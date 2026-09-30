/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "COM_utilities_dlss.hh"

#ifndef WITH_DLSS

namespace blender::compositor {

bool is_dlss_available()
{
  return false;
}

const char *dlss_last_error()
{
  return "Built without DLSS";
}

bool denoise_with_dlss(Context & /*context*/,
                       const Result & /*color*/,
                       const Result * /*albedo*/,
                       const Result * /*normal*/,
                       const Result * /*depth*/,
                       const Result * /*specular_albedo*/,
                       const Result * /*roughness*/,
                       const Result * /*motion*/,
                       const Result * /*specular_motion*/,
                       Result & /*output*/)
{
  return false;
}

}  // namespace blender::compositor

#else

#  include <algorithm>
#  include <cmath>
#  include <cstring>
#  include <mutex>
#  include <optional>
#  include <string>
#  include <vector>

#  include "BKE_appdir.hh"

#  include "BLI_array.hh"
#  include "BLI_fileops.hh"
#  include "BLI_math_vector.hh"
#  include "BLI_math_vector_types.hh"
#  include "BLI_path_utils.hh"
#  include "BLI_string.hh"
#  include "BLI_string_utf8.hh"
#  include "BLI_vector.hh"

#  include "CLG_log.h"

#  include "COM_utilities.hh"

#  include "GPU_texture.hh"

#  ifdef WITH_CUDA_DYNLOAD
#    include <cuew.h>
#  else
#    include <cuda.h>
#  endif

/* Do not define NVSDK_NGX_HEADER_ONLY here. Cycles already instantiates the
 * NGX CUDA wrappers in intern/cycles/integrator/denoiser_dlss.cpp; including
 * the header-only implementation again produces LNK2005 duplicates. */
#  include <nvsdk_ngx.h>
#  include <nvsdk_ngx_defs.h>
#  include <nvsdk_ngx_defs_dlssd.h>

static CLG_LogRef LOG = {"compositor.dlss"};

namespace blender::compositor {

/* Application ID for Blender from NVIDIA (same as Cycles). */
static const unsigned long long NGX_APPLICATION_ID = 100334311;

static char g_last_error[512] = "DLSS has not been initialized";

static void set_dlss_error(const char *msg)
{
  BLI_strncpy(g_last_error, msg ? msg : "", sizeof(g_last_error));
  CLOG_ERROR(&LOG, "%s", g_last_error);
}

static const char *ngx_result_name(const NVSDK_NGX_Result result)
{
  switch (unsigned(result)) {
    case unsigned(NVSDK_NGX_Result_Success):
      return "Success";
    case unsigned(NVSDK_NGX_Result_FAIL_FeatureNotSupported):
      return "FeatureNotSupported";
    case unsigned(NVSDK_NGX_Result_FAIL_PlatformError):
      return "PlatformError";
    case unsigned(NVSDK_NGX_Result_FAIL_FeatureAlreadyExists):
      return "FeatureAlreadyExists";
    case unsigned(NVSDK_NGX_Result_FAIL_FeatureNotFound):
      return "FeatureNotFound";
    case unsigned(NVSDK_NGX_Result_FAIL_InvalidParameter):
      return "InvalidParameter";
    case unsigned(NVSDK_NGX_Result_FAIL_NotInitialized):
      return "NotInitialized";
    case unsigned(NVSDK_NGX_Result_FAIL_MissingInput):
      return "MissingInput";
    case unsigned(NVSDK_NGX_Result_FAIL_UnableToInitializeFeature):
      return "UnableToInitializeFeature";
    case unsigned(NVSDK_NGX_Result_FAIL_OutOfDate):
      return "OutOfDate";
    case unsigned(NVSDK_NGX_Result_FAIL_OutOfGPUMemory):
      return "OutOfGPUMemory";
    case unsigned(NVSDK_NGX_Result_FAIL_Denied):
      return "Denied";
    default:
      return "Unknown";
  }
}

static std::wstring utf8_to_wstring(const char *src)
{
  wchar_t dst[1024];
  BLI_strncpy_wchar_from_utf8(dst, src ? src : "", 1024);
  return std::wstring(dst);
}

static bool cuda_check(const CUresult result, const char *what)
{
  if (result == CUDA_SUCCESS) {
    return true;
  }
  char msg[256];
  SNPRINTF(msg, "CUDA %s failed: %s", what, cuewErrorString(result));
  set_dlss_error(msg);
  return false;
}

struct CUDATexture {
  CUarray array = nullptr;
  CUtexObject texture_handle = 0;
  CUsurfObject surface_handle = 0;
  int width = 0;
  int height = 0;
  int channels = 0;

  bool init(const int width_in, const int height_in, const int num_channels)
  {
    this->destroy();
    width = width_in;
    height = height_in;
    channels = num_channels;

    CUDA_ARRAY_DESCRIPTOR desc = {};
    desc.Width = width;
    desc.Height = height;
    desc.Format = CU_AD_FORMAT_FLOAT;
    desc.NumChannels = num_channels;
    if (!cuda_check(cuArrayCreate(&array, &desc), "cuArrayCreate")) {
      return false;
    }

    CUDA_TEXTURE_DESC tex_desc = {};
    tex_desc.addressMode[0] = CU_TR_ADDRESS_MODE_CLAMP;
    tex_desc.addressMode[1] = CU_TR_ADDRESS_MODE_CLAMP;
    tex_desc.addressMode[2] = CU_TR_ADDRESS_MODE_CLAMP;
    tex_desc.flags = CU_TRSF_NORMALIZED_COORDINATES;

    CUDA_RESOURCE_DESC res_desc = {};
    res_desc.resType = CU_RESOURCE_TYPE_ARRAY;
    res_desc.res.array.hArray = array;

    if (!cuda_check(cuTexObjectCreate(&texture_handle, &res_desc, &tex_desc, nullptr),
                    "cuTexObjectCreate"))
    {
      this->destroy();
      return false;
    }
    if (!cuda_check(cuSurfObjectCreate(&surface_handle, &res_desc), "cuSurfObjectCreate")) {
      this->destroy();
      return false;
    }
    return true;
  }

  void destroy()
  {
    if (surface_handle) {
      cuSurfObjectDestroy(surface_handle);
      surface_handle = 0;
    }
    if (texture_handle) {
      cuTexObjectDestroy(texture_handle);
      texture_handle = 0;
    }
    if (array) {
      cuArrayDestroy(array);
      array = nullptr;
    }
    width = height = channels = 0;
  }

  bool upload(const float *host, CUstream stream) const
  {
    CUDA_MEMCPY2D copy = {};
    copy.srcMemoryType = CU_MEMORYTYPE_HOST;
    copy.srcHost = host;
    copy.srcPitch = size_t(width) * channels * sizeof(float);
    copy.dstMemoryType = CU_MEMORYTYPE_ARRAY;
    copy.dstArray = array;
    copy.WidthInBytes = copy.srcPitch;
    copy.Height = height;
    /* Same stream as EvaluateFeature, so the copies and the model overlap
     * instead of stalling once per guide buffer. */
    return cuda_check(cuMemcpy2DAsync(&copy, stream), "cuMemcpy2DAsync upload");
  }

  bool download(float *host, CUstream stream) const
  {
    CUDA_MEMCPY2D copy = {};
    copy.srcMemoryType = CU_MEMORYTYPE_ARRAY;
    copy.srcArray = array;
    copy.dstMemoryType = CU_MEMORYTYPE_HOST;
    copy.dstHost = host;
    copy.dstPitch = size_t(width) * channels * sizeof(float);
    copy.WidthInBytes = copy.dstPitch;
    copy.Height = height;
    return cuda_check(cuMemcpy2DAsync(&copy, stream), "cuMemcpy2DAsync download");
  }
};

class CompositorDLSS {
 public:
  static CompositorDLSS &instance()
  {
    static CompositorDLSS singleton;
    return singleton;
  }

  bool available()
  {
    std::lock_guard lock(mutex_);
    /* Pre-warm NGX so the first compositor eval is not a multi-hundred-ms hitch,
     * especially when Cycles viewport denoise is off and never inits DLSS. */
    return this->ensure_probe() && this->ensure_init();
  }

  bool denoise(const int2 size,
               const uint64_t view_key,
               const float *color,
               const float *albedo,
               const float *normal_roughness,
               const float *depth,
               const float *specular_albedo,
               const float *motion,
               const float *specular_motion,
               float *output,
               const bool upload_albedo,
               const bool upload_normal,
               const bool upload_depth,
               const bool upload_specular,
               const bool upload_motion,
               const bool upload_specular_motion)
  {
    std::lock_guard lock(mutex_);
    if (!this->ensure_probe()) {
      return false;
    }
    CUcontext previous = nullptr;
    cuCtxGetCurrent(&previous);
    auto restore_ctx = [&]() {
      if (previous != nullptr && previous != cu_context_) {
        cuCtxSetCurrent(previous);
      }
    };

    if (!this->ensure_init()) {
      restore_ctx();
      return false;
    }
    if (previous != cu_context_) {
      if (!cuda_check(cuCtxSetCurrent(cu_context_), "cuCtxSetCurrent")) {
        restore_ctx();
        return false;
      }
    }
    if (!this->ensure_feature(size.x, size.y)) {
      restore_ctx();
      return false;
    }

    const bool reset = (view_key != last_view_key_) || (last_view_key_ == uint64_t(-1));
    last_view_key_ = view_key;

    this->bind_host(reg_color_, host_color_);
    this->bind_host(reg_albedo_, host_albedo_);
    this->bind_host(reg_specular_, host_specular_);
    this->bind_host(reg_normal_, host_normal_rough_);
    this->bind_host(reg_depth_, host_depth_);
    this->bind_host(reg_motion_, host_motion_);
    this->bind_host(reg_spec_motion_, host_specular_motion_);
    this->bind_host(reg_output_, host_output_);

    if (!tex_color_.upload(color, cu_stream_)) {
      restore_ctx();
      return false;
    }
    if ((reset || upload_albedo) && !tex_diffuse_albedo_.upload(albedo, cu_stream_)) {
      restore_ctx();
      return false;
    }
    if ((reset || upload_normal) && !tex_normal_roughness_.upload(normal_roughness, cu_stream_)) {
      restore_ctx();
      return false;
    }
    if ((reset || upload_depth) && !tex_depth_.upload(depth, cu_stream_)) {
      restore_ctx();
      return false;
    }
    if ((reset || upload_specular) && !tex_specular_albedo_.upload(specular_albedo, cu_stream_)) {
      restore_ctx();
      return false;
    }
    if ((reset || upload_motion) && !tex_motion_.upload(motion, cu_stream_)) {
      restore_ctx();
      return false;
    }
    if ((reset || upload_specular_motion) &&
        !tex_specular_motion_.upload(specular_motion, cu_stream_))
    {
      restore_ctx();
      return false;
    }

    NVSDK_NGX_Parameter *params = nullptr;
    if (NVSDK_NGX_FAILED(NVSDK_NGX_CUDA_AllocateParameters(&params))) {
      set_dlss_error("Failed to allocate NGX parameters");
      restore_ctx();
      return false;
    }

    params->Set(NVSDK_NGX_Parameter_Reset, reset ? 1 : 0);
    params->Set(NVSDK_NGX_Parameter_Jitter_Offset_X, 0.0f);
    params->Set(NVSDK_NGX_Parameter_Jitter_Offset_Y, 0.0f);
    params->Set(NVSDK_NGX_Parameter_MV_Scale_X, 1.0f);
    params->Set(NVSDK_NGX_Parameter_MV_Scale_Y, 1.0f);
    params->Set(NVSDK_NGX_Parameter_Color, &tex_color_.texture_handle);
    params->Set(NVSDK_NGX_Parameter_Depth, &tex_depth_.texture_handle);
    params->Set(NVSDK_NGX_Parameter_DiffuseAlbedo, &tex_diffuse_albedo_.texture_handle);
    params->Set(NVSDK_NGX_Parameter_SpecularAlbedo, &tex_specular_albedo_.texture_handle);
    params->Set(NVSDK_NGX_Parameter_GBuffer_Normals, &tex_normal_roughness_.texture_handle);
    params->Set(NVSDK_NGX_Parameter_GBuffer_Roughness, &tex_normal_roughness_.texture_handle);
    params->Set(NVSDK_NGX_Parameter_MotionVectors, &tex_motion_.texture_handle);
    params->Set(NVSDK_NGX_Parameter_GBuffer_SpecularMvec, &tex_specular_motion_.texture_handle);
    params->Set(NVSDK_NGX_Parameter_Output, &tex_output_.surface_handle);
    params->Set(NVSDK_NGX_Parameter_DLSS_Render_Subrect_Dimensions_Width, size.x);
    params->Set(NVSDK_NGX_Parameter_DLSS_Render_Subrect_Dimensions_Height, size.y);
    params->Set(NVSDK_NGX_Parameter_DLSS_Indicator_Invert_Y_Axis, 1);

    const NVSDK_NGX_Result result = NVSDK_NGX_CUDA_EvaluateFeature(handle_, params, nullptr);
    NVSDK_NGX_CUDA_DestroyParameters(params);
    if (NVSDK_NGX_FAILED(result)) {
      char msg[256];
      SNPRINTF(msg, "DLSS EvaluateFeature failed (%s 0x%08x)", ngx_result_name(result), unsigned(result));
      set_dlss_error(msg);
      restore_ctx();
      return false;
    }

    if (!tex_output_.download(output, cu_stream_) ||
        !cuda_check(cuStreamSynchronize(cu_stream_), "cuStreamSynchronize"))
    {
      restore_ctx();
      return false;
    }
    restore_ctx();
    return true;
  }

 private:
  std::mutex mutex_;
  bool probed_ = false;
  bool probe_ok_ = false;
  bool initialized_ = false;
  bool owns_context_ = false;
  CUdevice cu_device_ = 0;
  int cu_device_index_ = 0;
  CUcontext cu_context_ = nullptr;
  CUstream cu_stream_ = nullptr;
  NVSDK_NGX_CUDADevice ngx_device_{};
  NVSDK_NGX_Handle *handle_ = nullptr;
  int feature_width_ = 0;
  int feature_height_ = 0;

  CUDATexture tex_color_;
  CUDATexture tex_depth_;
  CUDATexture tex_diffuse_albedo_;
  CUDATexture tex_specular_albedo_;
  CUDATexture tex_normal_roughness_;
  CUDATexture tex_motion_;
  CUDATexture tex_specular_motion_;
  CUDATexture tex_output_;

  std::wstring app_data_path_;
  std::vector<std::wstring> search_paths_;
  std::vector<const wchar_t *> search_path_ptrs_;
  bool primary_retained_ = false;
  uint64_t last_view_key_ = uint64_t(-1);
  Vector<float> host_color_;
  Vector<float> host_albedo_;
  Vector<float> host_specular_;
  Vector<float> host_normal_rough_;
  Vector<float> host_depth_;
  Vector<float> host_motion_;
  Vector<float> host_specular_motion_;
  Vector<float> host_output_;
  int2 host_size_ = int2(0, 0);
  bool default_guides_ready_ = false;

  /* Page-locked so cuMemcpy2DAsync does not bounce through a driver staging buffer. */
  struct HostReg {
    void *ptr = nullptr;
    size_t bytes = 0;
    bool pinned = false;

    void reset()
    {
      if (pinned && ptr != nullptr) {
        cuMemHostUnregister(ptr);
      }
      ptr = nullptr;
      bytes = 0;
      pinned = false;
    }

    void bind(void *p, const size_t n)
    {
      if (p == ptr && n == bytes) {
        return;
      }
      this->reset();
      if (p == nullptr || n == 0) {
        return;
      }
      if (cuMemHostRegister(p, n, CU_MEMHOSTREGISTER_PORTABLE) == CUDA_SUCCESS) {
        pinned = true;
      }
      ptr = p;
      bytes = n;
    }
  };
  HostReg reg_color_;
  HostReg reg_albedo_;
  HostReg reg_specular_;
  HostReg reg_normal_;
  HostReg reg_depth_;
  HostReg reg_motion_;
  HostReg reg_spec_motion_;
  HostReg reg_output_;

  void bind_host(HostReg &reg, const Vector<float> &buf)
  {
    reg.bind(const_cast<float *>(buf.data()), size_t(buf.size()) * sizeof(float));
  }

  void reset_host_regs()
  {
    reg_color_.reset();
    reg_albedo_.reset();
    reg_specular_.reset();
    reg_normal_.reset();
    reg_depth_.reset();
    reg_motion_.reset();
    reg_spec_motion_.reset();
    reg_output_.reset();
  }

 public:
  void ensure_host_buffers(const int2 size)
  {
    if (host_size_ == size) {
      return;
    }
    this->reset_host_regs();
    const int64_t n = int64_t(size.x) * size.y;
    host_color_.resize(n * 4);
    host_albedo_.resize(n * 4);
    host_specular_.resize(n * 4);
    host_normal_rough_.resize(n * 4);
    host_depth_.resize(n);
    host_motion_.resize(n * 2);
    host_specular_motion_.resize(n * 2);
    host_output_.resize(n * 4);
    host_size_ = size;
    default_guides_ready_ = false;
    last_view_key_ = uint64_t(-1);
  }

  Vector<float> &host_color()
  {
    return host_color_;
  }
  Vector<float> &host_albedo()
  {
    return host_albedo_;
  }
  Vector<float> &host_specular()
  {
    return host_specular_;
  }
  Vector<float> &host_normal_rough()
  {
    return host_normal_rough_;
  }
  Vector<float> &host_depth()
  {
    return host_depth_;
  }
  Vector<float> &host_motion()
  {
    return host_motion_;
  }
  Vector<float> &host_specular_motion()
  {
    return host_specular_motion_;
  }
  Vector<float> &host_output()
  {
    return host_output_;
  }
  bool &default_guides_ready()
  {
    return default_guides_ready_;
  }

 private:

  void collect_search_paths()
  {
    search_paths_.clear();
    auto add_dir = [&](const char *dir) {
      if (dir == nullptr || dir[0] == '\0') {
        return;
      }
      search_paths_.emplace_back(utf8_to_wstring(dir));
    };
    add_dir(BKE_appdir_program_dir());
    char extra[1024];
    BLI_path_join(extra, sizeof(extra), BKE_appdir_program_dir(), "dlss5");
    add_dir(extra);
    if (const std::optional<std::string> scripts = BKE_appdir_folder_id(BLENDER_SYSTEM_SCRIPTS,
                                                                        nullptr))
    {
      BLI_path_join(extra, sizeof(extra), scripts->c_str(), "addons_core", "cycles");
      add_dir(extra);
    }
    search_path_ptrs_.clear();
    search_path_ptrs_.reserve(search_paths_.size());
    for (const std::wstring &path : search_paths_) {
      search_path_ptrs_.push_back(path.c_str());
    }
  }

  bool ensure_probe()
  {
    if (probed_) {
      return probe_ok_;
    }
    probed_ = true;

#  ifdef WITH_CUDA_DYNLOAD
    if (cuewInit(CUEW_INIT_CUDA) != CUEW_SUCCESS) {
      set_dlss_error("Failed to load CUDA driver via CUEW");
      return false;
    }
#  endif
    if (!cuda_check(cuInit(0), "cuInit")) {
      return false;
    }

    int device_count = 0;
    if (!cuda_check(cuDeviceGetCount(&device_count), "cuDeviceGetCount") || device_count < 1) {
      set_dlss_error("No CUDA GPU found");
      return false;
    }

    bool found = false;
    for (int i = 0; i < device_count; i++) {
      CUdevice device = 0;
      if (!cuda_check(cuDeviceGet(&device, i), "cuDeviceGet")) {
        continue;
      }
      int major = 0;
      int minor = 0;
      cuDeviceGetAttribute(&major, CU_DEVICE_ATTRIBUTE_COMPUTE_CAPABILITY_MAJOR, device);
      cuDeviceGetAttribute(&minor, CU_DEVICE_ATTRIBUTE_COMPUTE_CAPABILITY_MINOR, device);
      if (major > 7 || (major == 7 && minor >= 5)) {
        cu_device_ = device;
        cu_device_index_ = i;
        found = true;
        break;
      }
    }
    if (!found) {
      set_dlss_error("No NVIDIA GPU with compute capability 7.5+");
      return false;
    }

    this->collect_search_paths();
    char cache[1024] = {};
    BKE_appdir_folder_caches(cache, sizeof(cache));
    char ngx_cache[1024];
    BLI_path_join(ngx_cache, sizeof(ngx_cache), cache, "dlss");
    BLI_dir_create_recursive(ngx_cache);
    app_data_path_ = utf8_to_wstring(ngx_cache);

    NVSDK_NGX_FeatureDiscoveryInfo discovery_info = {};
    discovery_info.SDKVersion = NVSDK_NGX_Version_API;
    discovery_info.FeatureID = NVSDK_NGX_Feature_RayReconstruction;
    discovery_info.Identifier.IdentifierType = NVSDK_NGX_Application_Identifier_Type_Application_Id;
    discovery_info.Identifier.v.ApplicationId = NGX_APPLICATION_ID;
    discovery_info.ApplicationDataPath = app_data_path_.c_str();

    NVSDK_NGX_FeatureRequirement requirement = {NVSDK_NGX_FeatureSupportResult_Supported};
    const NVSDK_NGX_Result result = NVSDK_NGX_CUDA_GetFeatureRequirements(
        cu_device_index_, &discovery_info, &requirement);
    if (NVSDK_NGX_FAILED(result) ||
        requirement.FeatureSupported != NVSDK_NGX_FeatureSupportResult_Supported)
    {
      char msg[256];
      SNPRINTF(msg,
               "DLSS Ray Reconstruction unsupported (%s 0x%08x)",
               ngx_result_name(result),
               unsigned(result));
      set_dlss_error(msg);
      return false;
    }

    probe_ok_ = true;
    return true;
  }

  bool ensure_init()
  {
    if (initialized_) {
      return true;
    }
    if (!this->ensure_probe()) {
      return false;
    }

    /* Own a dedicated primary context. Attaching to Cycles' current context made
     * compositor DLSS hitch whenever viewport denoise was off (Cycles never inits
     * NGX, and toggling it destroys the shared handle). */
    if (cuDevicePrimaryCtxRetain(&cu_context_, cu_device_) == CUDA_SUCCESS) {
      primary_retained_ = true;
      owns_context_ = false;
    }
    else if (!cuda_check(cuCtxCreate(&cu_context_, 0, cu_device_), "cuCtxCreate")) {
      return false;
    }
    else {
      owns_context_ = true;
    }
    if (!cuda_check(cuCtxSetCurrent(cu_context_), "cuCtxSetCurrent")) {
      return false;
    }
    if (!cuda_check(cuStreamCreate(&cu_stream_, 0), "cuStreamCreate")) {
      return false;
    }

    ngx_device_.cudaContext = cu_context_;
    ngx_device_.cudaStream = cu_stream_;

    if (search_path_ptrs_.empty()) {
      this->collect_search_paths();
    }
    NVSDK_NGX_FeatureCommonInfo feature_info = {};
    feature_info.PathListInfo.Path = search_path_ptrs_.data();
    feature_info.PathListInfo.Length = int(search_path_ptrs_.size());
    feature_info.LoggingInfo.MinimumLoggingLevel = NVSDK_NGX_LOGGING_LEVEL_ON;
    feature_info.LoggingInfo.LoggingCallback =
        [](const char *message, NVSDK_NGX_Logging_Level /*level*/, NVSDK_NGX_Feature) {
          CLOG_INFO(&LOG, "%s", message);
        };

    NVSDK_NGX_Result result = NVSDK_NGX_CUDA_Init1(
        NGX_APPLICATION_ID, app_data_path_.c_str(), &ngx_device_, &feature_info);
    if (NVSDK_NGX_FAILED(result) && result != NVSDK_NGX_Result_FAIL_FeatureAlreadyExists) {
      char msg[256];
      SNPRINTF(msg, "NGX CUDA init failed (%s 0x%08x)", ngx_result_name(result), unsigned(result));
      set_dlss_error(msg);
      return false;
    }

    initialized_ = true;
    return true;
  }

  void destroy_feature()
  {
    tex_color_.destroy();
    tex_depth_.destroy();
    tex_diffuse_albedo_.destroy();
    tex_specular_albedo_.destroy();
    tex_normal_roughness_.destroy();
    tex_motion_.destroy();
    tex_specular_motion_.destroy();
    tex_output_.destroy();
    if (handle_) {
      NVSDK_NGX_CUDA_ReleaseFeature(handle_);
      handle_ = nullptr;
    }
    feature_width_ = 0;
    feature_height_ = 0;
  }

  bool ensure_feature(const int width, const int height)
  {
    if (handle_ && feature_width_ == width && feature_height_ == height) {
      return true;
    }

    this->destroy_feature();

    if (width < 32 || height < 32) {
      set_dlss_error("DLSS requires at least 32x32 pixels");
      return false;
    }

    NVSDK_NGX_Parameter *params = nullptr;
    if (NVSDK_NGX_FAILED(NVSDK_NGX_CUDA_AllocateParameters(&params))) {
      set_dlss_error("Failed to allocate NGX create parameters");
      return false;
    }

    params->Set(NVSDK_NGX_Parameter_Input1, cu_context_);
    params->Set(NVSDK_NGX_Parameter_Input2, cu_stream_);
    params->Set(NVSDK_NGX_Parameter_Width, width);
    params->Set(NVSDK_NGX_Parameter_Height, height);
    params->Set(NVSDK_NGX_Parameter_OutWidth, width);
    params->Set(NVSDK_NGX_Parameter_OutHeight, height);
    params->Set(NVSDK_NGX_Parameter_PerfQualityValue, NVSDK_NGX_PerfQuality_Value_DLAA);
    params->Set(NVSDK_NGX_Parameter_DLSS_Denoise_Mode, NVSDK_NGX_DLSS_Denoise_Mode_DLUnified);
    params->Set(NVSDK_NGX_Parameter_DLSS_Feature_Create_Flags,
                NVSDK_NGX_DLSS_Feature_Flags_IsHDR | NVSDK_NGX_DLSS_Feature_Flags_MVLowRes);
    params->Set(NVSDK_NGX_Parameter_DLSS_Enable_Output_Subrects, 0);
    params->Set(NVSDK_NGX_Parameter_Use_HW_Depth, NVSDK_NGX_DLSS_Depth_Type_Linear);
    params->Set(NVSDK_NGX_Parameter_DLSS_Roughness_Mode, NVSDK_NGX_DLSS_Roughness_Mode_Packed);
    params->Set(NVSDK_NGX_Parameter_FreeMemOnReleaseFeature, 1);

    const NVSDK_NGX_Result result = NVSDK_NGX_CUDA_CreateFeature1(
        &ngx_device_, NVSDK_NGX_Feature_RayReconstruction, params, &handle_);
    NVSDK_NGX_CUDA_DestroyParameters(params);
    if (NVSDK_NGX_FAILED(result)) {
      char msg[256];
      SNPRINTF(msg, "Failed to create DLSS instance (%s 0x%08x)", ngx_result_name(result), unsigned(result));
      set_dlss_error(msg);
      handle_ = nullptr;
      return false;
    }

    if (!tex_color_.init(width, height, 4) || !tex_depth_.init(width, height, 1) ||
        !tex_diffuse_albedo_.init(width, height, 4) ||
        !tex_specular_albedo_.init(width, height, 4) ||
        !tex_normal_roughness_.init(width, height, 4) || !tex_motion_.init(width, height, 2) ||
        !tex_specular_motion_.init(width, height, 2) || !tex_output_.init(width, height, 4))
    {
      this->destroy_feature();
      return false;
    }

    feature_width_ = width;
    feature_height_ = height;
    return true;
  }
};

template<typename T>
static T sample_guide(const Result *result, const int2 texel, const T &fallback)
{
  if (result == nullptr || !result->is_allocated()) {
    return fallback;
  }
  return result->load_pixel_fallback<T, true>(texel, fallback);
}

bool is_dlss_available()
{
  return CompositorDLSS::instance().available();
}

const char *dlss_last_error()
{
  return g_last_error;
}

static bool is_varying_guide(const Result *result)
{
  return result != nullptr && result->is_allocated() && !result->is_single_value();
}

bool denoise_with_dlss(Context &context,
                       const Result &color,
                       const Result *albedo,
                       const Result *normal,
                       const Result *depth,
                       const Result *specular_albedo,
                       const Result *roughness,
                       const Result *motion,
                       const Result *specular_motion,
                       Result &output)
{
  if (!color.is_allocated() || color.is_single_value()) {
    return false;
  }

  const int2 size = color.domain().data_size;
  const int64_t pixel_count = int64_t(size.x) * int64_t(size.y);
  CompositorDLSS &dlss = CompositorDLSS::instance();
  dlss.ensure_host_buffers(size);

  const bool vary_albedo = is_varying_guide(albedo);
  const bool vary_normal = is_varying_guide(normal) || is_varying_guide(roughness);
  const bool vary_depth = is_varying_guide(depth);
  const bool vary_specular = is_varying_guide(specular_albedo);
  const bool vary_motion = is_varying_guide(motion);
  const bool vary_spec_motion = is_varying_guide(specular_motion);
  const bool fill_defaults = !dlss.default_guides_ready();

  /* One read straight into the CUDA staging buffer. Downloading to a temporary
   * Result and then memcpying it was a second full-frame copy every redraw. */
  if (color.is_stored_on_gpu()) {
    GPU_texture_read(color.gpu_texture(), GPU_DATA_FLOAT, 0, dlss.host_color().data());
  }
  else if (color.type() == ResultType::Color && color.cpu_data().size() == pixel_count) {
    memcpy(dlss.host_color().data(),
           color.cpu_data().data(),
           size_t(pixel_count) * sizeof(Color));
  }
  else {
    parallel_for(size, [&](const int2 texel) {
      const int64_t index = int64_t(texel.y) * size.x + texel.x;
      const Color pixel = color.load_pixel<Color>(texel);
      dlss.host_color()[index * 4 + 0] = pixel.r;
      dlss.host_color()[index * 4 + 1] = pixel.g;
      dlss.host_color()[index * 4 + 2] = pixel.b;
      dlss.host_color()[index * 4 + 3] = pixel.a;
    });
  }

  if (vary_albedo || vary_normal || vary_depth || vary_specular || vary_motion ||
      vary_spec_motion || fill_defaults)
  {
    parallel_for(size, [&](const int2 texel) {
      const int64_t index = int64_t(texel.y) * size.x + texel.x;
      if (vary_albedo || fill_defaults) {
        const Color albedo_pixel = sample_guide(albedo, texel, Color(1.0f, 1.0f, 1.0f, 1.0f));
        float3 albedo_value(albedo_pixel.r, albedo_pixel.g, albedo_pixel.b);
        albedo_value.x = std::clamp(albedo_value.x / (1.0f + albedo_value.x), 0.0f, 1.0f);
        albedo_value.y = std::clamp(albedo_value.y / (1.0f + albedo_value.y), 0.0f, 1.0f);
        albedo_value.z = std::clamp(albedo_value.z / (1.0f + albedo_value.z), 0.0f, 1.0f);
        dlss.host_albedo()[index * 4 + 0] = albedo_value.x;
        dlss.host_albedo()[index * 4 + 1] = albedo_value.y;
        dlss.host_albedo()[index * 4 + 2] = albedo_value.z;
        dlss.host_albedo()[index * 4 + 3] = 1.0f;
      }
      if (vary_specular || fill_defaults) {
        const Color spec_pixel = sample_guide(
            specular_albedo, texel, Color(0.0f, 0.0f, 0.0f, 1.0f));
        dlss.host_specular()[index * 4 + 0] = spec_pixel.r;
        dlss.host_specular()[index * 4 + 1] = spec_pixel.g;
        dlss.host_specular()[index * 4 + 2] = spec_pixel.b;
        dlss.host_specular()[index * 4 + 3] = 1.0f;
      }
      if (vary_normal || fill_defaults) {
        float3 n = sample_guide(normal, texel, float3(0.0f, 0.0f, 1.0f));
        if (math::length_squared(n) > 1.0e-8f) {
          n = math::normalize(n);
        }
        else {
          n = float3(0.0f, 0.0f, 1.0f);
        }
        dlss.host_normal_rough()[index * 4 + 0] = n.x;
        dlss.host_normal_rough()[index * 4 + 1] = n.y;
        dlss.host_normal_rough()[index * 4 + 2] = n.z;
        dlss.host_normal_rough()[index * 4 + 3] = sample_guide(roughness, texel, 0.5f);
      }
      if (vary_depth || fill_defaults) {
        float depth_value = sample_guide(depth, texel, 1.0f);
        if (!std::isfinite(depth_value)) {
          depth_value = 1.0e10f;
        }
        else if (depth_value < 0.0f) {
          depth_value = -depth_value;
        }
        else if (depth_value <= 1.0e-8f) {
          depth_value = 1.0e10f;
        }
        dlss.host_depth()[index] = depth_value;
      }
      if (vary_motion || fill_defaults) {
        const float2 mv = sample_guide(motion, texel, float2(0.0f));
        dlss.host_motion()[index * 2 + 0] = mv.x;
        dlss.host_motion()[index * 2 + 1] = mv.y;
      }
      if (vary_spec_motion || fill_defaults) {
        const float2 smv = sample_guide(specular_motion, texel, float2(0.0f));
        dlss.host_specular_motion()[index * 2 + 0] = smv.x;
        dlss.host_specular_motion()[index * 2 + 1] = smv.y;
      }
    });
    dlss.default_guides_ready() = true;
  }

  if (!dlss.denoise(size,
                    context.temporal_view_key(),
                    dlss.host_color().data(),
                    dlss.host_albedo().data(),
                    dlss.host_normal_rough().data(),
                    dlss.host_depth().data(),
                    dlss.host_specular().data(),
                    dlss.host_motion().data(),
                    dlss.host_specular_motion().data(),
                    dlss.host_output().data(),
                    vary_albedo,
                    vary_normal,
                    vary_depth,
                    vary_specular,
                    vary_motion,
                    vary_spec_motion))
  {
    return false;
  }

  output.allocate_texture(color.domain(), false, ResultStorageType::CPUImage);
  if (output.type() == ResultType::Color && output.cpu_data().size() == pixel_count) {
    memcpy(output.cpu_data_for_write().data(),
           dlss.host_output().data(),
           size_t(pixel_count) * sizeof(Color));
    Color *out_pixels = static_cast<Color *>(output.cpu_data_for_write().data());
    const float *src_a = dlss.host_color().data();
    parallel_for(size, [&](const int2 texel) {
      const int64_t index = int64_t(texel.y) * size.x + texel.x;
      out_pixels[index].a = src_a[index * 4 + 3];
    });
  }
  else {
    parallel_for(size, [&](const int2 texel) {
      const int64_t index = int64_t(texel.y) * size.x + texel.x;
      const float alpha = color.load_pixel<Color>(texel).a;
      output.store_pixel(texel,
                         Color(dlss.host_output()[index * 4 + 0],
                               dlss.host_output()[index * 4 + 1],
                               dlss.host_output()[index * 4 + 2],
                               alpha));
    });
  }
  return true;
}

}  // namespace blender::compositor

#endif
