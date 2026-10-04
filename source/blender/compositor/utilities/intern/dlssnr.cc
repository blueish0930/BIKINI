/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "COM_utilities_dlssnr.hh"

#ifndef WITH_DLSS

namespace blender::compositor {

bool is_dlssnr_available()
{
  return false;
}

const char *dlssnr_last_error()
{
  return "Built without DLSS";
}

bool apply_dlssnr(Context & /*context*/,
                  const Result & /*color*/,
                  const Result * /*depth*/,
                  const Result * /*motion*/,
                  const Result * /*mask*/,
                  const Result * /*albedo*/,
                  const Result * /*normal*/,
                  const DLSSNRParams & /*params*/,
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

#  include "GPU_context.hh"
#  include "GPU_texture.hh"

#  ifdef WITH_CUDA_DYNLOAD
#    include <cuew.h>
#  else
#    include <cuda.h>
#  endif

#  include "GHOST_IContext.hh"

#  ifdef _WIN32
#    define WIN32_LEAN_AND_MEAN
#    define NOMINMAX
#    include <Windows.h>
#    include <d3d12.h>
#    include <dxgi1_6.h>
#    include <wrl/client.h>
#  endif

#  include <nvsdk_ngx.h>
#  include <nvsdk_ngx_defs.h>
#  include <nvsdk_ngx_params.h>

static CLG_LogRef LOG = {"compositor.dlssnr"};

namespace blender::compositor {

static char g_last_error[512] = "nvngx_dlssnr.dll not found (place in blender/dlss5/)";

static void set_nr_error(const char *msg)
{
  BLI_strncpy(g_last_error, msg ? msg : "", sizeof(g_last_error));
  CLOG_ERROR(&LOG, "%s", g_last_error);
}

static std::wstring utf8_to_wstring(const char *src)
{
  wchar_t dst[1024];
  BLI_strncpy_wchar_from_utf8(dst, src ? src : "", 1024);
  return std::wstring(dst);
}

static uint16_t float_to_half(const float value)
{
  uint32_t bits = 0;
  memcpy(&bits, &value, sizeof(bits));
  const uint32_t sign = (bits >> 16) & 0x8000u;
  const int32_t exponent = int32_t((bits >> 23) & 0xFFu) - 127 + 15;
  uint32_t mantissa = bits & 0x7FFFFFu;
  if (exponent <= 0) {
    if (exponent < -10) {
      return uint16_t(sign);
    }
    mantissa = (mantissa | 0x800000u) >> (1 - exponent);
    return uint16_t(sign | ((mantissa + 0x1000u) >> 13));
  }
  if (exponent >= 31) {
    return uint16_t(sign | 0x7C00u);
  }
  return uint16_t(sign | (uint32_t(exponent) << 10) | ((mantissa + 0x1000u) >> 13));
}

static float half_to_float(const uint16_t value)
{
  const uint32_t sign = uint32_t(value & 0x8000u) << 16;
  uint32_t exponent = (value >> 10) & 0x1Fu;
  uint32_t mantissa = value & 0x3FFu;
  uint32_t bits;
  if (exponent == 0) {
    if (mantissa == 0) {
      bits = sign;
    }
    else {
      exponent = 127 - 15 + 1;
      while ((mantissa & 0x400u) == 0) {
        mantissa <<= 1;
        exponent--;
      }
      mantissa &= 0x3FFu;
      bits = sign | (exponent << 23) | (mantissa << 13);
    }
  }
  else if (exponent == 31) {
    bits = sign | 0x7F800000u | (mantissa << 13);
  }
  else {
    bits = sign | ((exponent + 127 - 15) << 23) | (mantissa << 13);
  }
  float out;
  memcpy(&out, &bits, sizeof(out));
  return out;
}

static bool find_nr_runtime_dir(char r_dir[1024])
{
  auto has_dll = [](const char *dir) -> bool {
    char path[1024];
    BLI_path_join(path, sizeof(path), dir, "nvngx_dlssnr.dll");
    return BLI_exists(path) && !BLI_is_dir(path);
  };

  const char *env = BLI_getenv("DLSS5_RUNTIME_DIR");
  if (env && env[0] && has_dll(env)) {
    BLI_strncpy(r_dir, env, 1024);
    return true;
  }

  const char *prog = BKE_appdir_program_dir();
  char candidate[1024];
  BLI_path_join(candidate, sizeof(candidate), prog, "dlss5");
  if (has_dll(candidate)) {
    BLI_strncpy(r_dir, candidate, 1024);
    return true;
  }
  if (has_dll(prog)) {
    BLI_strncpy(r_dir, prog, 1024);
    return true;
  }
  BLI_path_join(candidate, sizeof(candidate), prog, "5.3", "scripts", "addons_core", "cycles");
  if (has_dll(candidate)) {
    BLI_strncpy(r_dir, candidate, 1024);
    return true;
  }
  r_dir[0] = '\0';
  return false;
}

#  ifdef _WIN32

using Microsoft::WRL::ComPtr;

static constexpr unsigned long long kAppIdUnity = 0x0876232Cull;
static constexpr unsigned long long kAppIdPhoenix = 141959980ull;
static constexpr int kSdkSnippet = 0x15;
static constexpr int kFeatureId = 18;
static constexpr int kNgxSuccess = 0x1;

using NgxResult = int;
using GetModuleFileNameWFn = DWORD(WINAPI *)(HMODULE, LPWSTR, DWORD);
using NgxInitProjectFn = NgxResult(__cdecl *)(const char *,
                                              int,
                                              const char *,
                                              const wchar_t *,
                                              ID3D12Device *,
                                              const NVSDK_NGX_FeatureCommonInfo *,
                                              int);
using NgxInitProjectLegacyFn = NgxResult(__cdecl *)(const char *,
                                                    int,
                                                    const char *,
                                                    const wchar_t *,
                                                    ID3D12Device *,
                                                    int,
                                                    const NVSDK_NGX_FeatureCommonInfo *);
using NgxInitExtCoreFn = NgxResult(__cdecl *)(unsigned long long,
                                              const wchar_t *,
                                              ID3D12Device *,
                                              int,
                                              const NVSDK_NGX_FeatureCommonInfo *);
using NgxInitExtSnippetFn = NgxResult(__cdecl *)(unsigned long long,
                                                 const wchar_t *,
                                                 ID3D12Device *,
                                                 const NVSDK_NGX_FeatureCommonInfo *,
                                                 int);
using NgxShutdownFn = NgxResult(__cdecl *)(ID3D12Device *);
using NgxAllocFn = NgxResult(__cdecl *)(NVSDK_NGX_Parameter **);
using NgxDestroyPFn = NgxResult(__cdecl *)(NVSDK_NGX_Parameter *);
using NgxCreateFn = NgxResult(__cdecl *)(ID3D12GraphicsCommandList *,
                                         int,
                                         NVSDK_NGX_Parameter *,
                                         NVSDK_NGX_Handle **);
using NgxEvalFn = NgxResult(__cdecl *)(ID3D12GraphicsCommandList *,
                                       const NVSDK_NGX_Handle *,
                                       const NVSDK_NGX_Parameter *,
                                       void *);
using NgxReleaseFn = NgxResult(__cdecl *)(NVSDK_NGX_Handle *);

static HMODULE g_hooked_snippet = nullptr;
static HMODULE g_hooked_core = nullptr;
static GetModuleFileNameWFn g_original_get_module = nullptr;
static volatile LONG g_post_call_sink = 0;

static bool write_iat(void **slot, void *value)
{
  DWORD old_protect = 0;
  if (slot == nullptr || !VirtualProtect(slot, sizeof(void *), PAGE_READWRITE, &old_protect)) {
    return false;
  }
  InterlockedExchangePointer(slot, value);
  DWORD ignored = 0;
  VirtualProtect(slot, sizeof(void *), old_protect, &ignored);
  FlushInstructionCache(GetCurrentProcess(), slot, sizeof(void *));
  return true;
}

static DWORD WINAPI hooked_get_module_file_name_w(HMODULE module, LPWSTR filename, DWORD size)
{
  if (g_original_get_module == nullptr) {
    return 0;
  }
  if (g_hooked_snippet != nullptr && module == g_hooked_snippet) {
    return g_original_get_module(module, filename, size);
  }
  if (g_hooked_core != nullptr) {
    return g_original_get_module(g_hooked_core, filename, size);
  }
  if (filename == nullptr || size == 0) {
    return 0;
  }
  constexpr wchar_t fake_name[] = L"nvngx.dll";
  constexpr DWORD fake_length = ARRAYSIZE(fake_name) - 1;
  if (fake_length + 1 > size) {
    SetLastError(ERROR_INSUFFICIENT_BUFFER);
    return size;
  }
  memcpy(filename, fake_name, sizeof(fake_name));
  return fake_length;
}

static bool hook_snippet_get_module_file_name(HMODULE snippet, HMODULE core, void ***r_iat_slot)
{
  if (snippet == nullptr || r_iat_slot == nullptr) {
    return false;
  }
  auto *base = reinterpret_cast<BYTE *>(snippet);
  auto *dos = reinterpret_cast<IMAGE_DOS_HEADER *>(base);
  if (dos->e_magic != IMAGE_DOS_SIGNATURE) {
    return false;
  }
  auto *nt = reinterpret_cast<IMAGE_NT_HEADERS *>(base + dos->e_lfanew);
  if (nt->Signature != IMAGE_NT_SIGNATURE) {
    return false;
  }
  const auto &directory = nt->OptionalHeader.DataDirectory[IMAGE_DIRECTORY_ENTRY_IMPORT];
  if (directory.VirtualAddress == 0) {
    return false;
  }
  HMODULE kernel32 = GetModuleHandleW(L"kernel32.dll");
  const FARPROC wanted = kernel32 ? GetProcAddress(kernel32, "GetModuleFileNameW") : nullptr;
  auto *descriptor = reinterpret_cast<IMAGE_IMPORT_DESCRIPTOR *>(base + directory.VirtualAddress);
  for (; descriptor->Name != 0; ++descriptor) {
    const char *name = reinterpret_cast<const char *>(base + descriptor->Name);
    if (_stricmp(name, "KERNEL32.dll") != 0 && _stricmp(name, "KERNELBASE.dll") != 0) {
      continue;
    }
    auto *slots = reinterpret_cast<IMAGE_THUNK_DATA *>(base + descriptor->FirstThunk);
    auto *imports = descriptor->OriginalFirstThunk ?
                        reinterpret_cast<IMAGE_THUNK_DATA *>(base + descriptor->OriginalFirstThunk) :
                        nullptr;
    for (size_t index = 0; slots[index].u1.Function != 0; ++index) {
      bool match = false;
      if (imports != nullptr && imports[index].u1.AddressOfData != 0 &&
          !IMAGE_SNAP_BY_ORDINAL(imports[index].u1.Ordinal))
      {
        auto *entry = reinterpret_cast<IMAGE_IMPORT_BY_NAME *>(base + imports[index].u1.AddressOfData);
        match = strcmp(reinterpret_cast<const char *>(entry->Name), "GetModuleFileNameW") == 0;
      }
      else if (wanted && slots[index].u1.Function == reinterpret_cast<ULONG_PTR>(wanted)) {
        match = true;
      }
      if (!match) {
        continue;
      }
      auto *slot = reinterpret_cast<void **>(&slots[index].u1.Function);
      g_original_get_module = reinterpret_cast<GetModuleFileNameWFn>(*slot);
      g_hooked_snippet = snippet;
      g_hooked_core = core;
      if (!write_iat(slot, reinterpret_cast<void *>(&hooked_get_module_file_name_w))) {
        g_original_get_module = nullptr;
        g_hooked_snippet = nullptr;
        g_hooked_core = nullptr;
        return false;
      }
      *r_iat_slot = slot;
      return true;
    }
  }
  return false;
}

static __declspec(noinline) NgxResult finish_call(NgxResult result)
{
  g_post_call_sink = LONG(result);
  return result;
}

static __declspec(noinline) NgxResult call_snippet_init(NgxInitExtSnippetFn fn,
                                                        unsigned long long app_id,
                                                        const wchar_t *path,
                                                        ID3D12Device *device,
                                                        const NVSDK_NGX_FeatureCommonInfo *info,
                                                        int version)
{
  return finish_call(fn(app_id, path, device, info, version));
}

static __declspec(noinline) NgxResult call_snippet_init_core_abi(NgxInitExtCoreFn fn,
                                                                 unsigned long long app_id,
                                                                 const wchar_t *path,
                                                                 ID3D12Device *device,
                                                                 int version,
                                                                 const NVSDK_NGX_FeatureCommonInfo *info)
{
  return finish_call(fn(app_id, path, device, version, info));
}

static __declspec(noinline) NgxResult call_create(NgxCreateFn fn,
                                                  ID3D12GraphicsCommandList *list,
                                                  int feature,
                                                  NVSDK_NGX_Parameter *params,
                                                  NVSDK_NGX_Handle **handle)
{
  return finish_call(fn(list, feature, params, handle));
}

static __declspec(noinline) NgxResult call_eval(NgxEvalFn fn,
                                                ID3D12GraphicsCommandList *list,
                                                const NVSDK_NGX_Handle *handle,
                                                NVSDK_NGX_Parameter *params)
{
  return finish_call(fn(list, handle, params, nullptr));
}

static std::wstring find_driver_store_core()
{
  wchar_t windows_dir[MAX_PATH] = {};
  if (GetWindowsDirectoryW(windows_dir, MAX_PATH) == 0) {
    return {};
  }
  const std::wstring repo = std::wstring(windows_dir) + L"\\System32\\DriverStore\\FileRepository";
  const std::wstring pattern = repo + L"\\nv*.inf_*";
  struct Candidate {
    std::wstring path;
    unsigned long long stamp = 0;
    bool preferred = false;
  };
  std::vector<Candidate> candidates;
  WIN32_FIND_DATAW fd = {};
  HANDLE find = FindFirstFileW(pattern.c_str(), &fd);
  if (find == INVALID_HANDLE_VALUE) {
    return {};
  }
  do {
    if ((fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) == 0) {
      continue;
    }
    const std::wstring candidate = repo + L"\\" + fd.cFileName + L"\\_nvngx.dll";
    WIN32_FILE_ATTRIBUTE_DATA fad = {};
    if (!GetFileAttributesExW(candidate.c_str(), GetFileExInfoStandard, &fad) ||
        (fad.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY))
    {
      continue;
    }
    ULARGE_INTEGER stamp;
    stamp.LowPart = fad.ftLastWriteTime.dwLowDateTime;
    stamp.HighPart = fad.ftLastWriteTime.dwHighDateTime;
    candidates.push_back({candidate, stamp.QuadPart, wcsncmp(fd.cFileName, L"nv_dispi", 8) == 0});
  } while (FindNextFileW(find, &fd));
  FindClose(find);
  std::sort(candidates.begin(), candidates.end(), [](const Candidate &a, const Candidate &b) {
    if (a.preferred != b.preferred) {
      return a.preferred;
    }
    return a.stamp > b.stamp;
  });
  return candidates.empty() ? std::wstring() : candidates.front().path;
}

static std::wstring ngx_data_directory()
{
  const char *env = BLI_getenv("DLSS5_CACHE_DIR");
  if (env && env[0]) {
    return utf8_to_wstring(env);
  }
  char cache[1024] = {};
  BKE_appdir_folder_caches(cache, sizeof(cache));
  char path[1024];
  BLI_path_join(path, sizeof(path), cache, "dlss5");
  BLI_dir_create_recursive(path);
  return utf8_to_wstring(path);
}

class ScopedYieldBlenderGPU {
 public:
  /* Viewport compositor runs inside DRW_draw_view. Unbinding GHOST there destroys the window
   * GPU context and the next node's Result* can go null (mute crash in EEVEE viewport). Only
   * isolate on the compositor job thread. */
  explicit ScopedYieldBlenderGPU(const bool isolate)
  {
    if (!isolate) {
      return;
    }
    ghost_ = GHOST_IContext::getActiveDrawingContext();
    gpu_ctx_ = GPU_context_active_get();
    if (gpu_ctx_ != nullptr) {
      GPU_context_active_set(nullptr);
    }
    if (ghost_ != nullptr) {
      ghost_->releaseDrawingContext();
    }
  }

  ~ScopedYieldBlenderGPU()
  {
    if (ghost_ != nullptr) {
      ghost_->activateDrawingContext();
    }
    if (gpu_ctx_ != nullptr) {
      GPU_context_active_set(gpu_ctx_);
    }
  }

  ScopedYieldBlenderGPU(const ScopedYieldBlenderGPU &) = delete;
  ScopedYieldBlenderGPU &operator=(const ScopedYieldBlenderGPU &) = delete;

 private:
  GPUContext *gpu_ctx_ = nullptr;
  GHOST_IContext *ghost_ = nullptr;
};

class Dx12NR {
 public:
  bool available()
  {
    char dir[1024];
    if (!find_nr_runtime_dir(dir)) {
      set_nr_error("nvngx_dlssnr.dll not found. Put it in blender/dlss5/ "
                   "(or set DLSS5_RUNTIME_DIR)");
      return false;
    }
    return true;
  }

  bool apply(const int2 size,
             const float *color,
             const float *depth,
             const float *motion,
             const bool have_depth,
             const bool have_motion,
             const DLSSNRParams &nr,
             float *output,
             const bool isolate_gpu)
  {
    if (size.x < 32 || size.y < 32) {
      set_nr_error("DLSSNR requires at least 32x32 pixels");
      return false;
    }
    ScopedYieldBlenderGPU yield_gpu(isolate_gpu);
    if (!this->ensure_init()) {
      return false;
    }
    if (!this->ensure_feature(size.x, size.y, have_depth, have_motion, nr)) {
      return false;
    }
    /* New view / camera / first eval → Reset=1. Same EEVEE TAA view → keep history. */
    const bool do_reset = nr.reset || last_view_key_ != nr.view_key;
    this->set_params(size.x, size.y, have_depth, have_motion, nr, do_reset);
    if (!this->stage_inputs(color, depth, motion, size.x, size.y, have_depth, have_motion, nr.clamp_input))
    {
      return false;
    }

    const NgxResult eval = call_eval(nr_eval_, cmdlist_.Get(), handle_, params_);
    if (eval != kNgxSuccess) {
      char msg[256];
      SNPRINTF(msg, "EvaluateFeature failed (0x%08x)", unsigned(eval));
      set_nr_error(msg);
      this->execute_and_wait();
      return false;
    }

    this->barrier(tex_output_.Get(),
                  D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
                  D3D12_RESOURCE_STATE_COPY_SOURCE);
    if (!this->copy_texture_to_readback(tex_output_.Get(), size.x, size.y)) {
      return false;
    }
    this->barrier(tex_output_.Get(),
                  D3D12_RESOURCE_STATE_COPY_SOURCE,
                  D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
    if (!this->execute_and_wait()) {
      return false;
    }
    if (!this->download_color(output, color, size.x, size.y, nr.clamp_input)) {
      return false;
    }
    last_view_key_ = nr.view_key;
    has_history_ = !do_reset;
    return true;
  }

 private:
  bool probed_adapter_ = false;
  bool adapter_ok_ = false;
  bool initialized_ = false;
  bool has_history_ = false;
  uint64_t last_view_key_ = 0;
  bool have_depth_ = false;
  bool have_motion_ = false;
  int feature_width_ = 0;
  int feature_height_ = 0;
  int feature_style_ = -999;
  UINT64 fence_value_ = 0;
  UINT color_pitch_ = 0;
  UINT64 color_bytes_ = 0;

  HMODULE core_module_ = nullptr;
  HMODULE snippet_module_ = nullptr;
  void **iat_slot_ = nullptr;
  NgxInitProjectFn core_init_project_ = nullptr;
  NgxInitProjectLegacyFn core_init_project_legacy_ = nullptr;
  NgxInitExtCoreFn core_init_ext_ = nullptr;
  NgxShutdownFn core_shutdown_ = nullptr;
  NgxAllocFn alloc_params_ = nullptr;
  NgxDestroyPFn destroy_params_ = nullptr;
  NgxInitExtSnippetFn nr_init_snippet_ = nullptr;
  NgxInitExtCoreFn nr_init_core_abi_ = nullptr;
  NgxCreateFn nr_create_ = nullptr;
  NgxEvalFn nr_eval_ = nullptr;
  NgxReleaseFn nr_release_ = nullptr;

  ComPtr<IDXGIAdapter1> adapter_;
  ComPtr<ID3D12Device> device_;
  ComPtr<ID3D12CommandQueue> queue_;
  ComPtr<ID3D12CommandAllocator> allocator_;
  ComPtr<ID3D12GraphicsCommandList> cmdlist_;
  ComPtr<ID3D12Fence> fence_;
  HANDLE fence_event_ = nullptr;
  NVSDK_NGX_Parameter *params_ = nullptr;
  NVSDK_NGX_Handle *handle_ = nullptr;
  ComPtr<ID3D12Resource> tex_color_;
  ComPtr<ID3D12Resource> tex_output_;
  ComPtr<ID3D12Resource> tex_depth_;
  ComPtr<ID3D12Resource> tex_motion_;
  ComPtr<ID3D12Resource> upload_;
  ComPtr<ID3D12Resource> readback_;
  D3D12_PLACED_SUBRESOURCE_FOOTPRINT fp_color_ = {};
  D3D12_PLACED_SUBRESOURCE_FOOTPRINT fp_depth_ = {};
  D3D12_PLACED_SUBRESOURCE_FOOTPRINT fp_motion_ = {};
  D3D12_PLACED_SUBRESOURCE_FOOTPRINT fp_output_ = {};
  std::wstring runtime_dir_;
  std::wstring data_dir_;
  const wchar_t *feature_paths_[1] = {nullptr};

  static UINT64 align512(const UINT64 value)
  {
    return (value + 511ull) & ~511ull;
  }

  bool footprint_for(ID3D12Resource *resource,
                     const UINT64 base_offset,
                     D3D12_PLACED_SUBRESOURCE_FOOTPRINT &fp,
                     UINT64 &r_end)
  {
    const D3D12_RESOURCE_DESC desc = resource->GetDesc();
    UINT64 total = 0;
    device_->GetCopyableFootprints(&desc, 0, 1, base_offset, &fp, nullptr, nullptr, &total);
    r_end = fp.Offset + UINT64(fp.Footprint.RowPitch) * UINT64(fp.Footprint.Height);
    return fp.Footprint.RowPitch != 0 && fp.Footprint.Height != 0;
  }

  static bool ngx_ok(NgxResult result)
  {
    return result == kNgxSuccess;
  }

  bool ensure_adapter()
  {
    if (probed_adapter_) {
      return adapter_ok_;
    }
    probed_adapter_ = true;
    ComPtr<IDXGIFactory1> factory;
    if (FAILED(CreateDXGIFactory1(IID_PPV_ARGS(&factory)))) {
      set_nr_error("CreateDXGIFactory1 failed");
      return false;
    }
    for (UINT i = 0;; i++) {
      ComPtr<IDXGIAdapter1> adapter;
      if (factory->EnumAdapters1(i, &adapter) == DXGI_ERROR_NOT_FOUND) {
        break;
      }
      DXGI_ADAPTER_DESC1 desc = {};
      adapter->GetDesc1(&desc);
      if ((desc.Flags & DXGI_ADAPTER_FLAG_SOFTWARE) || desc.VendorId != 0x10DE) {
        continue;
      }
      adapter_ = adapter;
      adapter_ok_ = true;
      return true;
    }
    set_nr_error("No NVIDIA D3D12 adapter");
    return false;
  }

  ComPtr<ID3D12Resource> create_texture(const UINT width,
                                        const UINT height,
                                        const DXGI_FORMAT format,
                                        const D3D12_RESOURCE_STATES state,
                                        const D3D12_RESOURCE_FLAGS flags)
  {
    D3D12_RESOURCE_DESC desc = {};
    desc.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
    desc.Width = width;
    desc.Height = height;
    desc.DepthOrArraySize = 1;
    desc.MipLevels = 1;
    desc.Format = format;
    desc.SampleDesc.Count = 1;
    desc.Layout = D3D12_TEXTURE_LAYOUT_UNKNOWN;
    desc.Flags = flags;
    D3D12_HEAP_PROPERTIES heap = {};
    heap.Type = D3D12_HEAP_TYPE_DEFAULT;
    ComPtr<ID3D12Resource> resource;
    if (FAILED(device_->CreateCommittedResource(
            &heap, D3D12_HEAP_FLAG_NONE, &desc, state, nullptr, IID_PPV_ARGS(&resource))))
    {
      return nullptr;
    }
    return resource;
  }

  ComPtr<ID3D12Resource> create_buffer(const UINT64 bytes,
                                       const D3D12_HEAP_TYPE type,
                                       const D3D12_RESOURCE_STATES state)
  {
    D3D12_RESOURCE_DESC desc = {};
    desc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
    desc.Width = bytes;
    desc.Height = 1;
    desc.DepthOrArraySize = 1;
    desc.MipLevels = 1;
    desc.SampleDesc.Count = 1;
    desc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
    D3D12_HEAP_PROPERTIES heap = {};
    heap.Type = type;
    ComPtr<ID3D12Resource> resource;
    if (FAILED(device_->CreateCommittedResource(
            &heap, D3D12_HEAP_FLAG_NONE, &desc, state, nullptr, IID_PPV_ARGS(&resource))))
    {
      return nullptr;
    }
    return resource;
  }

  void barrier(ID3D12Resource *resource,
               const D3D12_RESOURCE_STATES before,
               const D3D12_RESOURCE_STATES after)
  {
    if (before == after || resource == nullptr) {
      return;
    }
    D3D12_RESOURCE_BARRIER b = {};
    b.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
    b.Transition.pResource = resource;
    b.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
    b.Transition.StateBefore = before;
    b.Transition.StateAfter = after;
    cmdlist_->ResourceBarrier(1, &b);
  }

  bool execute_and_wait()
  {
    if (FAILED(cmdlist_->Close())) {
      set_nr_error("D3D12 command list Close failed");
      return false;
    }
    ID3D12CommandList *lists[] = {cmdlist_.Get()};
    queue_->ExecuteCommandLists(1, lists);
    fence_value_++;
    if (FAILED(queue_->Signal(fence_.Get(), fence_value_))) {
      set_nr_error("D3D12 queue Signal failed");
      return false;
    }
    if (fence_->GetCompletedValue() < fence_value_) {
      if (FAILED(fence_->SetEventOnCompletion(fence_value_, fence_event_))) {
        set_nr_error("D3D12 SetEventOnCompletion failed");
        return false;
      }
      if (WaitForSingleObject(fence_event_, 5000) != WAIT_OBJECT_0) {
        set_nr_error("Timed out waiting for DLSSNR GPU work");
        return false;
      }
    }
    if (FAILED(allocator_->Reset()) || FAILED(cmdlist_->Reset(allocator_.Get(), nullptr))) {
      set_nr_error("D3D12 command list Reset failed");
      return false;
    }
    return true;
  }

  bool load_modules()
  {
    char dir_utf8[1024];
    if (!find_nr_runtime_dir(dir_utf8)) {
      set_nr_error("nvngx_dlssnr.dll not found. Put it in blender/dlss5/");
      return false;
    }
    runtime_dir_ = utf8_to_wstring(dir_utf8);
    data_dir_ = ngx_data_directory();

    const std::wstring local_core = runtime_dir_ + L"\\_nvngx.dll";
    const std::wstring packaged = runtime_dir_ + L"\\nvngx.dll";
    const std::wstring driver_core = find_driver_store_core();
    const wchar_t *core_candidates[] = {
        driver_core.c_str(), local_core.c_str(), packaged.c_str(), L"_nvngx.dll"};
    for (const wchar_t *path : core_candidates) {
      if (path == nullptr || path[0] == 0) {
        continue;
      }
      core_module_ = LoadLibraryExW(path, nullptr, LOAD_WITH_ALTERED_SEARCH_PATH);
      if (core_module_) {
        CLOG_INFO(&LOG, "DLSSNR NGX core loaded");
        break;
      }
    }
    if (!core_module_) {
      set_nr_error("Could not load NGX core (_nvngx.dll / driver)");
      return false;
    }

    const std::wstring snippet_path = runtime_dir_ + L"\\nvngx_dlssnr.dll";
    snippet_module_ = LoadLibraryExW(snippet_path.c_str(), nullptr, LOAD_WITH_ALTERED_SEARCH_PATH);
    if (!snippet_module_) {
      char msg[256];
      SNPRINTF(msg, "LoadLibrary nvngx_dlssnr.dll failed (Win32 %lu)", GetLastError());
      set_nr_error(msg);
      return false;
    }
    if (!hook_snippet_get_module_file_name(snippet_module_, core_module_, &iat_slot_)) {
      CLOG_WARN(&LOG, "DLSSNR GetModuleFileNameW hook failed; CreateFeature may return 0xBAD00002");
    }

    core_init_project_ = reinterpret_cast<NgxInitProjectFn>(
        GetProcAddress(core_module_, "NVSDK_NGX_D3D12_Init_with_ProjectID"));
    core_init_project_legacy_ = reinterpret_cast<NgxInitProjectLegacyFn>(
        GetProcAddress(core_module_, "NVSDK_NGX_D3D12_Init_ProjectID"));
    core_init_ext_ = reinterpret_cast<NgxInitExtCoreFn>(
        GetProcAddress(core_module_, "NVSDK_NGX_D3D12_Init_Ext"));
    core_shutdown_ = reinterpret_cast<NgxShutdownFn>(
        GetProcAddress(core_module_, "NVSDK_NGX_D3D12_Shutdown1"));
    alloc_params_ = reinterpret_cast<NgxAllocFn>(
        GetProcAddress(core_module_, "NVSDK_NGX_D3D12_AllocateParameters"));
    destroy_params_ = reinterpret_cast<NgxDestroyPFn>(
        GetProcAddress(core_module_, "NVSDK_NGX_D3D12_DestroyParameters"));
    nr_init_snippet_ = reinterpret_cast<NgxInitExtSnippetFn>(
        GetProcAddress(snippet_module_, "NVSDK_NGX_D3D12_Init_Ext"));
    nr_init_core_abi_ = reinterpret_cast<NgxInitExtCoreFn>(
        GetProcAddress(snippet_module_, "NVSDK_NGX_D3D12_Init_Ext"));
    nr_create_ = reinterpret_cast<NgxCreateFn>(
        GetProcAddress(snippet_module_, "NVSDK_NGX_D3D12_CreateFeature"));
    nr_eval_ = reinterpret_cast<NgxEvalFn>(
        GetProcAddress(snippet_module_, "NVSDK_NGX_D3D12_EvaluateFeature"));
    nr_release_ = reinterpret_cast<NgxReleaseFn>(
        GetProcAddress(snippet_module_, "NVSDK_NGX_D3D12_ReleaseFeature"));
    if (!alloc_params_ || !nr_create_ || !nr_eval_ || !nr_release_ || !nr_init_snippet_) {
      set_nr_error("nvngx_dlssnr.dll is missing D3D12 Feature 18 exports");
      return false;
    }
    return true;
  }

  bool init_ngx_session()
  {
    feature_paths_[0] = runtime_dir_.c_str();
    NVSDK_NGX_FeatureCommonInfo common = {};
    common.PathListInfo.Path = feature_paths_;
    common.PathListInfo.Length = 1;
    common.LoggingInfo.MinimumLoggingLevel = NVSDK_NGX_LOGGING_LEVEL_ON;

    bool core_ok = false;
    if (core_init_project_) {
      core_ok = ngx_ok(core_init_project_("e40890d0-da76-467b-8130-12f3ada8d7c8",
                                         NVSDK_NGX_ENGINE_TYPE_UNITY,
                                         "6000.3",
                                         data_dir_.c_str(),
                                         device_.Get(),
                                         &common,
                                         kSdkSnippet));
    }
    if (!core_ok && core_init_project_legacy_) {
      core_ok = ngx_ok(core_init_project_legacy_("e40890d0-da76-467b-8130-12f3ada8d7c8",
                                                NVSDK_NGX_ENGINE_TYPE_UNITY,
                                                "6000.3",
                                                data_dir_.c_str(),
                                                device_.Get(),
                                                kSdkSnippet,
                                                &common));
    }
    if (!core_ok && core_init_project_legacy_) {
      core_ok = ngx_ok(core_init_project_legacy_("53f803cc-a12f-4d69-90d5-19b7599cad19",
                                                NVSDK_NGX_ENGINE_TYPE_CUSTOM,
                                                "0.3.0",
                                                data_dir_.c_str(),
                                                device_.Get(),
                                                kSdkSnippet,
                                                &common));
    }
    if (!core_ok && core_init_ext_) {
      for (int ver = 0x13; ver <= 0x20 && !core_ok; ver++) {
        core_ok = ngx_ok(core_init_ext_(
            kAppIdUnity, data_dir_.c_str(), device_.Get(), ver, &common));
      }
    }
    if (!core_ok) {
      set_nr_error("NGX core Init failed (ProjectID / Init_Ext)");
      return false;
    }

    bool snippet_ok = ngx_ok(call_snippet_init(nr_init_snippet_,
                                               kAppIdUnity,
                                               data_dir_.c_str(),
                                               device_.Get(),
                                               &common,
                                               kSdkSnippet));
    if (!snippet_ok) {
      snippet_ok = ngx_ok(call_snippet_init(nr_init_snippet_,
                                           kAppIdPhoenix,
                                           data_dir_.c_str(),
                                           device_.Get(),
                                           &common,
                                           kSdkSnippet));
    }
    if (!snippet_ok && nr_init_core_abi_) {
      snippet_ok = ngx_ok(call_snippet_init_core_abi(nr_init_core_abi_,
                                                    kAppIdUnity,
                                                    data_dir_.c_str(),
                                                    device_.Get(),
                                                    kSdkSnippet,
                                                    &common));
    }
    if (!snippet_ok) {
      set_nr_error("nvngx_dlssnr.dll Init_Ext failed (0xBAD00002 = caller check)");
      return false;
    }
    if (!ngx_ok(alloc_params_(&params_)) || params_ == nullptr) {
      set_nr_error("AllocateParameters failed");
      return false;
    }
    return true;
  }

  bool ensure_init()
  {
    if (initialized_) {
      return true;
    }
    if (!this->ensure_adapter()) {
      return false;
    }
    if (FAILED(D3D12CreateDevice(adapter_.Get(), D3D_FEATURE_LEVEL_12_0, IID_PPV_ARGS(&device_)))) {
      set_nr_error("D3D12CreateDevice failed");
      return false;
    }
    D3D12_COMMAND_QUEUE_DESC qdesc = {};
    qdesc.Type = D3D12_COMMAND_LIST_TYPE_DIRECT;
    if (FAILED(device_->CreateCommandQueue(&qdesc, IID_PPV_ARGS(&queue_))) ||
        FAILED(device_->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT,
                                               IID_PPV_ARGS(&allocator_))) ||
        FAILED(device_->CreateCommandList(0,
                                          D3D12_COMMAND_LIST_TYPE_DIRECT,
                                          allocator_.Get(),
                                          nullptr,
                                          IID_PPV_ARGS(&cmdlist_))) ||
        FAILED(device_->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&fence_))))
    {
      set_nr_error("D3D12 queue/allocator/fence create failed");
      return false;
    }
    fence_event_ = CreateEventW(nullptr, FALSE, FALSE, nullptr);
    if (!fence_event_) {
      set_nr_error("CreateEventW failed");
      return false;
    }
    if (!this->load_modules() || !this->init_ngx_session()) {
      return false;
    }
    initialized_ = true;
    return true;
  }

  void set_params(const int width,
                  const int height,
                  const bool have_depth,
                  const bool have_motion,
                  const DLSSNRParams &nr,
                  const bool do_reset)
  {
    const unsigned reset = do_reset ? 1u : 0u;
    params_->Set("DLSSNR.Color", tex_color_.Get());
    params_->Set("DLSSNR.Output", tex_output_.Get());
    /* On reset, do not feed last frame's output as history. That is why orbiting the
     * viewport kept ghosting until the user re-ran the viewer several times. */
    params_->Set("DLSSNR.Backbuffer", do_reset ? tex_color_.Get() : tex_output_.Get());
    params_->Set("DLSSNR.Depth", have_depth ? tex_depth_.Get() : static_cast<ID3D12Resource *>(nullptr));
    params_->Set("DLSSNR.MVec", have_motion ? tex_motion_.Get() : static_cast<ID3D12Resource *>(nullptr));
    params_->Set("Width", width);
    params_->Set("Height", height);
    params_->Set("OutWidth", width);
    params_->Set("OutHeight", height);
    params_->Set("DLSSNR.Width", unsigned(width));
    params_->Set("DLSSNR.Height", unsigned(height));
    params_->Set("DLSSNR.Enabled", 1u);
    params_->Set("DLSSNR.Reset", reset);
    params_->Set(NVSDK_NGX_Parameter_Reset, reset);
    params_->Set("DLSSNR.JitterOffsetX", 0.0f);
    params_->Set("DLSSNR.JitterOffsetY", 0.0f);
    params_->Set("DLSSNR.DepthInverted", 1u);
    params_->Set("DLSSNR.ScalingRatio", 1.0f);
    params_->Set("DLSSNR.ColorSubrectBaseX", 0u);
    params_->Set("DLSSNR.ColorSubrectBaseY", 0u);
    params_->Set("DLSSNR.ColorSubrectWidth", unsigned(width));
    params_->Set("DLSSNR.ColorSubrectHeight", unsigned(height));
    params_->Set("DLSSNR.OutputSubrectBaseX", 0u);
    params_->Set("DLSSNR.OutputSubrectBaseY", 0u);
    params_->Set("DLSSNR.OutputSubrectWidth", unsigned(width));
    params_->Set("DLSSNR.OutputSubrectHeight", unsigned(height));
    params_->Set("DLSSNR.DepthSubrectBaseX", 0u);
    params_->Set("DLSSNR.DepthSubrectBaseY", 0u);
    params_->Set("DLSSNR.DepthSubrectWidth", have_depth ? unsigned(width) : 0u);
    params_->Set("DLSSNR.DepthSubrectHeight", have_depth ? unsigned(height) : 0u);
    params_->Set("DLSSNR.MVecSubrectBaseX", 0u);
    params_->Set("DLSSNR.MVecSubrectBaseY", 0u);
    params_->Set("DLSSNR.MVecSubrectWidth", have_motion ? unsigned(width) : 0u);
    params_->Set("DLSSNR.MVecSubrectHeight", have_motion ? unsigned(height) : 0u);
    params_->Set("DLSSNR.MVecScaleX", have_motion ? 1.0f : 1.0f);
    params_->Set("DLSSNR.MVecScaleY", have_motion ? 1.0f : 1.0f);
    params_->Set("DLSSNR.Intensity", std::clamp(nr.intensity, 0.0f, 2.0f));
    params_->Set("DLSSNR.LocalToneStrength", std::clamp(nr.tone, 0.0f, 2.0f));
    params_->Set("DLSSNR.LocalStructureStrength", std::clamp(nr.structure, 0.0f, 2.0f));
    params_->Set("DLSSNR.SkinStructureStrength", nr.skin);
    params_->Set("DLSSNR.Style", std::clamp(nr.style, 0, 2));
    params_->Set("DLSSNR.UseAutoMask", nr.auto_mask ? 1u : 0u);
    params_->Set("DLSSNR.UICorrection", nr.ui_correction ? 1u : 0u);
    params_->Set("CreationNodeMask", 1);
    params_->Set("VisibilityNodeMask", 1);
  }

  void release_feature()
  {
    if (handle_ && nr_release_) {
      nr_release_(handle_);
      handle_ = nullptr;
    }
    if (params_) {
      params_->Set("DLSSNR.MVec", static_cast<ID3D12Resource *>(nullptr));
      params_->Set("DLSSNR.Depth", static_cast<ID3D12Resource *>(nullptr));
    }
    tex_color_.Reset();
    tex_output_.Reset();
    tex_depth_.Reset();
    tex_motion_.Reset();
    upload_.Reset();
    readback_.Reset();
    feature_width_ = 0;
    feature_height_ = 0;
    feature_style_ = -999;
    have_depth_ = false;
    have_motion_ = false;
    has_history_ = false;
    last_view_key_ = 0;
  }

  bool ensure_feature(const int width,
                      const int height,
                      const bool have_depth,
                      const bool have_motion,
                      const DLSSNRParams &nr)
  {
    const int style = std::clamp(nr.style, 0, 2);
    if (handle_ && feature_width_ == width && feature_height_ == height &&
        feature_style_ == style && have_depth_ == have_depth && have_motion_ == have_motion)
    {
      return true;
    }
    this->release_feature();

    tex_color_ = this->create_texture(UINT(width),
                                      UINT(height),
                                      DXGI_FORMAT_R16G16B16A16_FLOAT,
                                      D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE,
                                      D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS);
    tex_output_ = this->create_texture(UINT(width),
                                       UINT(height),
                                       DXGI_FORMAT_R16G16B16A16_FLOAT,
                                       D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
                                       D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS);
    if (have_depth) {
      tex_depth_ = this->create_texture(UINT(width),
                                        UINT(height),
                                        DXGI_FORMAT_R32_FLOAT,
                                        D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE,
                                        D3D12_RESOURCE_FLAG_NONE);
    }
    if (have_motion) {
      tex_motion_ = this->create_texture(UINT(width),
                                         UINT(height),
                                         DXGI_FORMAT_R16G16_FLOAT,
                                         D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE,
                                         D3D12_RESOURCE_FLAG_NONE);
    }
    UINT64 upload_end = 0;
    if (!this->footprint_for(tex_color_.Get(), 0, fp_color_, upload_end)) {
      this->release_feature();
      return false;
    }
    if (have_depth &&
        !this->footprint_for(tex_depth_.Get(), this->align512(upload_end), fp_depth_, upload_end))
    {
      this->release_feature();
      return false;
    }
    if (have_motion &&
        !this->footprint_for(tex_motion_.Get(), this->align512(upload_end), fp_motion_, upload_end))
    {
      this->release_feature();
      return false;
    }
    UINT64 readback_end = 0;
    if (!this->footprint_for(tex_output_.Get(), 0, fp_output_, readback_end)) {
      this->release_feature();
      return false;
    }
    color_pitch_ = fp_output_.Footprint.RowPitch;
    color_bytes_ = readback_end;
    upload_ = this->create_buffer(
        this->align512(upload_end), D3D12_HEAP_TYPE_UPLOAD, D3D12_RESOURCE_STATE_GENERIC_READ);
    readback_ = this->create_buffer(
        this->align512(readback_end), D3D12_HEAP_TYPE_READBACK, D3D12_RESOURCE_STATE_COPY_DEST);
    if (!tex_color_ || !tex_output_ || !upload_ || !readback_ || (have_depth && !tex_depth_) ||
        (have_motion && !tex_motion_))
    {
      set_nr_error("Failed to create DLSSNR D3D12 textures");
      this->release_feature();
      return false;
    }

    this->set_params(width, height, have_depth, have_motion, nr, true);
    const NgxResult created = call_create(
        nr_create_, cmdlist_.Get(), kFeatureId, params_, &handle_);
    if (!ngx_ok(created) || handle_ == nullptr) {
      char msg[256];
      SNPRINTF(msg,
               "CreateFeature(18) failed (0x%08x). Need signed nvngx_dlssnr.dll + RTX 50",
               unsigned(created));
      set_nr_error(msg);
      handle_ = nullptr;
      this->execute_and_wait();
      this->release_feature();
      return false;
    }
    if (!this->execute_and_wait()) {
      this->release_feature();
      return false;
    }
    feature_width_ = width;
    feature_height_ = height;
    feature_style_ = style;
    have_depth_ = have_depth;
    have_motion_ = have_motion;
    CLOG_INFO(&LOG, "DLSSNR feature created %dx%d style=%d", width, height, style);
    return true;
  }

  void copy_texture_from_upload(ID3D12Resource *tex, const D3D12_PLACED_SUBRESOURCE_FOOTPRINT &fp)
  {
    this->barrier(tex,
                  D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE,
                  D3D12_RESOURCE_STATE_COPY_DEST);
    D3D12_TEXTURE_COPY_LOCATION dst = {};
    dst.pResource = tex;
    dst.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
    D3D12_TEXTURE_COPY_LOCATION src = {};
    src.pResource = upload_.Get();
    src.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
    src.PlacedFootprint = fp;
    cmdlist_->CopyTextureRegion(&dst, 0, 0, 0, &src, nullptr);
    this->barrier(tex, D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
  }

  bool stage_inputs(const float *color,
                    const float *depth,
                    const float *motion,
                    const int width,
                    const int height,
                    const bool have_depth,
                    const bool have_motion,
                    const bool clamp_input)
  {
    uint8_t *mapped = nullptr;
    if (FAILED(upload_->Map(0, nullptr, reinterpret_cast<void **>(&mapped)))) {
      set_nr_error("Upload Map failed");
      return false;
    }

    /* Compositor CPU buffers are bottom-up (same as image.pixels). D3D12/NR is top-left. */
    for (int y = 0; y < height; y++) {
      const int src_y = height - 1 - y;
      auto *row = reinterpret_cast<uint16_t *>(mapped + fp_color_.Offset +
                                               size_t(y) * fp_color_.Footprint.RowPitch);
      for (int x = 0; x < width; x++) {
        const float *px = color + (int64_t(src_y) * width + x) * 4;
        float r = px[0];
        float g = px[1];
        float b = px[2];
        if (clamp_input) {
          r = std::clamp(r, 0.0f, 1.0f);
          g = std::clamp(g, 0.0f, 1.0f);
          b = std::clamp(b, 0.0f, 1.0f);
        }
        row[x * 4 + 0] = float_to_half(r);
        row[x * 4 + 1] = float_to_half(g);
        row[x * 4 + 2] = float_to_half(b);
        row[x * 4 + 3] = float_to_half(1.0f);
      }
    }
    if (have_depth) {
      for (int y = 0; y < height; y++) {
        const int src_y = height - 1 - y;
        memcpy(mapped + fp_depth_.Offset + size_t(y) * fp_depth_.Footprint.RowPitch,
               depth + int64_t(src_y) * width,
               size_t(width) * 4);
      }
    }
    if (have_motion) {
      for (int y = 0; y < height; y++) {
        const int src_y = height - 1 - y;
        auto *row = reinterpret_cast<uint16_t *>(mapped + fp_motion_.Offset +
                                                 size_t(y) * fp_motion_.Footprint.RowPitch);
        for (int x = 0; x < width; x++) {
          const float *mv = motion + (int64_t(src_y) * width + x) * 2;
          row[x * 2 + 0] = float_to_half(mv[0]);
          row[x * 2 + 1] = float_to_half(-mv[1]);
        }
      }
    }
    upload_->Unmap(0, nullptr);

    this->copy_texture_from_upload(tex_color_.Get(), fp_color_);
    if (have_depth) {
      this->copy_texture_from_upload(tex_depth_.Get(), fp_depth_);
    }
    if (have_motion) {
      this->copy_texture_from_upload(tex_motion_.Get(), fp_motion_);
    }
    return true;
  }

  bool copy_texture_to_readback(ID3D12Resource *tex, const int /*width*/, const int /*height*/)
  {
    D3D12_TEXTURE_COPY_LOCATION dst = {};
    dst.pResource = readback_.Get();
    dst.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
    dst.PlacedFootprint = fp_output_;
    D3D12_TEXTURE_COPY_LOCATION src = {};
    src.pResource = tex;
    src.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
    cmdlist_->CopyTextureRegion(&dst, 0, 0, 0, &src, nullptr);
    return true;
  }

  bool download_color(float *out,
                      const float *original,
                      const int width,
                      const int height,
                      const bool clamp_input)
  {
    uint8_t *mapped = nullptr;
    if (FAILED(readback_->Map(0, nullptr, reinterpret_cast<void **>(&mapped)))) {
      set_nr_error("Readback Map failed");
      return false;
    }
    const int64_t n = int64_t(width) * height;
    Array<float> raw(n * 4);
    const UINT pitch = fp_output_.Footprint.RowPitch ? fp_output_.Footprint.RowPitch : color_pitch_;
    for (int y = 0; y < height; y++) {
      const int dst_y = height - 1 - y;
      const auto *row = reinterpret_cast<const uint16_t *>(mapped + fp_output_.Offset +
                                                           size_t(y) * pitch);
      for (int x = 0; x < width; x++) {
        const int64_t i = (int64_t(dst_y) * width + x) * 4;
        raw[i + 0] = half_to_float(row[x * 4 + 0]);
        raw[i + 1] = half_to_float(row[x * 4 + 1]);
        raw[i + 2] = half_to_float(row[x * 4 + 2]);
        raw[i + 3] = half_to_float(row[x * 4 + 3]);
      }
    }
    readback_->Unmap(0, nullptr);

    double diff_rgb = 0.0;
    double diff_bgr = 0.0;
    constexpr int stride = 17;
    for (int64_t i = 0; i < n; i += stride) {
      const float r = raw[i * 4 + 0];
      const float g = raw[i * 4 + 1];
      const float b = raw[i * 4 + 2];
      const float ir = original[i * 4 + 0];
      const float ig = original[i * 4 + 1];
      const float ib = original[i * 4 + 2];
      diff_rgb += std::fabs(r - ir) + std::fabs(g - ig) + std::fabs(b - ib);
      diff_bgr += std::fabs(b - ir) + std::fabs(g - ig) + std::fabs(r - ib);
    }
    const bool swap_rb = diff_bgr < diff_rgb;
    for (int64_t i = 0; i < n; i++) {
      const int64_t p = i * 4;
      float r = swap_rb ? raw[p + 2] : raw[p + 0];
      float g = raw[p + 1];
      float b = swap_rb ? raw[p + 0] : raw[p + 2];
      if (clamp_input) {
        r = std::clamp(r, 0.0f, 1.0f);
        g = std::clamp(g, 0.0f, 1.0f);
        b = std::clamp(b, 0.0f, 1.0f);
      }
      out[p + 0] = r;
      out[p + 1] = g;
      out[p + 2] = b;
      out[p + 3] = original[p + 3];
    }
    return true;
  }
};

/* Fast path: Feature 18 over CUDA (float textures, no D3D12, no Vulkan yield).
 * Falls back to Dx12NR when the snippet rejects the CUDA caller. */
class CudaNR {
 public:
  bool apply(const int2 size,
             const float *color,
             const float *depth,
             const float *motion,
             const bool have_depth,
             const bool have_motion,
             const DLSSNRParams &nr,
             float *output,
             const bool isolate_gpu)
  {
    if (failed_permanently_) {
      return false;
    }
    if (size.x < 32 || size.y < 32) {
      return false;
    }
    if (!this->ensure_init(isolate_gpu)) {
      failed_permanently_ = true;
      return false;
    }
    /* CUDA context is thread-local. Bind the context ensure_init created
     * (primary after a viewport warmup, private when F12 inits first). */
    CUcontext previous = nullptr;
    cuCtxGetCurrent(&previous);
    if (cuCtxSetCurrent(context_) != CUDA_SUCCESS) {
      set_nr_error("cuCtxSetCurrent failed on compositor thread");
      return false;
    }
    auto restore_ctx = [&]() {
      if (previous != nullptr && previous != context_) {
        cuCtxSetCurrent(previous);
      }
    };
    if (!this->ensure_feature(size.x, size.y, have_depth, have_motion, nr)) {
      restore_ctx();
      return false;
    }
    const bool do_reset = nr.reset || last_view_key_ != nr.view_key;
    if (!this->upload(color, depth, motion, size.x, size.y, have_depth, have_motion, nr.clamp_input))
    {
      restore_ctx();
      return false;
    }
    this->set_params(size.x, size.y, have_depth, have_motion, nr, do_reset);
    const NgxResult eval = cuda_eval_(handle_, params_, nullptr);
    if (eval != kNgxSuccess) {
      cuStreamSynchronize(stream_);
      char msg[256];
      SNPRINTF(msg, "CUDA EvaluateFeature(18) failed (0x%08x)", unsigned(eval));
      set_nr_error(msg);
      restore_ctx();
      return false;
    }
    if (!tex_output_.download_flipped(
            output, size.x, size.y, color, nr.clamp_input, stage_output_, stream_))
    {
      set_nr_error("CUDA download failed after DLSSNR");
      restore_ctx();
      return false;
    }
    last_view_key_ = nr.view_key;
    restore_ctx();
    return true;
  }

 private:
  /* Page-locked staging. One buffer per image: async copies must not share storage. */
  struct Stage {
    float *pinned_ptr = nullptr;
    size_t cap = 0;
    bool pinned = false;
    Vector<float> pageable;

    float *ensure(const size_t n)
    {
      if (n == 0) {
        return nullptr;
      }
      if (pinned && pinned_ptr != nullptr && cap >= n) {
        return pinned_ptr;
      }
      if (pinned_ptr != nullptr) {
        cuMemFreeHost(pinned_ptr);
        pinned_ptr = nullptr;
        cap = 0;
        pinned = false;
      }
      void *ptr = nullptr;
      if (cuMemHostAlloc(&ptr, n * sizeof(float), CU_MEMHOSTALLOC_PORTABLE) == CUDA_SUCCESS) {
        pinned_ptr = static_cast<float *>(ptr);
        cap = n;
        pinned = true;
        return pinned_ptr;
      }
      pinned = false;
      if (pageable.size() < int64_t(n)) {
        pageable.resize(int64_t(n));
      }
      return pageable.data();
    }
  };

  struct Tex {
    CUarray array = nullptr;
    CUtexObject tex = 0;
    CUsurfObject surf = 0;
    int width = 0;
    int height = 0;
    int channels = 0;

    bool init(const int w, const int h, const int ch)
    {
      this->destroy();
      width = w;
      height = h;
      channels = ch;
      CUDA_ARRAY_DESCRIPTOR desc = {};
      desc.Width = w;
      desc.Height = h;
      desc.Format = CU_AD_FORMAT_FLOAT;
      desc.NumChannels = ch;
      if (cuArrayCreate(&array, &desc) != CUDA_SUCCESS) {
        return false;
      }
      CUDA_TEXTURE_DESC td = {};
      td.addressMode[0] = td.addressMode[1] = td.addressMode[2] = CU_TR_ADDRESS_MODE_CLAMP;
      td.flags = CU_TRSF_NORMALIZED_COORDINATES;
      CUDA_RESOURCE_DESC rd = {};
      rd.resType = CU_RESOURCE_TYPE_ARRAY;
      rd.res.array.hArray = array;
      if (cuTexObjectCreate(&tex, &rd, &td, nullptr) != CUDA_SUCCESS) {
        this->destroy();
        return false;
      }
      if (cuSurfObjectCreate(&surf, &rd) != CUDA_SUCCESS) {
        this->destroy();
        return false;
      }
      return true;
    }
    void destroy()
    {
      if (surf) {
        cuSurfObjectDestroy(surf);
        surf = 0;
      }
      if (tex) {
        cuTexObjectDestroy(tex);
        tex = 0;
      }
      if (array) {
        cuArrayDestroy(array);
        array = nullptr;
      }
    }
    bool upload_flipped(const float *host,
                        const int src_channels,
                        const bool clamp_rgb,
                        Stage &stage,
                        CUstream stream) const
    {
      const size_t floats = size_t(width) * size_t(height) * size_t(channels);
      float *staged = stage.ensure(floats);
      if (channels == 4 && src_channels == 4 && !clamp_rgb) {
        parallel_for(int2(1, height), [&](const int2 texel) {
          const int y = texel.y;
          memcpy(staged + int64_t(y) * width * 4,
                 host + int64_t(height - 1 - y) * width * 4,
                 size_t(width) * 4 * sizeof(float));
        });
      }
      else {
        parallel_for(int2(1, height), [&](const int2 row) {
          const int y = row.y;
          const float *src_row = host + int64_t(height - 1 - y) * width * src_channels;
          float *dst_row = staged + int64_t(y) * width * channels;
          for (int x = 0; x < width; x++) {
            const float *px = src_row + x * src_channels;
            float *dst = dst_row + x * channels;
          if (channels == 1) {
            dst[0] = px[0];
          }
          else if (channels == 2) {
            dst[0] = px[0];
            dst[1] = -px[1];
          }
          else {
            float r = px[0], g = px[1], b = px[2];
            if (clamp_rgb) {
              r = std::clamp(r, 0.0f, 1.0f);
              g = std::clamp(g, 0.0f, 1.0f);
              b = std::clamp(b, 0.0f, 1.0f);
            }
            dst[0] = r;
            dst[1] = g;
            dst[2] = b;
            dst[3] = 1.0f;
          }
          }
        });
      }
      CUDA_MEMCPY2D copy = {};
      copy.srcMemoryType = CU_MEMORYTYPE_HOST;
      copy.srcHost = staged;
      copy.srcPitch = size_t(width) * channels * sizeof(float);
      copy.dstMemoryType = CU_MEMORYTYPE_ARRAY;
      copy.dstArray = array;
      copy.WidthInBytes = copy.srcPitch;
      copy.Height = height;
      if (stage.pinned && stream != nullptr) {
        return cuMemcpy2DAsync(&copy, stream) == CUDA_SUCCESS;
      }
      return cuMemcpy2D(&copy) == CUDA_SUCCESS;
    }
    bool download_flipped(float *out,
                          const int /*w*/,
                          const int /*h*/,
                          const float *original,
                          const bool clamp_rgb,
                          Stage &stage,
                          CUstream stream) const
    {
      const size_t floats = size_t(width) * size_t(height) * size_t(channels);
      float *staged = stage.ensure(floats);
      CUDA_MEMCPY2D copy = {};
      copy.srcMemoryType = CU_MEMORYTYPE_ARRAY;
      copy.srcArray = array;
      copy.dstMemoryType = CU_MEMORYTYPE_HOST;
      copy.dstHost = staged;
      copy.dstPitch = size_t(width) * channels * sizeof(float);
      copy.WidthInBytes = copy.dstPitch;
      copy.Height = height;
      const CUresult copied = (stage.pinned && stream != nullptr) ? cuMemcpy2DAsync(&copy, stream) :
                                                                    cuMemcpy2D(&copy);
      if (copied != CUDA_SUCCESS || (stream != nullptr && cuStreamSynchronize(stream) != CUDA_SUCCESS))
      {
        return false;
      }
      if (channels == 4 && !clamp_rgb) {
        parallel_for(int2(1, height), [&](const int2 texel) {
          const int y = texel.y;
          const int dst_y = height - 1 - y;
          float *dst = out + int64_t(dst_y) * width * 4;
          memcpy(dst, staged + int64_t(y) * width * 4, size_t(width) * 4 * sizeof(float));
          const float *src = original + int64_t(dst_y) * width * 4;
          for (int x = 0; x < width; x++) {
            dst[x * 4 + 3] = src[x * 4 + 3];
          }
        });
        return true;
      }
      parallel_for(int2(width, height), [&](const int2 texel) {
        const float *px = staged + (int64_t(texel.y) * width + texel.x) * channels;
        const int dst_y = height - 1 - texel.y;
        float *dst = out + (int64_t(dst_y) * width + texel.x) * 4;
        const float *src = original + (int64_t(dst_y) * width + texel.x) * 4;
        float r = px[0], g = px[1], b = px[2];
        if (clamp_rgb) {
          r = std::clamp(r, 0.0f, 1.0f);
          g = std::clamp(g, 0.0f, 1.0f);
          b = std::clamp(b, 0.0f, 1.0f);
        }
        dst[0] = r;
        dst[1] = g;
        dst[2] = b;
        dst[3] = src[3];
      });
      return true;
    }
  };

  using NgxCudaInit1Fn = NgxResult(__cdecl *)(unsigned long long,
                                              const wchar_t *,
                                              NVSDK_NGX_CUDADevice *,
                                              const NVSDK_NGX_FeatureCommonInfo *,
                                              int);
  using NgxCudaCreate1Fn = NgxResult(__cdecl *)(NVSDK_NGX_CUDADevice *,
                                                int,
                                                NVSDK_NGX_Parameter *,
                                                NVSDK_NGX_Handle **);
  using NgxCudaEvalFn = NgxResult(__cdecl *)(const NVSDK_NGX_Handle *,
                                             const NVSDK_NGX_Parameter *,
                                             void *);
  using NgxCudaReleaseFn = NgxResult(__cdecl *)(NVSDK_NGX_Handle *);
  using NgxCudaAllocFn = NVSDK_NGX_Result(NVSDK_CONV *)(NVSDK_NGX_Parameter **);

  bool failed_permanently_ = false;
  bool initialized_ = false;
  uint64_t last_view_key_ = 0;
  int feature_width_ = 0;
  int feature_height_ = 0;
  int feature_style_ = -999;
  bool have_depth_ = false;
  bool have_motion_ = false;
  CUdevice device_ = 0;
  CUcontext context_ = nullptr;
  CUstream stream_ = nullptr;
  bool primary_retained_ = false;
  /* F12-first init. Not Cycles' primary context (that one stays on the render thread). */
  bool private_context_ = false;
  NVSDK_NGX_CUDADevice ngx_device_{};
  HMODULE snippet_ = nullptr;
  HMODULE core_ = nullptr;
  void **iat_slot_ = nullptr;
  NgxCudaInit1Fn cuda_init1_ = nullptr;
  NgxCudaCreate1Fn cuda_create1_ = nullptr;
  NgxCudaEvalFn cuda_eval_ = nullptr;
  NgxCudaReleaseFn cuda_release_ = nullptr;
  NVSDK_NGX_Parameter *params_ = nullptr;
  NVSDK_NGX_Handle *handle_ = nullptr;
  Tex tex_color_;
  Tex tex_output_;
  Tex tex_depth_;
  Tex tex_motion_;
  std::wstring runtime_dir_;
  std::wstring data_dir_;
  const wchar_t *feature_paths_[1] = {nullptr};
  Stage stage_color_;
  Stage stage_depth_;
  Stage stage_motion_;
  Stage stage_output_;

  /* Official wrapper (Cycles' HEADER_ONLY NGX) and the dlssnr snippet do not
   * share init state. Snippet Init1 can succeed while
   * NVSDK_NGX_CUDA_AllocateParameters still returns NotInitialized — that is
   * the F12 failure when the viewport compositor never warmed this singleton
   * on the draw thread. */
  bool allocate_parameters()
  {
    if (cuCtxSetCurrent(context_) != CUDA_SUCCESS) {
      set_nr_error("cuCtxSetCurrent failed before AllocateParameters");
      return false;
    }

    NVSDK_NGX_FeatureCommonInfo official_info = {};
    official_info.PathListInfo.Path = feature_paths_;
    official_info.PathListInfo.Length = feature_paths_[0] ? 1 : 0;
    official_info.LoggingInfo.MinimumLoggingLevel = NVSDK_NGX_LOGGING_LEVEL_ON;
    /* Same application id as Cycles / compositor DLSS, so this thread joins
     * that NGX instance instead of allocating against an uninitialized wrapper. */
    const NVSDK_NGX_Result official = NVSDK_NGX_CUDA_Init1(
        100334311ull, data_dir_.c_str(), &ngx_device_, &official_info);
    if (NVSDK_NGX_FAILED(official) && official != NVSDK_NGX_Result_FAIL_FeatureAlreadyExists) {
      CLOG_WARN(&LOG,
                "Official NGX CUDA Init1 returned 0x%08x (continuing with snippet)",
                unsigned(official));
    }
    if (cuCtxSetCurrent(context_) != CUDA_SUCCESS) {
      set_nr_error("cuCtxSetCurrent failed after official NGX init");
      return false;
    }

    auto try_alloc = [&](NgxCudaAllocFn fn, const char *via) -> bool {
      if (fn == nullptr) {
        return false;
      }
      params_ = nullptr;
      const NVSDK_NGX_Result result = fn(&params_);
      if (!NVSDK_NGX_FAILED(result) && params_ != nullptr) {
        CLOG_INFO(&LOG, "DLSSNR parameters allocated via %s", via);
        return true;
      }
      CLOG_WARN(&LOG, "CUDA AllocateParameters via %s failed (0x%08x)", via, unsigned(result));
      params_ = nullptr;
      return false;
    };

    NgxCudaAllocFn snippet_alloc = nullptr;
    NgxCudaAllocFn core_alloc = nullptr;
    if (snippet_ != nullptr) {
      snippet_alloc = reinterpret_cast<NgxCudaAllocFn>(
          GetProcAddress(snippet_, "NVSDK_NGX_CUDA_AllocateParameters"));
    }
    if (core_ != nullptr) {
      core_alloc = reinterpret_cast<NgxCudaAllocFn>(
          GetProcAddress(core_, "NVSDK_NGX_CUDA_AllocateParameters"));
    }
    if (try_alloc(NVSDK_NGX_CUDA_AllocateParameters, "official") ||
        try_alloc(snippet_alloc, "snippet") || try_alloc(core_alloc, "core"))
    {
      return true;
    }
    set_nr_error("CUDA AllocateParameters failed");
    return false;
  }

  bool ensure_init(const bool isolate_gpu)
  {
    if (initialized_) {
      return true;
    }
#  ifdef WITH_CUDA_DYNLOAD
    if (cuewInit(CUEW_INIT_CUDA) != CUEW_SUCCESS) {
      set_nr_error("CUDA driver load failed for DLSSNR");
      return false;
    }
#  endif
    if (cuInit(0) != CUDA_SUCCESS) {
      set_nr_error("cuInit failed for DLSSNR");
      return false;
    }
    int count = 0;
    if (cuDeviceGetCount(&count) != CUDA_SUCCESS || count < 1) {
      set_nr_error("No CUDA GPU for DLSSNR");
      return false;
    }
    if (cuDeviceGet(&device_, 0) != CUDA_SUCCESS) {
      return false;
    }
    /* Viewport compositor inits on the draw thread and may keep the primary
     * context (Cycles uses it too). F12's first init runs on the job thread,
     * where retaining that primary context makes AllocateParameters fail and
     * the D3D12 fallback is disabled because it deadlocks. Own a private
     * context in that case. */
    if (isolate_gpu) {
      if (cuCtxCreate(&context_, 0, device_) != CUDA_SUCCESS) {
        set_nr_error("cuCtxCreate failed for DLSSNR");
        return false;
      }
      private_context_ = true;
    }
    else if (cuDevicePrimaryCtxRetain(&context_, device_) == CUDA_SUCCESS) {
      primary_retained_ = true;
    }
    else if (cuCtxCreate(&context_, 0, device_) != CUDA_SUCCESS) {
      set_nr_error("cuCtxCreate failed for DLSSNR");
      return false;
    }
    else {
      private_context_ = true;
    }
    if (cuCtxSetCurrent(context_) != CUDA_SUCCESS) {
      set_nr_error("cuCtxSetCurrent failed for DLSSNR");
      return false;
    }
    if (cuStreamCreate(&stream_, 0) != CUDA_SUCCESS) {
      set_nr_error("cuStreamCreate failed for DLSSNR");
      return false;
    }
    ngx_device_.cudaContext = context_;
    ngx_device_.cudaStream = stream_;

    char dir_utf8[1024];
    if (!find_nr_runtime_dir(dir_utf8)) {
      set_nr_error("nvngx_dlssnr.dll not found. Put it in blender/dlss5/");
      return false;
    }
    runtime_dir_ = utf8_to_wstring(dir_utf8);
    data_dir_ = ngx_data_directory();

    const std::wstring driver_core = find_driver_store_core();
    const std::wstring local_core = runtime_dir_ + L"\\_nvngx.dll";
    const wchar_t *cores[] = {driver_core.c_str(), local_core.c_str(), L"_nvngx.dll"};
    for (const wchar_t *path : cores) {
      if (path && path[0]) {
        core_ = LoadLibraryExW(path, nullptr, LOAD_WITH_ALTERED_SEARCH_PATH);
        if (core_) {
          break;
        }
      }
    }
    const std::wstring snippet_path = runtime_dir_ + L"\\nvngx_dlssnr.dll";
    snippet_ = LoadLibraryExW(snippet_path.c_str(), nullptr, LOAD_WITH_ALTERED_SEARCH_PATH);
    if (!snippet_) {
      set_nr_error("LoadLibrary nvngx_dlssnr.dll failed");
      return false;
    }
    hook_snippet_get_module_file_name(snippet_, core_, &iat_slot_);
    cuda_init1_ = reinterpret_cast<NgxCudaInit1Fn>(
        GetProcAddress(snippet_, "NVSDK_NGX_CUDA_Init1"));
    cuda_create1_ = reinterpret_cast<NgxCudaCreate1Fn>(
        GetProcAddress(snippet_, "NVSDK_NGX_CUDA_CreateFeature1"));
    cuda_eval_ = reinterpret_cast<NgxCudaEvalFn>(
        GetProcAddress(snippet_, "NVSDK_NGX_CUDA_EvaluateFeature"));
    cuda_release_ = reinterpret_cast<NgxCudaReleaseFn>(
        GetProcAddress(snippet_, "NVSDK_NGX_CUDA_ReleaseFeature"));
    if (!cuda_init1_ || !cuda_create1_ || !cuda_eval_ || !cuda_release_) {
      set_nr_error("nvngx_dlssnr.dll missing CUDA Feature 18 exports");
      return false;
    }

    feature_paths_[0] = runtime_dir_.c_str();
    NVSDK_NGX_FeatureCommonInfo common = {};
    common.PathListInfo.Path = feature_paths_;
    common.PathListInfo.Length = 1;
    common.LoggingInfo.MinimumLoggingLevel = NVSDK_NGX_LOGGING_LEVEL_ON;
    NgxResult init = cuda_init1_(
        kAppIdUnity, data_dir_.c_str(), &ngx_device_, &common, kSdkSnippet);
    if (init != kNgxSuccess && init != int(NVSDK_NGX_Result_FAIL_FeatureAlreadyExists)) {
      init = cuda_init1_(
          100334311ull, data_dir_.c_str(), &ngx_device_, &common, kSdkSnippet);
    }
    if (init != kNgxSuccess && init != int(NVSDK_NGX_Result_FAIL_FeatureAlreadyExists)) {
      char msg[256];
      SNPRINTF(msg, "CUDA Init1 for DLSSNR failed (0x%08x)", unsigned(init));
      set_nr_error(msg);
      return false;
    }
    if (!this->allocate_parameters()) {
      return false;
    }
    initialized_ = true;
    CLOG_INFO(&LOG, "DLSSNR CUDA path ready (%s context)", private_context_ ? "private" : "primary");
    return true;
  }

  void set_params(const int width,
                  const int height,
                  const bool have_depth,
                  const bool have_motion,
                  const DLSSNRParams &nr,
                  const bool do_reset)
  {
    const unsigned reset = do_reset ? 1u : 0u;
    params_->Set("DLSSNR.Color", &tex_color_.tex);
    params_->Set("DLSSNR.Output", &tex_output_.surf);
    params_->Set("DLSSNR.Backbuffer", do_reset ? &tex_color_.tex : &tex_output_.tex);
    params_->Set("DLSSNR.Depth", have_depth ? &tex_depth_.tex : nullptr);
    params_->Set("DLSSNR.MVec", have_motion ? &tex_motion_.tex : nullptr);
    params_->Set("Width", width);
    params_->Set("Height", height);
    params_->Set("OutWidth", width);
    params_->Set("OutHeight", height);
    params_->Set("DLSSNR.Width", unsigned(width));
    params_->Set("DLSSNR.Height", unsigned(height));
    params_->Set("DLSSNR.Enabled", 1u);
    params_->Set("DLSSNR.Reset", reset);
    params_->Set(NVSDK_NGX_Parameter_Reset, reset);
    params_->Set("DLSSNR.JitterOffsetX", 0.0f);
    params_->Set("DLSSNR.JitterOffsetY", 0.0f);
    params_->Set("DLSSNR.DepthInverted", 1u);
    params_->Set("DLSSNR.ScalingRatio", 1.0f);
    params_->Set("DLSSNR.Intensity", std::clamp(nr.intensity, 0.0f, 2.0f));
    params_->Set("DLSSNR.LocalToneStrength", std::clamp(nr.tone, 0.0f, 2.0f));
    params_->Set("DLSSNR.LocalStructureStrength", std::clamp(nr.structure, 0.0f, 2.0f));
    params_->Set("DLSSNR.SkinStructureStrength", nr.skin);
    params_->Set("DLSSNR.Style", std::clamp(nr.style, 0, 2));
    params_->Set("DLSSNR.UseAutoMask", nr.auto_mask ? 1u : 0u);
    params_->Set("DLSSNR.UICorrection", nr.ui_correction ? 1u : 0u);
  }

  bool upload(const float *color,
              const float *depth,
              const float *motion,
              const int width,
              const int height,
              const bool have_depth,
              const bool have_motion,
              const bool clamp_input)
  {
    if (!tex_color_.upload_flipped(color, 4, clamp_input, stage_color_, stream_)) {
      set_nr_error("CUDA color upload failed");
      return false;
    }
    if (have_depth && !tex_depth_.upload_flipped(depth, 1, false, stage_depth_, stream_)) {
      set_nr_error("CUDA depth upload failed");
      return false;
    }
    if (have_motion && !tex_motion_.upload_flipped(motion, 2, false, stage_motion_, stream_)) {
      set_nr_error("CUDA motion upload failed");
      return false;
    }
    (void)width;
    (void)height;
    return true;
  }

  bool ensure_feature(const int width,
                      const int height,
                      const bool have_depth,
                      const bool have_motion,
                      const DLSSNRParams &nr)
  {
    const int style = std::clamp(nr.style, 0, 2);
    if (handle_ && feature_width_ == width && feature_height_ == height &&
        feature_style_ == style && have_depth_ == have_depth && have_motion_ == have_motion)
    {
      return true;
    }
    if (handle_ && cuda_release_) {
      cuda_release_(handle_);
      handle_ = nullptr;
    }
    tex_color_.destroy();
    tex_output_.destroy();
    tex_depth_.destroy();
    tex_motion_.destroy();
    if (!tex_color_.init(width, height, 4) || !tex_output_.init(width, height, 4) ||
        (have_depth && !tex_depth_.init(width, height, 1)) ||
        (have_motion && !tex_motion_.init(width, height, 2)))
    {
      set_nr_error("Failed to create DLSSNR CUDA textures");
      return false;
    }
    this->set_params(width, height, have_depth, have_motion, nr, true);
    params_->Set(NVSDK_NGX_Parameter_Input1, context_);
    params_->Set(NVSDK_NGX_Parameter_Input2, stream_);
    const NgxResult created = cuda_create1_(&ngx_device_, kFeatureId, params_, &handle_);
    if (created != kNgxSuccess || handle_ == nullptr) {
      char msg[256];
      SNPRINTF(msg, "CUDA CreateFeature(18) failed (0x%08x)", unsigned(created));
      set_nr_error(msg);
      handle_ = nullptr;
      return false;
    }
    feature_width_ = width;
    feature_height_ = height;
    feature_style_ = style;
    have_depth_ = have_depth;
    have_motion_ = have_motion;
    CLOG_INFO(&LOG, "DLSSNR CUDA feature %dx%d", width, height);
    return true;
  }
};

#  endif /* _WIN32 */

class CompositorDLSSNR {
 public:
  static CompositorDLSSNR &instance()
  {
    static CompositorDLSSNR singleton;
    return singleton;
  }

  bool available()
  {
    std::lock_guard lock(mutex_);
#  ifdef _WIN32
    return dx12_.available();
#  else
    set_nr_error("DLSSNR compositor path is Windows D3D12 only");
    return false;
#  endif
  }

  bool apply(const int2 size,
             const float *color,
             const float *depth,
             const float *motion,
             const bool have_depth,
             const bool have_motion,
             const DLSSNRParams &nr,
             float *output,
             const bool isolate_gpu)
  {
    std::lock_guard lock(mutex_);
#  ifdef _WIN32
    if (cuda_.apply(
            size, color, depth, motion, have_depth, have_motion, nr, output, isolate_gpu))
    {
      return true;
    }
    /* F12 / compositor job thread: D3D12 NGX after Cycles CUDA deadlocks the GPU
     * (log shows CubinD3D12::Init then freeze). Viewport already uses CUDA. */
    if (isolate_gpu) {
      set_nr_error("DLSS Neural CUDA failed on the render thread. "
                   "D3D12 fallback is disabled for F12 because it deadlocks.");
      return false;
    }
    return dx12_.apply(
        size, color, depth, motion, have_depth, have_motion, nr, output, isolate_gpu);
#  else
    (void)size;
    (void)color;
    (void)depth;
    (void)motion;
    (void)have_depth;
    (void)have_motion;
    (void)nr;
    (void)output;
    (void)isolate_gpu;
    set_nr_error("DLSSNR compositor path is Windows D3D12 only");
    return false;
#  endif
  }

 private:
  std::mutex mutex_;
#  ifdef _WIN32
  CudaNR cuda_;
  Dx12NR dx12_;
#  endif
};

template<typename T>
static T sample_guide(const Result *result, const int2 texel, const T &fallback)
{
  if (result == nullptr || !result->is_allocated()) {
    return fallback;
  }
  return result->load_pixel_fallback<T, true>(texel, fallback);
}

bool is_dlssnr_available()
{
  return CompositorDLSSNR::instance().available();
}

const char *dlssnr_last_error()
{
  return g_last_error;
}

bool apply_dlssnr(Context &context,
                  const Result &color,
                  const Result *depth,
                  const Result *motion,
                  const Result *mask,
                  const Result * /*albedo*/,
                  const Result * /*normal*/,
                  const DLSSNRParams &params,
                  Result &output)
{
  if (context.is_canceled()) {
    return false;
  }
  if (!color.is_allocated() || color.is_single_value()) {
    return false;
  }

  const int2 size = color.domain().data_size;
  const int64_t pixel_count = int64_t(size.x) * int64_t(size.y);
  const bool have_depth = depth != nullptr && depth->is_allocated() && !depth->is_single_value();
  const bool have_motion = motion != nullptr && motion->is_allocated() &&
                           !motion->is_single_value();
  const bool have_mask = mask != nullptr && mask->is_allocated() && !mask->is_single_value();

  /* Reused across redraws. Allocating and zeroing a full float frame every
   * eval was a large part of the viewport hitch. */
  struct Host {
    Vector<float> color;
    Vector<float> depth;
    Vector<float> motion;
    Vector<float> mask;
    Vector<float> output;
  };
  static thread_local Host host;
  if (host.color.size() != pixel_count * 4) {
    host.color.resize(pixel_count * 4);
    host.output.resize(pixel_count * 4);
  }
  if (have_depth && host.depth.size() != pixel_count) {
    host.depth.resize(pixel_count);
  }
  if (have_motion && host.motion.size() != pixel_count * 2) {
    host.motion.resize(pixel_count * 2);
  }
  if (have_mask && host.mask.size() != pixel_count) {
    host.mask.resize(pixel_count);
  }

  auto bulk_read = [&](const Result &result, float *dst, const int channels) -> bool {
    if (result.is_stored_on_gpu()) {
      GPU_texture_read(result.gpu_texture(), GPU_DATA_FLOAT, 0, dst);
      return true;
    }
    if (result.cpu_data().size() == pixel_count) {
      memcpy(dst, result.cpu_data().data(), size_t(pixel_count) * size_t(channels) * sizeof(float));
      return true;
    }
    return false;
  };

  if (!bulk_read(color, host.color.data(), 4)) {
    parallel_for(size, [&](const int2 texel) {
      const int64_t index = int64_t(texel.y) * size.x + texel.x;
      const Color pixel = color.load_pixel<Color>(texel);
      host.color[index * 4 + 0] = pixel.r;
      host.color[index * 4 + 1] = pixel.g;
      host.color[index * 4 + 2] = pixel.b;
      host.color[index * 4 + 3] = pixel.a;
    });
  }
  if (have_depth && !bulk_read(*depth, host.depth.data(), 1)) {
    parallel_for(size, [&](const int2 texel) {
      const int64_t index = int64_t(texel.y) * size.x + texel.x;
      host.depth[index] = sample_guide(depth, texel, 1.0f);
    });
  }
  if (have_motion && !bulk_read(*motion, host.motion.data(), 2)) {
    parallel_for(size, [&](const int2 texel) {
      const int64_t index = int64_t(texel.y) * size.x + texel.x;
      const float2 mv = sample_guide(motion, texel, float2(0.0f));
      host.motion[index * 2 + 0] = mv.x;
      host.motion[index * 2 + 1] = mv.y;
    });
  }
  if (have_mask && !bulk_read(*mask, host.mask.data(), 1)) {
    parallel_for(size, [&](const int2 texel) {
      const int64_t index = int64_t(texel.y) * size.x + texel.x;
      host.mask[index] = sample_guide(mask, texel, 1.0f);
    });
  }

  if (context.is_canceled()) {
    return false;
  }
  /* CUDA Feature 18 does not unbind Vulkan. D3D12 fallback isolates itself. */
  if (!CompositorDLSSNR::instance().apply(size,
                                          host.color.data(),
                                          have_depth ? host.depth.data() : nullptr,
                                          have_motion ? host.motion.data() : nullptr,
                                          have_depth,
                                          have_motion,
                                          params,
                                          host.output.data(),
                                          !context.is_viewport()))
  {
    return false;
  }
  if (context.is_canceled()) {
    return false;
  }

  output.allocate_texture(color.domain(), false, ResultStorageType::CPUImage);
  Color *out_pixels = static_cast<Color *>(output.cpu_data_for_write().data());
  if (!have_mask) {
    memcpy(out_pixels, host.output.data(), size_t(pixel_count) * sizeof(Color));
  }
  else {
    parallel_for(size, [&](const int2 texel) {
      const int64_t index = int64_t(texel.y) * size.x + texel.x;
      const float amount = std::clamp(host.mask[index], 0.0f, 1.0f);
      const float src_r = host.color[index * 4 + 0];
      const float src_g = host.color[index * 4 + 1];
      const float src_b = host.color[index * 4 + 2];
      Color nr(host.output[index * 4 + 0],
               host.output[index * 4 + 1],
               host.output[index * 4 + 2],
               host.color[index * 4 + 3]);
      nr.r = src_r + (nr.r - src_r) * amount;
      nr.g = src_g + (nr.g - src_g) * amount;
      nr.b = src_b + (nr.b - src_b) * amount;
      out_pixels[index] = nr;
    });
  }
  return true;
}

}  // namespace blender::compositor

#endif
