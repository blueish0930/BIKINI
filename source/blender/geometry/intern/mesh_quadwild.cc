/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Thin Geometry Nodes wrapper around the bundled QRemeshify addon.
 *
 * All remesh/trace/quadrangulate + sharp/export logic runs in
 * qremeshify_bridge.py → QRemeshify/{operator,lib,util} — not reimplemented here.
 *
 * This file only:
 *   1) dumps Mesh → temporary OBJ (data transfer)
 *   2) runs: blender -b --python qremeshify_bridge.py -- ...
 *   3) loads result OBJ → Mesh
 */

#include "BLI_array.hh"
#include "BLI_fileops.hh"
#include "BLI_map.hh"
#include "BLI_math_base.hh"
#include "BLI_math_matrix.hh"
#include "BLI_math_vector.hh"
#include "BLI_offset_indices.hh"
#include "BLI_path_utils.hh"
#include "BLI_span.hh"
#include "BLI_string.hh"
#include "BLI_string_utils.hh"
#include "BLI_tempfile.hh"
#include "BLI_time.hh"
#include "BLI_vector.hh"

#include "BKE_appdir.hh"
#include "BKE_attribute.hh"
#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"

#include "DNA_mesh_types.h"

#include "MEM_guardedalloc.h"

#include "GEO_mesh_quadwild.hh"

#include <cmath>
#include <cstdio>
#include <sstream>
#include <string>
#include <vector>

#ifdef _WIN32
#  include <windows.h>
#else
#  include <sys/types.h>
#  include <sys/wait.h>
#  include <unistd.h>
#endif

namespace blender::geometry {

static bool path_is_file(const char *path)
{
  return BLI_exists(path) && !BLI_is_dir(path);
}

std::string quadwild_package_dir_find()
{
  auto ok = [](const char *dir) -> bool {
    char bridge[FILE_MAX], pkg[FILE_MAX];
    BLI_path_join(bridge, sizeof(bridge), dir, "qremeshify_bridge.py");
    BLI_path_join(pkg, sizeof(pkg), dir, "QRemeshify");
    return path_is_file(bridge) && BLI_is_dir(pkg);
  };

  if (const char *env = BLI_getenv("QUADWILD_DIR")) {
    if (env[0] && BLI_is_dir(env) && ok(env)) {
      return env;
    }
  }
  if (const char *program_dir = BKE_appdir_program_dir()) {
    char candidate[FILE_MAX];
    BLI_path_join(candidate, sizeof(candidate), program_dir, "quadwild");
    if (BLI_is_dir(candidate) && ok(candidate)) {
      return candidate;
    }
  }
  if (const std::optional<std::string> datafiles = BKE_appdir_folder_id(BLENDER_DATAFILES,
                                                                       "quadwild"))
  {
    if (BLI_is_dir(datafiles->c_str()) && ok(datafiles->c_str())) {
      return *datafiles;
    }
  }
  return {};
}

/** Minimal OBJ dump for data transfer into the plugin bridge (not remesh logic). */
static bool write_transfer_obj(const char *filepath, const Mesh &mesh)
{
  const Span<float3> positions = mesh.vert_positions();
  const Span<int> corner_verts = mesh.corner_verts();
  const OffsetIndices faces = mesh.faces();

  FILE *f = BLI_fopen(filepath, "wb");
  if (!f) {
    return false;
  }
  fprintf(f, "# transfer only — remesh by QRemeshify bridge\n");
  for (const int v : positions.index_range()) {
    const float3 &p = positions[v];
    fprintf(f, "v %.9g %.9g %.9g\n", double(p.x), double(p.y), double(p.z));
  }
  for (const int fi : faces.index_range()) {
    const IndexRange face = faces[fi];
    if (face.size() < 3) {
      continue;
    }
    fprintf(f, "f");
    for (const int c : face) {
      fprintf(f, " %d", corner_verts[c] + 1);
    }
    fprintf(f, "\n");
  }
  fclose(f);
  return true;
}

static Mesh *read_obj_mesh(const char *filepath)
{
  blender::fstream in(filepath, std::ios_base::in);
  if (!in) {
    return nullptr;
  }
  Vector<float3> verts;
  Vector<Vector<int>> faces_out;
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    if (line[0] == 'v' && (line.size() == 1 || line[1] == ' ' || line[1] == '\t')) {
      float x = 0, y = 0, z = 0;
      if (sscanf(line.c_str() + 1, "%f %f %f", &x, &y, &z) >= 3) {
        verts.append(float3(x, y, z));
      }
      continue;
    }
    if (line[0] == 'f' && (line.size() == 1 || line[1] == ' ' || line[1] == '\t')) {
      Vector<int> face;
      const char *p = line.c_str() + 1;
      while (*p) {
        while (*p == ' ' || *p == '\t') {
          p++;
        }
        if (*p == '\0') {
          break;
        }
        int idx = 0;
        if (sscanf(p, "%d", &idx) != 1) {
          break;
        }
        if (idx < 0) {
          idx = int(verts.size()) + idx + 1;
        }
        face.append(idx - 1);
        while (*p && *p != ' ' && *p != '\t') {
          p++;
        }
      }
      if (face.size() >= 3) {
        faces_out.append(std::move(face));
      }
    }
  }
  if (verts.is_empty() || faces_out.is_empty()) {
    return nullptr;
  }
  int corners_num = 0;
  for (const Vector<int> &face : faces_out) {
    corners_num += face.size();
  }
  Mesh *mesh = BKE_mesh_new_nomain(verts.size(), 0, faces_out.size(), corners_num);
  mesh->vert_positions_for_write().copy_from(verts.as_span());
  MutableSpan<int> face_offsets = mesh->face_offsets_for_write();
  MutableSpan<int> corner_verts = mesh->corner_verts_for_write();
  int corner = 0;
  face_offsets[0] = 0;
  for (const int fi : faces_out.index_range()) {
    for (const int vi : faces_out[fi]) {
      corner_verts[corner++] = math::clamp(vi, 0, int(verts.size()) - 1);
    }
    face_offsets[fi + 1] = corner;
  }
  bke::mesh_calc_edges(*mesh, false, false);
  return mesh;
}

#ifdef _WIN32
static std::wstring utf8_to_wide(const char *utf8)
{
  if (!utf8 || !utf8[0]) {
    return {};
  }
  const int n = MultiByteToWideChar(CP_UTF8, 0, utf8, -1, nullptr, 0);
  if (n <= 0) {
    return {};
  }
  std::wstring out(size_t(n - 1), L'\0');
  MultiByteToWideChar(CP_UTF8, 0, utf8, -1, out.data(), n);
  return out;
}
#endif

/**
 * Run: blender -b --python qremeshify_bridge.py -- <args...>
 * \return exit code, or -1 on launch failure.
 */
static int run_qremeshify_bridge(const char *blender_exe,
                                 const char *bridge_py,
                                 const std::vector<std::string> &py_args,
                                 const char *log_path,
                                 std::string *r_log)
{
#ifdef _WIN32
  std::string cmd;
  auto append_quoted = [&](const char *s) {
    if (!cmd.empty()) {
      cmd.push_back(' ');
    }
    cmd.push_back('"');
    for (const char *p = s; *p; p++) {
      if (*p == '"') {
        cmd.push_back('"');
        cmd.push_back('"');
      }
      else {
        cmd.push_back(*p);
      }
    }
    cmd.push_back('"');
  };

  append_quoted(blender_exe);
  append_quoted("-b");
  append_quoted("--python");
  append_quoted(bridge_py);
  append_quoted("--");
  for (const std::string &a : py_args) {
    append_quoted(a.c_str());
  }

  const std::wstring wexe = utf8_to_wide(blender_exe);
  std::wstring wcmd = utf8_to_wide(cmd.c_str());
  if (wexe.empty() || wcmd.empty()) {
    if (r_log) {
      *r_log = "UTF-8 path conversion failed";
    }
    return -1;
  }

  HANDLE log_handle = INVALID_HANDLE_VALUE;
  if (log_path && log_path[0]) {
    SECURITY_ATTRIBUTES sa{};
    sa.nLength = sizeof(sa);
    sa.bInheritHandle = TRUE;
    const std::wstring wlog = utf8_to_wide(log_path);
    log_handle = CreateFileW(wlog.c_str(),
                             GENERIC_WRITE,
                             FILE_SHARE_READ,
                             &sa,
                             CREATE_ALWAYS,
                             FILE_ATTRIBUTE_NORMAL,
                             nullptr);
  }

  STARTUPINFOW si{};
  PROCESS_INFORMATION pi{};
  si.cb = sizeof(si);
  si.dwFlags = STARTF_USESHOWWINDOW;
  si.wShowWindow = SW_HIDE;
  if (log_handle != INVALID_HANDLE_VALUE) {
    si.dwFlags |= STARTF_USESTDHANDLES;
    si.hStdInput = GetStdHandle(STD_INPUT_HANDLE);
    si.hStdOutput = log_handle;
    si.hStdError = log_handle;
  }

  std::vector<wchar_t> cmd_mutable(wcmd.begin(), wcmd.end());
  cmd_mutable.push_back(L'\0');

  /* Prevent nested Geometry Node remesh recursion / infinite blender spawn. */
  SetEnvironmentVariableA("QUADWILD_BRIDGE_CHILD", "1");

  const BOOL ok = CreateProcessW(wexe.c_str(),
                                 cmd_mutable.data(),
                                 nullptr,
                                 nullptr,
                                 log_handle != INVALID_HANDLE_VALUE ? TRUE : FALSE,
                                 CREATE_NO_WINDOW,
                                 nullptr,
                                 nullptr,
                                 &si,
                                 &pi);

  SetEnvironmentVariableA("QUADWILD_BRIDGE_CHILD", nullptr);

  if (!ok) {
    if (r_log) {
      *r_log = "CreateProcessW failed err=" + std::to_string(GetLastError());
    }
    if (log_handle != INVALID_HANDLE_VALUE) {
      CloseHandle(log_handle);
    }
    return -1;
  }

  WaitForSingleObject(pi.hProcess, INFINITE);
  DWORD code = 1;
  GetExitCodeProcess(pi.hProcess, &code);
  CloseHandle(pi.hProcess);
  CloseHandle(pi.hThread);
  if (log_handle != INVALID_HANDLE_VALUE) {
    CloseHandle(log_handle);
  }

  if (r_log && log_path && path_is_file(log_path)) {
    FILE *f = BLI_fopen(log_path, "rb");
    if (f) {
      fseek(f, 0, SEEK_END);
      const long sz = ftell(f);
      const long keep = 4000;
      if (sz > keep) {
        fseek(f, -keep, SEEK_END);
      }
      else {
        fseek(f, 0, SEEK_SET);
      }
      char buf[4001];
      const size_t n = fread(buf, 1, 4000, f);
      buf[n] = '\0';
      fclose(f);
      *r_log += buf;
    }
  }
  return int(code);
#else
  /* Non-Windows: fork/exec blender -b --python ... */
  std::vector<char *> argv;
  argv.push_back(const_cast<char *>(blender_exe));
  argv.push_back(const_cast<char *>("-b"));
  argv.push_back(const_cast<char *>("--python"));
  argv.push_back(const_cast<char *>(bridge_py));
  argv.push_back(const_cast<char *>("--"));
  for (const std::string &a : py_args) {
    argv.push_back(const_cast<char *>(a.c_str()));
  }
  argv.push_back(nullptr);

  setenv("QUADWILD_BRIDGE_CHILD", "1", 1);
  const pid_t pid = fork();
  if (pid < 0) {
    unsetenv("QUADWILD_BRIDGE_CHILD");
    return -1;
  }
  if (pid == 0) {
    execvp(blender_exe, argv.data());
    _exit(127);
  }
  unsetenv("QUADWILD_BRIDGE_CHILD");
  int status = 0;
  if (waitpid(pid, &status, 0) < 0) {
    return -1;
  }
  if (WIFEXITED(status)) {
    return WEXITSTATUS(status);
  }
  return -1;
#endif
}

/**
 * Apply 4-RoSy face attributes from a .rosy sidecar (fn, 4, then fn lines of xyz).
 * Writes `{prefix}0`…`{prefix}3` on the face domain.
 */
static bool apply_rosy_face_attributes(Mesh &mesh,
                                       const char *rosy_path,
                                       const std::string &prefix)
{
  FILE *f = BLI_fopen(rosy_path, "rb");
  if (!f) {
    return false;
  }
  int fn = 0, nsym = 0;
  if (fscanf(f, "%d", &fn) != 1 || fscanf(f, "%d", &nsym) != 1) {
    fclose(f);
    return false;
  }
  if (fn != mesh.faces_num || nsym != 4) {
    fclose(f);
    return false;
  }

  Array<float3> primary(fn);
  for (int i = 0; i < fn; i++) {
    float x = 0, y = 0, z = 0;
    if (fscanf(f, "%f %f %f", &x, &y, &z) != 3) {
      fclose(f);
      return false;
    }
    primary[i] = float3(x, y, z);
  }
  fclose(f);

  const Span<float3> positions = mesh.vert_positions();
  const Span<int> corner_verts = mesh.corner_verts();
  const OffsetIndices faces = mesh.faces();

  bke::MutableAttributeAccessor attributes = mesh.attributes_for_write();
  for (int k = 0; k < 4; k++) {
    const std::string name = prefix + std::to_string(k);
    bke::SpanAttributeWriter<float3> writer = attributes.lookup_or_add_for_write_only_span<float3>(
        name, bke::AttrDomain::Face);
    if (!writer) {
      return false;
    }
    for (const int fi : faces.index_range()) {
      const IndexRange face = faces[fi];
      float3 n(0, 0, 1);
      if (face.size() >= 3) {
        const float3 a = positions[corner_verts[face[0]]];
        const float3 b = positions[corner_verts[face[1]]];
        const float3 c = positions[corner_verts[face[2]]];
        n = math::normalize(math::cross(b - a, c - a));
        if (math::length_squared(n) < 1e-20f) {
          n = float3(0, 0, 1);
        }
      }
      float3 d = primary[fi];
      /* Project primary direction into the face tangent plane. */
      d = d - n * math::dot(d, n);
      if (math::length_squared(d) < 1e-20f) {
        /* Fallback orthogonal to n. */
        const float3 ax = math::abs(n.x) < 0.9f ? float3(1, 0, 0) : float3(0, 1, 0);
        d = math::normalize(math::cross(n, ax));
      }
      else {
        d = math::normalize(d);
      }
      /* Rotate by k * 90° around face normal. */
      float3 r = d;
      for (int step = 0; step < k; step++) {
        r = math::normalize(math::cross(n, r));
      }
      writer.span[fi] = r;
    }
    writer.finish();
  }
  return true;
}

/**
 * Read QuadWild `.patch` sidecar: face_count then one patch id per traced triangle face.
 * See E:\Notes\Blender_QuadWild_Layout_Separatrix_Implementation_2026-08-01.md
 */
static bool read_patch_ids(const char *patch_path, const int faces_num, Array<int> &r_ids)
{
  FILE *f = BLI_fopen(patch_path, "rb");
  if (!f) {
    return false;
  }
  int count = 0;
  if (fscanf(f, "%d", &count) != 1 || count != faces_num) {
    fclose(f);
    return false;
  }
  r_ids.reinitialize(faces_num);
  for (int i = 0; i < faces_num; i++) {
    if (fscanf(f, "%d", &r_ids[i]) != 1) {
      fclose(f);
      return false;
    }
  }
  fclose(f);
  return true;
}

static uint64_t undirected_edge_key(const int a, const int b)
{
  const uint32_t lo = uint32_t(math::min(a, b));
  const uint32_t hi = uint32_t(math::max(a, b));
  return (uint64_t(hi) << 32) | uint64_t(lo);
}

/**
 * Edge-only separatrix from traced mesh + per-face patch IDs.
 *
 * Keep:
 *   - open surface boundary edges (one incident face);
 *   - interior edges whose two faces have different patch IDs.
 *
 * Do NOT use `.corners` polylines or `.feature` alone — those yield wrong / sparse layout.
 */
static Mesh *build_separatrix_mesh_from_patch_ids(const Mesh &traced_mesh,
                                                  const Span<int> patch_ids,
                                                  const bool mark_seams,
                                                  const std::string &boundary_attr)
{
  if (traced_mesh.faces_num == 0 || patch_ids.size() != traced_mesh.faces_num) {
    return nullptr;
  }

  const Span<float3> positions = traced_mesh.vert_positions();
  const Span<int> corner_verts = traced_mesh.corner_verts();
  const OffsetIndices faces = traced_mesh.faces();

  /* edge key → incident face indices (usually 1–2). */
  Map<uint64_t, Vector<int>> edge_faces;
  for (const int fi : faces.index_range()) {
    const IndexRange face = faces[fi];
    for (const int i : face.index_range()) {
      const int v0 = corner_verts[face[i]];
      const int v1 = corner_verts[face[(i + 1) % face.size()]];
      edge_faces.lookup_or_add(undirected_edge_key(v0, v1), {}).append(fi);
    }
  }

  Vector<int2> keep_edges;
  keep_edges.reserve(edge_faces.size() / 4 + 16);
  for (const auto &item : edge_faces.items()) {
    const Span<int> incident = item.value;
    bool keep = false;
    if (incident.size() == 1) {
      keep = true; /* Original open boundary. */
    }
    else if (incident.size() >= 2) {
      const int p0 = patch_ids[incident[0]];
      for (const int fi : incident.drop_front(1)) {
        if (patch_ids[fi] != p0) {
          keep = true;
          break;
        }
      }
    }
    if (!keep) {
      continue;
    }
    const uint64_t key = item.key;
    const int v0 = int(key & 0xffffffffu);
    const int v1 = int(key >> 32);
    keep_edges.append(int2(v0, v1));
  }

  if (keep_edges.is_empty()) {
    return nullptr;
  }

  Map<int, int> old_to_new;
  Vector<float3> new_positions;
  new_positions.reserve(keep_edges.size());
  auto map_vert = [&](const int vi) -> int {
    if (const int *mapped = old_to_new.lookup_ptr(vi)) {
      return *mapped;
    }
    const int ni = new_positions.size();
    old_to_new.add(vi, ni);
    new_positions.append(positions[vi]);
    return ni;
  };

  Vector<int2> new_edges;
  new_edges.reserve(keep_edges.size());
  for (const int2 e : keep_edges) {
    new_edges.append(int2(map_vert(e[0]), map_vert(e[1])));
  }

  /* Edge-only mesh: no faces. */
  Mesh *mesh = BKE_mesh_new_nomain(new_positions.size(), new_edges.size(), 0, 0);
  mesh->vert_positions_for_write().copy_from(new_positions.as_span());
  mesh->edges_for_write().copy_from(new_edges.as_span());

  bke::MutableAttributeAccessor attributes = mesh->attributes_for_write();
  if (mark_seams) {
    bke::SpanAttributeWriter<bool> seam = attributes.lookup_or_add_for_write_only_span<bool>(
        "seam", bke::AttrDomain::Edge);
    if (seam) {
      seam.span.fill(true);
      seam.finish();
    }
  }
  if (!boundary_attr.empty()) {
    bke::SpanAttributeWriter<bool> boundary = attributes.lookup_or_add_for_write_only_span<bool>(
        boundary_attr, bke::AttrDomain::Edge);
    if (boundary) {
      boundary.span.fill(true);
      boundary.finish();
    }
  }

  return mesh;
}

Mesh *mesh_quadwild(const Mesh &src_mesh,
                    const QuadWildOptions &options,
                    QuadWildResultInfo *r_info)
{
  auto fail = [&](const std::string &msg) -> Mesh * {
    if (r_info) {
      r_info->success = false;
      r_info->message = msg;
    }
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  };

  if (const char *child = BLI_getenv("QUADWILD_BRIDGE_CHILD")) {
    if (child[0] == '1') {
      return fail("Nested QRemeshify bridge call blocked");
    }
  }

  if (src_mesh.verts_num < 3 || src_mesh.faces_num < 1) {
    return fail("Input mesh is empty");
  }

  const std::string package = quadwild_package_dir_find();
  if (package.empty()) {
    return fail(
        "QRemeshify package not found. Need <blender>/quadwild/qremeshify_bridge.py "
        "and <blender>/quadwild/QRemeshify/. Set QUADWILD_DIR if needed.");
  }
  if (r_info) {
    r_info->package_dir = package;
  }

  char blender_exe[FILE_MAX];
  const char *program_dir = BKE_appdir_program_dir();
  if (!program_dir || !program_dir[0]) {
    return fail("Cannot resolve blender.exe path");
  }
#ifdef _WIN32
  BLI_path_join(blender_exe, sizeof(blender_exe), program_dir, "blender.exe");
#else
  BLI_path_join(blender_exe, sizeof(blender_exe), program_dir, "blender");
#endif
  if (!path_is_file(blender_exe)) {
    return fail(std::string("blender executable not found: ") + blender_exe);
  }

  char bridge_py[FILE_MAX];
  BLI_path_join(bridge_py, sizeof(bridge_py), package.c_str(), "qremeshify_bridge.py");

  char temp_root[FILE_MAX];
  BLI_temp_directory_path_get(temp_root, sizeof(temp_root));
  BLI_path_slash_ensure(temp_root, sizeof(temp_root));
  char work_dir[FILE_MAX];
  SNPRINTF(work_dir, "%sqw_bridge_%lld", temp_root, BLI_time_now_seconds_i());
  if (!BLI_dir_create_recursive(work_dir)) {
    return fail(std::string("Cannot create work dir: ") + work_dir);
  }

  char input_obj[FILE_MAX], output_obj[FILE_MAX], log_path[FILE_MAX];
  BLI_path_join(input_obj, sizeof(input_obj), work_dir, "input.obj");
  BLI_path_join(output_obj, sizeof(output_obj), work_dir, "output.obj");
  BLI_path_join(log_path, sizeof(log_path), work_dir, "bridge_log.txt");

  if (!write_transfer_obj(input_obj, src_mesh)) {
    BLI_delete(work_dir, true, true);
    return fail("Failed to write transfer OBJ");
  }

  char hard_edges_path[FILE_MAX];
  hard_edges_path[0] = '\0';
  if (!options.hard_edges.is_empty()) {
    BLI_path_join(hard_edges_path, sizeof(hard_edges_path), work_dir, "input.hard_edges");
    FILE *hf = BLI_fopen(hard_edges_path, "wb");
    if (hf) {
      fprintf(hf, "%d\n", int(options.hard_edges.size()));
      for (const int2 &e : options.hard_edges) {
        fprintf(hf, "%d %d\n", e[0], e[1]);
      }
      fclose(hf);
    }
    else {
      hard_edges_path[0] = '\0';
    }
  }

  /* Sharp Angle < 0 → disable (organic). */
  const bool enable_sharp = options.sharp_feature_threshold_deg >= 0.0f;
  const float sharp_deg = enable_sharp ? options.sharp_feature_threshold_deg : -1.0f;

  const char *stage_str = "remesh";
  switch (options.mode) {
    case QuadWildMode::Field:
      stage_str = "field";
      break;
    case QuadWildMode::Layout:
      stage_str = "layout";
      break;
    case QuadWildMode::Remesh:
    default:
      stage_str = "remesh";
      break;
  }

  std::vector<std::string> py_args = {
      "--stage",
      stage_str,
      "--input",
      input_obj,
      "--output",
      output_obj,
      "--scale",
      std::to_string(math::max(options.scale_fact, 0.01f)),
      "--alpha",
      std::to_string(math::clamp(options.alpha, 0.0f, 0.999f)),
      "--sharp-angle",
      std::to_string(sharp_deg),
      "--remesh",
      options.do_remesh ? "1" : "0",
      "--smooth",
      options.smooth_output ? "1" : "0",
      "--enable-sharp",
      enable_sharp ? "1" : "0",
      "--align-singularities",
      options.align_singularities ? "1" : "0",
      "--flow-config",
      options.flow_config.empty() ? "SIMPLE" : options.flow_config,
      "--satsuma-config",
      options.satsuma_config.empty() ? "DEFAULT" : options.satsuma_config,
  };
  if (hard_edges_path[0]) {
    py_args.push_back("--hard-edges");
    py_args.push_back(hard_edges_path);
  }

  if (options.apply_rot_scale) {
    py_args.push_back("--rot-scale-matrix");
    for (int row = 0; row < 4; row++) {
      for (int col = 0; col < 4; col++) {
        py_args.push_back(std::to_string(options.rot_scale[col][row]));
      }
    }
  }

  std::string log;
  const int code = run_qremeshify_bridge(blender_exe, bridge_py, py_args, log_path, &log);

  if (!path_is_file(output_obj)) {
    BLI_delete(work_dir, true, true);
    return fail("QRemeshify bridge failed (exit " + std::to_string(code) + "). " + log);
  }

  Mesh *result = read_obj_mesh(output_obj);
  if (!result) {
    BLI_delete(work_dir, true, true);
    return fail("Failed to parse QRemeshify output OBJ. " + log);
  }

  char out_base[FILE_MAX];
  BLI_strncpy(out_base, output_obj, sizeof(out_base));
  BLI_path_extension_strip(out_base);

  if (options.mode == QuadWildMode::Field) {
    char rosy_path[FILE_MAX];
    SNPRINTF(rosy_path, "%s.rosy", out_base);
    if (path_is_file(rosy_path)) {
      const std::string prefix = options.field_attribute_prefix.empty() ?
                                     "qw_dir" :
                                     options.field_attribute_prefix;
      if (!apply_rosy_face_attributes(*result, rosy_path, prefix)) {
        /* Non-fatal: mesh still useful. */
        if (r_info) {
          r_info->message = "Field mesh OK but failed to apply .rosy attributes";
        }
      }
    }
  }
  if (options.mode == QuadWildMode::Layout) {
    /* Correct Layout (see Notes 2026-08-01): edge-only separatrix from .patch IDs on
     * traced mesh. Forbidden: .corners straight polygons, .feature-only sparse edges. */
    char patch_path[FILE_MAX];
    SNPRINTF(patch_path, "%s.patch", out_base);
    if (!path_is_file(patch_path)) {
      BKE_id_free(nullptr, result);
      BLI_delete(work_dir, true, true);
      return fail(
          "Layout missing .patch (need quadwild.exe mode 2 trace; not feature/corners only). " +
          log);
    }
    Array<int> patch_ids;
    if (!read_patch_ids(patch_path, result->faces_num, patch_ids)) {
      BKE_id_free(nullptr, result);
      BLI_delete(work_dir, true, true);
      return fail("Layout failed to read .patch (face count mismatch or parse error). " + log);
    }
    Mesh *separatrix = build_separatrix_mesh_from_patch_ids(
        *result, patch_ids, options.mark_seams, options.layout_boundary_attribute);
    BKE_id_free(nullptr, result);
    result = separatrix;
    if (!result || result->edges_num == 0) {
      if (result) {
        BKE_id_free(nullptr, result);
      }
      BLI_delete(work_dir, true, true);
      return fail("Layout produced empty separatrix network from .patch. " + log);
    }
  }

  BLI_delete(work_dir, true, true);

  BKE_mesh_copy_parameters(result, &src_mesh);
  if (r_info) {
    r_info->success = true;
    if (r_info->message.empty()) {
      r_info->message.clear();
    }
  }
  return result;
}

}  // namespace blender::geometry
