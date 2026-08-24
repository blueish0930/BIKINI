# SPDX-FileCopyrightText: 2026 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

"""Load ShaderToy shaders into ImageNodeShaderToy.

shadertoy.com is behind Cloudflare and returns 403 to Blender. Fetch order:

1. JSON/GLSL already in the URL field or clipboard
2. GitHub / jsDelivr snapshot of Public+API shaders (no Cloudflare)
3. Official API / unofficial POST (often 403)

No browser plugin required: copy the Image tab in the website (Ctrl+A, Ctrl+C)
and press Paste.
"""

from __future__ import annotations

import json
import os
import re
import ssl
import tempfile
import urllib.error
import urllib.parse
import urllib.request

import bpy
from bpy.types import Operator
from bpy_extras.io_utils import ImportHelper


_ID_RE = re.compile(
    r"(?:https?://)?(?:www\.)?shadertoy\.com/(?:view|embed)/([A-Za-z0-9]{5,8})",
    re.IGNORECASE,
)
_BARE_ID_RE = re.compile(r"^[A-Za-z0-9]{5,8}$")

_CHROME_UA = (
    "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 "
    "(KHTML, like Gecko) Chrome/131.0.0.0 Safari/537.36"
)

# Public+API dump (2024-10-05). jsDelivr first — GitHub raw is flaky in some regions.
_MIRRORS = (
    "https://cdn.jsdelivr.net/gh/GabeRundlett/shadertoy-api-shaders@master/shaders/{id}.json",
    "https://fastly.jsdelivr.net/gh/GabeRundlett/shadertoy-api-shaders@master/shaders/{id}.json",
    "https://gcore.jsdelivr.net/gh/GabeRundlett/shadertoy-api-shaders@master/shaders/{id}.json",
    "https://raw.githubusercontent.com/GabeRundlett/shadertoy-api-shaders/master/shaders/{id}.json",
    "https://cdn.jsdmirror.com/gh/GabeRundlett/shadertoy-api-shaders@master/shaders/{id}.json",
)

_LIST_URLS = (
    "https://cdn.jsdelivr.net/gh/GabeRundlett/shadertoy-api-shaders@master/shader-list.json",
    "https://raw.githubusercontent.com/GabeRundlett/shadertoy-api-shaders/master/shader-list.json",
)

_id_case_map: dict[str, str] | None = None


def parse_shader_id(text: str) -> str:
    text = (text or "").strip()
    if not text or text[0] in "{[":
        return ""
    m = _ID_RE.search(text)
    if m:
        return m.group(1)
    if _BARE_ID_RE.match(text):
        return text
    return ""


def _opener():
    ctx = ssl.create_default_context()
    return urllib.request.build_opener(urllib.request.HTTPSHandler(context=ctx))


def _get(url: str, timeout: float = 12):
    req = urllib.request.Request(url, headers={"User-Agent": _CHROME_UA, "Accept": "*/*"})
    return _opener().open(req, timeout=timeout)


def coerce_shader(data) -> dict:
    if isinstance(data, dict) and "Shader" in data:
        data = data["Shader"]
    if isinstance(data, list):
        if not data:
            raise ValueError("empty shader list")
        data = data[0]
        if isinstance(data, dict) and "Shader" in data:
            data = data["Shader"]
    if isinstance(data, dict) and "renderpass" in data:
        return data
    raise ValueError("not ShaderToy JSON (need renderpass)")


def try_parse_json_text(text: str) -> dict | None:
    text = (text or "").strip()
    if not text or text[0] not in "{[":
        return None
    try:
        return coerce_shader(json.loads(text))
    except (json.JSONDecodeError, ValueError, TypeError):
        return None


def try_parse_glsl(text: str) -> str | None:
    text = (text or "").strip()
    if not text or text[0] in "{[":
        return None
    if "mainImage" not in text:
        return None
    return text


def _shader_id_map() -> dict[str, str]:
    """Lowercase ID -> actual filename ID in the GitHub dump."""
    global _id_case_map
    if _id_case_map is not None:
        return _id_case_map
    _id_case_map = {}
    for url in _LIST_URLS:
        try:
            with _get(url, timeout=15) as resp:
                data = json.loads(resp.read().decode("utf-8"))
            ids = data.get("Results") if isinstance(data, dict) else data
            if not isinstance(ids, list):
                continue
            for item in ids:
                if isinstance(item, str) and item:
                    _id_case_map.setdefault(item.lower(), item)
            if _id_case_map:
                return _id_case_map
        except Exception:  # noqa: BLE001
            continue
    return _id_case_map


def _mirror_ids(shader_id: str) -> list[str]:
    """IDs to try, matching dump filename case when possible."""
    out: list[str] = []
    mapped = _shader_id_map().get(shader_id.lower())
    for cand in (mapped, shader_id, shader_id.swapcase()):
        if cand and cand not in out:
            out.append(cand)
    return out


def _fetch_mirrors(shader_id: str) -> dict:
    last = None
    for sid in _mirror_ids(shader_id):
        for tmpl in _MIRRORS:
            url = tmpl.format(id=sid)
            try:
                with _get(url, timeout=10) as resp:
                    data = json.loads(resp.read().decode("utf-8"))
                return coerce_shader(data)
            except Exception as ex:  # noqa: BLE001
                last = ex
                continue
    raise RuntimeError(str(last) if last else "not in public API snapshot")


def _fetch_official(shader_id: str, key: str) -> dict:
    url = (
        f"https://www.shadertoy.com/api/v1/shaders/{shader_id}"
        f"?key={urllib.parse.quote(key)}"
    )
    with _get(url, timeout=8) as resp:
        data = json.loads(resp.read().decode("utf-8"))
    if isinstance(data, dict) and data.get("Error"):
        raise RuntimeError(str(data["Error"]))
    return coerce_shader(data)


def fetch_shader_json(shader_id: str, api_key: str = "") -> dict:
    errors: list[str] = []
    key = (api_key or "").strip() or os.environ.get("SHADERTOY_KEY", "").strip()

    try:
        return _fetch_mirrors(shader_id)
    except Exception as ex:  # noqa: BLE001
        errors.append(f"archive: {ex}")

    if key:
        try:
            return _fetch_official(shader_id, key)
        except Exception as ex:  # noqa: BLE001
            errors.append(f"API: {ex}")

    raise RuntimeError(
        " | ".join(errors)
        + " — shadertoy.com is blocked (Cloudflare 403). "
        "Open the page in a browser, click the Image tab, Ctrl+A / Ctrl+C, then Paste."
    )


def _buffer_letter(name: str) -> str:
    cleaned = (name or "").replace("Buffer", "").strip()
    if cleaned[:1] in "ABCD":
        return cleaned[:1]
    return ""


def _buffer_id_map(shader: dict) -> dict:
    mapping = {}
    for rp in shader.get("renderpass") or []:
        if rp.get("type") != "buffer":
            continue
        letter = _buffer_letter(rp.get("name") or "")
        if not letter:
            continue
        for out in rp.get("outputs") or []:
            oid = out.get("id")
            if oid is not None:
                mapping[oid] = letter
    return mapping


def encode_inputs(inputs: list, shader: dict) -> str:
    idmap = _buffer_id_map(shader)
    parts = []
    for inp in inputs or []:
        try:
            ch = int(inp.get("channel", 0))
        except (TypeError, ValueError):
            continue
        ctype = inp.get("ctype") or ""
        if ctype == "buffer":
            letter = idmap.get(inp.get("id"), "?")
            parts.append(f"{ch}=buffer:{letter}")
        elif ctype == "texture":
            parts.append(f"{ch}=texture:{inp.get('src') or ''}")
        elif ctype == "keyboard":
            parts.append(f"{ch}=keyboard:")
        elif ctype:
            parts.append(f"{ch}=skip:{ctype}")
    return ";".join(parts)


def apply_shader_to_node(node, shader: dict) -> None:
    info = shader.get("info") or {}
    node.shader_id = info.get("id") or node.shader_id
    node.shader_name = info.get("name") or ""
    node.author = info.get("username") or ""

    codes = {"common": "", "A": "", "B": "", "C": "", "D": "", "image": ""}
    pass_maps: list[str] = []
    warnings: list[str] = []
    present: list[str] = []

    for rp in shader.get("renderpass") or []:
        ptype = rp.get("type") or ""
        name = rp.get("name") or ""
        code = rp.get("code") or ""
        if ptype == "common":
            codes["common"] = code
            present.append("Common")
        elif ptype == "image":
            codes["image"] = code
            present.append("Image")
            encoded = encode_inputs(rp.get("inputs") or [], shader)
            if encoded:
                pass_maps.append("IMAGE:" + encoded)
        elif ptype == "buffer":
            letter = _buffer_letter(name)
            if letter in "ABCD":
                codes[letter] = code
                present.append(f"Buffer {letter}")
                encoded = encode_inputs(rp.get("inputs") or [], shader)
                if encoded:
                    pass_maps.append(letter + ":" + encoded)
            else:
                warnings.append(f"Skipped buffer '{name}'")
        elif ptype in {"sound", "cubemap"}:
            warnings.append(f"Skipped {ptype}")
        elif ptype:
            warnings.append(f"Skipped {ptype} {name}".strip())

    node.code_common = codes["common"]
    node.code_buffer_a = codes["A"]
    node.code_buffer_b = codes["B"]
    node.code_buffer_c = codes["C"]
    node.code_buffer_d = codes["D"]
    node.code_image = codes["image"]
    node.channel_map = "|".join(pass_maps)
    node.status = " + ".join(present) if present else "Loaded (no passes?)"
    node.warning = "; ".join(warnings)
    node.error_log = ""


def apply_glsl_image(node, code: str) -> None:
    node.code_image = code
    node.status = "Pasted Image GLSL"
    node.warning = ""
    node.error_log = ""


def _active_shadertoy(context):
    node = getattr(context, "active_node", None)
    if node is None or getattr(node, "bl_idname", "") != "ImageNodeShaderToy":
        return None
    return node


def _apply_text(node, text: str) -> str | None:
    """Return a short status if text was applied, else None."""
    shader = try_parse_json_text(text)
    if shader is not None:
        apply_shader_to_node(node, shader)
        return "json"
    glsl = try_parse_glsl(text)
    if glsl is not None:
        apply_glsl_image(node, glsl)
        return "glsl"
    return None


class NODE_OT_shadertoy_fetch(Operator):
    bl_idname = "node.shadertoy_fetch"
    bl_label = "Fetch ShaderToy"
    bl_description = (
        "Load from GitHub Public+API archive, or from JSON/GLSL on the clipboard. "
        "Does not need a browser plugin"
    )
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        return _active_shadertoy(context) is not None

    def execute(self, context):
        node = _active_shadertoy(context)
        if node is None:
            self.report({"ERROR"}, "Select a ShaderToy node")
            return {"CANCELLED"}

        # URL box or clipboard may already hold JSON / GLSL.
        for source, text in (
            ("URL field", node.url),
            ("clipboard", context.window_manager.clipboard or ""),
        ):
            kind = _apply_text(node, text)
            if kind:
                self.report({"INFO"}, f"Loaded {kind} from {source}: {node.status}")
                return {"FINISHED"}

        shader_id = parse_shader_id(node.url) or parse_shader_id(node.shader_id)
        if not shader_id:
            self.report({"ERROR"}, "Paste a shadertoy.com/view/… URL or shader ID")
            return {"CANCELLED"}

        wm = context.window_manager
        wm.progress_begin(0, 1)
        try:
            shader = fetch_shader_json(shader_id, getattr(node, "api_key", ""))
            apply_shader_to_node(node, shader)
            node.url = f"https://www.shadertoy.com/view/{shader_id}"
            node.shader_id = shader_id
        except Exception as ex:  # noqa: BLE001
            view = f"https://www.shadertoy.com/view/{shader_id}"
            try:
                bpy.ops.wm.url_open(url=view)
            except Exception:  # noqa: BLE001
                pass
            node.status = "Fetch failed"
            node.warning = (
                f"{ex}  Browser opened. Image tab → Ctrl+A → Ctrl+C → Paste."
            )
            self.report(
                {"ERROR"},
                "Could not download (Cloudflare). Copied the page in your browser: "
                "Image tab, Ctrl+A, Ctrl+C, then click Paste.",
            )
            return {"CANCELLED"}
        finally:
            wm.progress_end()

        self.report({"INFO"}, f"Fetched {node.shader_name or shader_id}: {node.status}")
        return {"FINISHED"}


class NODE_OT_shadertoy_paste_json(Operator):
    bl_idname = "node.shadertoy_paste_json"
    bl_label = "Paste ShaderToy"
    bl_description = (
        "Paste clipboard: ShaderToy JSON, or GLSL that contains mainImage (Image pass)"
    )
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        return _active_shadertoy(context) is not None

    def execute(self, context):
        node = _active_shadertoy(context)
        if node is None:
            self.report({"ERROR"}, "Select a ShaderToy node")
            return {"CANCELLED"}
        text = context.window_manager.clipboard or ""
        kind = _apply_text(node, text)
        if kind is None:
            self.report(
                {"ERROR"},
                "Clipboard is not JSON or GLSL. On shadertoy.com: Image tab, Ctrl+A, Ctrl+C",
            )
            return {"CANCELLED"}
        self.report({"INFO"}, f"Pasted {kind}: {node.status}")
        return {"FINISHED"}


class NODE_OT_shadertoy_open_json(Operator, ImportHelper):
    bl_idname = "node.shadertoy_open_json"
    bl_label = "Open ShaderToy JSON"
    bl_description = "Load a ShaderToy .json file exported from the site or another app"
    bl_options = {"REGISTER", "UNDO"}

    filename_ext = ".json"
    filter_glob: bpy.props.StringProperty(default="*.json", options={"HIDDEN"})

    @classmethod
    def poll(cls, context):
        return _active_shadertoy(context) is not None

    def execute(self, context):
        node = _active_shadertoy(context)
        if node is None:
            self.report({"ERROR"}, "Select a ShaderToy node")
            return {"CANCELLED"}
        try:
            with open(self.filepath, "r", encoding="utf-8") as handle:
                text = handle.read()
        except OSError as ex:
            self.report({"ERROR"}, str(ex))
            return {"CANCELLED"}
        kind = _apply_text(node, text)
        if kind is None:
            self.report({"ERROR"}, "File is not ShaderToy JSON")
            return {"CANCELLED"}
        self.report({"INFO"}, f"Loaded {os.path.basename(self.filepath)}: {node.status}")
        return {"FINISHED"}


classes = (
    NODE_OT_shadertoy_fetch,
    NODE_OT_shadertoy_paste_json,
    NODE_OT_shadertoy_open_json,
)
