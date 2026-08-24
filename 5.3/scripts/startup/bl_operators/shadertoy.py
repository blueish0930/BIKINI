# SPDX-FileCopyrightText: 2026 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

"""Fetch ShaderToy JSON (official API or unofficial POST) into an ImageNodeShaderToy."""

from __future__ import annotations

import json
import os
import re
import urllib.error
import urllib.parse
import urllib.request

import bpy
from bpy.types import Operator


_ID_RE = re.compile(
    r"(?:https?://)?(?:www\.)?shadertoy\.com/(?:view|embed)/([A-Za-z0-9]{5,8})",
    re.IGNORECASE,
)
_BARE_ID_RE = re.compile(r"^[A-Za-z0-9]{5,8}$")


def parse_shader_id(text: str) -> str:
    text = (text or "").strip()
    if not text:
        return ""
    m = _ID_RE.search(text)
    if m:
        return m.group(1)
    if _BARE_ID_RE.match(text):
        return text
    return ""


def _headers() -> dict[str, str]:
    return {
        "User-Agent": "Blender-ShaderToy-Node/1.0",
        "Referer": "https://www.shadertoy.com/",
        "Accept": "*/*",
        "Origin": "https://www.shadertoy.com",
    }


def fetch_shader_json(shader_id: str, api_key: str = "") -> dict:
    """Return the inner shader object with info + renderpass.

    Tries official REST (Public+API only) when a key is present, then the
    unofficial POST used by standalone ShaderToy players.
    """
    errors: list[str] = []
    key = (api_key or "").strip() or os.environ.get("SHADERTOY_KEY", "").strip()

    if key:
        url = f"https://www.shadertoy.com/api/v1/shaders/{shader_id}?key={urllib.parse.quote(key)}"
        req = urllib.request.Request(url, headers=_headers())
        try:
            with urllib.request.urlopen(req, timeout=25) as resp:
                data = json.loads(resp.read().decode("utf-8"))
            if isinstance(data, dict) and "Shader" in data:
                return data["Shader"]
            errors.append(str(data.get("Error") or data.get("error") or "official API: no Shader"))
        except Exception as ex:  # noqa: BLE001
            errors.append(f"official API: {ex}")

    body = urllib.parse.urlencode(
        {"s": json.dumps({"shaders": [shader_id]})}
    ).encode("utf-8")
    headers = dict(_headers())
    headers["Content-Type"] = "application/x-www-form-urlencoded"
    req = urllib.request.Request(
        "https://www.shadertoy.com/shadertoy",
        data=body,
        headers=headers,
    )
    try:
        with urllib.request.urlopen(req, timeout=25) as resp:
            data = json.loads(resp.read().decode("utf-8"))
        if isinstance(data, list) and data:
            return data[0]
        if isinstance(data, dict) and "info" in data:
            return data
        errors.append("unofficial POST returned empty shader (need Public, or Public+API with a key)")
    except Exception as ex:  # noqa: BLE001
        errors.append(f"unofficial POST: {ex}")

    raise RuntimeError(" | ".join(errors) if errors else "Fetch failed")


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
    node.shader_id = info.get("id") or ""
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
    node.status = " + ".join(present) if present else "Fetched (no passes?)"
    node.warning = "; ".join(warnings)
    node.error_log = ""


class NODE_OT_shadertoy_fetch(Operator):
    bl_idname = "node.shadertoy_fetch"
    bl_label = "Fetch ShaderToy"
    bl_description = (
        "Download Common / Buffer A–D / Image GLSL from shadertoy.com into this node. "
        "Uses the official API when an API key is set, otherwise unofficial POST"
    )
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        space = getattr(context, "space_data", None)
        node = getattr(context, "active_node", None)
        if space is None or getattr(space, "tree_type", "") not in {
            "ImageNodeTree",
            "CompositorNodeTree",
        }:
            # Image editor tree type is ImageNodeTree.
            if node is None or node.bl_idname != "ImageNodeShaderToy":
                return False
        return node is not None and node.bl_idname == "ImageNodeShaderToy"

    def execute(self, context):
        node = context.active_node
        if node is None or node.bl_idname != "ImageNodeShaderToy":
            self.report({"ERROR"}, "Select a ShaderToy node")
            return {"CANCELLED"}

        shader_id = parse_shader_id(node.url) or parse_shader_id(node.shader_id)
        if not shader_id:
            self.report({"ERROR"}, "Paste a shadertoy.com/view/… URL or a shader ID first")
            return {"CANCELLED"}

        wm = context.window_manager
        wm.progress_begin(0, 1)
        try:
            shader = fetch_shader_json(shader_id, getattr(node, "api_key", ""))
            apply_shader_to_node(node, shader)
            node.url = f"https://www.shadertoy.com/view/{shader_id}"
            node.shader_id = shader_id
        except Exception as ex:  # noqa: BLE001
            node.status = "Fetch failed"
            node.warning = str(ex)
            self.report({"ERROR"}, f"ShaderToy fetch failed: {ex}")
            return {"CANCELLED"}
        finally:
            wm.progress_end()

        self.report({"INFO"}, f"Fetched {node.shader_name or shader_id}: {node.status}")
        return {"FINISHED"}


classes = (
    NODE_OT_shadertoy_fetch,
)
