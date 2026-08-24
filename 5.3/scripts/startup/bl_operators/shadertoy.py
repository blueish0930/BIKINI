# SPDX-FileCopyrightText: 2026 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

"""Fetch ShaderToy JSON into an ImageNodeShaderToy.

shadertoy.com is fronted by Cloudflare and commonly returns 403 to
urllib/curl. Order:

1. Official API (needs a key from shadertoy.com/howto, Public+API only)
2. Unofficial POST (browser-like cookies + headers)
3. Public GitHub snapshot of API shaders (no Cloudflare)
4. JSON already in the URL box, or the clipboard
"""

from __future__ import annotations

import http.cookiejar
import json
import os
import re
import ssl
import subprocess
import tempfile
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

_CHROME_UA = (
    "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 "
    "(KHTML, like Gecko) Chrome/131.0.0.0 Safari/537.36"
)

_MIRRORS = (
    "https://raw.githubusercontent.com/GabeRundlett/shadertoy-api-shaders/master/shaders/{id}.json",
    "https://cdn.jsdelivr.net/gh/GabeRundlett/shadertoy-api-shaders@master/shaders/{id}.json",
)


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


def _browser_headers(referer: str = "https://www.shadertoy.com/") -> dict[str, str]:
    return {
        "User-Agent": _CHROME_UA,
        "Accept": "application/json,text/html,application/xhtml+xml,*/*;q=0.8",
        "Accept-Language": "en-US,en;q=0.9",
        "Origin": "https://www.shadertoy.com",
        "Referer": referer,
    }


def coerce_shader(data) -> dict:
    """Accept official API wrap, unofficial array, or a bare shader object."""
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


def _urlopen(opener, req, timeout=25):
    try:
        return opener.open(req, timeout=timeout)
    except urllib.error.HTTPError as err:
        body = ""
        try:
            body = err.read()[:180].decode("utf-8", "replace")
        except Exception:  # noqa: BLE001
            pass
        raise RuntimeError(f"HTTP {err.code} {err.reason}" + (f" ({body})" if body else "")) from err


def _fetch_official(opener, shader_id: str, key: str) -> dict:
    url = (
        f"https://www.shadertoy.com/api/v1/shaders/{shader_id}"
        f"?key={urllib.parse.quote(key)}"
    )
    req = urllib.request.Request(url, headers=_browser_headers())
    with _urlopen(opener, req) as resp:
        data = json.loads(resp.read().decode("utf-8"))
    if isinstance(data, dict) and data.get("Error"):
        raise RuntimeError(str(data["Error"]))
    return coerce_shader(data)


def _fetch_unofficial_post(opener, shader_id: str) -> dict:
    view = f"https://www.shadertoy.com/view/{shader_id}"
    # Warm cookies; ShaderToy requires a session cookie.
    get_req = urllib.request.Request(view, headers=_browser_headers("https://www.shadertoy.com/"))
    try:
        with _urlopen(opener, get_req, timeout=15) as resp:
            resp.read(64)
    except Exception:  # noqa: BLE001
        pass

    payload = urllib.parse.urlencode(
        {"s": json.dumps({"shaders": [shader_id]})}
    ).encode("utf-8")
    headers = _browser_headers(view)
    headers["Content-Type"] = "application/x-www-form-urlencoded"
    headers["X-Requested-With"] = "XMLHttpRequest"
    for endpoint in ("https://www.shadertoy.com/shadertoy", "https://www.shadertoy.com/shadertoy/"):
        req = urllib.request.Request(endpoint, data=payload, headers=headers)
        with _urlopen(opener, req) as resp:
            data = json.loads(resp.read().decode("utf-8"))
        return coerce_shader(data)
    raise RuntimeError("unofficial POST empty")


def _fetch_curl(shader_id: str) -> dict:
    """Windows curl sometimes gets a different TLS fingerprint than urllib."""
    view = f"https://www.shadertoy.com/view/{shader_id}"
    payload = urllib.parse.urlencode({"s": json.dumps({"shaders": [shader_id]})})
    with tempfile.NamedTemporaryFile(prefix="stoy_", suffix=".txt", delete=False) as tmp:
        cookie_path = tmp.name
    try:
        subprocess.run(
            [
                "curl.exe",
                "-sS",
                "-L",
                "--max-time",
                "20",
                "-A",
                _CHROME_UA,
                "-c",
                cookie_path,
                "-b",
                cookie_path,
                "-o",
                os.devnull,
                view,
            ],
            check=False,
            capture_output=True,
            timeout=25,
        )
        proc = subprocess.run(
            [
                "curl.exe",
                "-sS",
                "-L",
                "--max-time",
                "20",
                "-A",
                _CHROME_UA,
                "-c",
                cookie_path,
                "-b",
                cookie_path,
                "-H",
                f"Referer: {view}",
                "-H",
                "Origin: https://www.shadertoy.com",
                "-H",
                "Content-Type: application/x-www-form-urlencoded",
                "--data",
                payload,
                "https://www.shadertoy.com/shadertoy",
            ],
            check=False,
            capture_output=True,
            timeout=25,
        )
    finally:
        try:
            os.unlink(cookie_path)
        except OSError:
            pass
    if proc.returncode != 0:
        err = (proc.stderr or b"").decode("utf-8", "replace")[:180]
        raise RuntimeError(err or f"curl exit {proc.returncode}")
    text = (proc.stdout or b"").decode("utf-8", "replace")
    if "403" in text and "Forbidden" in text and len(text) < 80:
        raise RuntimeError("HTTP 403 Forbidden")
    return coerce_shader(json.loads(text))


def _fetch_mirror(shader_id: str) -> dict:
    """Public+API snapshot (Oct 2024). Bypasses Cloudflare on shadertoy.com."""
    last = None
    ctx = ssl.create_default_context()
    opener = urllib.request.build_opener(urllib.request.HTTPSHandler(context=ctx))
    for tmpl in _MIRRORS:
        url = tmpl.format(id=shader_id)
        req = urllib.request.Request(url, headers={"User-Agent": _CHROME_UA})
        try:
            with opener.open(req, timeout=20) as resp:
                data = json.loads(resp.read().decode("utf-8"))
            return coerce_shader(data)
        except Exception as ex:  # noqa: BLE001
            last = ex
            continue
    raise RuntimeError(str(last) if last else "mirror miss")


def fetch_shader_json(shader_id: str, api_key: str = "") -> dict:
    errors: list[str] = []
    key = (api_key or "").strip() or os.environ.get("SHADERTOY_KEY", "").strip()
    cj = http.cookiejar.CookieJar()
    opener = urllib.request.build_opener(urllib.request.HTTPCookieProcessor(cj))

    attempts = []
    if key:
        attempts.append(("official API", lambda: _fetch_official(opener, shader_id, key)))
    attempts.append(("unofficial POST", lambda: _fetch_unofficial_post(opener, shader_id)))
    if os.name == "nt":
        attempts.append(("curl", lambda: _fetch_curl(shader_id)))
    attempts.append(("GitHub mirror", lambda: _fetch_mirror(shader_id)))

    for name, fn in attempts:
        try:
            return fn()
        except Exception as ex:  # noqa: BLE001
            errors.append(f"{name}: {ex}")

    hint = (
        "Cloudflare is blocking Blender (HTTP 403). "
        "Open the shader in a browser, install the ShaderToy unofficial plugin, "
        "Export JSON, then use Paste JSON — or put a key from shadertoy.com/howto "
        "in the node API Key (only Public+API shaders)."
    )
    raise RuntimeError(" | ".join(errors) + " — " + hint)


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
    node.status = " + ".join(present) if present else "Fetched (no passes?)"
    node.warning = "; ".join(warnings)
    node.error_log = ""


def _active_shadertoy(context):
    node = getattr(context, "active_node", None)
    if node is None or node.bl_idname != "ImageNodeShaderToy":
        return None
    return node


class NODE_OT_shadertoy_fetch(Operator):
    bl_idname = "node.shadertoy_fetch"
    bl_label = "Fetch ShaderToy"
    bl_description = (
        "Download Common / Buffer A–D / Image. Tries official API, unofficial POST, "
        "then a GitHub Public+API snapshot if Cloudflare returns 403"
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

        pasted = try_parse_json_text(node.url)
        if pasted is not None:
            apply_shader_to_node(node, pasted)
            self.report({"INFO"}, f"Loaded JSON from URL field: {node.status}")
            return {"FINISHED"}

        shader_id = parse_shader_id(node.url) or parse_shader_id(node.shader_id)
        if not shader_id:
            self.report({"ERROR"}, "Paste a shadertoy.com/view/… URL, a shader ID, or JSON")
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


class NODE_OT_shadertoy_paste_json(Operator):
    bl_idname = "node.shadertoy_paste_json"
    bl_label = "Paste ShaderToy JSON"
    bl_description = (
        "Load ShaderToy JSON from the clipboard (export from the unofficial browser plugin, "
        "or copy the /shadertoy POST response in DevTools)"
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
        shader = try_parse_json_text(text)
        if shader is None:
            self.report(
                {"ERROR"},
                "Clipboard is not ShaderToy JSON. In the unofficial plugin: Export. "
                "Or DevTools → Network → shadertoy → copy Response",
            )
            return {"CANCELLED"}
        apply_shader_to_node(node, shader)
        sid = parse_shader_id(getattr(node, "shader_id", "") or "")
        if sid:
            node.url = f"https://www.shadertoy.com/view/{sid}"
        self.report({"INFO"}, f"Pasted JSON: {node.status}")
        return {"FINISHED"}


classes = (
    NODE_OT_shadertoy_fetch,
    NODE_OT_shadertoy_paste_json,
)
