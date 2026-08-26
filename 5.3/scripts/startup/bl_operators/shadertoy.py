# SPDX-FileCopyrightText: 2026 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

"""Load ShaderToy shaders into ImageNodeShaderToy.

Fetch order:

1. JSON/GLSL already in the URL field or clipboard
2. Official ShaderToy API — only if an API key is set (live, Public+API only)
3. GitHub 2024 Public+API dump (jsDelivr) — offline fallback, not today's catalog
4. Unofficial POST / reader proxies (usually Cloudflare 403)

An API key is necessary for live fetch, not sufficient:
- Shader privacy must be Public + API (plain Public / Unlisted will 404)
- shadertoy.com may still 403 Blender (Cloudflare)
- Key needs a Silver/Gold ShaderToy account (shadertoy.com/howto)

Paste still works without any of that: Image tab, Ctrl+A, Ctrl+C, Paste.
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

# Offline fallback only (GabeRundlett dump, 2024-10-05). Not today's catalog.
_MIRRORS = (
    "https://cdn.jsdelivr.net/gh/GabeRundlett/shadertoy-api-shaders@master/shaders/{id}.json",
    "https://fastly.jsdelivr.net/gh/GabeRundlett/shadertoy-api-shaders@master/shaders/{id}.json",
    "https://raw.githubusercontent.com/GabeRundlett/shadertoy-api-shaders/master/shaders/{id}.json",
)


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


def _http_error_text(ex: Exception) -> str:
    if isinstance(ex, urllib.error.HTTPError):
        code = int(ex.code)
        if code == 403:
            return "HTTP 403 Cloudflare/forbidden"
        if code == 401:
            return "HTTP 401 bad or expired API key"
        if code == 404:
            return "HTTP 404 not found"
        if code == 429:
            return "HTTP 429 rate limited"
        return f"HTTP {code}"
    if isinstance(ex, urllib.error.URLError):
        return f"network: {ex.reason}"
    return str(ex)


def _get(url: str, timeout: float = 12):
    req = urllib.request.Request(
        url,
        headers={
            "User-Agent": _CHROME_UA,
            "Accept": "application/json, text/plain, */*",
        },
    )
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
    if not text:
        return None
    extracted = extract_mainimage(text)
    return extracted


def extract_mainimage(text: str) -> str | None:
    """Pull Image-pass GLSL out of raw GLSL or markdown fences. Skip huge HTML."""
    if not text or "mainImage" not in text:
        return None
    # Full-page copy from shadertoy.com is megabytes of HTML; stripping it stalls Fetch.
    if len(text) > 400_000 or "<html" in text[:2000].lower() or "<!doctype" in text[:200].lower():
        return None

    fences = re.findall(r"```(?:glsl|c|cpp|hs?lsl)?\s*\n(.*?)```", text, re.DOTALL | re.IGNORECASE)
    for block in fences:
        if "mainImage" in block:
            return block.strip() + "\n"

    idx = text.find("void mainImage")
    if idx < 0:
        idx = text.find("mainImage")
        if idx < 0:
            return None
        start = text.rfind("\n", 0, idx)
        idx = 0 if start < 0 else start + 1

    chunk = text[idx:]
    if chunk.startswith("{") or chunk.startswith("["):
        return None
    if "<" in chunk[:500]:
        chunk = re.sub(r"<[^>]+>", "", chunk[:80_000])
        chunk = chunk.replace("&lt;", "<").replace("&gt;", ">").replace("&amp;", "&")
    chunk = chunk.strip()
    if "mainImage" not in chunk:
        return None
    return chunk if chunk.endswith("\n") else chunk + "\n"


def _unescape_json_string(value: str) -> str:
    try:
        return json.loads(f'"{value}"')
    except json.JSONDecodeError:
        return (
            value.replace("\\n", "\n")
            .replace("\\t", "\t")
            .replace('\\"', '"')
            .replace("\\\\", "\\")
        )


def try_parse_embedded_shader(text: str) -> dict | None:
    """Best-effort: find a ShaderToy JSON blob or a \"code\" field in a page."""
    shader = try_parse_json_text(text)
    if shader is not None:
        return shader
    # gShaderToy / export JSON buried in HTML.
    for match in re.finditer(r"\{[^{}]{0,200}\"renderpass\"\s*:", text):
        start = match.start()
        snippet = text[start : start + 2_000_000]
        shader = try_parse_json_text(snippet)
        if shader is not None:
            return shader
    codes = re.findall(r'"code"\s*:\s*"((?:\\.|[^"\\])*)"', text)
    image = None
    for raw in codes:
        code = _unescape_json_string(raw)
        if "mainImage" in code:
            image = code
            break
    if image:
        return {
            "info": {"id": "", "name": "", "username": ""},
            "renderpass": [{"type": "image", "name": "Image", "code": image, "inputs": []}],
        }
    return None


def _fetch_mirrors(shader_id: str) -> dict:
    """One jsDelivr GET. 404 is instant; do not walk extra hosts/casings on a miss."""
    url = _MIRRORS[0].format(id=shader_id)
    try:
        with _get(url, timeout=4) as resp:
            data = json.loads(resp.read().decode("utf-8"))
        return coerce_shader(data)
    except Exception as ex:  # noqa: BLE001
        raise RuntimeError(
            "not in 2024 Public+API dump" + f" ({_http_error_text(ex)})"
        ) from ex


def _fetch_official(shader_id: str, key: str) -> dict:
    url = (
        f"https://www.shadertoy.com/api/v1/shaders/{urllib.parse.quote(shader_id)}"
        f"?key={urllib.parse.quote(key)}"
    )
    try:
        with _get(url, timeout=6) as resp:
            raw = resp.read().decode("utf-8", "replace")
    except urllib.error.HTTPError as ex:
        raise RuntimeError(_http_error_text(ex)) from ex
    data = json.loads(raw)
    if isinstance(data, dict) and data.get("Error"):
        err = str(data["Error"])
        low = err.lower()
        if "not found" in low or "invalid" in low:
            raise RuntimeError(
                f"{err} — shader is not Public+API (or bad ID). "
                "Webpage-visible Public/Unlisted shaders are not in the API"
            )
        raise RuntimeError(err)
    return coerce_shader(data)


def _fetch_unofficial(shader_id: str) -> dict:
    payload = urllib.parse.urlencode(
        {"s": json.dumps({"shaders": [shader_id]})}
    ).encode("utf-8")
    req = urllib.request.Request(
        "https://www.shadertoy.com/shadertoy",
        data=payload,
        method="POST",
        headers={
            "User-Agent": _CHROME_UA,
            "Accept": "application/json, text/javascript, */*; q=0.01",
            "Accept-Language": "en-US,en;q=0.9",
            "Content-Type": "application/x-www-form-urlencoded",
            "Origin": "https://www.shadertoy.com",
            "Referer": f"https://www.shadertoy.com/view/{shader_id}",
            "X-Requested-With": "XMLHttpRequest",
        },
    )
    with _opener().open(req, timeout=15) as resp:
        raw = resp.read().decode("utf-8", "replace")
    data = json.loads(raw)
    return coerce_shader(data)


def _fetch_proxies(shader_id: str) -> dict:
    urls = (
        f"https://r.jina.ai/https://www.shadertoy.com/view/{shader_id}",
        f"https://r.jina.ai/http://www.shadertoy.com/view/{shader_id}",
        f"https://api.allorigins.win/raw?url={urllib.parse.quote('https://www.shadertoy.com/view/' + shader_id, safe='')}",
    )
    last = None
    for url in urls:
        try:
            with _get(url, timeout=20) as resp:
                text = resp.read().decode("utf-8", "replace")
            shader = try_parse_embedded_shader(text)
            if shader is not None:
                info = shader.setdefault("info", {})
                info.setdefault("id", shader_id)
                return shader
            glsl = extract_mainimage(text)
            if glsl:
                return {
                    "info": {"id": shader_id, "name": shader_id, "username": ""},
                    "renderpass": [
                        {"type": "image", "name": "Image", "code": glsl, "inputs": []}
                    ],
                }
            last = RuntimeError("page had no mainImage")
        except Exception as ex:  # noqa: BLE001
            last = ex
            continue
    raise RuntimeError(str(last) if last else "proxies failed")


def resolve_api_key(api_key: str = "") -> str:
    return (
        (api_key or "").strip()
        or os.environ.get("SHADERTOY_KEY", "").strip()
        or os.environ.get("SHADERTOY_API_KEY", "").strip()
        or os.environ.get("SHADERTOY_APP_KEY", "").strip()
    )


def fetch_shader_json(shader_id: str, api_key: str = "") -> dict:
    errors: list[str] = []
    key = resolve_api_key(api_key)

    # Only two fast attempts. Unofficial POST / jina used to sit on Cloudflare for 15–20s each.
    if key:
        try:
            return _fetch_official(shader_id, key)
        except Exception as ex:  # noqa: BLE001
            errors.append(f"API: {ex}")
    else:
        errors.append("API: no key (node API Key, or SHADERTOY_KEY env)")

    try:
        return _fetch_mirrors(shader_id)
    except Exception as ex:  # noqa: BLE001
        errors.append(f"archive: {ex}")

    hint = "Image tab → Ctrl+A → Ctrl+C → Paste."
    if not key:
        hint = (
            "No API key: only the 2024 dump (or Paste) can work. "
            "Silver/Gold key at shadertoy.com/howto. "
            + hint
        )
    raise RuntimeError(" | ".join(errors) + " — " + hint)


def _buffer_letter(name: str) -> str:
    cleaned = (name or "").replace("Buffer", "").strip()
    if cleaned[:1] in "ABCD":
        return cleaned[:1]
    return ""


def _buffer_id_map(shader: dict, cubemap_slots: dict | None = None) -> dict:
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
    if cubemap_slots:
        mapping.update(cubemap_slots)
    return mapping


def _assign_cubemap_into_buffers(shader: dict, codes: dict) -> tuple[dict, list[str]]:
    """ShaderToy Cubemap A is not Buffer A–D. Store it in the first empty buffer slot."""
    mapping = {}
    notes = []
    taken = {L for L in "ABCD" if codes.get(L)}
    for rp in shader.get("renderpass") or []:
        if (rp.get("type") or "") != "cubemap":
            continue
        slot = next((L for L in "ABCD" if L not in taken), None)
        if slot is None:
            notes.append("Skipped Cubemap A (Buffer A–D already full)")
            continue
        codes[slot] = rp.get("code") or ""
        taken.add(slot)
        for out in rp.get("outputs") or []:
            oid = out.get("id")
            if oid is not None:
                mapping[oid] = slot
        notes.append(f"Cubemap A → Buffer {slot} (mainCubemap)")
    return mapping, notes


def encode_inputs(inputs: list, shader: dict, extra_idmap: dict | None = None) -> str:
    idmap = _buffer_id_map(shader, extra_idmap)
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
        elif ctype == "cubemap" and inp.get("id") in idmap:
            # Generated Cubemap A stored as Buffer A/B/C/D, not the dummy sky.
            parts.append(f"{ch}=buffer:{idmap[inp.get('id')]}")
        elif ctype == "texture":
            parts.append(f"{ch}=texture:{inp.get('src') or ''}")
        elif ctype == "keyboard":
            parts.append(f"{ch}=keyboard:")
        elif ctype == "cubemap":
            parts.append(f"{ch}=cubemap:")
        elif ctype == "volume":
            parts.append(f"{ch}=volume:")
        elif ctype:
            parts.append(f"{ch}=skip:{ctype}")
    return ";".join(parts)


def encode_channel_letters(inputs: list, shader: dict, extra_idmap: dict | None = None) -> str:
    """Compact iChannel0–3 wiring for the node (A–D / T / K / U / V).

    JSON `inputs` are not in channel-index order (mstfzS Image lists the cubemap
    first as channel 3). The GPU runtime reads positional letters, so this must
    fill slots 0–3 rather than appending in list order.
    """
    idmap = _buffer_id_map(shader, extra_idmap)
    slots = ["", "", "", ""]
    for inp in inputs or []:
        try:
            ch = int(inp.get("channel", 0))
        except (TypeError, ValueError):
            continue
        if ch < 0 or ch > 3:
            continue
        ctype = inp.get("ctype") or ""
        if ctype == "buffer":
            letter = idmap.get(inp.get("id"), "")
            slots[ch] = letter if letter in "ABCD" else "T"
        elif ctype == "texture":
            slots[ch] = "T"
        elif ctype == "keyboard":
            slots[ch] = "K"
        elif ctype == "cubemap":
            # Site HDRI cubemap vs this shader's Cubemap A pass.
            slots[ch] = idmap.get(inp.get("id"), "U")
        elif ctype == "volume":
            slots[ch] = "V"
        elif ctype:
            slots[ch] = "K"
    # Keep unused middle slots as Skip (`-`). `A,T,,K` used to collapse to
    # `A,T,K` and put the keyboard on iChannel2 (Xst3Dj reset froze).
    for i, val in enumerate(slots):
        if val == "":
            slots[i] = "-"
    while slots and slots[-1] == "-":
        slots.pop()
    return ",".join(slots)


def apply_shader_to_node(node, shader: dict) -> None:
    info = shader.get("info") or {}
    node.shader_id = info.get("id") or node.shader_id
    node.shader_name = info.get("name") or ""
    node.author = info.get("username") or ""

    codes = {"common": "", "A": "", "B": "", "C": "", "D": "", "image": ""}
    pass_maps: list[str] = []
    letters = {"image": "", "A": "", "B": "", "C": "", "D": ""}
    warnings: list[str] = []
    present: list[str] = []
    pending: list[dict] = []

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
            pending.append(rp)
        elif ptype == "buffer":
            letter = _buffer_letter(name)
            if letter in "ABCD":
                codes[letter] = code
                present.append(f"Buffer {letter}")
                pending.append(rp)
            else:
                warnings.append(f"Skipped buffer '{name}'")
        elif ptype == "cubemap":
            pending.append(rp)
        elif ptype == "sound":
            warnings.append("Skipped sound")
        elif ptype:
            warnings.append(f"Skipped {ptype} {name}".strip())

    cube_slots, cube_notes = _assign_cubemap_into_buffers(shader, codes)
    warnings.extend(cube_notes)
    if cube_slots:
        present.append("Cubemap A")
    idmap = _buffer_id_map(shader, cube_slots)

    for rp in pending:
        ptype = rp.get("type") or ""
        if ptype == "image":
            encoded = encode_inputs(rp.get("inputs") or [], shader, cube_slots)
            if encoded:
                pass_maps.append("IMAGE:" + encoded)
            letters["image"] = encode_channel_letters(rp.get("inputs") or [], shader, cube_slots)
        elif ptype == "buffer":
            letter = _buffer_letter(rp.get("name") or "")
            if letter in "ABCD":
                encoded = encode_inputs(rp.get("inputs") or [], shader, cube_slots)
                if encoded:
                    pass_maps.append(letter + ":" + encoded)
                letters[letter] = encode_channel_letters(rp.get("inputs") or [], shader, cube_slots)
        elif ptype == "cubemap":
            slot = None
            for out in rp.get("outputs") or []:
                slot = cube_slots.get(out.get("id"))
                if slot:
                    break
            if slot:
                encoded = encode_inputs(rp.get("inputs") or [], shader, cube_slots)
                if encoded:
                    pass_maps.append(slot + ":" + encoded)
                letters[slot] = encode_channel_letters(rp.get("inputs") or [], shader, cube_slots)

    node.code_common = codes["common"]
    node.code_buffer_a = codes["A"]
    node.code_buffer_b = codes["B"]
    node.code_buffer_c = codes["C"]
    node.code_buffer_d = codes["D"]
    node.code_image = codes["image"]
    node.channel_map = "|".join(pass_maps)
    # Runtime reads these, not channel_map. Clear leftovers from the previous shader.
    node.channels_image = letters["image"]
    node.channels_buffer_a = letters["A"]
    node.channels_buffer_b = letters["B"]
    node.channels_buffer_c = letters["C"]
    node.channels_buffer_d = letters["D"]
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


def _try_compile():
    try:
        bpy.ops.node.shadertoy_compile()
    except Exception:  # noqa: BLE001
        pass


def _apply_text(node, text: str) -> str | None:
    """Return a short status if text was applied, else None."""
    shader = try_parse_json_text(text) or try_parse_embedded_shader(text)
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
        "Fetch via ShaderToy API key (Public+API, live), then the 2024 dump, "
        "or load JSON/GLSL already on the clipboard"
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

        # Only treat the URL box as JSON/GLSL if it actually looks like source.
        # Do not scan the clipboard here — a copied shadertoy.com page is huge HTML
        # and used to stall Fetch for many seconds. Use the Paste button for that.
        url_text = node.url or ""
        if url_text.lstrip()[:1] in "{[" or "mainImage" in url_text:
            kind = _apply_text(node, url_text)
            if kind:
                self.report({"INFO"}, f"Loaded {kind} from URL field: {node.status}")
                return {"FINISHED"}

        shader_id = parse_shader_id(url_text) or parse_shader_id(node.shader_id)
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
            msg = str(ex)
            node.status = "Fetch failed"
            node.warning = msg
            # Don't auto-open the browser — that hid the real error.
            short = msg if len(msg) < 240 else msg[:237] + "..."
            self.report({"ERROR"}, short)
            return {"CANCELLED"}
        finally:
            wm.progress_end()

        _try_compile()
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
        if kind == "json":
            _try_compile()
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
        if kind == "json":
            _try_compile()
        self.report({"INFO"}, f"Loaded {os.path.basename(self.filepath)}: {node.status}")
        return {"FINISHED"}


classes = (
    NODE_OT_shadertoy_fetch,
    NODE_OT_shadertoy_paste_json,
    NODE_OT_shadertoy_open_json,
)
