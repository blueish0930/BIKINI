# SPDX-FileCopyrightText: 2023 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

'''
Usage:
    python discover_nodes.py
        <sources/root>
        <path/to/output.cc>
        <generated_function_name>
        <source>...

The goal is to make it easy for nodes to register themselves without having to have
a central place that registers all nodes manually. A node can use this mechanism by
invoking `NOD_REGISTER_NODE(register_function_name)`.

This scripts finds all those macro invocations generates code that calls the functions.
'''
__all__ = (
    "main",
)

import os
import re
import sys


def filepath_is_older(filepath_test: str, filepath_compare: tuple[str, ...]) -> bool:
    import stat
    mtime = os.stat(filepath_test)[stat.ST_MTIME]
    for filepath_other in filepath_compare:
        if mtime < os.stat(filepath_other)[stat.ST_MTIME]:
            return True
    return False


def main() -> int:
    # The build system requires the generated file to be touched if any files used to generate it are newer.
    try:
        sys.argv.remove("--use-makefile-workaround")
        use_makefile_workaround = True
    except ValueError:
        use_makefile_workaround = False

    # NOTE: avoid `pathlib`, pulls in many modules indirectly, path handling is simple enough.
    source_root = sys.argv[1]
    output_cc_file = sys.argv[2]
    function_to_generate = sys.argv[3]

    def append_cc_path(dst: list[str], path: str) -> None:
        path = path.strip().strip('"')
        if not path.endswith(".cc"):
            return
        if not os.path.isabs(path):
            path = os.path.join(source_root, path)
        dst.append(path)

    source_cc_files: list[str] = []
    argv_rest = sys.argv[4:]
    i = 0
    while i < len(argv_rest):
        arg = argv_rest[i]
        if arg == "--sources-from":
            list_path = argv_rest[i + 1]
            i += 2
            with open(list_path, "r", encoding="utf-8") as fh:
                for line in fh:
                    append_cc_path(source_cc_files, line)
            continue
        append_cc_path(source_cc_files, arg)
        i += 1

    # Sidecar next to the generated file: survives Windows argv truncation
    # (cmd.exe 8191) when CMake still passes the full source list as arguments.
    sidecar = output_cc_file + ".sources.txt"
    if os.path.isfile(sidecar):
        with open(sidecar, "r", encoding="utf-8") as fh:
            for line in fh:
                append_cc_path(source_cc_files, line)

    seen: set[str] = set()
    unique_source_cc_files: list[str] = []
    for path in source_cc_files:
        key = os.path.normcase(os.path.abspath(path))
        if key in seen:
            continue
        seen.add(key)
        unique_source_cc_files.append(path)
    source_cc_files = unique_source_cc_files

    macro_name = "NOD_REGISTER_NODE"
    discover_suffix = "_discover"

    include_lines: list[str] = []
    decl_lines: list[str] = []
    func_lines: list[str] = []

    # Add forward declaration to avoid warning.
    func_lines.append("namespace blender {")
    func_lines.append(f"void {function_to_generate}();")
    func_lines.append(f"void {function_to_generate}()")
    func_lines.append("{")

    # Use a single regular expression to search for opening name-spaces, closing name-spaces
    # and macro invocations. This makes it easy to iterate over the matches in order.
    re_namespace_begin = r"^namespace ([\w:]+) \{"
    re_namespace_end = r"^\}  // namespace ([\w:]+)"
    re_macro = r"MACRO\((\w+)\)".replace("MACRO", macro_name)
    re_all = f"({re_namespace_begin})|({re_namespace_end})|({re_macro})"
    re_all_compiled = re.compile(re_all, flags=re.MULTILINE)

    for path in source_cc_files:
        # Stale sidecar / source lists can name files that were removed.
        if not os.path.isfile(path):
            continue
        # Read the source code.
        with open(path, "r", encoding="utf-8") as fh:
            code = fh.read()

        # Keeps track of the current name-space we're in.
        namespace_parts: list[str] = []

        for match in re_all_compiled.finditer(code):
            if entered_namespace := match.group(2):
                # Enter a (nested) name-space.
                namespace_parts += entered_namespace.split("::")
            elif exited_namespace := match.group(4):
                # Exit a (nested) name-space.
                del namespace_parts[-len(exited_namespace.split("::")):]
            elif function_name := match.group(6):
                # Macro invocation in the current name-space.
                namespace_str = "::".join(namespace_parts)
                # Add suffix so that this refers to the function created by the macro.
                auto_run_name = function_name + discover_suffix

                # Declare either outside of any named name-space or in a (nested) name-space.
                # Can't declare it in an anonymous name-space because that would make the
                # declared function static.
                if namespace_str:
                    decl_lines.append(f"namespace {namespace_str} {{")
                decl_lines.append(f"void {auto_run_name}();")
                if namespace_str:
                    decl_lines.append("}")

                # Call the function.
                func_lines.append(f"  {namespace_str}::{auto_run_name}();")

    func_lines.append("}")
    func_lines.append("}  // namespace blender")

    # Write the generated code if it changed. If the newly generated code is the same as before,
    # don't overwrite the existing file to avoid unnecessary rebuilds.
    try:
        with open(output_cc_file, "r", encoding="utf-8") as fh:
            old_generated_code = fh.read()
    except Exception:
        old_generated_code = ""
    new_generated_code = "\n".join(include_lines + decl_lines + [""] + func_lines)

    if old_generated_code != new_generated_code:
        with open(output_cc_file, "w", encoding="utf-8") as fh:
            fh.write(new_generated_code)
    elif use_makefile_workaround and filepath_is_older(output_cc_file, (__file__, *source_cc_files)):
        # If the generated file is older than this command, this file would be generated every time.
        os.utime(output_cc_file)

    return 0


if __name__ == "__main__":
    sys.exit(main())
