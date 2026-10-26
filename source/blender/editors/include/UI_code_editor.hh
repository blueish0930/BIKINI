/* SPDX-FileCopyrightText: 2026 Blender Authors
 * SPDX-License-Identifier: GPL-2.0-or-later */
#pragma once

#include <algorithm>
#include <cctype>
#include <cstring>
#include <string>
#include <string_view>
#include <vector>

struct PointerRNA;
namespace blender {
struct TextboxState;
namespace ui {
struct Button;
struct Layout;

/** Shared by the geometry and shader Wrangle editors. Offsets are UTF-8 byte offsets. */
struct CodeSearchState {
  char find[256] = {};
  char replace[256] = {};
  std::string source;
  std::string query;
  std::vector<int> matches;
  int current = -1;
  bool reveal = false;

  void update(const std::string_view code)
  {
    if (source == code && query == find) {
      return;
    }
    const bool changed_query = query != find;
    const int old_offset = current >= 0 && current < int(matches.size()) ? matches[current] : 0;
    source = code;
    query = find;
    matches.clear();
    if (!query.empty()) {
      for (size_t pos = 0; (pos = source.find(query, pos)) != std::string::npos;
           pos += query.size()) {
        matches.push_back(int(pos));
      }
    }
    current = matches.empty() ? -1 :
              changed_query ? 0 :
              std::min(int(std::lower_bound(matches.begin(), matches.end(), old_offset) -
                           matches.begin()),
                       int(matches.size()) - 1);
    reveal = current >= 0 && (reveal || changed_query);
  }

  void step(const int direction)
  {
    if (!matches.empty()) {
      current = (current + direction + int(matches.size())) % int(matches.size());
      reveal = true;
    }
  }
};

/** Tabs pasted from other editors use the same two columns as the Tab key. Strings stay intact. */
inline std::string code_editor_normalize_paste(const std::string_view text)
{
  std::string result;
  bool leading = true;
  for (const char c : text) {
    if (c == '\r') {
      continue;
    }
    if (c == '\t' && leading) {
      result += "  ";
    }
    else {
      result += c;
    }
    if (c == '\n') {
      leading = true;
    }
    else if (c != ' ' && c != '\t') {
      leading = false;
    }
  }
  return result;
}

/** Whitespace-only formatter. Strings, comments and for-loop separators are lexical tokens. */
inline std::string code_editor_format(const std::string_view code)
{
  std::string result, line;
  int indent = 0, parens = 0, brackets = 0;
  auto flush = [&]() {
    while (!line.empty() && std::isspace(static_cast<unsigned char>(line.back()))) {
      line.pop_back();
    }
    if (!line.empty()) {
      result.append(size_t(std::max(indent, 0)) * 2, ' ');
      result += line;
      result += '\n';
      line.clear();
    }
  };
  auto word_at = [&](const size_t pos, const std::string_view word) {
    return code.substr(pos, word.size()) == word &&
           (pos + word.size() == code.size() ||
            !(std::isalnum(static_cast<unsigned char>(code[pos + word.size()])) ||
              code[pos + word.size()] == '_'));
  };
  for (size_t i = 0; i < code.size();) {
    const char c = code[i];
    if (std::isspace(static_cast<unsigned char>(c))) {
      if (!line.empty() && line.back() != ' ') {
        line += ' ';
      }
      i++;
      continue;
    }
    if (c == '"' || c == '\'') {
      const char quote = c;
      line += code[i++];
      while (i < code.size()) {
        const char v = code[i++];
        line += v;
        if (v == '\\' && i < code.size()) {
          line += code[i++];
        }
        else if (v == quote) {
          break;
        }
      }
      continue;
    }
    if (c == '/' && i + 1 < code.size() && code[i + 1] == '/') {
      while (i < code.size() && code[i] != '\n') {
        line += code[i++];
      }
      flush();
      continue;
    }
    if (c == '/' && i + 1 < code.size() && code[i + 1] == '*') {
      line += "/*";
      i += 2;
      while (i < code.size()) {
        if (code[i] == '*' && i + 1 < code.size() && code[i + 1] == '/') {
          line += "*/";
          i += 2;
          break;
        }
        line += code[i++];
      }
      continue;
    }
    if (c == '#') {
      flush();
      while (i < code.size()) {
        const char v = code[i++];
        line += v;
        if (v == '\n' && (line.size() < 2 || line[line.size() - 2] != '\\')) {
          break;
        }
      }
      flush();
      continue;
    }
    if (c == '{') {
      line += c;
      flush();
      indent++;
    }
    else if (c == '}') {
      flush();
      indent = std::max(indent - 1, 0);
      line += c;
      size_t next = i + 1;
      while (next < code.size() && std::isspace(static_cast<unsigned char>(code[next]))) {
        next++;
      }
      if (word_at(next, "else") || word_at(next, "while")) {
        line += ' ';
        i = next;
        continue;
      }
      if (next == code.size() || (code[next] != ';' && code[next] != ',' &&
                                  code[next] != ')' && code[next] != ']')) {
        flush();
      }
    }
    else {
      line += c;
      if (c == '(') parens++;
      if (c == ')') parens = std::max(parens - 1, 0);
      if (c == '[') brackets++;
      if (c == ']') brackets = std::max(brackets - 1, 0);
      if (c == ';' && parens == 0 && brackets == 0) {
        flush();
      }
    }
    i++;
  }
  flush();
  return result;
}

void code_editor_draw_tools(Layout &layout,
                            PointerRNA node_ptr,
                            CodeSearchState &search,
                            TextboxState &editor);
void button_code_search_set(Button *button, CodeSearchState *search);
}  // namespace ui
}  // namespace blender
