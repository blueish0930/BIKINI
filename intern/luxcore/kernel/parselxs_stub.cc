/* SPDX-FileCopyrightText: 2026 BIKINI Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <cstdio>
#include <string>

#include "luxrays/utils/properties.h"

#ifdef _MSC_VER
#  pragma comment(lib, "E:/Blender_Source/lib/windows_x64/imath/lib/Imath.lib")
#endif

FILE *luxcore_parserlxs_yyin = nullptr;

int luxcore_parserlxs_yyparse(void)
{
  return 1;
}

void luxcore_parserlxs_yyrestart(FILE * /*new_file*/) {}

namespace luxcore {
namespace parselxs {

void IncludeClear() {}
void ResetParser() {}

std::string currentFile;
unsigned int lineNum = 0;
luxrays::Properties overwriteProps;
luxrays::observer_ptr<luxrays::Properties> renderConfigProps;
luxrays::observer_ptr<luxrays::Properties> sceneProps;

}  // namespace parselxs
}  // namespace luxcore
