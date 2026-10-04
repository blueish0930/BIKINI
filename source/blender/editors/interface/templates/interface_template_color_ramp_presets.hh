/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edinterface
 *
 * sRGB samples for color-ramp presets. Scientific maps are subsampled from
 * matplotlib 3.9 (viridis family and turbo) and from Moreland cool-warm /
 * ColorBrewer RdBu. Included inside namespace blender::ui.
 */

#pragma once

/* Include after BLT_translation.hh. `N_` marks the strings for translation.
 * Include from inside namespace blender::ui. */

struct ColorRampPreset {
  const char *label;
  const char *description;
  const float (*srgb)[3];
  int stops;
  /** 0 = debug charts, 1 = paper and visualization maps. */
  int group;
};

static const float ramp_gray[][3] = {
    {0.000000f, 0.000000f, 0.000000f},
    {1.000000f, 1.000000f, 1.000000f},
};

static const float ramp_hue[][3] = {
    {1.000000f, 0.000000f, 0.000000f},
    {1.000000f, 0.500000f, 0.000000f},
    {1.000000f, 1.000000f, 0.000000f},
    {0.500000f, 1.000000f, 0.000000f},
    {0.000000f, 1.000000f, 0.000000f},
    {0.000000f, 1.000000f, 0.500000f},
    {0.000000f, 1.000000f, 1.000000f},
    {0.000000f, 0.500000f, 1.000000f},
    {0.000000f, 0.000000f, 1.000000f},
    {0.500000f, 0.000000f, 1.000000f},
    {1.000000f, 0.000000f, 1.000000f},
    {1.000000f, 0.000000f, 0.500000f},
    {1.000000f, 0.000000f, 0.000000f},
};

static const float ramp_heat[][3] = {
    {0.000000f, 0.000000f, 0.000000f},
    {0.000000f, 0.000000f, 1.000000f},
    {0.000000f, 1.000000f, 1.000000f},
    {0.000000f, 1.000000f, 0.000000f},
    {1.000000f, 1.000000f, 0.000000f},
    {1.000000f, 0.000000f, 0.000000f},
    {1.000000f, 1.000000f, 1.000000f},
};

static const float ramp_rgb[][3] = {
    {0.000000f, 0.000000f, 0.000000f},
    {1.000000f, 0.000000f, 0.000000f},
    {0.000000f, 1.000000f, 0.000000f},
    {0.000000f, 0.000000f, 1.000000f},
    {1.000000f, 1.000000f, 1.000000f},
};

static const float ramp_jet[][3] = {
    {0.000000f, 0.000000f, 0.500000f},
    {0.000000f, 0.000000f, 1.000000f},
    {0.000000f, 1.000000f, 1.000000f},
    {1.000000f, 1.000000f, 0.000000f},
    {1.000000f, 0.000000f, 0.000000f},
    {0.500000f, 0.000000f, 0.000000f},
};

static const float ramp_viridis[][3] = {
    {0.267004f, 0.004874f, 0.329415f},
    {0.282656f, 0.100196f, 0.422160f},
    {0.277134f, 0.185228f, 0.489898f},
    {0.253935f, 0.265254f, 0.529983f},
    {0.221989f, 0.339161f, 0.548752f},
    {0.190631f, 0.407061f, 0.556089f},
    {0.163625f, 0.471133f, 0.558148f},
    {0.139147f, 0.533812f, 0.555298f},
    {0.120565f, 0.596422f, 0.543611f},
    {0.134692f, 0.658636f, 0.517649f},
    {0.208030f, 0.718701f, 0.472873f},
    {0.327796f, 0.773980f, 0.406640f},
    {0.477504f, 0.821444f, 0.318195f},
    {0.647257f, 0.858400f, 0.209861f},
    {0.824940f, 0.884720f, 0.106217f},
    {0.993248f, 0.906157f, 0.143936f},
};

static const float ramp_plasma[][3] = {
    {0.050383f, 0.029803f, 0.527975f},
    {0.200445f, 0.017902f, 0.593364f},
    {0.312543f, 0.008239f, 0.635700f},
    {0.417642f, 0.000564f, 0.658390f},
    {0.517933f, 0.021563f, 0.654109f},
    {0.610667f, 0.090204f, 0.619951f},
    {0.692840f, 0.165141f, 0.564522f},
    {0.764193f, 0.240396f, 0.502126f},
    {0.826588f, 0.315714f, 0.441316f},
    {0.881443f, 0.392529f, 0.383229f},
    {0.928329f, 0.472975f, 0.326067f},
    {0.965024f, 0.559118f, 0.268513f},
    {0.988260f, 0.652325f, 0.211364f},
    {0.994141f, 0.753137f, 0.161404f},
    {0.977995f, 0.861432f, 0.142808f},
    {0.940015f, 0.975158f, 0.131326f},
};

static const float ramp_inferno[][3] = {
    {0.001462f, 0.000466f, 0.013866f},
    {0.046915f, 0.030324f, 0.150164f},
    {0.142378f, 0.046242f, 0.308553f},
    {0.258234f, 0.038571f, 0.406485f},
    {0.366529f, 0.071579f, 0.431994f},
    {0.472328f, 0.110547f, 0.428334f},
    {0.578304f, 0.148039f, 0.404411f},
    {0.682656f, 0.189501f, 0.360757f},
    {0.780517f, 0.243327f, 0.299523f},
    {0.865006f, 0.316822f, 0.226055f},
    {0.929644f, 0.411479f, 0.145367f},
    {0.970919f, 0.522853f, 0.058367f},
    {0.987622f, 0.645320f, 0.039886f},
    {0.978806f, 0.774545f, 0.176037f},
    {0.950018f, 0.903409f, 0.380271f},
    {0.988362f, 0.998364f, 0.644924f},
};

static const float ramp_magma[][3] = {
    {0.001462f, 0.000466f, 0.013866f},
    {0.043830f, 0.033830f, 0.141886f},
    {0.123833f, 0.067295f, 0.295879f},
    {0.232077f, 0.059889f, 0.437695f},
    {0.341482f, 0.080564f, 0.492631f},
    {0.445163f, 0.122724f, 0.506901f},
    {0.550287f, 0.161158f, 0.505719f},
    {0.658483f, 0.196027f, 0.490253f},
    {0.767398f, 0.233705f, 0.457755f},
    {0.868793f, 0.287728f, 0.409303f},
    {0.944006f, 0.377643f, 0.365136f},
    {0.981000f, 0.498428f, 0.369734f},
    {0.994738f, 0.624350f, 0.427397f},
    {0.997228f, 0.747981f, 0.516859f},
    {0.993170f, 0.870024f, 0.626189f},
    {0.987053f, 0.991438f, 0.749504f},
};

static const float ramp_cividis[][3] = {
    {0.000000f, 0.135112f, 0.304751f},
    {0.000000f, 0.181610f, 0.421859f},
    {0.117612f, 0.225935f, 0.434308f},
    {0.208926f, 0.272546f, 0.424809f},
    {0.279411f, 0.318677f, 0.423031f},
    {0.342246f, 0.364939f, 0.428559f},
    {0.401418f, 0.411790f, 0.440708f},
    {0.458366f, 0.459552f, 0.460457f},
    {0.517920f, 0.508454f, 0.472707f},
    {0.582087f, 0.558670f, 0.468118f},
    {0.648222f, 0.610553f, 0.454801f},
    {0.716177f, 0.664384f, 0.432386f},
    {0.785965f, 0.720438f, 0.399613f},
    {0.857809f, 0.778969f, 0.353259f},
    {0.932180f, 0.840159f, 0.285880f},
    {0.995737f, 0.909344f, 0.217772f},
};

static const float ramp_turbo[][3] = {
    {0.189950f, 0.071760f, 0.232170f},
    {0.253690f, 0.263270f, 0.654060f},
    {0.276910f, 0.441450f, 0.913280f},
    {0.244270f, 0.609370f, 0.996970f},
    {0.132780f, 0.771650f, 0.885800f},
    {0.103420f, 0.896000f, 0.715000f},
    {0.275970f, 0.970920f, 0.516530f},
    {0.532550f, 0.999190f, 0.305810f},
    {0.725960f, 0.964700f, 0.206400f},
    {0.883310f, 0.865530f, 0.217190f},
    {0.980000f, 0.730000f, 0.221610f},
    {0.992970f, 0.552140f, 0.154170f},
    {0.940840f, 0.355660f, 0.070310f},
    {0.839260f, 0.206540f, 0.023050f},
    {0.686020f, 0.095360f, 0.004810f},
    {0.479600f, 0.015830f, 0.010550f},
};

static const float ramp_coolwarm[][3] = {
    {0.229806f, 0.298718f, 0.753683f},
    {0.303869f, 0.406535f, 0.844959f},
    {0.383013f, 0.509419f, 0.917388f},
    {0.466667f, 0.604563f, 0.968155f},
    {0.552953f, 0.688929f, 0.995376f},
    {0.639176f, 0.759600f, 0.998151f},
    {0.722193f, 0.813953f, 0.976575f},
    {0.798692f, 0.849786f, 0.931689f},
    {0.865395f, 0.865410f, 0.865396f},
    {0.924128f, 0.827385f, 0.774508f},
    {0.958853f, 0.769768f, 0.678008f},
    {0.969954f, 0.694267f, 0.579375f},
    {0.958003f, 0.602842f, 0.481776f},
    {0.923945f, 0.497309f, 0.387970f},
    {0.869187f, 0.378313f, 0.300267f},
    {0.795632f, 0.241284f, 0.220526f},
    {0.705673f, 0.015556f, 0.150233f},
};

static const float ramp_rdbu[][3] = {
    {0.019608f, 0.188235f, 0.380392f},
    {0.129412f, 0.400000f, 0.674510f},
    {0.262745f, 0.576471f, 0.764706f},
    {0.572549f, 0.772549f, 0.870588f},
    {0.819608f, 0.898039f, 0.941176f},
    {0.968627f, 0.968627f, 0.968627f},
    {0.992157f, 0.858824f, 0.780392f},
    {0.956863f, 0.647059f, 0.509804f},
    {0.839216f, 0.376471f, 0.301961f},
    {0.698039f, 0.094118f, 0.168627f},
    {0.403922f, 0.000000f, 0.121569f},
};


static const ColorRampPreset color_ramp_presets[] = {
    {N_("Gray"),
     N_("Black to white. Scalar and shader value debug"),
     ramp_gray,
     int(sizeof(ramp_gray) / sizeof(*ramp_gray)),
     0},
    {N_("Hue"),
     N_("Full hue sweep. Angle, UV and direction debug"),
     ramp_hue,
     int(sizeof(ramp_hue) / sizeof(*ramp_hue)),
     0},
    {N_("Heat"),
     N_("Black, blue, cyan, green, yellow, red, white. Thermal debug"),
     ramp_heat,
     int(sizeof(ramp_heat) / sizeof(*ramp_heat)),
     0},
    {N_("RGB"),
     N_("Black, red, green, blue, white. Channel debug"),
     ramp_rgb,
     int(sizeof(ramp_rgb) / sizeof(*ramp_rgb)),
     0},
    {N_("Jet"),
     N_("Classic jet rainbow. Common in older papers; weak perceptual uniformity"),
     ramp_jet,
     int(sizeof(ramp_jet) / sizeof(*ramp_jet)),
     0},
    {N_("Viridis"),
     N_("Perceptually uniform sequential map. Usual default in papers"),
     ramp_viridis,
     int(sizeof(ramp_viridis) / sizeof(*ramp_viridis)),
     1},
    {N_("Plasma"),
     N_("Perceptually uniform sequential map, purple to yellow"),
     ramp_plasma,
     int(sizeof(ramp_plasma) / sizeof(*ramp_plasma)),
     1},
    {N_("Inferno"),
     N_("Perceptually uniform sequential map, black through red to yellow"),
     ramp_inferno,
     int(sizeof(ramp_inferno) / sizeof(*ramp_inferno)),
     1},
    {N_("Magma"),
     N_("Perceptually uniform sequential map, black through purple to white"),
     ramp_magma,
     int(sizeof(ramp_magma) / sizeof(*ramp_magma)),
     1},
    {N_("Cividis"),
     N_("Colorblind-safe sequential map"),
     ramp_cividis,
     int(sizeof(ramp_cividis) / sizeof(*ramp_cividis)),
     1},
    {N_("Turbo"),
     N_("Even rainbow for visualization. Smoother replacement for jet"),
     ramp_turbo,
     int(sizeof(ramp_turbo) / sizeof(*ramp_turbo)),
     1},
    {N_("Coolwarm"),
     N_("Moreland diverging map. Blue is low, red is high"),
     ramp_coolwarm,
     int(sizeof(ramp_coolwarm) / sizeof(*ramp_coolwarm)),
     1},
    {N_("RdBu"),
     N_("ColorBrewer diverging map. Blue is low, red is high"),
     ramp_rdbu,
     int(sizeof(ramp_rdbu) / sizeof(*ramp_rdbu)),
     1},
};
