/* SPDX-FileCopyrightText: 2026 Blender Authors
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Member implementation shared with MatrixLowering. Cycles' shader registers do not store
 * rotations: retain four symbolic quaternion components until a scalar/vector consumer. */

  Scalar dot3(const Vector3 &a, const Vector3 &b)
  {
    return add(add(mul(a[0], b[0]), mul(a[1], b[1])), mul(a[2], b[2]));
  }
  Vector3 cross3(const Vector3 &a, const Vector3 &b)
  {
    return {sub(mul(a[1], b[2]), mul(a[2], b[1])),
            sub(mul(a[2], b[0]), mul(a[0], b[2])),
            sub(mul(a[0], b[1]), mul(a[1], b[0]))};
  }
  Vector3 scale3(Vector3 v, const Scalar &factor)
  {
    for (Scalar &x : v) {
      x = mul(x, factor);
    }
    return v;
  }
  Vector3 add3(Vector3 a, const Vector3 &b)
  {
    for (int i = 0; i < 3; i++) {
      a[i] = add(a[i], b[i]);
    }
    return a;
  }
  Vector3 choose3(const Scalar &test, const Vector3 &a, const Vector3 &b)
  {
    return {select(test, a[0], b[0]), select(test, a[1], b[1]),
            select(test, a[2], b[2])};
  }
  Vector3 normalize3(const Vector3 &v)
  {
    const Scalar length = op(NODE_MATH_SQRT, dot3(v, v));
    return {div(v[0], length), div(v[1], length), div(v[2], length)};
  }
  Quaternion normalize_rotation(Quaternion q)
  {
    Scalar norm = 0.0f;
    for (const Scalar &x : q) {
      norm = add(norm, mul(x, x));
    }
    const Scalar nonzero = op(NODE_MATH_GREATER_THAN, norm, 0.0f);
    norm = op(NODE_MATH_SQRT, norm);
    for (int i = 0; i < 4; i++) {
      q[i] = select(nonzero, div(q[i], norm), float(i == 0));
    }
    return q;
  }
  Quaternion multiply_rotation(const Quaternion &a, const Quaternion &b)
  {
    return {sub(sub(sub(mul(a[0], b[0]), mul(a[1], b[1])), mul(a[2], b[2])),
                mul(a[3], b[3])),
            add(add(sub(mul(a[0], b[1]), mul(a[3], b[2])), mul(a[1], b[0])),
                mul(a[2], b[3])),
            add(add(sub(mul(a[0], b[2]), mul(a[1], b[3])), mul(a[2], b[0])),
                mul(a[3], b[1])),
            add(add(sub(mul(a[0], b[3]), mul(a[2], b[1])), mul(a[3], b[0])),
                mul(a[1], b[2]))};
  }
  Vector3 rotate_vector(const Quaternion &q, const Vector3 &v)
  {
    const Vector3 xyz{q[1], q[2], q[3]};
    return add3(v, scale3(cross3(xyz, add3(cross3(xyz, v), scale3(v, q[0]))), 2.0f));
  }
  Quaternion axis_angle_rotation(const Vector3 &axis, const Scalar &angle)
  {
    const Vector3 unit = normalize3(axis);
    const Scalar half = mul(angle, 0.5f);
    const Scalar sine = op(NODE_MATH_SINE, half);
    const Scalar nonzero = op(NODE_MATH_GREATER_THAN, dot3(axis, axis), 0.0f);
    return {select(nonzero, op(NODE_MATH_COSINE, half), 1.0f),
            mul(unit[0], sine), mul(unit[1], sine), mul(unit[2], sine)};
  }
  Vector3 orthogonal3(const Vector3 &v)
  {
    const Scalar xy = op(NODE_MATH_GREATER_THAN, op(NODE_MATH_ABSOLUTE, add(v[0], v[1])), 0.0f);
    const Scalar xz = op(NODE_MATH_GREATER_THAN, op(NODE_MATH_ABSOLUTE, add(v[0], v[2])), 0.0f);
    return choose3(xy, {mul(-1.0f, v[1]), v[0], 0.0f},
                   choose3(xz, {mul(-1.0f, v[2]), 0.0f, v[0]},
                           {0.0f, mul(-1.0f, v[2]), v[1]}));
  }
  Quaternion matrix_rotation(const std::array<Vector3, 3> &m)
  {
    /* Select the largest diagonal of the equivalent quaternion outer product. This avoids
     * division by a near-zero component, including half turns and tied matrix diagonals. */
    const Scalar trace = add(add(m[0][0], m[1][1]), m[2][2]);
    const std::array<Scalar, 4> diagonal{
        add(1.0f, trace), sub(add(1.0f, m[0][0]), add(m[1][1], m[2][2])),
        sub(add(1.0f, m[1][1]), add(m[0][0], m[2][2])),
        sub(add(1.0f, m[2][2]), add(m[0][0], m[1][1]))};
    const Scalar wx = sub(m[1][2], m[2][1]);
    const Scalar wy = sub(m[2][0], m[0][2]);
    const Scalar wz = sub(m[0][1], m[1][0]);
    const Scalar xy = add(m[1][0], m[0][1]);
    const Scalar xz = add(m[2][0], m[0][2]);
    const Scalar yz = add(m[2][1], m[1][2]);
    const std::array<Quaternion, 4> numerator{
        Quaternion{diagonal[0], wx, wy, wz}, Quaternion{wx, diagonal[1], xy, xz},
        Quaternion{wy, xy, diagonal[2], yz}, Quaternion{wz, xz, yz, diagonal[3]}};
    Quaternion result = numerator[0];
    Scalar largest = diagonal[0];
    for (int i = 1; i < 4; i++) {
      const Scalar use = op(NODE_MATH_GREATER_THAN, diagonal[i], largest);
      for (int c = 0; c < 4; c++) {
        result[c] = select(use, numerator[i][c], result[c]);
      }
      largest = op(NODE_MATH_MAXIMUM, largest, diagonal[i]);
    }
    result = normalize_rotation(result);
    const Scalar sign = select(op(NODE_MATH_LESS_THAN, result[0], 0.0f), -1.0f, 1.0f);
    for (Scalar &x : result) {
      x = mul(x, sign);
    }
    return result;
  }
  Quaternion axes_rotation(bNode &n)
  {
    if (n.custom1 == n.custom2) {
      return {1.0f, 0.0f, 0.0f, 0.0f};
    }
    Vector3 primary = normalize3(vector(input(n, 0)));
    Vector3 secondary = vector(input(n, 1));
    const Scalar has_primary = op(NODE_MATH_GREATER_THAN, dot3(primary, primary), 0.0f);
    const Scalar has_secondary = op(NODE_MATH_GREATER_THAN, dot3(secondary, secondary), 0.0f);
    Vector3 tertiary = cross3(primary, secondary);
    tertiary = normalize3(choose3(op(NODE_MATH_GREATER_THAN, dot3(tertiary, tertiary), 0.0f),
                                 tertiary, orthogonal3(primary)));
    const Vector3 both_secondary = cross3(tertiary, primary);
    const Vector3 primary_secondary = normalize3(orthogonal3(primary));
    const Vector3 secondary_unit = normalize3(secondary);
    const Vector3 secondary_primary = normalize3(orthogonal3(secondary_unit));
    const Vector3 p = choose3(has_primary, primary, secondary_primary);
    const Vector3 s = choose3(has_primary,
                              choose3(has_secondary, both_secondary, primary_secondary),
                              secondary_unit);
    tertiary = cross3(p, s);
    if ((n.custom2 + 1) % 3 == n.custom1) {
      tertiary = scale3(tertiary, -1.0f);
    }
    std::array<Vector3, 3> matrix;
    matrix[n.custom1] = p;
    matrix[n.custom2] = s;
    matrix[3 - n.custom1 - n.custom2] = tertiary;
    return matrix_rotation(matrix);
  }
  Quaternion align_rotation(bNode &n)
  {
    const Quaternion old = rotation(input(n, 0));
    const Vector3 v = vector(input(n, 2));
    Vector3 main{0.0f, 0.0f, 0.0f};
    main[n.custom1] = 1.0f;
    const Vector3 old_axis = rotate_vector(old, main);
    Vector3 axis;
    Scalar angle;
    if (n.custom2 == FN_NODE_ALIGN_EULER_TO_VECTOR_PIVOT_AXIS_AUTO) {
      const Vector3 target = normalize3(v);
      axis = cross3(old_axis, target);
      Vector3 fallback = cross3(old_axis, {1.0f, 0.0f, 0.0f});
      fallback = choose3(op(NODE_MATH_GREATER_THAN, dot3(fallback, fallback), 0.0f),
                         fallback, cross3(old_axis, {0.0f, 1.0f, 0.0f}));
      axis = choose3(op(NODE_MATH_GREATER_THAN, dot3(axis, axis), 0.0f), axis, fallback);
      angle = op(NODE_MATH_ARCCOSINE,
                 op(NODE_MATH_MINIMUM, 1.0f, op(NODE_MATH_MAXIMUM, -1.0f, dot3(old_axis, target))));
    }
    else {
      if (n.custom1 == n.custom2 - 1) {
        return old;
      }
      Vector3 pivot{0.0f, 0.0f, 0.0f};
      pivot[n.custom2 - 1] = 1.0f;
      axis = rotate_vector(old, pivot);
      const Vector3 a = add3(old_axis, scale3(axis, mul(-1.0f, dot3(old_axis, axis))));
      const Vector3 b = add3(v, scale3(axis, mul(-1.0f, dot3(v, axis))));
      angle = op(NODE_MATH_ARCTAN2, dot3(cross3(a, b), axis), dot3(a, b));
    }
    const Quaternion result = multiply_rotation(
        axis_angle_rotation(axis, mul(scalar(input(n, 1)), angle)), old);
    const Scalar nonzero = op(NODE_MATH_GREATER_THAN, dot3(v, v), 0.0f);
    Quaternion q;
    for (int i = 0; i < 4; i++) {
      q[i] = select(nonzero, result[i], old[i]);
    }
    return q;
  }
  Quaternion mix_rotation(bNode &n)
  {
    const auto &storage = *static_cast<const NodeShaderMix *>(n.storage);
    Scalar factor = scalar(input(n, 0));
    if (storage.clamp_factor) {
      factor = op(NODE_MATH_MINIMUM, 1.0f, op(NODE_MATH_MAXIMUM, 0.0f, factor));
    }
    const Quaternion a = rotation(input(n, 8)), b = rotation(input(n, 9));
    const Quaternion conjugate{a[0], mul(-1.0f, a[1]), mul(-1.0f, a[2]), mul(-1.0f, a[3])};
    Quaternion delta = multiply_rotation(conjugate, b);
    const Scalar sign = select(op(NODE_MATH_LESS_THAN, delta[0], 0.0f), -1.0f, 1.0f);
    for (Scalar &x : delta) {
      x = mul(x, sign);
    }
    const Vector3 axis{delta[1], delta[2], delta[3]};
    const Scalar half_angle = op(NODE_MATH_ARCTAN2, op(NODE_MATH_SQRT, dot3(axis, axis)), delta[0]);
    return multiply_rotation(a, axis_angle_rotation(axis, mul(2.0f, mul(factor, half_angle))));
  }

  const char *attribute_name(bNode &n)
  {
    const char *name = static_cast<const NodeShaderAttribute *>(n.storage)->name;
    if (const bNodeSocket *s = bke::node_find_socket(n, SOCK_IN, "Name"_ustr)) {
      const auto *value = s->default_value_typed<bNodeSocketValueString>();
      if (value && value->value[0]) {
        name = value->value;
      }
    }
    return name;
  }
  bNode &legacy_attribute(bNode &n)
  {
    if (auto found = attribute_nodes_.find(&n); found != attribute_nodes_.end()) {
      return *found->second;
    }
    bNode &attr = node("ShaderNodeAttribute"_ustr);
    auto &storage = *static_cast<NodeShaderAttribute *>(attr.storage);
    storage.type = static_cast<const NodeShaderAttribute *>(n.storage)->type;
    const bool rotation = static_cast<const NodeShaderAttribute *>(n.storage)->data_type ==
                          SHD_ATTRIBUTE_DATA_ROTATION;
    const std::string name = rotation && storage.type == SHD_ATTRIBUTE_GEOMETRY ?
                                 shader_rotation_attribute_name(attribute_name(n)) : attribute_name(n);
    STRNCPY(storage.name, name.c_str());
    /* Uniform object properties have no geometry attribute request to carry the default. */
    attr.custom2 = rotation ? SHD_ATTRIBUTE_DATA_ROTATION : 0;
    attribute_nodes_[&n] = &attr;
    return attr;
  }
