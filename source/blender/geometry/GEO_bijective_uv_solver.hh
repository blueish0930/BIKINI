/* SPDX-FileCopyrightText: 2026 Blender Authors
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SparseCholesky>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include <vector>

/* Independent numerical kernel. All optimization state stays in double
 * precision. Smith and Schaefer (2015), ED + EB; the two alternative direction
 * generators deliberately share the objective, continuous collision bound and
 * line search. */
namespace blender::geometry::bijective_uv {
using Vec = Eigen::VectorXd;
using V2 = Eigen::Vector2d;
using V3 = Eigen::Vector3d;
using M2 = Eigen::Matrix2d;
using Sparse = Eigen::SparseMatrix<double>;
using Triplet = Eigen::Triplet<double>;
constexpr double infinity = std::numeric_limits<double>::infinity();

struct Options
{
  int method = 1;
  int iterations = 500;
  int history = 8;
  double tolerance = 1e-8;
  double epsilon = 0;
  double weight = 1;
};
struct Result
{
  Vec uv;
  int iterations = 0, trials = 0, status = 0;
  double energy = 0, distortion = 0, barrier = 0, gradient = 0;
};
struct Triangle
{
  std::array<int, 3> v;
  std::array<V2, 3> basis;
  double area;
};
inline V2 point(const Vec &u, int i)
{
  return u.segment<2>(2 * i);
}
inline double cross(const V2 &a, const V2 &b)
{
  return a.x() * b.y() - a.y() * b.x();
}
inline std::array<double, 2> roots(double a, double b, double c)
{
  if (a == 0) {
    return {b == 0 ? infinity : -c / b, infinity};
  }
  const double d = b * b - 4 * a * c;
  if (d < 0) {
    return {infinity, infinity};
  }
  const double q = -.5 * (b + std::copysign(std::sqrt(d), b));
  return {q / a, q == 0 ? infinity : c / q};
}

/* Sorted sweep index avoids a dense boundary-vertex x boundary-edge scan,
 * including unusually large swept boxes where a uniform hash would enumerate
 * empty cells. */
class BoundaryIndex
{
  const Vec &u_;
  std::vector<int> order_;

public:
  BoundaryIndex(const Vec &u, const std::vector<int> &vertices) : u_(u), order_(vertices)
  {
    std::sort(order_.begin(), order_.end(), [&](int a, int b) { return u[2 * a] < u[2 * b]; });
  }
  template <class Fn> void query(const V2 &lo, const V2 &hi, Fn fn) const
  {
    auto it = std::lower_bound(order_.begin(), order_.end(), lo.x(),
                               [&](int a, double x) { return u_[2 * a] < x; });
    for (; it != order_.end() && u_[2 * (*it)] <= hi.x(); ++it) {
      const int v = *it;
      if (u_[2 * v + 1] >= lo.y() && u_[2 * v + 1] <= hi.y()) {
        fn(v);
      }
    }
  }
};

class Solver
{
public:
  std::vector<V3> positions;
  std::vector<Triangle> triangles;
  std::vector<std::array<int, 2>> boundary;
  std::vector<int> boundary_vertices;
  double area = 0, epsilon = 0;
  Options options;

  Solver(const std::vector<V3> &p, const std::vector<std::array<int, 3>> &faces, const Options &o)
      : positions(p), options(o)
  {
    if (o.method < 0 || o.method > 2 || !std::isfinite(o.tolerance) || o.tolerance < 0 ||
        !std::isfinite(o.epsilon) || o.epsilon < 0) {
      throw std::runtime_error("Invalid solver settings");
    }
    std::map<std::pair<int, int>, std::pair<int, int>> edges;
    for (auto f : faces) {
      for (int v : f) {
        if (v < 0 || v >= int(p.size())) {
          throw std::runtime_error("Invalid triangle index");
        }
      }
      const V3 e = p[f[1]] - p[f[0]], d = p[f[2]] - p[f[0]];
      const double l = e.norm(), h = e.cross(d).norm() / l, x = e.dot(d) / l;
      if (!(l > 0 && h > 0) || !std::isfinite(l * h)) {
        throw std::runtime_error("Degenerate reference triangle");
      }
      Triangle t;
      t.v = f;
      t.area = l * h * .5;
      t.basis[1] = V2(1 / l, -x / (l * h));
      t.basis[2] = V2(0, 1 / h);
      t.basis[0] = -t.basis[1] - t.basis[2];
      triangles.push_back(t);
      area += t.area;
      for (int k = 0; k < 3; k++) {
        int a = f[k], b = f[(k + 1) % 3];
        auto &item = edges[std::minmax(a, b)];
        item.first++;
        item.second += a < b ? 1 : -1;
      }
    }
    if (triangles.empty()) {
      throw std::runtime_error("An open triangle chart is required");
    }
    std::vector<int> degree(p.size(), 0);
    for (auto [e, c] : edges) {
      if (c.first > 2 || (c.first == 2 && c.second != 0)) {
        throw std::runtime_error("Charts must be oriented manifold surfaces");
      }
      if (c.first == 1) {
        boundary.push_back({e.first, e.second});
        degree[e.first]++;
        degree[e.second]++;
        epsilon += (p[e.first] - p[e.second]).norm();
      }
    }
    if (boundary.empty()) {
      throw std::runtime_error("Closed chart: split mesh edges along seams first");
    }
    for (int i = 0; i < int(p.size()); i++) {
      if (degree[i] != 0 && degree[i] != 2) {
        throw std::runtime_error("Non-manifold chart boundary");
      }
      if (degree[i]) {
        boundary_vertices.push_back(i);
      }
    }
    epsilon = o.epsilon > 0 ? o.epsilon : epsilon / double(boundary.size()) * .25;
    if (!std::isfinite(epsilon) || !(epsilon > 0) || !std::isfinite(o.weight) || o.weight < 0) {
      throw std::runtime_error("Invalid barrier parameters");
    }
  }

  Vec initialize() const
  {
    const int n = int(positions.size());
    std::vector<std::vector<int>> adj(n), badj(n);
    for (const auto &t : triangles) {
      for (int k = 0; k < 3; k++) {
        adj[t.v[k]].push_back(t.v[(k + 1) % 3]);
        adj[t.v[(k + 1) % 3]].push_back(t.v[k]);
      }
    }
    for (auto e : boundary) {
      badj[e[0]].push_back(e[1]);
      badj[e[1]].push_back(e[0]);
    }
    std::vector<int> loop;
    int prev = -1, cur = boundary_vertices[0];
    do {
      loop.push_back(cur);
      int next = badj[cur][0] == prev ? badj[cur][1] : badj[cur][0];
      prev = cur;
      cur = next;
    } while (cur != loop[0] && loop.size() <= boundary_vertices.size());
    if (loop.size() != boundary_vertices.size()) {
      throw std::runtime_error(
          "Automatic Tutte initialization needs a disk; supply a valid initial "
          "UV for charts with holes");
    }
    Vec u = Vec::Zero(2 * n);
    std::vector<int> inner(n, -1);
    int count = 0;
    double perimeter = 0;
    for (size_t k = 0; k < loop.size(); k++) {
      perimeter += (positions[loop[k]] - positions[loop[(k + 1) % loop.size()]]).norm();
    }
    double angle = 0, radius = std::sqrt(area / 3.14159265358979323846);
    for (size_t k = 0; k < loop.size(); k++) {
      u.segment<2>(2 * loop[k]) = radius * V2(std::cos(angle), std::sin(angle));
      angle += 2 * 3.14159265358979323846 *
               (positions[loop[k]] - positions[loop[(k + 1) % loop.size()]]).norm() / perimeter;
    }
    for (int i = 0; i < n; i++) {
      if (badj[i].empty()) {
        inner[i] = count++;
      }
    }
    std::vector<Triplet> trip;
    Eigen::MatrixXd rhs = Eigen::MatrixXd::Zero(count, 2);
    for (int i = 0; i < n; i++) {
      if (inner[i] >= 0) {
        auto &a = adj[i];
        std::sort(a.begin(), a.end());
        a.erase(std::unique(a.begin(), a.end()), a.end());
        if (a.empty()) {
          throw std::runtime_error("Isolated vertex in chart");
        }
        trip.emplace_back(inner[i], inner[i], double(a.size()));
        for (int j : a) {
          if (inner[j] >= 0) {
            trip.emplace_back(inner[i], inner[j], -1);
          } else {
            rhs.row(inner[i]) += point(u, j).transpose();
          }
        }
      }
    }
    if (count) {
      Sparse A(count, count);
      A.setFromTriplets(trip.begin(), trip.end());
      Eigen::SimplicialLDLT<Sparse> factor(A);
      if (factor.info() != Eigen::Success) {
        throw std::runtime_error("Tutte initialization failed");
      }
      Eigen::MatrixXd x = factor.solve(rhs);
      for (int i = 0; i < n; i++) {
        if (inner[i] >= 0) {
          u.segment<2>(2 * i) = x.row(inner[i]).transpose();
        }
      }
    }
    const auto &t = triangles[0];
    if (cross(point(u, t.v[1]) - point(u, t.v[0]), point(u, t.v[2]) - point(u, t.v[0])) < 0) {
      for (int i = 0; i < n; i++) {
        u[2 * i + 1] = -u[2 * i + 1];
      }
    }
    return u;
  }

  bool boundary_valid(const Vec &u) const
  {
    for (size_t i = 0; i < boundary.size(); i++) {
      for (size_t j = i + 1; j < boundary.size(); j++) {
        auto a = boundary[i], b = boundary[j];
        if (a[0] == b[0] || a[0] == b[1] || a[1] == b[0] || a[1] == b[1]) {
          const int shared = a[0] == b[0] || a[0] == b[1] ? a[0] : a[1];
          const V2 va = point(u, a[0] == shared ? a[1] : a[0]) - point(u, shared);
          const V2 vb = point(u, b[0] == shared ? b[1] : b[0]) - point(u, shared);
          if (cross(va, vb) == 0 && va.dot(vb) > 0) {
            return false;
          }
          continue;
        }
        V2 p = point(u, a[0]), q = point(u, a[1]), r = point(u, b[0]), s = point(u, b[1]);
        if ((p.cwiseMin(q).array() > r.cwiseMax(s).array()).any() ||
            (r.cwiseMin(s).array() > p.cwiseMax(q).array()).any()) {
          continue;
        }
        double c1 = cross(q - p, r - p), c2 = cross(q - p, s - p), c3 = cross(s - r, p - r),
               c4 = cross(s - r, q - r);
        if (((c1 <= 0 && c2 >= 0) || (c1 >= 0 && c2 <= 0)) &&
            ((c3 <= 0 && c4 >= 0) || (c3 >= 0 && c4 <= 0))) {
          return false;
        }
      }
    }
    return true;
  }

  double evaluate(const Vec &u, Vec &g, double &ed, double &eb,
                  std::vector<Triplet> *hessian = nullptr) const
  {
    g = Vec::Zero(u.size());
    ed = eb = 0;
    for (const Triangle &t : triangles) {
      M2 J = M2::Zero();
      /* Subtract a vertex to preserve translation invariance numerically. */
      for (int k = 1; k < 3; k++) {
        J += (point(u, t.v[k]) - point(u, t.v[0])) * t.basis[k].transpose();
      }
      double det = J.determinant(), norm = J.squaredNorm();
      if (!(det > 0)) {
        return infinity;
      }
      double inv = 1 / det, inv2 = inv * inv;
      ed += t.area * norm * (1 + inv2);
      M2 cofactor;
      cofactor << J(1, 1), -J(1, 0), -J(0, 1), J(0, 0);
      M2 grad = 2 * t.area * ((1 + inv2) * J - norm * inv2 * inv * cofactor);
      for (int k = 0; k < 3; k++) {
        g.segment<2>(2 * t.v[k]) += grad * t.basis[k];
      }
      if (hessian) {
        /* Analytic Hessian in row-major Jacobian coordinates, projected to PSD.
         * Projection changes only the direction model, never ED or its
         * gradient. */
        Eigen::Vector4d j(J(0, 0), J(0, 1), J(1, 0), J(1, 1));
        Eigen::Vector4d d(J(1, 1), -J(1, 0), -J(0, 1), J(0, 0));
        Eigen::Matrix4d D = Eigen::Matrix4d::Zero();
        D(0, 3) = D(3, 0) = 1;
        D(1, 2) = D(2, 1) = -1;
        Eigen::Matrix4d H =
            t.area * (2 * (1 + inv2) * Eigen::Matrix4d::Identity() -
                      4 * inv2 * inv * (j * d.transpose() + d * j.transpose()) +
                      6 * norm * inv2 * inv2 * d * d.transpose() - 2 * norm * inv2 * inv * D);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eig(H);
        if (eig.info() != Eigen::Success) {
          throw std::runtime_error("Non-finite local Hessian");
        }
        H = eig.eigenvectors() * eig.eigenvalues().cwiseMax(0).asDiagonal() *
            eig.eigenvectors().transpose();
        Eigen::Matrix<double, 4, 6> B = Eigen::Matrix<double, 4, 6>::Zero();
        for (int k = 0; k < 3; k++) {
          B.block<2, 1>(0, 2 * k) = t.basis[k];
          B.block<2, 1>(2, 2 * k + 1) = t.basis[k];
        }
        Eigen::Matrix<double, 6, 6> local = B.transpose() * H * B;
        for (int a = 0; a < 6; a++) {
          for (int b = 0; b < 6; b++) {
            hessian->emplace_back(2 * t.v[a / 2] + a % 2, 2 * t.v[b / 2] + b % 2, local(a, b));
          }
        }
      }
    }
    BoundaryIndex index(u, boundary_vertices);
    for (auto e : boundary) {
      V2 a = point(u, e[0]), b = point(u, e[1]), ab = b - a;
      const double len2 = ab.squaredNorm();
      if (!(len2 > 0)) {
        return infinity;
      }
      index.query(
          a.cwiseMin(b) - V2::Constant(epsilon), a.cwiseMax(b) + V2::Constant(epsilon), [&](int v) {
            if (v == e[0] || v == e[1]) {
              return;
            }
            double s = std::clamp((point(u, v) - a).dot(ab) / len2, 0., 1.);
            V2 r = point(u, v) - a - s * ab;
            double dist = r.norm();
            if (dist >= epsilon) {
              return;
            }
            if (!(dist > 0)) {
              eb = infinity;
              return;
            }
            double q = epsilon / dist - 1, w = options.weight;
            eb += w * q * q;
            V2 gp = (-2 * w * epsilon * q / (dist * dist * dist)) * r;
            g.segment<2>(2 * v) += gp;
            g.segment<2>(2 * e[0]) -= (1 - s) * gp;
            g.segment<2>(2 * e[1]) -= s * gp;
            if (hessian) {
              /* Positive radial curvature approximation for the barrier. */
              double curvature =
                  w * (6 * epsilon * epsilon / std::pow(dist, 4) - 4 * epsilon / std::pow(dist, 3));
              std::array<int, 3> ids = {v, e[0], e[1]};
              std::array<double, 3> coeff = {1, s - 1, -s};
              M2 H = curvature * (r / dist) * (r / dist).transpose();
              for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                  for (int c = 0; c < 2; c++) {
                    for (int d = 0; d < 2; d++) {
                      hessian->emplace_back(2 * ids[i] + c, 2 * ids[j] + d,
                                            coeff[i] * coeff[j] * H(c, d));
                    }
                  }
                }
              }
            }
          });
    }
    return ed + eb;
  }

  double step_cap(const Vec &u, const Vec &p) const
  {
    double limit = 8;
    for (const auto &t : triangles) {
      V2 a = point(u, t.v[1]) - point(u, t.v[0]), b = point(u, t.v[2]) - point(u, t.v[0]);
      V2 da = point(p, t.v[1]) - point(p, t.v[0]), db = point(p, t.v[2]) - point(p, t.v[0]);
      for (double r : roots(cross(da, db), cross(a, db) + cross(da, b), cross(a, b))) {
        if (r > 0) {
          limit = std::min(limit, r);
        }
      }
    }
    /* Conservative swept point bounds. Narrow phase solves exact quadratic
     * collinearity and checks the segment parameter at each positive root. */
    V2 max_move = V2::Zero();
    for (int v : boundary_vertices) {
      max_move = max_move.cwiseMax(point(p, v).cwiseAbs());
    }
    BoundaryIndex index(u, boundary_vertices);
    for (auto e : boundary) {
      V2 a = point(u, e[0]), b = point(u, e[1]), da = point(p, e[0]), db = point(p, e[1]);
      V2 lo = a.cwiseMin(b).cwiseMin(a + limit * da).cwiseMin(b + limit * db) - limit * max_move;
      V2 hi = a.cwiseMax(b).cwiseMax(a + limit * da).cwiseMax(b + limit * db) + limit * max_move;
      index.query(lo, hi, [&](int v) {
        if (v == e[0] || v == e[1]) {
          return;
        }
        V2 ab = b - a, dab = db - da, q = point(u, v) - a, dq = point(p, v) - da;
        const double qa = cross(dab, dq), qb = cross(ab, dq) + cross(dab, q), qc = cross(ab, q);
        if (qa == 0 && qb == 0 && qc == 0) {
          /* Permanently collinear motion has no isolated polynomial root.
           * Its first contact is a point meeting one of the edge endpoints. */
          for (int endpoint : e) {
            const V2 relative = point(u, v) - point(u, endpoint);
            const V2 velocity = point(p, v) - point(p, endpoint);
            const int axis = std::abs(velocity.x()) > std::abs(velocity.y()) ? 0 : 1;
            if (velocity[axis] == 0) {
              continue;
            }
            const double t = -relative[axis] / velocity[axis];
            if (t > 0 && t < limit &&
                (relative + t * velocity).norm() <=
                    32 * std::numeric_limits<double>::epsilon() * std::max(1., relative.norm())) {
              limit = t;
            }
          }
        }
        for (double r : roots(qa, qb, qc)) {
          if (!(r > 0 && r < limit)) {
            continue;
          }
          V2 edge = ab + r * dab, rel = q + r * dq;
          double s = rel.dot(edge) / edge.squaredNorm();
          if (s >= 0 && s <= 1) {
            limit = r;
          }
        }
      });
    }
    return .8 * limit;
  }

  Result solve(Vec u) const
  {
    if (u.size() != 2 * int(positions.size()) || !u.allFinite() || !boundary_valid(u)) {
      throw std::runtime_error("Initial UV must be finite with non-intersecting boundaries");
    }
    Result result;
    Vec g;
    double ed, eb, f = evaluate(u, g, ed, eb);
    if (!std::isfinite(f) || !g.allFinite()) {
      throw std::runtime_error("Initial UV contains flipped or degenerate triangles");
    }
    const int n = int(u.size()), m = std::clamp(options.history, 1, 32);
    Eigen::SimplicialLDLT<Sparse> metric;
    if (options.method == 1) {
      std::vector<Triplet> trip;
      for (auto &t : triangles) {
        for (int a = 0; a < 3; a++) {
          for (int b = 0; b < 3; b++) {
            for (int c = 0; c < 2; c++) {
              trip.emplace_back(2 * t.v[a] + c, 2 * t.v[b] + c,
                                8 * t.area * t.basis[a].dot(t.basis[b]));
            }
          }
        }
      }
      for (int i = 0; i < n; i++) {
        trip.emplace_back(i, i, 1e-4);
      }
      Sparse P(n, n);
      P.setFromTriplets(trip.begin(), trip.end());
      metric.compute(P);
      if (metric.info() != Eigen::Success) {
        throw std::runtime_error("Reference metric factorization failed");
      }
    }
    std::vector<Vec> ss(m), ys(m);
    std::vector<double> rho(m), alpha(m);
    int head = 0, count = 0, small = 0;
    double gamma = options.method == 0 ? .05 : 1.;
    for (int iteration = 0; iteration < std::max(options.iterations, 0); iteration++) {
      if (g.norm() <= 1e-12 * std::max(1., std::sqrt(area))) {
        result.status = 1;
        break;
      }
      Vec direction;
      if (options.method == 2) {
        std::vector<Triplet> trip;
        trip.reserve(triangles.size() * 36 + n);
        evaluate(u, g, ed, eb, &trip);
        for (int i = 0; i < n; i++) {
          trip.emplace_back(i, i, 1e-4);
        }
        Sparse H(n, n);
        H.setFromTriplets(trip.begin(), trip.end());
        Eigen::SimplicialLDLT<Sparse> factor(H);
        if (factor.info() != Eigen::Success) {
          throw std::runtime_error("Assembled system factorization failed");
        }
        direction = factor.solve(-g);
      } else {
        Vec q = g;
        for (int k = count - 1; k >= 0; k--) {
          int j = (head + k) % m;
          alpha[j] = rho[j] * ss[j].dot(q);
          q.noalias() -= alpha[j] * ys[j];
        }
        if (options.method == 1) {
          q = metric.solve(q).eval();
        }
        q *= gamma;
        for (int k = 0; k < count; k++) {
          int j = (head + k) % m;
          double beta = rho[j] * ys[j].dot(q);
          q.noalias() += ss[j] * (alpha[j] - beta);
        }
        direction = -q;
      }
      double slope = g.dot(direction);
      if (!direction.allFinite() || !(slope < 0)) {
        count = 0;
        head = 0;
        direction = -g;
        slope = -g.squaredNorm();
      }
      const double cap = step_cap(u, direction);
      double step = std::min(1., cap), lo = 0, hi = infinity, fn = 0, en = 0, bn = 0;
      Vec un, gn;
      bool accepted = false;
      for (int search = 0; search < 60; search++) {
        result.trials++;
        un = u + step * direction;
        fn = evaluate(un, gn, en, bn);
        double dn = gn.dot(direction);
        bool arm = std::isfinite(fn) && fn < f && fn <= f + 1e-4 * step * slope;
        if (arm && (std::abs(dn) <= -.9 * slope || (step >= cap && dn < 0))) {
          accepted = true;
          break;
        }
        if (!arm || dn >= 0) {
          hi = step;
        } else {
          lo = step;
        }
        double next = std::isfinite(hi) ? .5 * (lo + hi) : std::min(2 * step, cap);
        if (next == step) {
          break;
        }
        step = next;
      }
      if (!accepted) {
        result.status = 2;
        break;
      }
      Vec s = un - u, y = gn - g;
      double sy = s.dot(y);
      if (options.method != 2 && sy > 1e-10 * s.norm() * y.norm()) {
        int j = (head + count) % m;
        if (count == m) {
          head = (head + 1) % m;
        } else {
          count++;
        }
        ss[j] = std::move(s);
        ys[j] = std::move(y);
        rho[j] = 1 / sy;
        double yy = options.method == 1 ? ys[j].dot(metric.solve(ys[j])) : ys[j].squaredNorm();
        gamma = sy / yy;
      }
      double relative = (f - fn) / std::max(std::abs(f), 1e-30);
      small = (options.tolerance > 0 && relative <= options.tolerance) ? small + 1 : 0;
      u.swap(un);
      g.swap(gn);
      f = fn;
      ed = en;
      eb = bn;
      result.iterations++;
      if (small >= 5) {
        result.status = 1;
        break;
      }
    }
    result.uv = std::move(u);
    result.energy = f;
    result.distortion = ed / area;
    result.barrier = eb;
    result.gradient = g.norm();
    return result;
  }
};
} // namespace blender::geometry::bijective_uv
