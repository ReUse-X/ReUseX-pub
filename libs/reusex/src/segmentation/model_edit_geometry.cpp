// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/segmentation/model_edit_geometry.hpp"

#include "reusex/core/guid.hpp"
#include "reusex/core/logging.hpp"
#include "reusex/core/space_identity.hpp"
#include "reusex/core/space_polygon.hpp"

#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_set_2.h>
#include <CGAL/Polygon_with_holes_2.h>

#include <Eigen/Dense>
#include <fmt/format.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <limits>
#include <map>
#include <numbers>
#include <optional>
#include <random>
#include <set>
#include <stdexcept>
#include <tuple>

namespace reusex::segmentation {

namespace {

using K = CGAL::Exact_predicates_exact_constructions_kernel;
using Point = K::Point_2;
using Segment = K::Segment_2;
using Traits = CGAL::Arr_segment_traits_2<K>;
using Arr = CGAL::Arrangement_2<Traits>;
using Ring = std::vector<core::PlanPoint>;
using V = std::array<double, 2>;

double ring_signed_area(const Ring &r) {
  double a = 0.0;
  for (size_t i = 0; i < r.size(); ++i) {
    const auto &p = r[i];
    const auto &q = r[(i + 1) % r.size()];
    a += p[0] * q[1] - q[0] * p[1];
  }
  return 0.5 * a;
}

bool in_ring(const Ring &r, double x, double y) {
  bool in = false;
  for (size_t i = 0, j = r.size() - 1; i < r.size(); j = i++) {
    const auto &a = r[i];
    const auto &b = r[j];
    if ((a[1] > y) != (b[1] > y) &&
        x < (b[0] - a[0]) * (y - a[1]) / (b[1] - a[1]) + a[0])
      in = !in;
  }
  return in;
}

bool in_polygon(const core::SpacePolygon &p, double x, double y) {
  if (!in_ring(p.outer, x, y))
    return false;
  for (const auto &h : p.holes)
    if (in_ring(h, x, y))
      return false;
  return true;
}

/// Start the ring at its lexicographically smallest vertex.
Ring canonical(Ring r) {
  if (r.empty())
    return r;
  const auto it = std::min_element(r.begin(), r.end());
  std::rotate(r.begin(), it, r.end());
  return r;
}

template <typename Ccb> Ring ring_of(Ccb first) {
  Ring r;
  auto c = first;
  const auto same = [](const core::PlanPoint &a, const core::PlanPoint &b) {
    return std::abs(a[0] - b[0]) < 1e-9 && std::abs(a[1] - b[1]) < 1e-9;
  };
  do {
    const auto &p = c->source()->point();
    const core::PlanPoint q{CGAL::to_double(p.x()), CGAL::to_double(p.y())};
    // Vertices an exact construction puts apart but doubles round together.
    if (r.empty() || !same(r.back(), q))
      r.push_back(q);
  } while (++c != first);
  while (r.size() > 1 && same(r.front(), r.back()))
    r.pop_back();
  // Collinear vertices (a run split by a crossing that was later removed)
  // are kept: harmless.
  return r;
}

double ring_length(const Ring &r) {
  double l = 0.0;
  for (size_t i = 0; i < r.size(); ++i) {
    const auto &p = r[i];
    const auto &q = r[(i + 1) % r.size()];
    l += std::hypot(q[0] - p[0], q[1] - p[1]);
  }
  return l;
}

/// A point strictly inside @p poly: an edge midpoint nudged inwards.
std::optional<V> interior_point(const core::SpacePolygon &poly) {
  const Ring &r = poly.outer;
  const double sign = ring_signed_area(r) >= 0.0 ? 1.0 : -1.0;
  for (const double eps : {1e-3, 1e-2, 5e-2}) {
    for (size_t i = 0; i < r.size(); ++i) {
      const auto &p = r[i];
      const auto &q = r[(i + 1) % r.size()];
      const double dx = q[0] - p[0], dy = q[1] - p[1];
      const double len = std::hypot(dx, dy);
      if (len < 4.0 * eps)
        continue;
      // Left of the edge is inside a counter-clockwise ring.
      const V m{0.5 * (p[0] + q[0]) - sign * eps * dy / len,
                0.5 * (p[1] + q[1]) + sign * eps * dx / len};
      if (in_polygon(poly, m[0], m[1]))
        return m;
    }
  }
  return std::nullopt;
}

double distance_to_segment(V p, V a, V b) {
  const double abx = b[0] - a[0], aby = b[1] - a[1];
  const double l2 = abx * abx + aby * aby;
  const double t =
      l2 > 0.0 ? std::clamp(((p[0] - a[0]) * abx + (p[1] - a[1]) * aby) / l2,
                            0.0, 1.0)
               : 0.0;
  return std::hypot(p[0] - (a[0] + t * abx), p[1] - (a[1] + t * aby));
}

struct WallGeom {
  const core::WallRecord *w;
  V p0, p1, d, n; ///< ends, unit direction, left normal
  double len, half;
};

WallGeom geom_of(const core::WallRecord &w) {
  WallGeom g;
  g.w = &w;
  g.p0 = {w.x0, w.y0};
  g.p1 = {w.x1, w.y1};
  g.len = std::hypot(w.x1 - w.x0, w.y1 - w.y0);
  g.d = {(w.x1 - w.x0) / g.len, (w.y1 - w.y0) / g.len};
  g.n = {-g.d[1], g.d[0]};
  g.half = w.thickness > 0.0 ? 0.5 * w.thickness : 0.0;
  return g;
}

V at(const WallGeom &g, double s, double off) {
  return {g.p0[0] + s * g.d[0] + off * g.n[0],
          g.p0[1] + s * g.d[1] + off * g.n[1]};
}

void fill_metrics(core::SpaceRecord &s) {
  s.area = core::space_polygon_area(s.polygon);
  s.perimeter = ring_length(s.polygon.outer);
  s.volume = s.area * std::max(0.0, s.z_ceiling - s.z_floor);
  s.compactness = s.perimeter > 0.0 ? 4.0 * std::numbers::pi * s.area /
                                          (s.perimeter * s.perimeter)
                                    : -1.0;
}

/// The stretches of @p walls' faces the edges of @p s lie on (within
/// @p tolerance_m, parallel), one boundary per (wall, side).
std::vector<core::SpaceBoundaryRecord>
find_boundaries(const core::SpaceRecord &s, const std::vector<WallGeom> &walls,
                double tolerance_m) {
  std::vector<core::SpaceBoundaryRecord> out;
  std::map<std::pair<std::string, std::string>, std::pair<double, double>> hull;
  const auto edges = [&](const Ring &r) {
    for (size_t i = 0; i < r.size(); ++i) {
      const V a{r[i][0], r[i][1]};
      const V b{r[(i + 1) % r.size()][0], r[(i + 1) % r.size()][1]};
      const double el = std::hypot(b[0] - a[0], b[1] - a[1]);
      if (el < 1e-6)
        continue;
      const V ed{(b[0] - a[0]) / el, (b[1] - a[1]) / el};
      const V mid{0.5 * (a[0] + b[0]), 0.5 * (a[1] + b[1])};
      for (const auto &g : walls) {
        if (std::abs(ed[0] * g.n[0] + ed[1] * g.n[1]) > 0.01)
          continue; // not parallel
        const auto off_of = [&](V p) {
          return (p[0] - g.p0[0]) * g.n[0] + (p[1] - g.p0[1]) * g.n[1];
        };
        const double oa = off_of(a), ob = off_of(b);
        std::string side;
        for (const double face : {g.half, -g.half}) {
          if (std::abs(oa - face) <= tolerance_m &&
              std::abs(ob - face) <= tolerance_m) {
            // The space is on the far side of the face from the axis; a
            // one-sided wall: the side its interior lies on.
            double sgn = face;
            if (g.half <= 0.0) {
              const V in{mid[0] - 1e-3 * ed[1], mid[1] + 1e-3 * ed[0]};
              const V probe =
                  in_polygon(s.polygon, in[0], in[1])
                      ? in
                      : V{mid[0] + 1e-3 * ed[1], mid[1] - 1e-3 * ed[0]};
              sgn = off_of(probe);
            }
            side = sgn >= 0.0 ? "a" : "b";
            break;
          }
        }
        if (side.empty())
          continue;
        const auto s_of = [&](V p) {
          return (p[0] - g.p0[0]) * g.d[0] + (p[1] - g.p0[1]) * g.d[1];
        };
        double s0 = std::clamp(std::min(s_of(a), s_of(b)), 0.0, g.len);
        double s1 = std::clamp(std::max(s_of(a), s_of(b)), 0.0, g.len);
        if (s1 - s0 < 1e-3)
          continue;
        const auto key = std::make_pair(g.w->guid, side);
        if (auto it = hull.find(key); it != hull.end()) {
          it->second.first = std::min(it->second.first, s0);
          it->second.second = std::max(it->second.second, s1);
        } else {
          hull.emplace(key, std::make_pair(s0, s1));
        }
      }
    }
  };
  edges(s.polygon.outer);
  for (const auto &h : s.polygon.holes)
    edges(h);
  for (const auto &[key, range] : hull)
    out.push_back({s.guid, key.first, key.second, range.first, range.second});
  return out;
}

} // namespace

RebuildSpacesResult
rebuild_spaces_from_walls(const std::vector<core::WallRecord> &walls_in,
                          const std::vector<core::SpaceRecord> &stored_in,
                          const core::StoreyRecord &storey,
                          std::string_view project_guid,
                          const RebuildSpacesOptions &opt) {
  (void)core::deterministic_guid(project_guid, "spaces"); // validates
  RebuildSpacesResult res;
  std::vector<WallGeom> walls;
  for (const auto &w : walls_in)
    if (w.storey == storey.idx && std::hypot(w.x1 - w.x0, w.y1 - w.y0) > 1e-6)
      walls.push_back(geom_of(w));
  std::vector<const core::SpaceRecord *> stored;
  for (const auto &s : stored_in)
    if (s.storey == storey.idx)
      stored.push_back(&s);
  res.walls = static_cast<int>(walls.size());

  // 1. The arrangement of the wall faces, extended runs and end caps.
  std::vector<Segment> segs;
  const auto seg = [&](V a, V b) {
    if (std::hypot(b[0] - a[0], b[1] - a[1]) > 1e-9)
      segs.emplace_back(Point(a[0], a[1]), Point(b[0], b[1]));
  };
  const double ext = opt.join_extension_m;
  for (const auto &g : walls) {
    if (g.half > 0.0) {
      for (const double off : {g.half, -g.half})
        seg(at(g, -ext, off), at(g, g.len + ext, off));
      seg(at(g, 0.0, g.half), at(g, 0.0, -g.half));
      seg(at(g, g.len, g.half), at(g, g.len, -g.half));
    } else {
      seg(at(g, -ext, 0.0), at(g, g.len + ext, 0.0));
    }
  }
  Arr arr;
  CGAL::insert(arr, segs.begin(), segs.end());
  // Dangling stubs: an edge with the same face on both sides.
  for (bool again = true; again;) {
    again = false;
    for (auto e = arr.edges_begin(); e != arr.edges_end(); ++e)
      if (e->face() == e->twin()->face()) {
        arr.remove_edge(e);
        again = true;
        break;
      }
  }

  // 2. Bounded faces -> candidate polygons.
  std::vector<core::SpacePolygon> polys;
  for (auto f = arr.faces_begin(); f != arr.faces_end(); ++f) {
    if (f->is_unbounded() || !f->has_outer_ccb())
      continue;
    ++res.faces;
    core::SpacePolygon poly;
    poly.outer = canonical(ring_of(f->outer_ccb()));
    if (ring_signed_area(poly.outer) < 0.0)
      std::reverse(poly.outer.begin(), poly.outer.end());
    for (auto h = f->inner_ccbs_begin(); h != f->inner_ccbs_end(); ++h) {
      Ring hole = canonical(ring_of(*h));
      if (hole.size() >= 3 && std::abs(ring_signed_area(hole)) > 1e-6)
        poly.holes.push_back(std::move(hole));
    }
    if (poly.outer.size() < 3)
      continue;
    const double area = core::space_polygon_area(poly);
    const double width = 2.0 * area / std::max(1e-9, ring_length(poly.outer));
    if (area < opt.min_area_m2 || width < opt.min_width_m) {
      ++res.too_small;
      continue;
    }
    const auto p = interior_point(poly);
    const bool body =
        p && std::any_of(walls.begin(), walls.end(), [&](const WallGeom &g) {
          return g.half > 0.0 &&
                 distance_to_segment(*p, g.p0, g.p1) < g.half + 1e-6;
        });
    if (body) {
      ++res.wall_bodies;
      continue;
    }
    polys.push_back(std::move(poly));
  }

  // 3. Names and guids from the stored spaces, by overlap, one-to-one.
  std::vector<std::tuple<double, size_t, size_t>> pairs; // -share, new, old
  for (size_t i = 0; i < polys.size(); ++i) {
    const double ai = core::space_polygon_area(polys[i]);
    for (size_t j = 0; j < stored.size(); ++j) {
      const double aj = core::space_polygon_area(stored[j]->polygon);
      const double ov = core::space_overlap_area(polys[i], stored[j]->polygon);
      const double share = ov / std::max(1e-9, std::min(ai, aj));
      if (share >= opt.min_name_overlap)
        pairs.emplace_back(-share, i, j);
    }
  }
  std::sort(pairs.begin(), pairs.end());
  std::vector<int> took(polys.size(), -1);
  std::vector<bool> used(stored.size(), false);
  for (const auto &[neg, i, j] : pairs)
    if (took[i] < 0 && !used[j]) {
      took[i] = static_cast<int>(j);
      used[j] = true;
    }

  std::set<std::string> taken;
  for (const auto &s : stored_in)
    taken.insert(s.guid);
  for (size_t i = 0; i < polys.size(); ++i) {
    core::SpaceRecord s;
    if (took[i] >= 0) {
      s = *stored[static_cast<size_t>(took[i])];
      ++res.matched;
    } else {
      s.storey = storey.idx;
      s.kind = "room";
      s.z_floor = storey.z_floor;
      s.z_ceiling = storey.z_ceiling;
      s.source = "manual";
    }
    s.id = -1;
    s.polygon = polys[i];
    if (took[i] < 0) {
      const std::string key = core::space_natural_key(s);
      std::string g = core::deterministic_guid(project_guid, key);
      for (int k = 2; taken.count(g); ++k)
        g = core::deterministic_guid(project_guid,
                                     fmt::format("{}#{}", key, k));
      s.guid = g;
      taken.insert(g);
    }
    fill_metrics(s);
    res.spaces.push_back(std::move(s));
  }
  for (size_t j = 0; j < stored.size(); ++j)
    if (!used[j])
      res.unmatched.push_back(stored[j]->guid);

  // 4. Boundaries: polygon edges on a wall face.
  for (const auto &s : res.spaces) {
    auto bs = find_boundaries(s, walls, opt.boundary_tolerance_m);
    res.boundaries.insert(res.boundaries.end(), bs.begin(), bs.end());
  }

  if (res.spaces.empty())
    reusex::warn("rebuild_spaces_from_walls: storey {}: no space from {} "
                 "wall(s): {} bounded face(s), {} wall bod(y/ies), {} below "
                 "{:.2f} m^2; do the walls close a room?",
                 storey.idx, res.walls, res.faces, res.wall_bodies,
                 res.too_small, opt.min_area_m2);
  return res;
}

std::vector<core::ModelEditOp>
rebuild_spaces_ops(const RebuildSpacesResult &result, bool remove_unmatched) {
  std::vector<core::ModelEditOp> ops;
  for (const auto &s : result.spaces) {
    std::vector<core::SpaceBoundaryRecord> bs;
    for (const auto &b : result.boundaries)
      if (b.space == s.guid)
        bs.push_back(b);
    ops.push_back(core::ModelEditOp::put(s, std::move(bs)));
  }
  if (!remove_unmatched)
    return ops;
  for (const auto &g : result.unmatched)
    ops.push_back(core::ModelEditOp::remove(core::ModelElementKind::space, g));
  return ops;
}

CeilingFitResult fit_ceiling_plane(const Cloud &cloud,
                                   const core::StoreyRecord &storey,
                                   const core::SpacePolygon &region,
                                   double z_lo, double z_hi,
                                   const CeilingFitOptions &opt) {
  CeilingFitResult r;
  if (region.outer.size() < 3) {
    r.code = "bad_region";
    r.reason = "the region has fewer than 3 points";
    return r;
  }
  const auto &T = storey.level_transform;
  std::vector<Eigen::Vector3d> pts;
  for (const auto &p : cloud.points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
      continue;
    const double x = T[0] * p.x + T[1] * p.y + T[2] * p.z + T[3];
    const double y = T[4] * p.x + T[5] * p.y + T[6] * p.z + T[7];
    const double z = T[8] * p.x + T[9] * p.y + T[10] * p.z + T[11];
    if (z < z_lo || z > z_hi || !in_polygon(region, x, y))
      continue;
    pts.emplace_back(x, y, z);
  }
  r.points = static_cast<int>(pts.size());
  if (r.points < opt.min_points) {
    r.code = "too_few_points";
    r.reason = fmt::format("{} point(s) in the region between z {:.2f} and "
                           "{:.2f} m (need {})",
                           r.points, z_lo, z_hi, opt.min_points);
    return r;
  }

  // Seed (reusex-g72u): least squares over every band point settles between
  // surfaces (walls, furniture, a sloped roof); start the trimming from the
  // best plane of a deterministic RANSAC instead, the one most points lie
  // on within inlier_m (vertical residual, as the trimming measures it).
  std::vector<char> in(pts.size(), 1);
  if (opt.ransac_iterations > 0 && pts.size() >= 3) {
    std::mt19937 rng(opt.ransac_seed); // fixed seed: deterministic (§6)
    const auto n = static_cast<std::uint32_t>(pts.size());
    const double cos_max = std::cos(opt.max_slope_deg * std::numbers::pi / 180);
    int best = 0;
    Eigen::Vector3d best_p = Eigen::Vector3d::Zero(),
                    best_n = Eigen::Vector3d::UnitZ();
    for (int it = 0; it < opt.ransac_iterations; ++it) {
      const auto i = rng() % n, j = rng() % n, k = rng() % n;
      if (i == j || j == k || i == k)
        continue;
      Eigen::Vector3d nrm = (pts[j] - pts[i]).cross(pts[k] - pts[i]);
      const double len = nrm.norm();
      if (len < 1e-9)
        continue;
      nrm /= len;
      if (std::abs(nrm.z()) < cos_max)
        continue; // too steep (or vertical): never a ceiling
      int count = 0;
      for (const auto &q : pts)
        if (std::abs(nrm.dot(q - pts[i]) / nrm.z()) <= opt.inlier_m)
          ++count;
      if (count > best) {
        best = count;
        best_p = pts[i];
        best_n = nrm;
      }
    }
    if (best >= 3)
      for (size_t q = 0; q < pts.size(); ++q)
        in[q] =
            std::abs(best_n.dot(pts[q] - best_p) / best_n.z()) <= opt.inlier_m
                ? 1
                : 0;
  }
  Eigen::Vector3d coef = Eigen::Vector3d::Zero(); // a, b, c about (cx, cy)
  double cx = 0.0, cy = 0.0;
  for (int round = 0; round < std::max(1, opt.max_rounds); ++round) {
    int n = 0;
    cx = cy = 0.0;
    for (size_t i = 0; i < pts.size(); ++i)
      if (in[i]) {
        cx += pts[i].x();
        cy += pts[i].y();
        ++n;
      }
    if (n < opt.min_points) {
      r.code = "few_inliers";
      r.reason = fmt::format("only {} point(s) lie within {:.3f} m of a "
                             "plane (need {})",
                             n, opt.inlier_m, opt.min_points);
      return r;
    }
    cx /= n;
    cy /= n;
    Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
    Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < pts.size(); ++i) {
      if (!in[i])
        continue;
      const Eigen::Vector3d row(1.0, pts[i].x() - cx, pts[i].y() - cy);
      A += row * row.transpose();
      rhs += row * pts[i].z();
    }
    coef = A.colPivHouseholderQr().solve(rhs);
    bool changed = false;
    for (size_t i = 0; i < pts.size(); ++i) {
      const double res = pts[i].z() - (coef[0] + coef[1] * (pts[i].x() - cx) +
                                       coef[2] * (pts[i].y() - cy));
      const char keep = std::abs(res) <= opt.inlier_m ? 1 : 0;
      if (keep != in[i]) {
        in[i] = keep;
        changed = true;
      }
    }
    if (!changed)
      break;
  }

  double ss = 0.0;
  int n = 0;
  for (size_t i = 0; i < pts.size(); ++i)
    if (in[i]) {
      const double res = pts[i].z() - (coef[0] + coef[1] * (pts[i].x() - cx) +
                                       coef[2] * (pts[i].y() - cy));
      ss += res * res;
      ++n;
    }
  r.inliers = n;
  if (n < opt.min_points) {
    r.code = "few_inliers";
    r.reason = fmt::format("only {} point(s) lie within {:.3f} m of the fit "
                           "(need {})",
                           n, opt.inlier_m, opt.min_points);
    return r;
  }
  r.rms_m = std::sqrt(ss / n);
  r.slope_deg =
      std::atan(std::hypot(coef[1], coef[2])) * 180.0 / std::numbers::pi;
  r.plane.at = {cx, cy, coef[0]};
  r.plane.slope = {coef[1], coef[2]};
  if (r.slope_deg > opt.max_slope_deg) {
    r.code = "too_steep";
    r.reason = fmt::format("the fit slopes {:.1f} deg (max {:.1f}): not a "
                           "ceiling",
                           r.slope_deg, opt.max_slope_deg);
    return r;
  }
  if (std::isfinite(opt.z_floor)) {
    // The plane's highest point over the region (it is linear, so at a
    // vertex of the outer ring): a room has standing height somewhere,
    // while an attic's sloped ceiling may sit low over most of it (S2-27
    // Sygehusgrunden: 1.8 m over the floor at its middle, 3.1 m at the
    // ridge side; reusex-g72u).
    double highest = -std::numeric_limits<double>::infinity();
    for (const auto &v : region.outer)
      highest = std::max(highest, r.plane.z_at(v[0], v[1]));
    r.clearance_m = highest - opt.z_floor;
    if (r.clearance_m < opt.min_clearance_m) {
      r.code = "too_low";
      r.reason = fmt::format("the fit lies at most {:.2f} m over the floor "
                             "(min {:.2f} m): not this room's ceiling",
                             r.clearance_m, opt.min_clearance_m);
      return r;
    }
  }
  const double share = static_cast<double>(n) / r.points;
  if (share < opt.min_inlier_share) {
    r.code = "low_inlier_share";
    r.reason =
        fmt::format("only {} of {} point(s) ({:.0f} %) lie on the fit "
                    "(need {:.0f} %): the band holds more than one "
                    "surface",
                    n, r.points, 100.0 * share, 100.0 * opt.min_inlier_share);
    return r;
  }
  r.ok = true;
  return r;
}

CeilingBand
default_ceiling_fit_band(double z_floor, double z_ceiling,
                         const std::vector<core::StoreyRecord> &storeys,
                         const CeilingBandOptions &opt) {
  CeilingBand b;
  b.z_lo = z_floor + 0.5 * std::max(0.0, z_ceiling - z_floor);
  b.z_hi = std::max(z_ceiling, b.z_lo) + opt.above_ceiling_m;
  // The next storey: the lowest whose floor lies over this ceiling. One
  // whose floor is under it (a split level, a mis-measured storey) is no
  // reason to cut the band short.
  double next = std::numeric_limits<double>::infinity();
  for (const auto &s : storeys)
    if (s.z_floor > z_ceiling)
      next = std::min(next, s.z_floor);
  if (std::isfinite(next)) {
    const double cap = std::max(next - opt.below_next_floor_m,
                                std::max(z_ceiling, b.z_lo) + 0.05);
    if (cap < b.z_hi) {
      b.z_hi = cap;
      b.capped = true;
    }
  }
  return b;
}

// --- Room tools (reusex-pccw)
// -------------------------------------------------

namespace {

using Polygon2 = CGAL::Polygon_2<K>;
using PolygonH = CGAL::Polygon_with_holes_2<K>;
using PolySet = CGAL::Polygon_set_2<K>;

std::vector<WallGeom> storey_walls(const std::vector<core::WallRecord> &walls,
                                   int storey) {
  std::vector<WallGeom> out;
  for (const auto &w : walls)
    if (w.storey == storey && std::hypot(w.x1 - w.x0, w.y1 - w.y0) > 1e-6)
      out.push_back(geom_of(w));
  return out;
}

/// Drop vertices within @p eps of the line through their neighbours
/// (collinear runs, the hair-thin notches a union of rounded rings leaves).
Ring simplify(Ring r, double eps) {
  for (bool changed = true; changed && r.size() > 3;) {
    changed = false;
    for (size_t i = 0; i < r.size() && r.size() > 3; ++i) {
      const auto &p = r[(i + r.size() - 1) % r.size()];
      const auto &q = r[i];
      const auto &n = r[(i + 1) % r.size()];
      const double dx = n[0] - p[0], dy = n[1] - p[1];
      const double l = std::hypot(dx, dy);
      const double d =
          l < 1e-12 ? std::hypot(q[0] - p[0], q[1] - p[1])
                    : std::abs((q[0] - p[0]) * dy - (q[1] - p[1]) * dx) / l;
      if (d < eps) {
        r.erase(r.begin() + static_cast<std::ptrdiff_t>(i));
        changed = true;
        break;
      }
    }
  }
  return r;
}

core::SpacePolygon simplify(core::SpacePolygon p, double eps) {
  p.outer = canonical(simplify(std::move(p.outer), eps));
  for (auto &h : p.holes)
    h = canonical(simplify(std::move(h), eps));
  std::erase_if(p.holes, [](const Ring &h) {
    return h.size() < 3 || std::abs(ring_signed_area(h)) < 1e-9;
  });
  return p;
}

std::optional<Polygon2> cgal_ring(const Ring &r, bool ccw) {
  Polygon2 p;
  for (const auto &q : r)
    p.push_back(Point(q[0], q[1]));
  if (p.size() < 3 || !p.is_simple())
    return std::nullopt;
  if (p.is_counterclockwise_oriented() != ccw)
    p.reverse_orientation();
  return p;
}

std::optional<PolygonH> cgal_polygon(const core::SpacePolygon &poly) {
  auto outer = cgal_ring(poly.outer, true);
  if (!outer)
    return std::nullopt;
  PolygonH out(*outer);
  for (const auto &h : poly.holes) {
    auto hole = cgal_ring(h, false);
    if (!hole)
      return std::nullopt;
    out.add_hole(*hole);
  }
  return out;
}

Ring ring_from(const Polygon2 &p) {
  Ring r;
  for (auto v = p.vertices_begin(); v != p.vertices_end(); ++v)
    r.push_back({CGAL::to_double(v->x()), CGAL::to_double(v->y())});
  return r;
}

core::SpacePolygon polygon_from(const PolygonH &p) {
  core::SpacePolygon out;
  out.outer = ring_from(p.outer_boundary());
  if (ring_signed_area(out.outer) < 0.0)
    std::reverse(out.outer.begin(), out.outer.end());
  for (auto h = p.holes_begin(); h != p.holes_end(); ++h)
    out.holes.push_back(ring_from(*h));
  return out;
}

} // namespace

PlaceSpaceResult place_space_at(const std::vector<core::WallRecord> &walls,
                                const std::vector<core::SpaceRecord> &stored,
                                const core::StoreyRecord &storey,
                                std::string_view project_guid,
                                const core::PlanPoint &seed,
                                const RebuildSpacesOptions &opt) {
  PlaceSpaceResult r;
  const auto rebuilt =
      rebuild_spaces_from_walls(walls, stored, storey, project_guid, opt);
  for (const auto &s : rebuilt.spaces) {
    if (!in_polygon(s.polygon, seed[0], seed[1]))
      continue;
    r.ok = true;
    r.space = s;
    r.existing = std::any_of(stored.begin(), stored.end(), [&](const auto &o) {
      return o.storey == storey.idx && o.guid == s.guid;
    });
    for (const auto &b : rebuilt.boundaries)
      if (b.space == s.guid)
        r.boundaries.push_back(b);
    return r;
  }
  for (const auto &g : storey_walls(walls, storey.idx))
    if (distance_to_segment({seed[0], seed[1]}, g.p0, g.p1) <=
        std::max(g.half, 1e-3)) {
      r.reason = fmt::format("({:.2f}, {:.2f}) lies in wall {}; click inside "
                             "a room",
                             seed[0], seed[1], g.w->guid);
      return r;
    }
  r.reason = fmt::format(
      "no closed area of storey {}'s walls holds ({:.2f}, {:.2f}) ({} "
      "wall(s), {} room(s) found); close the walls or draw the room as a "
      "polygon",
      storey.idx, seed[0], seed[1], rebuilt.walls, rebuilt.spaces.size());
  return r;
}

std::vector<core::ModelEditOp> place_space_ops(const PlaceSpaceResult &r) {
  std::vector<core::ModelEditOp> ops;
  if (r.ok)
    ops.push_back(core::ModelEditOp::put(r.space, r.boundaries));
  return ops;
}

MergeSpacesResult merge_spaces(const core::SpaceRecord &a,
                               const core::SpaceRecord &b,
                               const std::vector<core::WallRecord> &walls_in,
                               const RoomToolOptions &opt) {
  MergeSpacesResult r;
  if (a.guid == b.guid) {
    r.reason = "merge: a space cannot be merged with itself";
    return r;
  }
  if (a.storey != b.storey) {
    r.reason = fmt::format("merge: spaces {} and {} are on storeys {} and {}",
                           a.guid, b.guid, a.storey, b.storey);
    return r;
  }
  const auto pa = cgal_polygon(a.polygon);
  const auto pb = cgal_polygon(b.polygon);
  if (!pa || !pb) {
    r.reason = fmt::format("merge: the polygon of space {} is not simple",
                           pa ? b.guid : a.guid);
    return r;
  }
  const auto walls = storey_walls(walls_in, a.storey);
  const auto ba = find_boundaries(a, walls, opt.boundary_tolerance_m);
  const auto bb = find_boundaries(b, walls, opt.boundary_tolerance_m);
  PolySet set(*pa);
  set.join(*pb);
  constexpr double kOverlap = 1e-4; // into both rooms, past rounding
  for (const auto &g : walls) {
    double lo = 0.0, hi = -1.0;
    for (const auto &x : ba)
      for (const auto &y : bb)
        if (x.wall == g.w->guid && y.wall == g.w->guid && x.side != y.side) {
          const double l = std::max(x.s_start, y.s_start);
          const double h = std::min(x.s_end, y.s_end);
          if (h - l > hi - lo) {
            lo = l;
            hi = h;
          }
        }
    if (hi - lo < opt.min_shared_m)
      continue;
    const double off = g.half + kOverlap;
    Ring rect = {at(g, lo, -off), at(g, hi, -off), at(g, hi, off),
                 at(g, lo, off)};
    if (const auto p = cgal_ring(rect, true)) {
      set.join(*p);
      r.bridged_walls.push_back(g.w->guid);
    }
  }
  std::vector<PolygonH> out;
  set.polygons_with_holes(std::back_inserter(out));
  if (out.size() != 1) {
    r.reason = fmt::format(
        "merge: spaces {} and {} neither touch nor share a wall ({} piece(s) "
        "after the union)",
        a.guid, b.guid, out.size());
    r.bridged_walls.clear();
    return r;
  }
  r.ok = true;
  r.space = a;
  r.space.id = -1;
  r.space.polygon = simplify(polygon_from(out.front()), opt.simplify_m);
  fill_metrics(r.space);
  r.boundaries = find_boundaries(r.space, walls, opt.boundary_tolerance_m);
  r.removed = b.guid;
  std::sort(r.bridged_walls.begin(), r.bridged_walls.end());
  return r;
}

std::vector<core::ModelEditOp> merge_spaces_ops(const MergeSpacesResult &r) {
  std::vector<core::ModelEditOp> ops;
  if (!r.ok)
    return ops;
  ops.push_back(core::ModelEditOp::put(r.space, r.boundaries));
  ops.push_back(
      core::ModelEditOp::remove(core::ModelElementKind::space, r.removed));
  return ops;
}

SplitSpaceResult split_space(const core::SpaceRecord &space,
                             const core::PlanPoint &p0,
                             const core::PlanPoint &p1,
                             const std::vector<core::WallRecord> &walls_in,
                             std::string_view project_guid,
                             const std::vector<std::string> &taken_in,
                             const RoomToolOptions &opt) {
  (void)core::deterministic_guid(project_guid, "spaces"); // validates
  SplitSpaceResult r;
  const double len = std::hypot(p1[0] - p0[0], p1[1] - p0[1]);
  if (!(len > 1e-6)) {
    r.reason = "split: the line has no length";
    return r;
  }
  if (space.polygon.outer.size() < 3) {
    r.reason = "split: space " + space.guid + " has no polygon";
    return r;
  }
  const V d{(p1[0] - p0[0]) / len, (p1[1] - p0[1]) / len};
  const double ext = opt.split_extension_m;
  std::vector<Segment> segs;
  const auto seg = [&](V a, V b) {
    if (std::hypot(b[0] - a[0], b[1] - a[1]) > 1e-9)
      segs.emplace_back(Point(a[0], a[1]), Point(b[0], b[1]));
  };
  const auto ring_segs = [&](const Ring &ring) {
    for (size_t i = 0; i < ring.size(); ++i)
      seg({ring[i][0], ring[i][1]},
          {ring[(i + 1) % ring.size()][0], ring[(i + 1) % ring.size()][1]});
  };
  ring_segs(space.polygon.outer);
  for (const auto &h : space.polygon.holes)
    ring_segs(h);
  seg({p0[0] - ext * d[0], p0[1] - ext * d[1]},
      {p1[0] + ext * d[0], p1[1] + ext * d[1]});
  Arr arr;
  CGAL::insert(arr, segs.begin(), segs.end());
  for (bool again = true; again;) {
    again = false;
    for (auto e = arr.edges_begin(); e != arr.edges_end(); ++e)
      if (e->face() == e->twin()->face()) {
        arr.remove_edge(e);
        again = true;
        break;
      }
  }
  std::vector<core::SpacePolygon> pieces;
  for (auto f = arr.faces_begin(); f != arr.faces_end(); ++f) {
    if (f->is_unbounded() || !f->has_outer_ccb())
      continue;
    core::SpacePolygon poly;
    poly.outer = ring_of(f->outer_ccb());
    if (ring_signed_area(poly.outer) < 0.0)
      std::reverse(poly.outer.begin(), poly.outer.end());
    for (auto h = f->inner_ccbs_begin(); h != f->inner_ccbs_end(); ++h) {
      Ring hole = ring_of(*h);
      if (hole.size() >= 3 && std::abs(ring_signed_area(hole)) > 1e-9)
        poly.holes.push_back(std::move(hole));
    }
    poly = simplify(std::move(poly), opt.simplify_m);
    if (poly.outer.size() < 3)
      continue;
    const auto ip = interior_point(poly);
    if (!ip || !in_polygon(space.polygon, (*ip)[0], (*ip)[1]))
      continue; // a hole of the space
    pieces.push_back(std::move(poly));
  }
  if (pieces.size() < 2) {
    r.reason = fmt::format(
        "split: the line ({:.2f}, {:.2f}) - ({:.2f}, {:.2f}) does not cut "
        "space {} in two; draw it across the room, edge to edge",
        p0[0], p0[1], p1[0], p1[1], space.guid);
    return r;
  }
  std::stable_sort(
      pieces.begin(), pieces.end(), [](const auto &x, const auto &y) {
        return core::space_polygon_area(x) > core::space_polygon_area(y);
      });
  std::set<std::string> taken(taken_in.begin(), taken_in.end());
  taken.insert(space.guid);
  const auto walls = storey_walls(walls_in, space.storey);
  for (size_t k = 0; k < pieces.size(); ++k) {
    core::SpaceRecord s = space;
    s.id = -1;
    s.polygon = pieces[k];
    if (k > 0) {
      const std::string key = fmt::format("spaces/split/{}/{}", space.guid, k);
      std::string g = core::deterministic_guid(project_guid, key);
      for (int n = 2; taken.count(g); ++n)
        g = core::deterministic_guid(project_guid,
                                     fmt::format("{}#{}", key, n));
      taken.insert(g);
      s.guid = g;
      if (!s.name.empty())
        s.name = fmt::format("{} ({})", space.name, k + 1);
      s.room_label = -1;
      s.metadata.clear();
    }
    fill_metrics(s);
    auto bs = find_boundaries(s, walls, opt.boundary_tolerance_m);
    r.boundaries.insert(r.boundaries.end(), bs.begin(), bs.end());
    r.spaces.push_back(std::move(s));
  }
  r.ok = true;
  return r;
}

std::vector<core::ModelEditOp> split_space_ops(const SplitSpaceResult &r) {
  std::vector<core::ModelEditOp> ops;
  if (!r.ok)
    return ops;
  for (const auto &s : r.spaces) {
    std::vector<core::SpaceBoundaryRecord> bs;
    for (const auto &b : r.boundaries)
      if (b.space == s.guid)
        bs.push_back(b);
    ops.push_back(core::ModelEditOp::put(s, std::move(bs)));
  }
  return ops;
}

} // namespace reusex::segmentation
