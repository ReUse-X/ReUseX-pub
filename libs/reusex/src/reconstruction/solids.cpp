// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/reconstruction/solids.hpp"

#include "reusex/core/guid.hpp"
#include "reusex/core/logging.hpp"
#include "reusex/core/sloped_ceiling.hpp"
#include "reusex/geometry/transform_utils.hpp"

#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_mesh_processing/clip.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Polygon_mesh_processing/self_intersections.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Polygon_set_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/boost/graph/helpers.h>
#include <CGAL/convex_hull_2.h>

#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>

#include <fmt/format.h>
#include <fmt/ranges.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <list>
#include <map>
#include <optional>
#include <queue>
#include <unordered_map>
#include <utility>

namespace reusex::geometry {
namespace {

using K = CGAL::Exact_predicates_inexact_constructions_kernel;
using Mesh = CGAL::Surface_mesh<K::Point_3>;
using EK = CGAL::Exact_predicates_exact_constructions_kernel;
using EPolygon = CGAL::Polygon_2<EK>;
using EPolygonWithHoles = CGAL::Polygon_with_holes_2<EK>;
using EPolygonSet = CGAL::Polygon_set_2<EK>;
namespace PMP = CGAL::Polygon_mesh_processing;

using Vec2 = Eigen::Vector2d;
using Ring = std::vector<Vec2>;

// ── Constrained triangulation of a polygon with holes ───────────────────────

struct FaceInfo {
  int nesting = -1;
  bool in_domain() const { return nesting % 2 == 1; }
};
using Vb = CGAL::Triangulation_vertex_base_2<K>;
using Fbi = CGAL::Triangulation_face_base_with_info_2<FaceInfo, K>;
using Fb = CGAL::Constrained_triangulation_face_base_2<K, Fbi>;
using Tds = CGAL::Triangulation_data_structure_2<Vb, Fb>;
using Cdt =
    CGAL::Constrained_Delaunay_triangulation_2<K, Tds,
                                               CGAL::Exact_predicates_tag>;

/// Flood the faces from @p start with nesting level @p index, stopping at
/// constrained edges, which open the next level (the CGAL polygon example).
void mark_domains(Cdt &cdt, Cdt::Face_handle start, int index,
                  std::list<Cdt::Edge> &border) {
  if (start->info().nesting != -1)
    return;
  std::list<Cdt::Face_handle> queue{start};
  while (!queue.empty()) {
    auto fh = queue.front();
    queue.pop_front();
    if (fh->info().nesting != -1)
      continue;
    fh->info().nesting = index;
    for (int i = 0; i < 3; ++i) {
      const Cdt::Edge e(fh, i);
      auto n = fh->neighbor(i);
      if (n->info().nesting == -1) {
        if (cdt.is_constrained(e))
          border.push_back(e);
        else
          queue.push_back(n);
      }
    }
  }
}

void mark_domains(Cdt &cdt) {
  for (auto f = cdt.all_faces_begin(); f != cdt.all_faces_end(); ++f)
    f->info().nesting = -1;
  std::list<Cdt::Edge> border;
  mark_domains(cdt, cdt.infinite_face(), 0, border);
  while (!border.empty()) {
    const Cdt::Edge e = border.front();
    border.pop_front();
    auto n = e.first->neighbor(e.second);
    if (n->info().nesting == -1)
      mark_domains(cdt, n, e.first->info().nesting + 1, border);
  }
}

double signed_area(const Ring &r) {
  double a = 0.0;
  for (std::size_t i = 0; i < r.size(); ++i) {
    const Vec2 &p = r[i], &q = r[(i + 1) % r.size()];
    a += p.x() * q.y() - q.x() * p.y();
  }
  return 0.5 * a;
}

/// Drop repeated points (and a closing repeat); orient CCW (outer) or CW.
Ring clean_ring(const Ring &in, bool ccw) {
  Ring r;
  for (const auto &p : in) {
    if (!p.allFinite())
      continue;
    if (!r.empty() && (p - r.back()).norm() < 1e-9)
      continue;
    r.push_back(p);
  }
  while (r.size() > 1 && (r.front() - r.back()).norm() < 1e-9)
    r.pop_back();
  if (r.size() >= 3 && ((signed_area(r) > 0.0) != ccw))
    std::reverse(r.begin(), r.end());
  return r;
}

/// A closed prism: @p outer (with @p holes) from z0 up to z1, outward faces.
/// std::nullopt for a degenerate polygon or z1 <= z0.
std::optional<Mesh> extrude(const Ring &outer_in,
                            const std::vector<Ring> &holes_in, double z0,
                            double z1) {
  if (!(z1 > z0) || !std::isfinite(z0) || !std::isfinite(z1))
    return std::nullopt;
  const Ring outer = clean_ring(outer_in, true);
  if (outer.size() < 3 || std::abs(signed_area(outer)) < 1e-9)
    return std::nullopt;
  std::vector<Ring> holes;
  for (const auto &h : holes_in) {
    Ring c = clean_ring(h, false);
    if (c.size() >= 3 && std::abs(signed_area(c)) >= 1e-9)
      holes.push_back(std::move(c));
  }

  Cdt cdt;
  std::vector<std::vector<Cdt::Vertex_handle>> rings;
  const auto insert_ring = [&](const Ring &r) {
    std::vector<Cdt::Vertex_handle> vh;
    for (const auto &p : r)
      vh.push_back(cdt.insert(K::Point_2(p.x(), p.y())));
    for (std::size_t i = 0; i < vh.size(); ++i)
      if (vh[i] != vh[(i + 1) % vh.size()])
        cdt.insert_constraint(vh[i], vh[(i + 1) % vh.size()]);
    rings.push_back(std::move(vh));
  };
  insert_ring(outer);
  for (const auto &h : holes)
    insert_ring(h);
  mark_domains(cdt);

  Mesh m;
  std::map<Cdt::Vertex_handle,
           std::pair<Mesh::Vertex_index, Mesh::Vertex_index>>
      vmap;
  const auto vert = [&](Cdt::Vertex_handle v) {
    auto it = vmap.find(v);
    if (it != vmap.end())
      return it->second;
    const auto &p = v->point();
    auto b = m.add_vertex(K::Point_3(p.x(), p.y(), z0));
    auto t = m.add_vertex(K::Point_3(p.x(), p.y(), z1));
    return vmap.emplace(v, std::make_pair(b, t)).first->second;
  };
  bool ok = true;
  const auto face = [&](Mesh::Vertex_index a, Mesh::Vertex_index b,
                        Mesh::Vertex_index c) {
    if (m.add_face(a, b, c) == Mesh::null_face())
      ok = false;
  };
  for (auto f = cdt.finite_faces_begin(); f != cdt.finite_faces_end(); ++f) {
    if (!f->info().in_domain())
      continue;
    const auto a = vert(f->vertex(0)), b = vert(f->vertex(1)),
               c = vert(f->vertex(2));
    face(a.second, b.second, c.second); // top, +z
    face(a.first, c.first, b.first);    // bottom, -z
  }
  // Sides: an outer ring runs CCW and a hole CW, so (b - a) x up points out of
  // the solid either way.
  for (const auto &vh : rings)
    for (std::size_t i = 0; i < vh.size(); ++i) {
      const auto va = vh[i], vb = vh[(i + 1) % vh.size()];
      if (va == vb)
        continue;
      const auto a = vert(va), b = vert(vb);
      face(a.first, b.first, b.second);
      face(a.first, b.second, a.second);
    }
  if (!ok)
    debug("solids: extrusion: a face could not be added (non-manifold "
          "polygon); the solid is not closed");
  return m;
}

// ── Frames ──────────────────────────────────────────────────────────────────

/// Level -> world of each storey (the inverse of its level_transform).
struct Frames {
  std::map<int, Eigen::Affine3d> to_world;
  const Eigen::Affine3d &of(int storey) const {
    static const Eigen::Affine3d identity = Eigen::Affine3d::Identity();
    auto it = to_world.find(storey);
    return it == to_world.end() ? identity : it->second;
  }
};

Frames make_frames(const core::SolidModel &model) {
  Frames f;
  for (const auto &s : model.storeys) {
    const Eigen::Affine3d T = to_affine(s.level_transform);
    if (!T.matrix().allFinite() || std::abs(T.linear().determinant()) < 1e-12) {
      warn("solids: storey {} has a singular level_transform; its elements "
           "are placed as if in world coordinates",
           s.idx);
      continue;
    }
    f.to_world[s.idx] = T.inverse();
  }
  return f;
}

std::array<double, 16> row_major(const Eigen::Affine3d &a) {
  std::array<double, 16> out{};
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      out[static_cast<std::size_t>(4 * r + c)] = a.matrix()(r, c);
  return out;
}

/// A local frame in level coordinates: origin @p o (x, y, z), x along @p dx
/// (in plan), z up.
Eigen::Affine3d local_frame(const Vec2 &o, double z, const Vec2 &dx) {
  Eigen::Affine3d l = Eigen::Affine3d::Identity();
  l.linear().col(0) = Eigen::Vector3d(dx.x(), dx.y(), 0.0);
  l.linear().col(1) = Eigen::Vector3d(-dx.y(), dx.x(), 0.0);
  l.linear().col(2) = Eigen::Vector3d::UnitZ();
  l.translation() = Eigen::Vector3d(o.x(), o.y(), z);
  return l;
}

std::vector<core::PlanPoint> to_plan(const Ring &r) {
  std::vector<core::PlanPoint> out;
  out.reserve(r.size());
  for (const auto &p : r)
    out.push_back({p.x(), p.y()});
  return out;
}

Ring to_ring(const std::vector<core::PlanPoint> &r) {
  Ring out;
  out.reserve(r.size());
  for (const auto &p : r)
    out.emplace_back(p[0], p[1]);
  return out;
}

/// Close, volume, world-space PCL triangles of @p m (level frame @p T).
void finish(ElementSolid &e, Mesh &m, const Eigen::Affine3d &T) {
  if (!CGAL::is_triangle_mesh(m))
    PMP::triangulate_faces(m);
  m.collect_garbage();
  e.closed = CGAL::is_closed(m) && m.number_of_faces() > 0;
  e.volume = e.closed ? PMP::volume(m) : 0.0;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.reserve(m.number_of_vertices());
  for (const auto v : m.vertices()) {
    const auto &p = m.point(v);
    const Eigen::Vector3d w = T * Eigen::Vector3d(p.x(), p.y(), p.z());
    cloud.push_back(pcl::PointXYZ(static_cast<float>(w.x()),
                                  static_cast<float>(w.y()),
                                  static_cast<float>(w.z())));
  }
  pcl::toPCLPointCloud2(cloud, e.mesh.cloud);
  e.mesh.polygons.clear();
  e.mesh.polygons.reserve(m.number_of_faces());
  for (const auto f : m.faces()) {
    pcl::Vertices tri;
    for (const auto v : CGAL::vertices_around_face(m.halfedge(f), m))
      tri.vertices.push_back(static_cast<std::uint32_t>(v.idx()));
    e.mesh.polygons.push_back(std::move(tri));
  }
}

// ── Sloped ceilings (reusex-546) ───────────────────────────────────────────

/// The level-frame plane z = p(x, y) + dz as a CGAL plane whose negative side
/// is below it: -sx x - sy y + z + (sx x0 + sy y0 - z0 - dz) = 0.
K::Plane_3 below_plane(const core::CeilingPlane &p, double dz) {
  return {-p.slope[0], -p.slope[1], 1.0,
          p.slope[0] * p.at[0] + p.slope[1] * p.at[1] - p.at[2] - dz};
}

// The sloped-ceiling booleans run on exact constructions: with inexact ones
// the clip / corefinement of a long thin roof piece meets "unauthorized
// intersections of constraints" on real scans (Sygehusgrunden).
using EMesh = CGAL::Surface_mesh<EK::Point_3>;

EMesh to_exact(const Mesh &m) {
  EMesh out;
  std::vector<EMesh::Vertex_index> v(m.num_vertices());
  for (const auto vi : m.vertices()) {
    const auto &p = m.point(vi);
    v[vi.idx()] = out.add_vertex(EK::Point_3(p.x(), p.y(), p.z()));
  }
  for (const auto f : m.faces()) {
    std::vector<EMesh::Vertex_index> ids;
    for (const auto vi : CGAL::vertices_around_face(m.halfedge(f), m))
      ids.push_back(v[vi.idx()]);
    out.add_face(ids);
  }
  return out;
}

Mesh to_inexact(const EMesh &m) {
  Mesh out;
  std::vector<Mesh::Vertex_index> v(m.num_vertices());
  for (const auto vi : m.vertices()) {
    const auto &p = m.point(vi);
    v[vi.idx()] = out.add_vertex(K::Point_3(CGAL::to_double(p.x()),
                                            CGAL::to_double(p.y()),
                                            CGAL::to_double(p.z())));
  }
  for (const auto f : m.faces()) {
    std::vector<Mesh::Vertex_index> ids;
    for (const auto vi : CGAL::vertices_around_face(m.halfedge(f), m))
      ids.push_back(v[vi.idx()]);
    out.add_face(ids);
  }
  return out;
}

/// Keep the part of the closed mesh @p m below every plane (raised by
/// @p dz), exactly. False when a clip failed or left nothing.
bool clip_below(EMesh &m, const std::vector<core::CeilingPlane> &planes,
                double dz) {
  for (const auto &p : planes) {
    const EK::FT sx(p.slope[0]), sy(p.slope[1]);
    const EK::Plane_3 plane(-sx, -sy, EK::FT(1),
                            sx * EK::FT(p.at[0]) + sy * EK::FT(p.at[1]) -
                                EK::FT(p.at[2]) - EK::FT(dz));
    try {
      PMP::clip(m, plane, CGAL::parameters::clip_volume(true));
    } catch (const std::exception &e) {
      debug("solids: clipping by a ceiling plane threw: {}", e.what());
      return false;
    }
    if (m.number_of_faces() == 0)
      return false;
  }
  return true;
}

/// clip_below() on an inexact mesh, through the exact kernel.
bool clip_below(Mesh &m, const std::vector<core::CeilingPlane> &planes,
                double dz) {
  EMesh e = to_exact(m);
  if (!clip_below(e, planes, dz) || !CGAL::is_closed(e))
    return false;
  m = to_inexact(e);
  return true;
}

/// @p p (level frame) as a world plane {a, b, c, d}, a x + b y + c z + d = 0,
/// the kept side negative.
std::array<double, 4> world_plane(const K::Plane_3 &p,
                                  const Eigen::Affine3d &T) {
  const Eigen::Vector3d n_l(p.a(), p.b(), p.c());
  const Eigen::Vector3d n_w = T.linear() * n_l;
  // T is rigid: n_l . x_l + d = n_w . (x_w - t) + d.
  return {n_w.x(), n_w.y(), n_w.z(), p.d() - n_w.dot(T.translation())};
}

bool inside_ring(const Ring &r, const Vec2 &p) {
  bool in = false;
  for (std::size_t i = 0, j = r.size() - 1; i < r.size(); j = i++) {
    const Vec2 &a = r[i], &b = r[j];
    if ((a.y() > p.y()) != (b.y() > p.y()) &&
        p.x() < (b.x() - a.x()) * (p.y() - a.y()) / (b.y() - a.y()) + a.x())
      in = !in;
  }
  return in;
}

bool inside_space(const core::SpaceRecord &s, const Vec2 &p) {
  if (s.polygon.outer.size() < 3 || !inside_ring(to_ring(s.polygon.outer), p))
    return false;
  for (const auto &h : s.polygon.holes)
    if (h.size() >= 3 && inside_ring(to_ring(h), p))
      return false;
  return true;
}

// ── Walls ───────────────────────────────────────────────────────────────────

/// One wall's box in its frame: s0..s1 along the axis from (x0, y0), lo..hi
/// across it (left of the axis positive), z0..z1.
struct WallBox {
  const core::WallRecord *wall = nullptr;
  Vec2 p0, d;
  double len = 0.0, s0 = 0.0, s1 = 0.0, lo = 0.0, hi = 0.0, z0 = 0.0, z1 = 0.0;
  bool valid = false;
  /// Sloped top (s, z), capped at z1 (reusex-546); empty = flat at z1.
  std::vector<Vec2> top;
  /// The sloped space whose ceiling the derived top follows (index into
  /// model.spaces), -1 for none: the wall's roof goes with that space's.
  int slope_space = -1;

  /// The top at @p s: the profile interpolated (constant past its ends),
  /// capped at z1.
  [[nodiscard]] double top_at(double s) const {
    if (top.empty())
      return z1;
    double z = top.front().y();
    if (s >= top.back().x()) {
      z = top.back().y();
    } else if (s > top.front().x()) {
      std::size_t i = 1;
      while (i + 1 < top.size() && top[i].x() < s)
        ++i;
      const Vec2 &a = top[i - 1], &b = top[i];
      const double w = b.x() > a.x() ? (s - a.x()) / (b.x() - a.x()) : 0.0;
      z = a.y() + w * (b.y() - a.y());
    }
    return std::min(z1, z);
  }

  Vec2 at(double s, double t) const {
    return p0 + d * s + Vec2(-d.y(), d.x()) * t;
  }
  /// The footprint, counter-clockwise.
  Ring footprint() const {
    return {at(s0, lo), at(s1, lo), at(s1, hi), at(s0, hi)};
  }
};

std::vector<WallBox> wall_boxes(const core::SolidModel &model,
                                const SolidsOptions &opts,
                                std::size_t &skipped) {
  std::vector<WallBox> boxes(model.walls.size());
  std::unordered_map<std::string, std::size_t> by_guid;
  for (std::size_t i = 0; i < model.walls.size(); ++i) {
    const auto &w = model.walls[i];
    WallBox &b = boxes[i];
    b.wall = &w;
    b.p0 = Vec2(w.x0, w.y0);
    const Vec2 p1(w.x1, w.y1);
    b.len = (p1 - b.p0).norm();
    if (!(b.len > 1e-6) || !std::isfinite(b.len) || !(w.height > 0.0) ||
        !std::isfinite(w.z_base) || !std::isfinite(w.height)) {
      ++skipped;
      continue;
    }
    b.d = (p1 - b.p0) / b.len;
    b.s0 = 0.0;
    b.s1 = b.len;
    b.z0 = w.z_base;
    b.z1 = w.z_base + w.height;
    for (const auto &q : w.top)
      b.top.emplace_back(q[0], q[1]);
    if (w.thickness > 0.0) {
      b.lo = -0.5 * w.thickness;
      b.hi = 0.5 * w.thickness;
    } else {
      // The axis is the seen face: grow away from the space it faces.
      const double t = opts.unknown_wall_thickness_m;
      const Vec2 mid = b.p0 + b.d * (0.5 * b.len);
      const Vec2 n(-b.d.y(), b.d.x());
      const double probe = std::max(0.15, t);
      bool left = false, right = false;
      for (const auto &s : model.spaces) {
        if (s.storey != w.storey)
          continue;
        left = left || inside_space(s, mid + n * probe);
        right = right || inside_space(s, mid - n * probe);
      }
      if (left && !right) {
        b.lo = -t;
        b.hi = 0.0;
      } else if (right && !left) {
        b.lo = 0.0;
        b.hi = t;
      } else {
        b.lo = -0.5 * t;
        b.hi = 0.5 * t;
      }
    }
    b.valid = true;
    by_guid.emplace(w.guid, i);
  }
  if (opts.extend_at_joins) {
    // Reach the far face of the wall met at a corner or a tee.
    const auto reach = [&](const WallBox &other) {
      return std::max(std::abs(other.lo), std::abs(other.hi));
    };
    for (const auto &j : model.wall_joins) {
      if (j.kind != "corner" && j.kind != "tee")
        continue;
      const auto a = by_guid.find(j.wall_a), b = by_guid.find(j.wall_b);
      if (a == by_guid.end() || b == by_guid.end())
        continue;
      WallBox &A = boxes[a->second], &B = boxes[b->second];
      const auto extend = [&](WallBox &w, const std::string &at, double r) {
        if (at == "start")
          w.s0 = std::min(w.s0, -r);
        else if (at == "end")
          w.s1 = std::max(w.s1, w.len + r);
      };
      extend(A, j.at_a, reach(B));
      extend(B, j.at_b, reach(A));
    }
  }
  return boxes;
}

/// The s values where a wall's top bends: its ends, the profile's vertices
/// between them, and where the profile crosses the cap z1.
std::vector<double> top_breaks(const WallBox &b) {
  std::vector<double> ss{b.s0, b.s1};
  for (std::size_t i = 0; i < b.top.size(); ++i) {
    const Vec2 &q = b.top[i];
    if (q.x() > b.s0 && q.x() < b.s1)
      ss.push_back(q.x());
    if (i == 0)
      continue;
    const Vec2 &a = b.top[i - 1];
    if ((a.y() - b.z1) * (q.y() - b.z1) < 0.0) {
      const double s =
          a.x() + (b.z1 - a.y()) / (q.y() - a.y()) * (q.x() - a.x());
      if (s > b.s0 && s < b.s1)
        ss.push_back(s);
    }
  }
  std::sort(ss.begin(), ss.end());
  ss.erase(std::unique(ss.begin(), ss.end(),
                       [](double x, double y) { return y - x < 1e-9; }),
           ss.end());
  return ss;
}

/// Does the top dip below z1 anywhere along the wall (by more than 1 mm)?
bool has_sloped_top(const WallBox &b) {
  if (b.top.empty())
    return false;
  for (const double s : top_breaks(b))
    if (b.top_at(s) < b.z1 - 1e-3)
      return true;
  return false;
}

/// The wall's elevation in (s, z), counter-clockwise: the bottom at z0 from
/// s0 to s1, then the top back from s1 to s0 (never lower than 1 cm above
/// the bottom).
Ring wall_elevation(const WallBox &b) {
  const auto ss = top_breaks(b);
  Ring r{{b.s0, b.z0}, {b.s1, b.z0}};
  for (auto it = ss.rbegin(); it != ss.rend(); ++it)
    r.emplace_back(*it, std::max(b.z0 + 0.01, b.top_at(*it)));
  return r;
}

/// The wall's closed solid before openings: its box, or with a sloped top its
/// elevation extruded across the thickness (local (s, z, w) mapped back to
/// the level frame, w along minus the left normal so the frame stays
/// right-handed and the faces outward).
std::optional<Mesh> wall_mesh(const WallBox &b) {
  if (!has_sloped_top(b))
    return extrude(b.footprint(), {}, b.z0, b.z1);
  auto m = extrude(wall_elevation(b), {}, -b.hi, -b.lo);
  if (!m)
    return m;
  const Vec2 n(-b.d.y(), b.d.x());
  for (const auto v : m->vertices()) {
    const auto p = m->point(v);
    const Vec2 xy = b.p0 + b.d * p.x() - n * p.z();
    m->point(v) = K::Point_3(xy.x(), xy.y(), p.y());
  }
  return m;
}

ElementSolid build_wall(const WallBox &b,
                        const std::vector<const core::OpeningRecord *> &ops,
                        const Frames &frames, const SolidsOptions &opts,
                        SolidsResult &result) {
  const core::WallRecord &w = *b.wall;
  const Eigen::Affine3d &T = frames.of(w.storey);
  ElementSolid e;
  e.type = SolidType::wall;
  e.source_guid = w.guid;
  e.storey = w.storey;
  e.predefined = "STANDARD";
  e.profile.outer = {{b.s0, b.lo}, {b.s1, b.lo}, {b.s1, b.hi}, {b.s0, b.hi}};
  e.profile.depth = b.z1 - b.z0;
  e.profile.placement = row_major(T * local_frame(b.p0, b.z0, b.d));

  if (has_sloped_top(b))
    for (const auto &q : wall_elevation(b))
      if (q.y() > b.z0 + 1e-12)
        e.top.push_back({q.x(), q.y() - b.z0});
  std::reverse(e.top.begin(), e.top.end());
  auto box = wall_mesh(b);
  Mesh m = box ? std::move(*box) : Mesh{};
  const double eps = 1e-3, over = opts.opening_overcut_m;
  for (const auto *o : ops) {
    double a = std::max(o->s_start, b.s0), c = std::min(o->s_end, b.s1);
    double zs = o->z_sill, zh = o->z_head;
    if (c - a < eps || zh <= b.z0 + eps || zs >= b.z1 - eps ||
        !std::isfinite(a + c + zs + zh)) {
      ++result.openings_skipped;
      debug("solids: opening {} lies outside wall {} (s {:.2f}..{:.2f} of "
            "{:.2f}..{:.2f}, z {:.2f}..{:.2f} of {:.2f}..{:.2f}); not cut",
            o->guid, w.guid, o->s_start, o->s_end, b.s0, b.s1, zs, zh, b.z0,
            b.z1);
      continue;
    }
    // Run the cut through every face it touches, so no coplanar sliver
    // remains: across the wall always, past the ends, floor and top when the
    // opening reaches them.
    if (a <= b.s0 + eps)
      a = b.s0 - over;
    if (c >= b.s1 - eps)
      c = b.s1 + over;
    if (zs <= b.z0 + eps)
      zs = b.z0 - over;
    if (zh >= std::min(b.top_at(a), b.top_at(c)) - eps)
      zh = b.z1 + over;
    const double lo = b.lo - over, hi = b.hi + over;
    OpeningVoid v;
    v.guid = o->guid;
    v.kind = o->kind;
    v.profile.outer = {
        {0.0, 0.0}, {c - a, 0.0}, {c - a, hi - lo}, {0.0, hi - lo}};
    v.profile.depth = zh - zs;
    v.profile.placement = row_major(T * local_frame(b.at(a, lo), zs, b.d));
    auto cut = extrude({b.at(a, lo), b.at(c, lo), b.at(c, hi), b.at(a, hi)}, {},
                       zs, zh);
    bool done = false;
    if (cut && box) {
      Mesh keep = m;
      try {
        Mesh out;
        done = PMP::corefine_and_compute_difference(m, *cut, out) &&
               CGAL::is_closed(out);
        if (done)
          m = std::move(out);
        else
          m = std::move(keep);
      } catch (const std::exception &ex) {
        debug("solids: corefinement threw for opening {} in wall {}: {}",
              o->guid, w.guid, ex.what());
        m = std::move(keep);
        done = false;
      }
    }
    if (done) {
      ++result.openings_cut;
      e.voids.push_back(std::move(v));
    } else {
      ++result.openings_failed;
      ++e.voids_failed;
      warn("solids: could not subtract opening {} ({}) from wall {}; the wall "
           "is left uncut there",
           o->guid, o->kind, w.guid);
    }
  }
  finish(e, m, T);
  return e;
}

/// Where the ceilings of @p spaces bend along @p b's axis (two of a space's
/// planes, or a plane and its cap, meet), plus every 0.25 m and the ends:
/// the s values a top that follows them is sampled at.
std::vector<double>
ceiling_breaks(const WallBox &b,
               const std::vector<const core::SpaceRecord *> &spaces) {
  std::vector<double> ss{b.s0, b.s1};
  for (double v = std::ceil(b.s0 / 0.25) * 0.25; v < b.s1; v += 0.25)
    ss.push_back(v);
  const auto along = [&](const core::SpaceRecord &s, std::size_t k, double t) {
    const Vec2 p = b.at(t, 0.0);
    return k < s.ceiling_planes.size() ? s.ceiling_planes[k].z_at(p.x(), p.y())
                                       : s.z_ceiling;
  };
  for (const auto *sp : spaces) {
    const auto &s = *sp;
    const std::size_t n = s.ceiling_planes.size() + 1; // planes + the cap
    for (std::size_t i = 0; i < n; ++i)
      for (std::size_t j = i + 1; j < n; ++j) {
        const double ai = along(s, i, 0.0), bi = along(s, i, 1.0) - ai;
        const double aj = along(s, j, 0.0), bj = along(s, j, 1.0) - aj;
        if (std::abs(bi - bj) < 1e-9)
          continue;
        const double t = (aj - ai) / (bi - bj);
        if (t > b.s0 && t < b.s1)
          ss.push_back(t);
      }
  }
  std::sort(ss.begin(), ss.end());
  ss.erase(std::unique(ss.begin(), ss.end(),
                       [](double x, double y) { return y - x < 1e-6; }),
           ss.end());
  return ss;
}

/// @p top without the points on a straight line between their neighbours
/// (past its ends a top holds its last value, top_at()).
std::vector<Vec2> simplify_top(const std::vector<Vec2> &top) {
  if (top.size() < 3)
    return top;
  std::vector<Vec2> simple{top.front()};
  for (std::size_t i = 1; i + 1 < top.size(); ++i) {
    const Vec2 &a = simple.back(), &c = top[i + 1], &q = top[i];
    const double w = (q.x() - a.x()) / (c.x() - a.x());
    if (std::abs(a.y() + w * (c.y() - a.y()) - q.y()) > 1e-4)
      simple.push_back(q);
  }
  simple.push_back(top.back());
  return simple;
}

/// A detected wall beside a space with a sloped ceiling takes its top from
/// the ceilings it meets (gable and knee walls, reusex-546): at every s the
/// highest ceiling of the spaces on either side there, evaluated on the
/// axis. Sampled every 0.25 m plus where each sloped space's ceiling bends
/// along the axis (two of its planes, or a plane and its cap, meet), so a
/// gable's peak is kept exactly; capped at z1, simplified to its bends.
void follow_sloped_ceilings(WallBox &b, const core::SolidModel &model,
                            const std::vector<std::size_t> &beside,
                            double off) {
  std::vector<std::size_t> sloped;
  for (const auto si : beside)
    if (!model.spaces[si].ceiling_planes.empty())
      sloped.push_back(si);
  if (sloped.empty())
    return;
  std::vector<const core::SpaceRecord *> of;
  for (const auto si : sloped)
    of.push_back(&model.spaces[si]);
  const auto ss = ceiling_breaks(b, of);

  const int storey = b.wall->storey;
  std::vector<Vec2> top;
  int best_space = -1;
  double best_area = -1.0;
  for (const double sv : ss) {
    const Vec2 p = b.at(std::clamp(sv, 0.0, b.len), 0.0);
    double z = -std::numeric_limits<double>::infinity();
    for (const double t : {off, -off}) {
      const Vec2 q = b.at(std::clamp(sv, 0.0, b.len), t);
      for (std::size_t si = 0; si < model.spaces.size(); ++si) {
        const auto &s = model.spaces[si];
        if (s.storey != storey || !(s.z_ceiling > s.z_floor) ||
            !inside_space(s, q))
          continue;
        z = std::max(z, core::ceiling_at(s, p.x(), p.y()));
        if (!s.ceiling_planes.empty() && std::max(s.area, 0.0) > best_area) {
          best_area = std::max(s.area, 0.0);
          best_space = static_cast<int>(si);
        }
      }
    }
    if (std::isfinite(z))
      top.emplace_back(sv, std::clamp(z, b.z0 + 0.05, b.z1));
  }
  if (top.size() < 2 || best_space < 0)
    return;
  b.top = simplify_top(top);
  if (has_sloped_top(b))
    b.slope_space = best_space;
  else
    b.top.clear();
}

// ── Slabs ───────────────────────────────────────────────────────────────────

/// One piece of a storey's floor polygon.
struct Piece {
  Ring outer;
  std::vector<Ring> holes;
  double area = 0.0;
};

EPolygon epolygon(const Ring &r) {
  EPolygon p;
  for (const auto &q : r)
    p.push_back(EK::Point_2(q.x(), q.y()));
  return p;
}

Ring dring(const EPolygon &p) {
  Ring r;
  for (auto v = p.vertices_begin(); v != p.vertices_end(); ++v)
    r.emplace_back(CGAL::to_double(v->x()), CGAL::to_double(v->y()));
  return r;
}

/// The union of @p rings (each simple; a non-simple one is counted in
/// @p not_simple and left out).
EPolygonSet ring_set(const std::vector<Ring> &rings, std::size_t &not_simple) {
  EPolygonSet set;
  for (const auto &r0 : rings) {
    const Ring r = clean_ring(r0, true);
    if (r.size() < 3)
      continue;
    EPolygon p = epolygon(r);
    if (!p.is_simple()) {
      ++not_simple;
      continue;
    }
    if (p.orientation() == CGAL::CLOCKWISE)
      p.reverse_orientation();
    set.join(p);
  }
  return set;
}

/// @p set split into pieces, largest first; small holes filled, small
/// pieces dropped.
std::vector<Piece> set_pieces(const EPolygonSet &set,
                              const SolidsOptions &opts) {
  std::vector<EPolygonWithHoles> pwhs;
  set.polygons_with_holes(std::back_inserter(pwhs));
  std::vector<Piece> out;
  for (const auto &pwh : pwhs) {
    if (pwh.is_unbounded())
      continue;
    Piece piece;
    piece.outer = dring(pwh.outer_boundary());
    piece.area = std::abs(signed_area(piece.outer));
    for (auto h = pwh.holes_begin(); h != pwh.holes_end(); ++h) {
      Ring hr = dring(*h);
      const double ha = std::abs(signed_area(hr));
      if (ha < opts.slab_min_hole_area_m2)
        continue;
      piece.area -= ha;
      piece.holes.push_back(std::move(hr));
    }
    if (piece.area >= opts.slab_min_area_m2)
      out.push_back(std::move(piece));
  }
  std::stable_sort(out.begin(), out.end(), [](const Piece &a, const Piece &b) {
    return a.area > b.area;
  });
  return out;
}

/// The heights a storey's spaces sit at (floors, or ceilings), grouped:
/// ascending, each the area-weighted mean of a chain of heights less than
/// tol apart, snapped to the storey's own height when within tol of it.
struct Levels {
  std::vector<double> z;
  /// Index of the level nearest @p v (ties: the lower).
  [[nodiscard]] std::size_t of(double v) const {
    std::size_t best = 0;
    for (std::size_t i = 1; i < z.size(); ++i)
      if (std::abs(z[i] - v) < std::abs(z[best] - v))
        best = i;
    return best;
  }
  [[nodiscard]] double snap(double v) const { return z.empty() ? v : z[of(v)]; }
};

Levels group_levels(std::vector<std::pair<double, double>> z_area,
                    double storey_z, double tol) {
  Levels out;
  std::stable_sort(z_area.begin(), z_area.end());
  for (std::size_t i = 0; i < z_area.size();) {
    double sum = 0.0, weight = 0.0;
    std::size_t j = i;
    do {
      const double w = std::max(z_area[j].second, 1e-6);
      sum += w * z_area[j].first;
      weight += w;
      ++j;
    } while (j < z_area.size() && z_area[j].first - z_area[j - 1].first < tol);
    double z = sum / weight;
    if (std::abs(z - storey_z) <= tol)
      z = storey_z;
    if (out.z.empty() || std::abs(out.z.back() - z) > 1e-9)
      out.z.push_back(z);
    i = j;
  }
  std::sort(out.z.begin(), out.z.end());
  out.z.erase(std::unique(out.z.begin(), out.z.end()), out.z.end());
  if (out.z.empty())
    out.z.push_back(storey_z);
  return out;
}

/// Regions of one storey per level, the higher level first taking what two
/// levels share: rings[k] are level k's spaces and walls. Returns per level
/// its pieces (ascending level order).
std::vector<std::vector<Piece>>
level_regions(const std::vector<std::vector<Ring>> &rings,
              const SolidsOptions &opts, std::size_t &not_simple) {
  std::vector<std::vector<Piece>> out(rings.size());
  EPolygonSet higher;
  for (std::size_t k = rings.size(); k-- > 0;) {
    EPolygonSet set = ring_set(rings[k], not_simple);
    EPolygonSet own = set;
    if (!higher.is_empty())
      own.difference(higher);
    out[k] = set_pieces(own, opts);
    higher.join(set);
  }
  return out;
}

std::vector<Piece> hull_piece(const std::vector<Ring> &rings) {
  std::vector<K::Point_2> pts, hull;
  for (const auto &r : rings)
    for (const auto &p : r)
      pts.emplace_back(p.x(), p.y());
  if (pts.size() < 3)
    return {};
  CGAL::convex_hull_2(pts.begin(), pts.end(), std::back_inserter(hull));
  Piece piece;
  for (const auto &p : hull)
    piece.outer.emplace_back(p.x(), p.y());
  piece.area = std::abs(signed_area(piece.outer));
  if (piece.outer.size() < 3 || piece.area < 1e-9)
    return {};
  return {piece};
}

ElementSolid build_slab(const core::StoreyRecord &st, const Piece &piece,
                        int part, double z_bottom, double thickness,
                        const char *predefined, const Frames &frames) {
  const Eigen::Affine3d &T = frames.of(st.idx);
  ElementSolid e;
  e.type = SolidType::slab;
  e.source_guid = st.guid;
  e.storey = st.idx;
  e.predefined = predefined;
  e.part = part;
  e.profile.outer = to_plan(clean_ring(piece.outer, true));
  for (const auto &h : piece.holes)
    e.profile.holes.push_back(to_plan(clean_ring(h, false)));
  e.profile.depth = thickness;
  e.profile.placement =
      row_major(T * local_frame(Vec2::Zero(), z_bottom, Vec2::UnitX()));
  auto m = extrude(piece.outer, piece.holes, z_bottom, z_bottom + thickness);
  Mesh mesh = m ? std::move(*m) : Mesh{};
  finish(e, mesh, T);
  return e;
}

/// A roof slab over a space with a sloped ceiling (reusex-546): the layer
/// between the ceiling and the ceiling raised by @p thickness over @p piece,
/// as (the piece's prism under the raised ceiling) minus (a wider prism
/// under the ceiling). The two never share a face, so the corefinement has
/// no coplanar case. Falls back to a flat slab at the space's z_ceiling,
/// with a warning, when the boolean fails.
/// @p r without vertices within 0.1 mm of the previous one: a union of
/// footprints leaves such near-duplicates, and the triangles they make break
/// the booleans of a sloped roof.
Ring tidy_ring(const Ring &r) {
  Ring out;
  for (const auto &p : r)
    if (out.empty() || (p - out.back()).norm() >= 1e-4)
      out.push_back(p);
  while (out.size() > 1 && (out.front() - out.back()).norm() < 1e-4)
    out.pop_back();
  return out;
}

ElementSolid build_sloped_roof(const core::StoreyRecord &st,
                               const Piece &piece_in, int part,
                               const core::SpaceRecord &space, double thickness,
                               const Frames &frames) {
  Piece piece = piece_in;
  piece.outer = tidy_ring(piece.outer);
  for (auto &h : piece.holes)
    h = tidy_ring(h);
  double zlow = std::numeric_limits<double>::infinity();
  double x0 = zlow, y0 = zlow, x1 = -zlow, y1 = -zlow;
  for (const auto &v : piece.outer) {
    zlow = std::min(zlow, core::ceiling_at(space, v.x(), v.y()));
    x0 = std::min(x0, v.x());
    y0 = std::min(y0, v.y());
    x1 = std::max(x1, v.x());
    y1 = std::max(y1, v.y());
  }
  zlow -= 0.05;
  const double cap = space.z_ceiling;
  std::optional<Mesh> above, below;
  if (std::isfinite(zlow) && cap > zlow) {
    above = extrude(piece.outer, piece.holes, zlow, cap + thickness);
    below = extrude({{x0 - 1.0, y0 - 1.0},
                     {x1 + 1.0, y0 - 1.0},
                     {x1 + 1.0, y1 + 1.0},
                     {x0 - 1.0, y1 + 1.0}},
                    {}, zlow - 1.0, cap);
  }
  Mesh out;
  EMesh a, b, diff;
  if (above && below) {
    a = to_exact(*above);
    b = to_exact(*below);
  }
  bool ok = above && below && CGAL::is_closed(a) &&
            !PMP::does_self_intersect(a) &&
            clip_below(a, space.ceiling_planes, thickness) &&
            clip_below(b, space.ceiling_planes, 0.0);
  if (ok) {
    try {
      ok = PMP::corefine_and_compute_difference(a, b, diff) &&
           CGAL::is_closed(diff) && diff.number_of_faces() > 0;
      if (ok)
        out = to_inexact(diff);
    } catch (const std::exception &e) {
      debug("solids: sloped roof over space {}: corefinement threw: {}",
            space.guid, e.what());
      ok = false;
    }
  }
  if (!ok) {
    warn("solids: the roof over space {} could not follow its sloped "
         "ceiling; it is a flat slab at {:.2f} m",
         space.guid, cap);
    return build_slab(st, piece, part, cap, thickness, "ROOF", frames);
  }
  const Eigen::Affine3d &T = frames.of(st.idx);
  ElementSolid e;
  e.type = SolidType::slab;
  e.source_guid = st.guid;
  e.storey = st.idx;
  e.predefined = "ROOF";
  e.part = part;
  e.profile.outer = to_plan(clean_ring(piece.outer, true));
  for (const auto &h : piece.holes)
    e.profile.holes.push_back(to_plan(clean_ring(h, false)));
  e.profile.depth = cap + thickness - zlow;
  e.profile.placement =
      row_major(T * local_frame(Vec2::Zero(), zlow, Vec2::UnitX()));
  finish(e, out, T);
  return e;
}

Vec2 centroid(const Ring &r) {
  Vec2 c = Vec2::Zero();
  for (const auto &p : r)
    c += p;
  return r.empty() ? c : Vec2(c / static_cast<double>(r.size()));
}

} // namespace

std::string_view to_string(SolidType type) {
  switch (type) {
  case SolidType::wall:
    return "wall";
  case SolidType::slab:
    return "slab";
  case SolidType::space:
    return "space";
  }
  return "unknown";
}

// ── Eaves: knee walls and dormers (reusex-ow8p) ───────────────────────────

/// A knee wall under @p space's slope, not beside the space (the strip
/// between them belongs to no space): its top is the space's ceiling on its
/// axis, where the slope meets the facade, capped at z1.
void follow_slope_over(WallBox &b, const core::SpaceRecord &space) {
  std::vector<Vec2> top;
  for (const double sv : ceiling_breaks(b, {&space})) {
    const Vec2 p = b.at(sv, 0.0);
    top.emplace_back(sv, std::clamp(core::ceiling_at(space, p.x(), p.y()),
                                    b.z0 + 0.05, b.z1));
  }
  if (top.size() >= 2)
    b.top = simplify_top(top);
}

/// @p b's top raised to each dormer's ceiling over its stretch of the wall:
/// the facade rises through the slope there.
void raise_for_dormers(WallBox &b,
                       const std::vector<const core::Dormer *> &dormers) {
  if (dormers.empty())
    return;
  constexpr double eps = 1e-3;
  std::vector<double> ss = top_breaks(b);
  const std::vector<double> base = ss;
  for (const auto *d : dormers) {
    for (const double v : {d->s0 - eps, d->s0, d->s1, d->s1 + eps})
      if (v > b.s0 && v < b.s1)
        ss.push_back(v);
    // Where the top crosses the dormer's ceiling inside its stretch.
    for (std::size_t i = 1; i < base.size(); ++i) {
      const double a = base[i - 1], c = base[i];
      const double za = b.top_at(a) - d->z, zc = b.top_at(c) - d->z;
      if (za * zc < 0.0) {
        const double t = a + za / (za - zc) * (c - a);
        if (t > d->s0 && t < d->s1)
          ss.push_back(t);
      }
    }
  }
  std::sort(ss.begin(), ss.end());
  ss.erase(std::unique(ss.begin(), ss.end(),
                       [](double x, double y) { return y - x < 1e-9; }),
           ss.end());
  std::vector<Vec2> top;
  for (const double sv : ss) {
    double z = b.top_at(sv);
    for (const auto *d : dormers)
      if (sv >= d->s0 && sv <= d->s1)
        z = std::max(z, std::min(d->z, b.z1));
    top.emplace_back(sv, z);
  }
  b.top = simplify_top(top);
}

/// A dormer's plan reaching out over its facade wall's footprint, so its
/// roof covers the wall it stands on.
Ring dormer_roof_ring(const core::Dormer &d, const WallBox *wall) {
  Ring r = to_ring(d.outline);
  if (r.size() != 4 || wall == nullptr)
    return r;
  const Vec2 in = 0.5 * ((r[3] - r[0]) + (r[2] - r[1]));
  if (!(in.norm() > 1e-6))
    return r;
  const Vec2 out = -in.normalized() *
                   (std::max(std::abs(wall->lo), std::abs(wall->hi)) + 0.01);
  r[0] += out;
  r[1] += out;
  return r;
}

/// The space's volume with its dormers added: the part of each dormer's
/// plan inside the space, from its floor to the dormer's ceiling, joined on
/// exactly. False (and @p m untouched) when a boolean fails.
bool add_dormers(Mesh &m, const core::SpaceRecord &space,
                 const std::vector<core::Dormer> &dormers,
                 const SolidsOptions &opts) {
  std::size_t not_simple = 0;
  std::vector<Ring> outer{to_ring(space.polygon.outer)};
  EPolygonSet in_space = ring_set(outer, not_simple);
  std::vector<Ring> holes;
  for (const auto &h : space.polygon.holes)
    holes.push_back(to_ring(h));
  if (!holes.empty())
    in_space.difference(ring_set(holes, not_simple));
  EMesh acc = to_exact(m);
  bool any = false;
  for (const auto &d : dormers) {
    if (!(d.z > space.z_floor))
      continue;
    EPolygonSet part = ring_set({to_ring(d.outline)}, not_simple);
    part.intersection(in_space);
    for (const auto &piece : set_pieces(part, opts)) {
      auto prism = extrude(piece.outer, piece.holes, space.z_floor, d.z);
      if (!prism)
        continue;
      EMesh add = to_exact(*prism), out;
      try {
        if (!PMP::corefine_and_compute_union(acc, add, out) ||
            !CGAL::is_closed(out))
          return false;
      } catch (const std::exception &e) {
        debug("solids: space {}: joining a dormer threw: {}", space.guid,
              e.what());
        return false;
      }
      acc = std::move(out);
      any = true;
    }
  }
  if (any)
    m = to_inexact(acc);
  return true;
}

namespace {

/// Namespace of the eave walls' GUIDs (any fixed UUID: the name carries the
/// space GUID).
constexpr const char *kEaveNamespace = "b4c1f3a2-6d0e-5f7a-9c8b-2e1d0a3f4b5c";

/// @p model with a knee wall for every eave with no wall (core::Eave,
/// reusex-wyi0): a one-sided wall whose axis (its seen face) is the eave,
/// from the space's floor up to the space's own slope along it (its top
/// profile; locked, so the rooms beside it do not re-derive it);
/// build_solids() grows it away from the space. Its GUID is deterministic:
/// the space's and the eave's index. nullopt when there are none.
std::optional<core::SolidModel> with_eave_walls(const core::SolidModel &model) {
  std::vector<core::WallRecord> added;
  for (const auto &s : model.spaces) {
    const auto edges = core::roof_edges(s);
    for (std::size_t k = 0; k < edges.eaves.size(); ++k) {
      const auto &e = edges.eaves[k];
      core::WallRecord w;
      w.guid = core::deterministic_guid(kEaveNamespace,
                                        fmt::format("eave/{}/{}", s.guid, k));
      w.storey = s.storey;
      w.x0 = e.a[0];
      w.y0 = e.a[1];
      w.x1 = e.b[0];
      w.y1 = e.b[1];
      // From the floor slab (the storey's floor level; the space's own
      // measured floor sits a few cm off it) to the space's top.
      w.z_base = s.z_floor;
      for (const auto &st : model.storeys)
        if (st.idx == s.storey && std::isfinite(st.z_floor))
          w.z_base = std::min(w.z_base, st.z_floor);
      w.height = s.z_ceiling - w.z_base;
      w.thickness = -1.0;
      w.assumed = true;
      w.is_external = true;
      w.source = "detected";
      // Its top: the slope of its own space along it, every 0.25 m; kept as
      // drawn (locked), so a flat room beside it cannot raise it.
      w.locked = true;
      w.metadata = R"({"inferred":"eave"})";
      const double len = std::hypot(w.x1 - w.x0, w.y1 - w.y0);
      const int n = std::max(1, static_cast<int>(std::ceil(len / 0.25)));
      bool ok = len > 1e-6 && std::isfinite(w.height) && w.height > 0.0;
      for (int i = 0; ok && i <= n; ++i) {
        const double t = static_cast<double>(i) / n;
        const double z = core::ceiling_at(s, w.x0 + t * (w.x1 - w.x0),
                                          w.y0 + t * (w.y1 - w.y0));
        ok = std::isfinite(z) && z > w.z_base + 0.05;
        w.top.push_back({t * len, z});
      }
      if (ok)
        added.push_back(std::move(w));
    }
  }
  if (added.empty())
    return std::nullopt;
  core::SolidModel m = model;
  for (auto &w : added)
    m.walls.push_back(std::move(w));
  return m;
}

} // namespace

SolidsResult build_solids(const core::SolidModel &model_in,
                          const SolidsOptions &opts) {
  // Eaves with no wall get one (reusex-wyi0).
  const auto eaves = with_eave_walls(model_in);
  const core::SolidModel &model = eaves ? *eaves : model_in;
  if (eaves)
    debug("solids: {} knee wall(s) drawn at eaves with no wall",
          eaves->walls.size() - model_in.walls.size());
  SolidsResult result;
  const Frames frames = make_frames(model);

  // Floor and ceiling levels of each storey's spaces (split levels).
  std::map<int, Levels> floor_levels, ceiling_levels;
  {
    std::map<int, std::vector<std::pair<double, double>>> floors, ceilings;
    for (const auto &s : model.spaces) {
      if (!std::isfinite(s.z_floor) || !std::isfinite(s.z_ceiling) ||
          !(s.z_ceiling > s.z_floor))
        continue;
      const double area = std::max(s.area, 0.0);
      floors[s.storey].emplace_back(s.z_floor, area);
      ceilings[s.storey].emplace_back(s.z_ceiling, area);
    }
    for (const auto &st : model.storeys) {
      floor_levels[st.idx] =
          group_levels(floors[st.idx], st.z_floor, opts.level_tol_m);
      ceiling_levels[st.idx] =
          group_levels(ceilings[st.idx], st.z_ceiling, opts.level_tol_m);
    }
  }
  const auto levels_of = [](const std::map<int, Levels> &m, int storey,
                            double fallback) {
    const auto it = m.find(storey);
    return it == m.end() ? Levels{{fallback}} : it->second;
  };

  // Walls, with their openings cut out.
  auto boxes = wall_boxes(model, opts, result.walls_skipped);
  if (opts.walls_follow_spaces) {
    // From the floor level of the lowest space beside the wall to the
    // ceiling level of the highest one.
    for (auto &b : boxes) {
      const core::WallRecord &w = *b.wall;
      if (!b.valid || w.locked || w.source != "detected")
        continue;
      const double off =
          std::max(0.15, std::max(std::abs(b.lo), std::abs(b.hi)) + 0.1);
      double lo = std::numeric_limits<double>::infinity();
      double hi = -std::numeric_limits<double>::infinity();
      std::vector<std::size_t> beside;
      for (const double f : {0.1, 0.3, 0.5, 0.7, 0.9}) {
        const double sv = f * b.len;
        for (const double t : {off, -off}) {
          const Vec2 p = b.at(sv, t);
          for (std::size_t si = 0; si < model.spaces.size(); ++si) {
            const auto &s = model.spaces[si];
            if (s.storey != w.storey || !(s.z_ceiling > s.z_floor) ||
                !inside_space(s, p))
              continue;
            lo = std::min(lo, s.z_floor);
            hi = std::max(hi, s.z_ceiling);
            if (std::find(beside.begin(), beside.end(), si) == beside.end())
              beside.push_back(si);
          }
        }
      }
      if (!std::isfinite(lo) || !std::isfinite(hi))
        continue;
      const auto *st = [&]() -> const core::StoreyRecord * {
        for (const auto &x : model.storeys)
          if (x.idx == w.storey)
            return &x;
        return nullptr;
      }();
      const double z0 =
          levels_of(floor_levels, w.storey, st ? st->z_floor : lo).snap(lo);
      const double z1 =
          levels_of(ceiling_levels, w.storey, st ? st->z_ceiling : hi).snap(hi);
      if (z1 > z0) {
        b.z0 = z0;
        b.z1 = z1;
        follow_sloped_ceilings(b, model, beside, off);
      }
    }
  }
  // Eaves (reusex-ow8p): a facade wall under a slope that is not beside its
  // space (the strip between them belongs to no space) stops where the slope
  // meets it, a knee wall; over a dormer it rises to the dormer's ceiling.
  // Both were measured from the cloud by the spaces stage.
  std::vector<core::RoofEdges> roof_edges(model.spaces.size());
  for (std::size_t si = 0; si < model.spaces.size(); ++si)
    roof_edges[si] = core::roof_edges(model.spaces[si]);
  std::unordered_map<std::string, std::size_t> box_of;
  for (std::size_t i = 0; i < boxes.size(); ++i)
    if (boxes[i].valid)
      box_of.emplace(boxes[i].wall->guid, i);
  const auto follows = [&](const WallBox &b) {
    return opts.walls_follow_spaces && !b.wall->locked &&
           b.wall->source == "detected";
  };
  std::map<std::size_t, std::vector<const core::Dormer *>> dormers_of_box;
  std::map<std::size_t, int> dormer_space_of_box;
  for (std::size_t si = 0; si < model.spaces.size(); ++si) {
    for (const auto &g : roof_edges[si].knee_walls) {
      const auto it = box_of.find(g);
      if (it == box_of.end())
        continue;
      WallBox &b = boxes[it->second];
      if (!follows(b) || b.slope_space >= 0 ||
          b.wall->storey != model.spaces[si].storey)
        continue;
      follow_slope_over(b, model.spaces[si]);
      if (has_sloped_top(b)) {
        b.slope_space = static_cast<int>(si);
        debug("solids: wall {} is a knee wall under space {}: {:.2f}..{:.2f} "
              "m above its base",
              g, model.spaces[si].guid, b.top_at(b.s0) - b.z0,
              b.top_at(b.s1) - b.z0);
      } else {
        b.top.clear();
      }
    }
    for (const auto &d : roof_edges[si].dormers) {
      const auto it = box_of.find(d.wall);
      if (it == box_of.end() || !follows(boxes[it->second]))
        continue;
      dormers_of_box[it->second].push_back(&d);
      dormer_space_of_box.emplace(it->second, static_cast<int>(si));
    }
  }
  for (const auto &[i, ds] : dormers_of_box) {
    WallBox &b = boxes[i];
    raise_for_dormers(b, ds);
    if (b.slope_space < 0)
      b.slope_space = dormer_space_of_box[i];
  }

  // The roof over a wall beside a sloped space goes with that space's roof
  // (the largest such space beside it), so it follows the same planes.
  for (auto &b : boxes) {
    if (!b.valid || b.slope_space >= 0)
      continue;
    const double off =
        std::max(0.15, std::max(std::abs(b.lo), std::abs(b.hi)) + 0.1);
    double best = -1.0;
    for (const double f : {0.1, 0.3, 0.5, 0.7, 0.9})
      for (const double t : {off, -off}) {
        const Vec2 p = b.at(f * b.len, t);
        for (std::size_t si = 0; si < model.spaces.size(); ++si) {
          const auto &s = model.spaces[si];
          if (s.storey != b.wall->storey || s.ceiling_planes.empty() ||
              !inside_space(s, p))
            continue;
          const double area =
              std::max(s.area, std::abs(signed_area(to_ring(s.polygon.outer))));
          if (area > best) {
            best = area;
            b.slope_space = static_cast<int>(si);
          }
        }
      }
  }
  std::unordered_map<std::string, std::vector<const core::OpeningRecord *>>
      openings;
  std::unordered_map<std::string, bool> valid_wall;
  for (const auto &b : boxes)
    valid_wall[b.wall->guid] = valid_wall[b.wall->guid] || b.valid;
  for (const auto &o : model.openings) {
    if (const auto it = valid_wall.find(o.wall);
        it == valid_wall.end() || !it->second) {
      ++result.openings_skipped;
      debug("solids: opening {} names wall {}, which the model does not hold "
            "(or which has no solid)",
            o.guid, o.wall);
      continue;
    }
    openings[o.wall].push_back(&o);
  }
  for (const auto &b : boxes) {
    if (!b.valid)
      continue;
    static const std::vector<const core::OpeningRecord *> none;
    const auto it = openings.find(b.wall->guid);
    result.walls.push_back(build_wall(
        b, it == openings.end() ? none : it->second, frames, opts, result));
  }

  // Spaces.
  for (const auto &s : model.spaces) {
    const Eigen::Affine3d &T = frames.of(s.storey);
    std::vector<Ring> holes;
    for (const auto &h : s.polygon.holes)
      holes.push_back(to_ring(h));
    auto m = extrude(to_ring(s.polygon.outer), holes, s.z_floor, s.z_ceiling);
    if (!m) {
      ++result.spaces_skipped;
      continue;
    }
    ElementSolid e;
    e.type = SolidType::space;
    e.source_guid = s.guid;
    e.storey = s.storey;
    e.predefined = s.kind.empty() ? "unknown" : s.kind;
    e.profile.outer = to_plan(clean_ring(to_ring(s.polygon.outer), true));
    for (const auto &h : holes)
      e.profile.holes.push_back(to_plan(clean_ring(h, false)));
    e.profile.depth = s.z_ceiling - s.z_floor;
    e.profile.placement =
        row_major(T * local_frame(Vec2::Zero(), s.z_floor, Vec2::UnitX()));
    // A sloped ceiling cuts the prism (reusex-546).
    if (!s.ceiling_planes.empty()) {
      Mesh keep = *m;
      if (clip_below(*m, s.ceiling_planes, 0.0) && CGAL::is_closed(*m)) {
        for (const auto &p : s.ceiling_planes)
          e.clip_planes.push_back(world_plane(below_plane(p, 0.0), T));
      } else {
        warn("solids: space {}: its ceiling planes could not be cut from "
             "its volume; it is left flat at {:.2f} m",
             s.guid, s.z_ceiling);
        *m = std::move(keep);
      }
      // Its dormers rise above the slope (reusex-ow8p).
      const auto edges = core::roof_edges(s);
      if (!edges.dormers.empty() && !add_dormers(*m, s, edges.dormers, opts))
        warn("solids: space {}: its {} dormer(s) could not be joined to its "
             "volume; it stops at the slope there",
             s.guid, edges.dormers.size());
    }
    finish(e, *m, T);
    result.spaces.push_back(std::move(e));
  }

  // Slabs: per storey, one floor region per floor level (its spaces and the
  // walls standing on it), all down to the storey's slab bottom; a roof per
  // ceiling level over the top storey.
  std::vector<const core::StoreyRecord *> storeys;
  for (const auto &s : model.storeys)
    storeys.push_back(&s);
  std::stable_sort(
      storeys.begin(), storeys.end(),
      [](const auto *a, const auto *b) { return a->idx < b->idx; });
  std::size_t not_simple = 0;
  for (std::size_t k = 0; k < storeys.size(); ++k) {
    const auto &st = *storeys[k];
    const Levels fl = levels_of(floor_levels, st.idx, st.z_floor);
    const Levels cl = levels_of(ceiling_levels, st.idx, st.z_ceiling);
    std::vector<std::vector<Ring>> floor_rings(fl.z.size()),
        roof_rings(cl.z.size());
    // Roof regions past cl.z.size() each belong to one space with a sloped
    // ceiling (reusex-546): its own polygon and the walls whose top follows
    // it, last so that they take what they share with a flat level.
    constexpr std::size_t kFlat = std::numeric_limits<std::size_t>::max();
    std::vector<std::size_t> roof_space(cl.z.size(), kFlat);
    std::map<std::size_t, std::size_t> sloped_group;
    bool any_space = false;
    for (std::size_t si = 0; si < model.spaces.size(); ++si) {
      const auto &s = model.spaces[si];
      if (s.storey == st.idx && s.polygon.outer.size() >= 3) {
        floor_rings[fl.of(s.z_floor)].push_back(to_ring(s.polygon.outer));
        if (s.ceiling_planes.empty()) {
          roof_rings[cl.of(s.z_ceiling)].push_back(to_ring(s.polygon.outer));
        } else {
          sloped_group[si] = roof_rings.size();
          roof_rings.push_back({to_ring(s.polygon.outer)});
          roof_space.push_back(si);
        }
        any_space = true;
      }
    }
    // Dormer roofs (reusex-ow8p), last again: each takes its plan from the
    // sloped roof it rises through. roof_z: a dormer roof's height.
    std::vector<double> roof_z(roof_rings.size(),
                               std::numeric_limits<double>::quiet_NaN());
    for (std::size_t si = 0; si < model.spaces.size(); ++si) {
      if (model.spaces[si].storey != st.idx)
        continue;
      for (const auto &d : roof_edges[si].dormers) {
        const auto it = box_of.find(d.wall);
        const WallBox *wb = it == box_of.end() ? nullptr : &boxes[it->second];
        roof_rings.push_back({dormer_roof_ring(d, wb)});
        roof_space.push_back(kFlat);
        roof_z.push_back(d.z);
      }
    }
    std::vector<Ring> wall_rings;
    for (const auto &b : boxes)
      if (b.valid && b.wall->storey == st.idx) {
        wall_rings.push_back(b.footprint());
        floor_rings[fl.of(b.z0)].push_back(b.footprint());
        const auto g =
            b.slope_space >= 0
                ? sloped_group.find(static_cast<std::size_t>(b.slope_space))
                : sloped_group.end();
        if (g != sloped_group.end())
          roof_rings[g->second].push_back(b.footprint());
        else
          roof_rings[cl.of(b.z1)].push_back(b.footprint());
      }
    std::vector<std::vector<Piece>> floors, roofs;
    if (any_space) {
      floors = level_regions(floor_rings, opts, not_simple);
      roofs = level_regions(roof_rings, opts, not_simple);
    } else {
      // No space: the hull of the walls; fl / cl hold only the storey's
      // own floor and ceiling then.
      floors = {hull_piece(wall_rings)};
      roofs = floors;
    }
    const Levels &fl_used = fl;
    const Levels &cl_used = cl;
    std::size_t n_floor = 0;
    for (const auto &f : floors)
      n_floor += f.size();
    if (n_floor == 0) {
      warn("solids: storey {} has no space and no wall to draw a floor from; "
           "no slab",
           st.idx);
      continue;
    }
    // The gap to the storey below, in world z at the floor's centre.
    const double z_low = fl_used.z.front();
    double thickness = opts.slab_thickness_m;
    if (k > 0) {
      const auto &below = *storeys[k - 1];
      const Piece *first = nullptr;
      for (const auto &f : floors)
        if (!f.empty() && (!first || f.front().area > first->area))
          first = &f.front();
      const Vec2 c = centroid(first->outer);
      const double z_floor =
          (frames.of(st.idx) * Eigen::Vector3d(c.x(), c.y(), z_low)).z();
      const double z_ceiling = (frames.of(below.idx) *
                                Eigen::Vector3d(c.x(), c.y(), below.z_ceiling))
                                   .z();
      const double gap = z_floor - z_ceiling;
      if (gap > 1e-3 && gap <= opts.max_slab_thickness_m)
        thickness = gap;
      else
        info("solids: storey {}: the gap to storey {} is {:.2f} m (not in "
             "(0, {:.2f}]); its floor slab gets the default {:.2f} m",
             st.idx, below.idx, gap, opts.max_slab_thickness_m,
             opts.slab_thickness_m);
    }
    const double z_bottom = z_low - thickness;
    if (fl_used.z.size() > 1)
      info("solids: storey {} is split into {} floor levels ({}); each "
           "level's slab fills down to {:.2f} m",
           st.idx, fl_used.z.size(), fmt::join(fl_used.z, ", "), z_bottom);
    int part = 0;
    for (std::size_t lv = 0; lv < floors.size(); ++lv)
      for (const auto &piece : floors[lv])
        result.slabs.push_back(build_slab(st, piece, part++, z_bottom,
                                          fl_used.z[lv] - z_bottom, "FLOOR",
                                          frames));
    // The flat roof over the top storey; a sloped ceiling's roof on every
    // storey (a sloped ceiling is under a roof wherever it is: Sygehusgrunden's
    // Etage 2 has its own pitched part beside Etage 3).
    if (opts.roof) {
      const bool top = k + 1 == storeys.size();
      int roof_part = 0;
      for (std::size_t lv = 0; lv < roofs.size(); ++lv)
        for (const auto &piece : roofs[lv]) {
          if (lv < roof_z.size() && std::isfinite(roof_z[lv]))
            result.slabs.push_back(build_slab(st, piece, roof_part++,
                                              roof_z[lv], opts.slab_thickness_m,
                                              "ROOF", frames));
          else if (lv < roof_space.size() && roof_space[lv] != kFlat)
            result.slabs.push_back(build_sloped_roof(
                st, piece, roof_part++, model.spaces[roof_space[lv]],
                opts.slab_thickness_m, frames));
          else if (top)
            result.slabs.push_back(
                build_slab(st, piece, roof_part++, cl_used.z[lv],
                           opts.slab_thickness_m, "ROOF", frames));
        }
    }
  }
  if (not_simple > 0)
    warn("solids: {} space / wall outline(s) are not simple polygons and were "
         "left out of the slabs",
         not_simple);

  for (const auto *v : {&result.walls, &result.slabs, &result.spaces})
    for (const auto &e : *v)
      if (!e.closed)
        ++result.not_closed;
  return result;
}

} // namespace reusex::geometry
