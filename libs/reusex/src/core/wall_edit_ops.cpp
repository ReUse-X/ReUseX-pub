// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/wall_edit_ops.hpp"

#include "core/guid.hpp"

#include <fmt/format.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <optional>
#include <stdexcept>

namespace reusex::core {

namespace {

using V = std::array<double, 2>;

V sub(V a, V b) { return {a[0] - b[0], a[1] - b[1]}; }
V add(V a, V b) { return {a[0] + b[0], a[1] + b[1]}; }
V mul(V a, double k) { return {a[0] * k, a[1] * k}; }
double dot(V a, V b) { return a[0] * b[0] + a[1] * b[1]; }
double cross(V a, V b) { return a[0] * b[1] - a[1] * b[0]; }
double norm(V a) { return std::hypot(a[0], a[1]); }
V start_of(const WallRecord &w) { return {w.x0, w.y0}; }
V end_of(const WallRecord &w) { return {w.x1, w.y1}; }

V unit_dir(const WallRecord &w) {
  const V d = sub(end_of(w), start_of(w));
  const double n = norm(d);
  if (!(n > 0.0))
    throw std::invalid_argument("wall " + w.guid + " has a zero-length axis");
  return mul(d, 1.0 / n);
}

void set_axis(WallRecord &w, V p0, V p1) {
  w.x0 = p0[0];
  w.y0 = p0[1];
  w.x1 = p1[0];
  w.y1 = p1[1];
}

/// The top profile shifted by @p ds (s' = s + ds) and cut to [0, len],
/// with the profile's value at each cut end added: strictly increasing s.
std::vector<PlanPoint> remeasure_top(const std::vector<PlanPoint> &top,
                                     double ds, double len) {
  if (top.empty())
    return {};
  const auto z_at = [&](double s) {
    if (s <= top.front()[0])
      return top.front()[1];
    if (s >= top.back()[0])
      return top.back()[1];
    for (size_t i = 1; i < top.size(); ++i)
      if (s <= top[i][0]) {
        const double t = (s - top[i - 1][0]) / (top[i][0] - top[i - 1][0]);
        return top[i - 1][1] + t * (top[i][1] - top[i - 1][1]);
      }
    return top.back()[1];
  };
  std::vector<PlanPoint> out;
  out.push_back({0.0, z_at(-ds)});
  for (const auto &p : top) {
    const double s = p[0] + ds;
    if (s > 1e-9 && s < len - 1e-9)
      out.push_back({s, p[1]});
  }
  out.push_back({len, z_at(len - ds)});
  return out;
}

/// The plan point of an opening's centre on its host.
V opening_centre(const OpeningRecord &o, const WallRecord &host) {
  return add(start_of(host), mul(unit_dir(host), 0.5 * (o.s_start + o.s_end)));
}

/// @p o moved onto @p host (same width), its centre projected on the axis,
/// clamped to the wall. @returns whether it had to be clipped.
bool rehost(OpeningRecord &o, const WallRecord &host, V centre) {
  const double len = wall_length(host);
  const double width = o.s_end - o.s_start;
  const double c = dot(sub(centre, start_of(host)), unit_dir(host));
  o.wall = host.guid;
  o.s_start = c - 0.5 * width;
  o.s_end = c + 0.5 * width;
  bool clipped = false;
  if (o.s_start < 0.0) {
    o.s_start = 0.0;
    clipped = true;
  }
  if (o.s_end > len) {
    o.s_end = len;
    clipped = true;
  }
  if (!(o.s_end > o.s_start)) // fully off: keep a sliver at the nearer end
    o.s_start = std::max(0.0, o.s_end - 0.01);
  return clipped;
}

double distance_to_segment(V p, V a, V b) {
  const V ab = sub(b, a);
  const double l2 = dot(ab, ab);
  const double t =
      l2 > 0.0 ? std::clamp(dot(sub(p, a), ab) / l2, 0.0, 1.0) : 0.0;
  return norm(sub(p, add(a, mul(ab, t))));
}

} // namespace

double wall_length(const WallRecord &w) {
  return std::hypot(w.x1 - w.x0, w.y1 - w.y0);
}

WallSplitResult split_wall(const WallRecord &wall, double s,
                           const std::vector<OpeningRecord> &openings,
                           const std::vector<WallJoinRecord> &joins,
                           const std::vector<WallRecord> &others,
                           const std::string &new_guid, double min_part_m) {
  const double len = wall_length(wall);
  if (!(s >= min_part_m && s <= len - min_part_m))
    throw std::invalid_argument(fmt::format(
        "split_wall: s = {:.3f} m is not inside wall {} (length {:.3f} m) by "
        "{:.2f} m",
        s, wall.guid, len, min_part_m));
  const V d = unit_dir(wall);
  const V cut = add(start_of(wall), mul(d, s));

  WallSplitResult r;
  r.a = wall;
  r.b = wall;
  r.b.id = -1;
  r.b.guid = new_guid.empty() ? generate_guid() : new_guid;
  set_axis(r.a, start_of(wall), cut);
  set_axis(r.b, cut, end_of(wall));
  r.a.top = remeasure_top(wall.top, 0.0, s);
  r.b.top = remeasure_top(wall.top, -s, len - s);

  for (auto o : openings) {
    if (o.wall != wall.guid)
      continue;
    const double c = 0.5 * (o.s_start + o.s_end);
    WallRecord &host = c < s ? r.a : r.b;
    if (rehost(o, host, opening_centre(o, wall)))
      r.clipped.push_back(o.guid);
    r.openings.push_back(std::move(o));
  }

  for (auto j : joins) {
    const bool on_a = j.wall_a == wall.guid, on_b = j.wall_b == wall.guid;
    if (!on_a && !on_b)
      continue;
    const std::string &at = on_a ? j.at_a : j.at_b;
    bool to_b = at == "end";
    if (at == "path") {
      // Where the other wall touches: its endpoint nearest this axis.
      const std::string &other = on_a ? j.wall_b : j.wall_a;
      const auto it =
          std::find_if(others.begin(), others.end(),
                       [&](const auto &w) { return w.guid == other; });
      if (it != others.end()) {
        const V p0 = start_of(*it), p1 = end_of(*it);
        const V p =
            distance_to_segment(p0, start_of(wall), end_of(wall)) <=
                    distance_to_segment(p1, start_of(wall), end_of(wall))
                ? p0
                : p1;
        to_b = dot(sub(p, start_of(wall)), d) > s;
      }
    }
    if (to_b)
      (on_a ? j.wall_a : j.wall_b) = r.b.guid;
    r.joins.push_back(std::move(j));
  }
  r.joins.push_back({r.a.guid, r.b.guid, "collinear", "end", "start"});
  return r;
}

WallMergeResult merge_walls(const WallRecord &a, const WallRecord &b,
                            const std::vector<OpeningRecord> &openings,
                            const std::vector<WallJoinRecord> &joins,
                            const WallMergeOptions &opt) {
  if (a.guid == b.guid)
    throw std::invalid_argument("merge_walls: a wall cannot merge with itself");
  if (a.storey != b.storey)
    throw std::invalid_argument(
        fmt::format("merge_walls: walls {} and {} are on storeys {} and {}",
                    a.guid, b.guid, a.storey, b.storey));
  const V da = unit_dir(a);
  V db = unit_dir(b);
  if (dot(da, db) < 0.0)
    db = mul(db, -1.0);
  const double angle =
      std::acos(std::clamp(dot(da, db), -1.0, 1.0)) * 180.0 / std::numbers::pi;
  if (angle > opt.max_angle_deg)
    throw std::invalid_argument(fmt::format(
        "merge_walls: walls {} and {} differ by {:.1f} deg (max {:.1f})",
        a.guid, b.guid, angle, opt.max_angle_deg));
  const double la = wall_length(a), lb = wall_length(b);
  const V ma = mul(add(start_of(a), end_of(a)), 0.5);
  const V mb = mul(add(start_of(b), end_of(b)), 0.5);
  const V na{-da[1], da[0]}, nb{-db[1], db[0]};
  const double off =
      std::max(std::abs(dot(sub(mb, ma), na)), std::abs(dot(sub(ma, mb), nb)));
  if (off > opt.max_offset_m)
    throw std::invalid_argument(
        fmt::format("merge_walls: walls {} and {} are {:.2f} m apart across "
                    "their axes (max {:.2f} m)",
                    a.guid, b.guid, off, opt.max_offset_m));

  // Length-weighted axis: direction and a point on it.
  V dir = add(mul(da, la), mul(db, lb));
  dir = mul(dir, 1.0 / norm(dir));
  const V mid = mul(add(mul(ma, la), mul(mb, lb)), 1.0 / (la + lb));
  double smin = 0.0, smax = 0.0;
  bool first = true;
  for (const V p : {start_of(a), end_of(a), start_of(b), end_of(b)}) {
    const double t = dot(sub(p, mid), dir);
    smin = first ? t : std::min(smin, t);
    smax = first ? t : std::max(smax, t);
    first = false;
  }

  WallMergeResult r;
  r.wall = a;
  r.removed = b.guid;
  set_axis(r.wall, add(mid, mul(dir, smin)), add(mid, mul(dir, smax)));
  if (a.thickness > 0.0 && b.thickness > 0.0)
    r.wall.thickness = (a.thickness * la + b.thickness * lb) / (la + lb);
  else
    r.wall.thickness = std::max(a.thickness, b.thickness);
  r.wall.thickness_ci_lo = r.wall.thickness_ci_hi = -1.0;
  const double top = std::max(a.z_base + a.height, b.z_base + b.height);
  r.wall.z_base = std::min(a.z_base, b.z_base);
  r.wall.height = top - r.wall.z_base;
  r.wall.is_external = a.is_external || b.is_external;
  r.wall.assumed = a.assumed && b.assumed;
  r.wall.face_a_points = a.face_a_points + b.face_a_points;
  r.wall.face_b_points = a.face_b_points + b.face_b_points;
  if (a.seen_share >= 0.0 && b.seen_share >= 0.0)
    r.wall.seen_share = (a.seen_share * la + b.seen_share * lb) / (la + lb);
  // The top profiles, re-read along the merged axis.
  r.wall.top.clear();
  if (!a.top.empty() || !b.top.empty()) {
    std::vector<PlanPoint> pts;
    for (const WallRecord *w : {&a, &b}) {
      const V d = unit_dir(*w);
      const auto profile =
          w->top.empty()
              ? std::vector<PlanPoint>{{0.0, w->z_base + w->height},
                                       {wall_length(*w), w->z_base + w->height}}
              : w->top;
      for (const auto &p : profile) {
        const V at = add(start_of(*w), mul(d, p[0]));
        pts.push_back({dot(sub(at, start_of(r.wall)), unit_dir(r.wall)), p[1]});
      }
    }
    std::sort(pts.begin(), pts.end());
    for (const auto &p : pts)
      if (r.wall.top.empty() || p[0] > r.wall.top.back()[0] + 1e-6)
        r.wall.top.push_back(p);
    if (r.wall.top.size() < 2)
      r.wall.top.clear();
  }

  for (auto o : openings) {
    const WallRecord *host =
        o.wall == a.guid ? &a : (o.wall == b.guid ? &b : nullptr);
    if (!host)
      continue;
    rehost(o, r.wall, opening_centre(o, *host));
    r.openings.push_back(std::move(o));
  }

  const double len = wall_length(r.wall);
  for (auto j : joins) {
    const bool a_side = j.wall_a == a.guid || j.wall_a == b.guid;
    const bool b_side = j.wall_b == a.guid || j.wall_b == b.guid;
    if (a_side == b_side) // between a and b, or not theirs
      continue;
    const WallRecord &old = (a_side ? j.wall_a : j.wall_b) == a.guid ? a : b;
    std::string &at = a_side ? j.at_a : j.at_b;
    (a_side ? j.wall_a : j.wall_b) = r.wall.guid;
    if (at == "path")
      continue;
    const V p = at == "start" ? start_of(old) : end_of(old);
    const double t = dot(sub(p, start_of(r.wall)), unit_dir(r.wall));
    const double tol = std::max(0.05, r.wall.thickness);
    if (t < tol)
      at = "start";
    else if (t > len - tol)
      at = "end";
    else {
      at = "path";
      if (j.kind == "corner" || j.kind == "collinear")
        j.kind = "tee";
    }
    r.joins.push_back(std::move(j));
  }
  return r;
}

WallJoinResult join_walls(const WallRecord &a, const WallRecord &b,
                          const std::vector<OpeningRecord> &openings,
                          double min_angle_deg, double max_trim_m) {
  if (a.guid == b.guid)
    throw std::invalid_argument("join_walls: a wall cannot join itself");
  if (a.storey != b.storey)
    throw std::invalid_argument(
        fmt::format("join_walls: walls {} and {} are on storeys {} and {}",
                    a.guid, b.guid, a.storey, b.storey));
  const V da = unit_dir(a), db = unit_dir(b);
  const double sine = std::abs(cross(da, db));
  if (sine < std::sin(min_angle_deg * std::numbers::pi / 180.0))
    throw std::invalid_argument(fmt::format(
        "join_walls: walls {} and {} are parallel within {:.1f} deg", a.guid,
        b.guid, min_angle_deg));
  // start(a) + t da = start(b) + u db
  const V w0 = sub(start_of(b), start_of(a));
  const double t = cross(w0, db) / cross(da, db);
  const V x = add(start_of(a), mul(da, t));

  WallJoinResult r;
  r.a = a;
  r.b = b;
  // Per wall: which end moves (none for a tee), and how far its start moved
  // along the axis (openings and the top are measured from it).
  struct Moved {
    std::string at;
    double ds = 0.0;
  };
  const auto move = [&](WallRecord &w, const WallRecord &orig) {
    const V d = unit_dir(orig);
    const double len = wall_length(orig);
    const double s = dot(sub(x, start_of(orig)), d);
    if (std::min(s, len - s) > max_trim_m) {
      // Deep inside: a tee onto this wall, which stays as it is.
      return Moved{"path", 0.0};
    }
    Moved m;
    if (std::abs(s) <= std::abs(s - len)) { // the start is nearer
      m.at = "start";
      m.ds = -s; // the start moves to s: every s shifts by -s
      set_axis(w, x, end_of(orig));
    } else {
      m.at = "end";
      set_axis(w, start_of(orig), x);
    }
    if (wall_length(w) < 0.05)
      throw std::invalid_argument(
          fmt::format("join_walls: joining would leave wall {} {:.3f} m long",
                      orig.guid, wall_length(w)));
    w.top = remeasure_top(orig.top, m.ds, wall_length(w));
    return m;
  };
  Moved ma = move(r.a, a), mb = move(r.b, b);
  if (ma.at == "path" && mb.at == "path")
    throw std::invalid_argument(fmt::format(
        "join_walls: walls {} and {} cross inside both; split one first",
        a.guid, b.guid));
  r.join = {a.guid, b.guid,
            (ma.at == "path" || mb.at == "path") ? "tee" : "corner", ma.at,
            mb.at};

  for (auto o : openings) {
    const Moved *m =
        o.wall == a.guid ? &ma : (o.wall == b.guid ? &mb : nullptr);
    if (!m)
      continue;
    const WallRecord &host = o.wall == a.guid ? r.a : r.b;
    o.s_start += m->ds;
    o.s_end += m->ds;
    const double len = wall_length(host);
    o.s_start = std::clamp(o.s_start, 0.0, len);
    o.s_end = std::clamp(o.s_end, 0.0, len);
    if (!(o.s_end > o.s_start))
      o.s_start = std::max(0.0, o.s_end - 0.01);
    r.openings.push_back(std::move(o));
  }
  return r;
}

} // namespace reusex::core
