// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/wall_edit_ops.hpp"

#include "core/guid.hpp"

#include <fmt/format.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <optional>
#include <set>
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

// --- extend_wall_to / transform_selection (reusex-so4t) ---------------------

namespace reusex::core {

namespace {

constexpr double kMinWallLengthM = 0.05;

/// The intersection of the two walls' axis lines.
/// @throws std::invalid_argument when they are parallel within
///         @p min_angle_deg.
V axes_intersection(const WallRecord &a, const WallRecord &b,
                    double min_angle_deg) {
  const V da = unit_dir(a), db = unit_dir(b);
  if (std::abs(cross(da, db)) <
      std::sin(min_angle_deg * std::numbers::pi / 180.0))
    throw std::invalid_argument(
        fmt::format("walls {} and {} are parallel within {:.1f} deg", a.guid,
                    b.guid, min_angle_deg));
  const double t = cross(sub(start_of(b), start_of(a)), db) / cross(da, db);
  return add(start_of(a), mul(da, t));
}

/// Move the end ("end" when @p at_end, else the start) of @p w to the point
/// of its axis nearest @p p, keeping the other end; the top profile is
/// re-measured. @throws std::invalid_argument when that would leave the
/// wall shorter than 5 cm or turn it round.
void move_wall_end(WallRecord &w, bool at_end, V p) {
  const V fixed = at_end ? start_of(w) : end_of(w);
  const V moving = at_end ? end_of(w) : start_of(w);
  const V d = mul(sub(moving, fixed), 1.0 / norm(sub(moving, fixed)));
  const double along = dot(sub(p, fixed), d);
  if (!(along >= kMinWallLengthM))
    throw std::invalid_argument(fmt::format(
        "moving the {} of wall {} there would {}", at_end ? "end" : "start",
        w.guid,
        along < 0.0 ? "turn it round"
                    : fmt::format("leave it {:.3f} m long", along)));
  const V q = add(fixed, mul(d, along));
  // s is measured from the start: a moved start shifts every s.
  const double ds = at_end ? 0.0 : dot(sub(start_of(w), q), mul(d, -1.0));
  if (at_end)
    set_axis(w, fixed, q);
  else
    set_axis(w, q, fixed);
  w.top = remeasure_top(w.top, ds, along);
}

/// Where the @p at_end end of @p w meets @p t's near face: the face on the
/// side of w's other end (the axis when t's thickness is unknown).
/// @throws std::invalid_argument when the axes miss @p t's extent.
V tee_point(const WallRecord &w, bool at_end, const WallRecord &t,
            double min_angle_deg) {
  const V x = axes_intersection(w, t, min_angle_deg);
  const V dt = unit_dir(t);
  const double u = dot(sub(x, start_of(t)), dt);
  if (u < 0.0 || u > wall_length(t))
    throw std::invalid_argument(fmt::format(
        "wall {} no longer meets wall {} ({:.2f} m off its {})", w.guid, t.guid,
        u < 0.0 ? -u : u - wall_length(t), u < 0.0 ? "start" : "end"));
  const double h = t.thickness > 0.0 ? 0.5 * t.thickness : 0.0;
  double side = cross(dt, sub(at_end ? start_of(w) : end_of(w), start_of(t)));
  if (std::abs(side) < 1e-9)
    side = cross(dt, sub(at_end ? end_of(w) : start_of(w), start_of(t)));
  if (h == 0.0 || std::abs(side) < 1e-9)
    return x;
  const V n{-dt[1], dt[0]};
  const V face = add(start_of(t), mul(n, side > 0.0 ? h : -h));
  WallRecord f = t;
  set_axis(f, face, add(face, dt));
  return axes_intersection(w, f, min_angle_deg);
}

bool parse_end(const std::string &end) {
  if (end != "start" && end != "end")
    throw std::invalid_argument("wall end must be \"start\" or \"end\", not '" +
                                end + "'");
  return end == "end";
}

/// @p after's openings re-measured from @p before's start (their plan
/// place kept), into @p out; nothing when the start did not move.
void remeasure_openings(const WallRecord &before, const WallRecord &after,
                        const std::vector<OpeningRecord> &openings,
                        std::vector<OpeningRecord> &out) {
  if (before.x0 == after.x0 && before.y0 == after.y0)
    return;
  const double ds =
      dot(sub(start_of(before), start_of(after)), unit_dir(after));
  const double len = wall_length(after);
  for (auto o : openings) {
    if (o.wall != before.guid)
      continue;
    o.s_start = std::clamp(o.s_start + ds, 0.0, len);
    o.s_end = std::clamp(o.s_end + ds, 0.0, len);
    if (!(o.s_end > o.s_start))
      o.s_start = std::max(0.0, o.s_end - 0.01);
    out.push_back(std::move(o));
  }
}

/// A copy is a new hand-drawn row: no edit marks from the original.
std::string copy_metadata(const std::string &text) {
  if (text.empty())
    return text;
  auto meta = nlohmann::json::parse(text, nullptr, false);
  if (!meta.is_object())
    return text;
  meta.erase("edited");
  meta.erase("edited_from");
  return meta.empty() ? std::string() : meta.dump();
}

} // namespace

WallExtendResult extend_wall_to(const WallRecord &wall, const std::string &end,
                                const WallRecord &target,
                                const std::vector<OpeningRecord> &openings,
                                double min_angle_deg) {
  const bool at_end = parse_end(end);
  if (wall.guid == target.guid)
    throw std::invalid_argument("extend_wall_to: a wall cannot extend to "
                                "itself");
  if (wall.storey != target.storey)
    throw std::invalid_argument(
        fmt::format("extend_wall_to: walls {} and {} are on storeys {} and {}",
                    wall.guid, target.guid, wall.storey, target.storey));
  try {
    const V x = axes_intersection(wall, target, min_angle_deg);
    const double lt = wall_length(target);
    const double u = dot(sub(x, start_of(target)), unit_dir(target));
    const double snap = std::max(target.thickness, 0.1);
    WallExtendResult r;
    r.wall = wall;
    if (u > snap && u < lt - snap) {
      move_wall_end(r.wall, at_end,
                    tee_point(wall, at_end, target, min_angle_deg));
      r.join = {wall.guid, target.guid, "tee", end, "path"};
    } else {
      move_wall_end(r.wall, at_end, x);
      const bool t_end = u > 0.5 * lt;
      const V tp = t_end ? end_of(target) : start_of(target);
      if (norm(sub(tp, x)) > 1e-9) {
        WallRecord t = target;
        move_wall_end(t, t_end, x);
        r.target = std::move(t);
      }
      r.join = {wall.guid, target.guid, "corner", end, t_end ? "end" : "start"};
    }
    remeasure_openings(wall, r.wall, openings, r.openings);
    if (r.target)
      remeasure_openings(target, *r.target, openings, r.openings);
    return r;
  } catch (const std::invalid_argument &e) {
    throw std::invalid_argument(fmt::format("extend_wall_to: wall {} to {}: {}",
                                            wall.guid, target.guid, e.what()));
  }
}

PlanPoint PlanTransform::apply(PlanPoint p) const {
  double x = p[0], y = p[1];
  if (rotate_deg != 0.0) {
    const double a = rotate_deg * std::numbers::pi / 180.0;
    const double c = std::cos(a), s = std::sin(a);
    const double rx = x - pivot[0], ry = y - pivot[1];
    x = pivot[0] + c * rx - s * ry;
    y = pivot[1] + s * rx + c * ry;
  }
  return {x + dx, y + dy};
}

SelectionTransformResult
transform_selection(const std::vector<std::string> &guids,
                    const PlanTransform &t, bool copy,
                    const std::vector<WallRecord> &walls,
                    const std::vector<OpeningRecord> &openings,
                    const std::vector<SpaceRecord> &spaces,
                    const std::vector<WallJoinRecord> &joins,
                    const std::vector<SpaceBoundaryRecord> &boundaries,
                    const std::map<std::string, std::string> &new_guids) {
  const auto index = [](const auto &rows) {
    std::map<std::string, size_t> m;
    for (size_t i = 0; i < rows.size(); ++i)
      m.emplace(rows[i].guid, i);
    return m;
  };
  const auto wall_at = index(walls), opening_at = index(openings),
             space_at = index(spaces);
  std::vector<std::string> sel_walls, sel_openings, sel_spaces;
  std::set<std::string> seen, moved;
  for (const auto &g : guids) {
    if (!seen.insert(g).second)
      continue;
    if (wall_at.count(g)) {
      sel_walls.push_back(g);
      moved.insert(g);
    } else if (opening_at.count(g))
      sel_openings.push_back(g);
    else if (space_at.count(g))
      sel_spaces.push_back(g);
    else
      throw std::invalid_argument("transform_selection: " + g +
                                  " is no stored wall, opening or space");
  }

  SelectionTransformResult r;
  std::map<std::string, std::string> copy_of;
  const auto copy_guid = [&](const std::string &g) {
    const auto it = new_guids.find(g);
    std::string n = it != new_guids.end() && !it->second.empty()
                        ? it->second
                        : generate_guid();
    r.copies.emplace_back(g, n);
    copy_of[g] = n;
    return n;
  };
  const double rot = t.rotate_deg * std::numbers::pi / 180.0;
  const auto rotate_vec = [&](std::array<double, 2> v) {
    if (t.rotate_deg == 0.0)
      return v;
    const double c = std::cos(rot), s = std::sin(rot);
    return std::array<double, 2>{c * v[0] - s * v[1], s * v[0] + c * v[1]};
  };

  // Walls: the rigid motion (a moved wall's openings, top profile and
  // boundaries are measured along it, so they move with it as they are).
  std::vector<std::string> order;
  std::map<std::string, WallRecord> cur;   // the walls to put, as they are
  std::map<std::string, WallRecord> rigid; // ...as the motion left them
  for (const auto &g : sel_walls) {
    WallRecord w = walls[wall_at.at(g)];
    const PlanPoint p0 = t.apply({w.x0, w.y0}), p1 = t.apply({w.x1, w.y1});
    set_axis(w, p0, p1);
    if (copy) {
      w.guid = copy_guid(g);
      w.id = -1;
      w.source = "manual";
      w.metadata = copy_metadata(w.metadata);
    }
    order.push_back(w.guid);
    rigid[w.guid] = w;
    cur[w.guid] = std::move(w);
  }

  // Joins: kept, copied, or re-solved against the unmoved wall.
  const auto get = [&](const std::string &g) -> WallRecord & {
    if (!cur.count(g)) {
      cur[g] = walls[wall_at.at(g)];
      rigid[g] = cur[g];
      order.push_back(g);
    }
    return cur.at(g);
  };
  constexpr double kMinAngle = 5.0;
  for (const auto &j : joins) {
    const bool am = moved.count(j.wall_a) > 0, bm = moved.count(j.wall_b) > 0;
    if (copy) {
      r.joins.push_back(j);
      if (am && bm) {
        WallJoinRecord c = j;
        c.wall_a = copy_of.at(j.wall_a);
        c.wall_b = copy_of.at(j.wall_b);
        r.joins.push_back(std::move(c));
      }
      continue;
    }
    if (am == bm || !wall_at.count(am ? j.wall_b : j.wall_a)) {
      r.joins.push_back(j);
      continue;
    }
    const std::string &mg = am ? j.wall_a : j.wall_b;
    const std::string &ug = am ? j.wall_b : j.wall_a;
    const std::string &at_m = am ? j.at_a : j.at_b;
    const std::string &at_u = am ? j.at_b : j.at_a;
    try {
      if (j.kind == "collinear")
        throw std::invalid_argument("a collinear join cannot follow");
      if (at_m == "path" && at_u == "path")
        throw std::invalid_argument("a crossing cannot follow");
      WallRecord m = get(mg), u = get(ug);
      if (at_m != "path" && at_u != "path") {
        const V x = axes_intersection(m, u, kMinAngle);
        move_wall_end(m, at_m == "end", x);
        move_wall_end(u, at_u == "end", x);
      } else if (at_u == "path") {
        move_wall_end(m, at_m == "end",
                      tee_point(m, at_m == "end", u, kMinAngle));
      } else {
        move_wall_end(u, at_u == "end",
                      tee_point(u, at_u == "end", m, kMinAngle));
      }
      cur[mg] = std::move(m);
      cur[ug] = std::move(u);
      r.joins.push_back(j);
    } catch (const std::invalid_argument &e) {
      r.warnings.push_back(
          fmt::format("the {} join between wall {} and wall {} was dropped: {}",
                      j.kind, mg, ug, e.what()));
    }
  }
  for (const auto &g : order) {
    const WallRecord &w = cur.at(g);
    const WallRecord &was = rigid.at(g);
    // An unmoved wall a failed re-solve only looked at stays out.
    if (!moved.count(g) && !copy && w.x0 == was.x0 && w.y0 == was.y0 &&
        w.x1 == was.x1 && w.y1 == was.y1)
      continue;
    r.walls.push_back(w);
  }

  // Openings, by guid, last write wins.
  std::vector<std::string> o_order;
  std::map<std::string, OpeningRecord> o_out;
  const auto emit = [&](OpeningRecord o) {
    if (!o_out.count(o.guid))
      o_order.push_back(o.guid);
    o_out[o.guid] = std::move(o);
  };
  if (copy) {
    for (const auto &o : openings) {
      if (!moved.count(o.wall))
        continue;
      OpeningRecord c = o;
      c.guid = copy_guid(o.guid);
      c.wall = copy_of.at(o.wall);
      c.id = -1;
      c.source = "manual";
      c.source_instance_guid.clear();
      c.metadata = copy_metadata(c.metadata);
      emit(std::move(c));
    }
  } else {
    // An end a re-solved join moved re-measures that wall's openings.
    for (const auto &g : order) {
      std::vector<OpeningRecord> re;
      remeasure_openings(rigid.at(g), cur.at(g), openings, re);
      for (auto &o : re)
        emit(std::move(o));
    }
  }
  for (const auto &g : sel_openings) {
    const OpeningRecord &o = openings[opening_at.at(g)];
    if (moved.count(o.wall))
      continue; // it moves (or is copied) with its host
    if (!wall_at.count(o.wall)) {
      r.warnings.push_back(fmt::format(
          "opening {} was not moved: its host wall {} is not stored", g,
          o.wall));
      continue;
    }
    const WallRecord &was = walls[wall_at.at(o.wall)];
    const WallRecord &host = cur.count(o.wall) ? cur.at(o.wall) : was;
    const PlanPoint c = opening_centre(o, was);
    OpeningRecord n = o_out.count(g) ? o_out.at(g) : o;
    if (copy) {
      n = o;
      n.guid = copy_guid(g);
      n.id = -1;
      n.source = "manual";
      n.source_instance_guid.clear();
      n.metadata = copy_metadata(n.metadata);
    }
    rehost(n, host, t.apply(c));
    if (!copy && n.s_start == o.s_start && n.s_end == o.s_end &&
        !o_out.count(g))
      continue;
    emit(std::move(n));
  }
  for (const auto &g : o_order)
    r.openings.push_back(o_out.at(g));

  // Spaces.
  for (const auto &g : sel_spaces) {
    SpaceRecord s = spaces[space_at.at(g)];
    for (auto &p : s.polygon.outer)
      p = t.apply(p);
    for (auto &ring : s.polygon.holes)
      for (auto &p : ring)
        p = t.apply(p);
    for (auto &plane : s.ceiling_planes) {
      const PlanPoint at = t.apply({plane.at[0], plane.at[1]});
      plane.at[0] = at[0];
      plane.at[1] = at[1];
      plane.slope = rotate_vec(plane.slope); // the gradient turns with it
    }
    if (copy) {
      s.guid = copy_guid(g);
      s.id = -1;
      s.source = "manual";
      s.room_label = -1;
      s.metadata = copy_metadata(s.metadata);
      for (const auto &b : boundaries)
        if (b.space == g && copy_of.count(b.wall)) {
          SpaceBoundaryRecord c = b;
          c.space = s.guid;
          c.wall = copy_of.at(b.wall);
          r.boundaries.push_back(std::move(c));
        }
    }
    r.spaces.push_back(std::move(s));
  }
  return r;
}

} // namespace reusex::core
