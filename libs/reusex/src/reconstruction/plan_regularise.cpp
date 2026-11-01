// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/reconstruction/plan_regularise.hpp"

#include "reusex/geometry/transform_utils.hpp"

#include <Eigen/Geometry>
#include <fmt/format.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <numbers>
#include <numeric>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>

namespace reusex::geometry {
namespace {

constexpr double kDeg = std::numbers::pi / 180.0;
constexpr double kMoved = 1e-4; ///< below this a coordinate did not move [m]

Eigen::Vector2d tangent_of(const Eigen::Vector2d &d) { return {-d.y(), d.x()}; }

bool finite(double v) { return std::isfinite(v); }

/// Signed angle (deg) of @p a against @p b modulo 90, in (-45, 45].
double residual90(double a, double b) {
  double r = std::fmod(a - b, 90.0);
  if (r <= -45.0)
    r += 90.0;
  else if (r > 45.0)
    r -= 90.0;
  return r;
}

/// Level frame of storey k -> level frame of storey 0, in the plane.
struct PlaneMap {
  Eigen::Matrix2d linear = Eigen::Matrix2d::Identity();
  Eigen::Vector2d shift = Eigen::Vector2d::Zero();
  Eigen::Vector2d point(const Eigen::Vector2d &p) const {
    return linear * p + shift;
  }
  Eigen::Vector2d direction(const Eigen::Vector2d &v) const {
    return (linear * v).normalized();
  }
};

PlaneMap plane_map(const core::StoreyRecord &from,
                   const core::StoreyRecord &to) {
  PlaneMap m;
  if (from.level_transform == to.level_transform)
    return m;
  const Eigen::Affine3d A =
      to_affine(to.level_transform) * to_affine(from.level_transform).inverse();
  const double z = 0.5 * (from.z_floor + from.z_ceiling);
  const Eigen::Vector3d o = A * Eigen::Vector3d(0, 0, z);
  const Eigen::Vector3d ex = A * Eigen::Vector3d(1, 0, z) - o;
  const Eigen::Vector3d ey = A * Eigen::Vector3d(0, 1, z) - o;
  m.linear.col(0) = ex.head<2>();
  m.linear.col(1) = ey.head<2>();
  m.shift = o.head<2>();
  return m;
}

/// A wall seen from the common frame (storey 0's level frame).
struct Line {
  size_t g = 0, w = 0;                          ///< graph, wall
  int storey = 0;                               ///< StoreyRecord::idx
  Eigen::Vector2d n = Eigen::Vector2d::UnitX(); ///< its normal, common frame
  Eigen::Vector2d mid = Eigen::Vector2d::Zero();
  double half = 0.0;                                        ///< half its length
  double rel_lo = std::numeric_limits<double>::quiet_NaN(); ///< face - axis
  double rel_hi = std::numeric_limits<double>::quiet_NaN();
  double length() const { return 2.0 * half; }
};

Line line_of(const WallSegment &s, const PlaneMap &m, size_t g, size_t w,
             int storey) {
  Line l;
  l.g = g;
  l.w = w;
  l.storey = storey;
  const Eigen::Vector2d t = tangent_of(s.d);
  const Eigen::Vector2d mid = s.axis_offset * s.d + 0.5 * (s.s0 + s.s1) * t;
  l.n = m.direction(s.d);
  l.mid = m.point(mid);
  l.half = 0.5 * (s.s1 - s.s0);
  if (finite(s.face_lo))
    l.rel_lo = s.face_lo - s.axis_offset;
  if (finite(s.face_hi))
    l.rel_hi = s.face_hi - s.axis_offset;
  if (!finite(l.rel_lo) && !finite(l.rel_hi)) // no face: the axis is all
    l.rel_lo = l.rel_hi = 0.0;
  return l;
}

/// Sign of @p l's normal against @p n.
double sign_against(const Line &l, const Eigen::Vector2d &n) {
  return l.n.dot(n) >= 0.0 ? 1.0 : -1.0;
}

/// @p l's faces in the frame of normal @p n: {lo (toward -n), hi}; NaN for
/// a side not seen.
std::array<double, 2> faces_in(const Line &l, const Eigen::Vector2d &n) {
  const double c = n.dot(l.mid);
  if (sign_against(l, n) > 0)
    return {c + l.rel_lo, c + l.rel_hi};
  return {c - l.rel_hi, c - l.rel_lo};
}

/// [start, end] of @p l along tangent_of(n).
std::array<double, 2> extent_in(const Line &l, const Eigen::Vector2d &n) {
  const double s = tangent_of(n).dot(l.mid);
  return {s - l.half, s + l.half};
}

double overlap_of(const Line &a, const Line &b, const Eigen::Vector2d &n) {
  const auto ea = extent_in(a, n), eb = extent_in(b, n);
  return std::min(ea[1], eb[1]) - std::max(ea[0], eb[0]);
}

/// The nearest same-side face distance, or +inf when no side is shared.
double face_distance(const Line &a, const Line &b, const Eigen::Vector2d &n) {
  const auto fa = faces_in(a, n), fb = faces_in(b, n);
  double best = std::numeric_limits<double>::infinity();
  for (int s = 0; s < 2; ++s)
    if (finite(fa[s]) && finite(fb[s]))
      best = std::min(best, std::abs(fa[s] - fb[s]));
  return best;
}

/// The length-weighted mean of {value, weight}: the line that moves the
/// wall length least (in the least-squares sense).
double weighted_mean(const std::vector<std::pair<double, double>> &v) {
  double sum = 0.0, w = 0.0;
  for (const auto &x : v) {
    sum += x.first * x.second;
    w += x.second;
  }
  return w > 0.0 ? sum / w : v.front().first;
}

void add_rule(WallRegularisation &r, const char *rule) {
  if (std::find(r.rules.begin(), r.rules.end(), rule) == r.rules.end())
    r.rules.emplace_back(rule);
}

void check_options(const PlanRegulariseOptions &o) {
  const auto bad = [](const char *what) {
    return std::invalid_argument(
        fmt::format("regularise_walls: {} is out of range", what));
  };
  if (!(o.plan_angle_deg >= 0.0 && o.plan_angle_deg < 45.0))
    throw bad("plan_angle_deg");
  if (!(o.plan_collinear_m >= 0.0) || !(o.plan_stack_m >= 0.0) ||
      !(o.plan_collinear_gap_m >= 0.0) || !(o.plan_max_overlap_m >= 0.0) ||
      !(o.plan_stack_min_overlap_m >= 0.0))
    throw bad("a distance");
  if (!(o.plan_grid_deg >= 0.0 && o.plan_grid_deg < 45.0))
    throw bad("plan_grid_deg");
  if (!(o.plan_round_m >= 0.0))
    throw bad("plan_round_m");
}

} // namespace

PlanRegulariseResult
regularise_walls(std::vector<WallGraph> &graphs,
                 const std::vector<core::StoreyRecord> &storeys,
                 const PlanRegulariseOptions &opt) {
  if (graphs.size() != storeys.size())
    throw std::invalid_argument(
        fmt::format("regularise_walls: {} wall graph(s) for {} storey(s)",
                    graphs.size(), storeys.size()));
  check_options(opt);
  PlanRegulariseResult res;
  if (!opt.plan_regularise || graphs.empty())
    return res;

  std::vector<PlaneMap> maps;
  for (const auto &st : storeys)
    maps.push_back(plane_map(st, storeys.front()));

  // The measured axes, before anything moves.
  for (auto &g : graphs)
    for (auto &s : g.walls) {
      const Eigen::Vector2d t = tangent_of(s.d);
      s.regularised = {};
      s.regularised.measured_a = s.axis_offset * s.d + s.s0 * t;
      s.regularised.measured_b = s.axis_offset * s.d + s.s1 * t;
    }

  // 1. Grid: the orthogonal pair with the most wall length within
  //    plan_grid_deg; walls within plan_grid_deg of it turn onto it about
  //    their own midpoint (a family turns as one, so its walls stay
  //    parallel; collinear ones drift apart by length x angle, which step 2
  //    puts back on one line).
  if (opt.plan_grid_deg > 0.0) {
    struct Angle {
      double theta, weight;
    };
    std::vector<Angle> angles;
    for (size_t k = 0; k < graphs.size(); ++k)
      for (const auto &s : graphs[k].walls) {
        const Eigen::Vector2d n = maps[k].direction(s.d);
        double th = std::atan2(n.y(), n.x()) / kDeg;
        th = std::fmod(std::fmod(th, 90.0) + 90.0, 90.0);
        angles.push_back({th, std::max(0.0, s.s1 - s.s0)});
      }
    // Seed: the wall angle that gathers most length within plan_grid_deg,
    // ties broken by the length at that very angle (the heaviest family's
    // own orientation, not a compromise between storeys); refined as the
    // 4-theta mean of the walls within kGridTightDeg of it.
    constexpr double kGridTightDeg = 0.1;
    double best_w = -1.0, best_exact = -1.0, seed = 0.0;
    for (const auto &c : angles) {
      double w = 0.0, exact = 0.0;
      for (const auto &a : angles) {
        const double r = std::abs(residual90(a.theta, c.theta));
        if (r <= opt.plan_grid_deg)
          w += a.weight;
        if (r <= kGridTightDeg)
          exact += a.weight;
      }
      if (w > best_w + 1e-9 ||
          (std::abs(w - best_w) <= 1e-9 && exact > best_exact + 1e-9)) {
        best_w = w;
        best_exact = exact;
        seed = c.theta;
      }
    }
    double sx = 0.0, sy = 0.0;
    for (const auto &a : angles)
      if (std::abs(residual90(a.theta, seed)) <= kGridTightDeg) {
        sx += a.weight * std::cos(4.0 * a.theta * kDeg);
        sy += a.weight * std::sin(4.0 * a.theta * kDeg);
      }
    double phi = seed;
    if (std::hypot(sx, sy) > 0.0)
      phi = std::atan2(sy, sx) / kDeg / 4.0;
    phi = std::fmod(std::fmod(phi, 90.0) + 90.0, 90.0);
    res.grid_angle_deg = phi;
    for (size_t k = 0; k < graphs.size(); ++k)
      for (auto &s : graphs[k].walls) {
        const Eigen::Vector2d n = maps[k].direction(s.d);
        double th = std::atan2(n.y(), n.x()) / kDeg;
        const double r = residual90(th, phi);
        if (std::abs(r) > opt.plan_grid_deg || std::abs(r) < 1e-9)
          continue;
        const Eigen::Vector2d t = tangent_of(s.d);
        const Eigen::Vector2d mid =
            s.axis_offset * s.d + 0.5 * (s.s0 + s.s1) * t;
        const double half = 0.5 * (s.s1 - s.s0);
        const double lo = s.face_lo - s.axis_offset;
        const double hi = s.face_hi - s.axis_offset;
        s.d = Eigen::Rotation2Dd(-r * kDeg) * s.d;
        s.d.normalize();
        const Eigen::Vector2d t2 = tangent_of(s.d);
        s.axis_offset = s.d.dot(mid);
        s.s0 = t2.dot(mid) - half;
        s.s1 = t2.dot(mid) + half;
        s.face_lo = s.axis_offset + lo; // NaN stays NaN
        s.face_hi = s.axis_offset + hi;
        s.regularised.angle_delta_deg = -r;
        add_rule(s.regularised, "grid");
        ++res.walls_turned;
        res.max_turn_deg = std::max(res.max_turn_deg, std::abs(r));
      }
  }

  // 2. Lines: walls that continue each other on one storey ("collinear") or
  //    stand over each other on adjacent storeys ("stack") share one line.
  std::vector<Line> lines;
  for (size_t k = 0; k < graphs.size(); ++k)
    for (size_t w = 0; w < graphs[k].walls.size(); ++w)
      lines.push_back(
          line_of(graphs[k].walls[w], maps[k], k, w, storeys[k].idx));
  const size_t n = lines.size();
  const double cos_par = std::cos(opt.plan_angle_deg * kDeg);

  struct Candidate {
    double dist;
    size_t i, j;
  };
  std::vector<Candidate> cands;
  for (size_t i = 0; i < n; ++i)
    for (size_t j = i + 1; j < n; ++j) {
      const Line &a = lines[i], &b = lines[j];
      if (std::abs(a.n.dot(b.n)) < cos_par)
        continue;
      const double ov = overlap_of(a, b, a.n);
      double tol = 0.0;
      if (a.storey == b.storey) {
        if (ov > opt.plan_max_overlap_m || -ov > opt.plan_collinear_gap_m)
          continue;
        tol = opt.plan_collinear_m;
      } else if (opt.plan_stack && std::abs(a.storey - b.storey) == 1) {
        if (ov < opt.plan_stack_min_overlap_m)
          continue;
        tol = opt.plan_stack_m;
      } else {
        continue;
      }
      const double dist = face_distance(a, b, a.n);
      if (dist <= tol)
        cands.push_back({dist, i, j});
    }
  std::stable_sort(
      cands.begin(), cands.end(), [](const Candidate &x, const Candidate &y) {
        return std::tie(x.dist, x.i, x.j) < std::tie(y.dist, y.i, y.j);
      });

  std::vector<size_t> owner(n);
  std::iota(owner.begin(), owner.end(), size_t{0});
  std::vector<std::vector<size_t>> members(n);
  for (size_t i = 0; i < n; ++i)
    members[i] = {i};
  const double spread_tol =
      std::max(opt.plan_collinear_m, opt.plan_stack ? opt.plan_stack_m : 0.0);
  const auto valid = [&](const std::vector<size_t> &m) {
    const Eigen::Vector2d ref = lines[m.front()].n;
    for (size_t x = 0; x < m.size(); ++x)
      for (size_t y = x + 1; y < m.size(); ++y) {
        const Line &a = lines[m[x]], &b = lines[m[y]];
        if (std::abs(a.n.dot(b.n)) < cos_par)
          return false;
        // Two walls of one storey standing side by side are two walls.
        if (a.storey == b.storey &&
            overlap_of(a, b, ref) > opt.plan_max_overlap_m)
          return false;
      }
    for (int side = 0; side < 2; ++side) {
      double lo = std::numeric_limits<double>::infinity(), hi = -lo;
      for (const size_t i : m) {
        const double f = faces_in(lines[i], ref)[static_cast<size_t>(side)];
        if (!finite(f))
          continue;
        lo = std::min(lo, f);
        hi = std::max(hi, f);
      }
      if (hi - lo > spread_tol + 1e-12)
        return false;
    }
    return true;
  };
  for (const auto &c : cands) {
    const size_t a = owner[c.i], b = owner[c.j];
    if (a == b)
      continue;
    std::vector<size_t> merged = members[a];
    merged.insert(merged.end(), members[b].begin(), members[b].end());
    std::sort(merged.begin(), merged.end());
    if (!valid(merged))
      continue;
    for (const size_t i : members[b])
      owner[i] = a;
    members[a] = std::move(merged);
    members[b].clear();
  }

  // 3. Each line's target: the length-weighted mean of the faces on its
  //    better-seen side (optionally rounded); every member moves its face on
  //    that side there, its thickness unchanged. A member seen only from the
  //    other side goes to the mean of the other-side faces.
  std::vector<double> shift(n, 0.0);
  std::vector<char> on_line(n, 0);
  const auto round_to = [&](double v) {
    return opt.plan_round_m > 0.0
               ? std::round(v / opt.plan_round_m) * opt.plan_round_m
               : v;
  };
  for (size_t root = 0; root < n; ++root) {
    const auto &m = members[root];
    if (m.empty() || (m.size() < 2 && opt.plan_round_m <= 0.0))
      continue;
    size_t longest = m.front();
    for (const size_t i : m)
      if (lines[i].length() > lines[longest].length() + 1e-12)
        longest = i;
    const Eigen::Vector2d ref = lines[longest].n;
    std::array<double, 2> weight{0.0, 0.0}, lo{1e300, 1e300},
        hi{-1e300, -1e300};
    for (const size_t i : m) {
      const auto f = faces_in(lines[i], ref);
      for (size_t s = 0; s < 2; ++s)
        if (finite(f[s])) {
          weight[s] += lines[i].length();
          lo[s] = std::min(lo[s], f[s]);
          hi[s] = std::max(hi[s], f[s]);
        }
    }
    size_t side = 0;
    if (weight[1] > weight[0] + 1e-9 ||
        (std::abs(weight[1] - weight[0]) <= 1e-9 &&
         hi[1] - lo[1] < hi[0] - lo[0] - 1e-12))
      side = 1;
    const size_t other = 1 - side;
    std::vector<std::pair<double, double>> on_side; // face, length
    for (const size_t i : m) {
      const double f = faces_in(lines[i], ref)[side];
      if (finite(f))
        on_side.emplace_back(f, lines[i].length());
    }
    if (on_side.empty())
      continue;
    const double target = round_to(weighted_mean(on_side));
    std::vector<double> delta(m.size(),
                              std::numeric_limits<double>::quiet_NaN());
    for (size_t x = 0; x < m.size(); ++x) {
      const double f = faces_in(lines[m[x]], ref)[side];
      if (finite(f))
        delta[x] = target - f;
    }
    std::vector<std::pair<double, double>> on_other;
    for (size_t x = 0; x < m.size(); ++x) {
      const double f = faces_in(lines[m[x]], ref)[other];
      if (finite(f))
        on_other.emplace_back(f + (finite(delta[x]) ? delta[x] : 0.0),
                              lines[m[x]].length());
    }
    for (size_t x = 0; x < m.size(); ++x)
      if (!finite(delta[x])) {
        const double f = faces_in(lines[m[x]], ref)[other];
        delta[x] = finite(f) && !on_other.empty()
                       ? round_to(weighted_mean(on_other)) - f
                       : 0.0;
      }
    bool storeys_differ = false;
    for (size_t x = 0; x < m.size(); ++x) {
      shift[m[x]] = delta[x] * sign_against(lines[m[x]], ref);
      on_line[m[x]] = 1;
      storeys_differ |= lines[m[x]].storey != lines[m.front()].storey;
    }
    if (m.size() >= 2) {
      ++res.lines;
      res.stacked_lines += storeys_differ ? 1 : 0;
    }
    for (const size_t i : m) {
      auto &s = graphs[lines[i].g].walls[lines[i].w];
      if (std::abs(shift[i]) <= kMoved)
        continue;
      bool same_storey = false, other_storey = false;
      for (const size_t j : m)
        if (j != i) {
          same_storey |= lines[j].storey == lines[i].storey;
          other_storey |= lines[j].storey != lines[i].storey;
        }
      if (same_storey)
        add_rule(s.regularised, "collinear");
      if (other_storey)
        add_rule(s.regularised, "stack");
      if (opt.plan_round_m > 0.0)
        add_rule(s.regularised, "round");
    }
  }
  for (size_t i = 0; i < n; ++i) {
    if (!on_line[i] || std::abs(shift[i]) <= kMoved)
      continue;
    auto &s = graphs[lines[i].g].walls[lines[i].w];
    s.axis_offset += shift[i];
    s.face_lo += shift[i]; // NaN stays NaN
    s.face_hi += shift[i];
  }

  // 4. Corner and tee ends follow the moved lines: each joined end moves to
  //    the crossing of the two (new) axes, if that leaves the wall a run and
  //    moves the end by at most a snap's reach.
  const double reach =
      std::max({opt.plan_collinear_m, opt.plan_stack_m, opt.plan_round_m}) +
      1.0;
  for (auto &g : graphs)
    for (const auto &j : g.joins) {
      if (j.kind == WallJoinKind::collinear || j.a < 0 || j.b < 0 ||
          static_cast<size_t>(j.a) >= g.walls.size() ||
          static_cast<size_t>(j.b) >= g.walls.size())
        continue;
      auto &a = g.walls[static_cast<size_t>(j.a)];
      auto &b = g.walls[static_cast<size_t>(j.b)];
      Eigen::Matrix2d m;
      m.row(0) = a.d.transpose();
      m.row(1) = b.d.transpose();
      if (std::abs(m.determinant()) < 1e-6)
        continue;
      const Eigen::Vector2d x =
          m.inverse() * Eigen::Vector2d(a.axis_offset, b.axis_offset);
      const auto move = [&](WallSegment &s, WallEnd at) {
        if (at == WallEnd::path)
          return;
        const double v = x.dot(tangent_of(s.d));
        double &end = at == WallEnd::start ? s.s0 : s.s1;
        if (std::abs(v - end) <= kMoved || std::abs(v - end) > reach)
          return;
        const double s0 = at == WallEnd::start ? v : s.s0;
        const double s1 = at == WallEnd::end ? v : s.s1;
        if (s1 - s0 < 0.1)
          return;
        end = v;
        add_rule(s.regularised, "join");
        ++res.ends_rejoined;
      };
      move(a, j.at_a);
      move(b, j.at_b);
    }

  // 5. What moved, against the measured axes.
  double sq = 0.0;
  for (auto &g : graphs)
    for (auto &s : g.walls) {
      auto &r = s.regularised;
      const Eigen::Vector2d t = tangent_of(s.d);
      const Eigen::Vector2d a = s.axis_offset * s.d + s.s0 * t;
      const Eigen::Vector2d b = s.axis_offset * s.d + s.s1 * t;
      const Eigen::Vector2d old_mid = 0.5 * (r.measured_a + r.measured_b);
      r.offset_delta_m = s.d.dot(0.5 * (a + b)) - s.d.dot(old_mid);
      r.applied = (a - r.measured_a).norm() > kMoved ||
                  (b - r.measured_b).norm() > kMoved;
      if (!r.applied) {
        r.rules.clear();
        continue;
      }
      if (std::abs(r.offset_delta_m) > kMoved ||
          std::abs(r.angle_delta_deg) > 1e-9) {
        ++res.walls_moved;
        sq += r.offset_delta_m * r.offset_delta_m;
        res.max_shift_m = std::max(res.max_shift_m, std::abs(r.offset_delta_m));
      }
    }
  if (res.walls_moved > 0)
    res.rms_shift_m = std::sqrt(sq / res.walls_moved);
  return res;
}

// --------------------------------------------------------------------------
// Openings
// --------------------------------------------------------------------------

std::vector<core::WallRecord>
measured_walls(const std::vector<core::WallRecord> &walls) {
  std::vector<core::WallRecord> out = walls;
  for (auto &w : out) {
    if (w.metadata.find("regularised") == std::string::npos)
      continue;
    const auto meta = nlohmann::json::parse(w.metadata, nullptr, false);
    if (!meta.is_object() || !meta.contains("regularised"))
      continue;
    const auto &r = meta["regularised"];
    if (!r.is_object() || !r.contains("measured") || !r["measured"].is_object())
      continue;
    const auto &m = r["measured"];
    const auto num = [&](const char *k, double fallback) {
      return m.contains(k) && m[k].is_number() ? m[k].get<double>() : fallback;
    };
    w.x0 = num("x0", w.x0);
    w.y0 = num("y0", w.y0);
    w.x1 = num("x1", w.x1);
    w.y1 = num("y1", w.y1);
  }
  return out;
}

void openings_onto_walls(std::vector<core::OpeningRecord> &openings,
                         const std::vector<core::WallRecord> &measured,
                         const std::vector<core::WallRecord> &walls) {
  std::map<std::string, const core::WallRecord *> from, to;
  for (const auto &w : measured)
    from[w.guid] = &w;
  for (const auto &w : walls)
    to[w.guid] = &w;
  for (auto &o : openings) {
    const auto f = from.find(o.wall), t = to.find(o.wall);
    if (f == from.end() || t == to.end())
      continue;
    const Eigen::Vector2d a0(f->second->x0, f->second->y0),
        a1(f->second->x1, f->second->y1);
    const Eigen::Vector2d b0(t->second->x0, t->second->y0),
        b1(t->second->x1, t->second->y1);
    const double la = (a1 - a0).norm(), lb = (b1 - b0).norm();
    if (la <= 1e-9 || lb <= 1e-9 ||
        ((a0 - b0).norm() <= 1e-12 && (a1 - b1).norm() <= 1e-12))
      continue;
    const Eigen::Vector2d ua = (a1 - a0) / la, ub = (b1 - b0) / lb;
    const auto onto = [&](double s) {
      return std::clamp((a0 + s * ua - b0).dot(ub), 0.0, lb);
    };
    double s0 = onto(o.s_start), s1 = onto(o.s_end);
    if (s1 < s0)
      std::swap(s0, s1);
    if (s1 - s0 <= 1e-6)
      continue; // squeezed off the wall: keep it where it was measured
    o.s_start = s0;
    o.s_end = s1;
  }
}

namespace {

struct Window {
  size_t index = 0; ///< into the openings
  int storey = 0;
  const core::WallRecord *host = nullptr;
  double length = 0.0;       ///< the host's
  Eigen::Vector2d u, centre; ///< host direction and centre, common frame
};

double median_of(std::vector<double> v) {
  std::sort(v.begin(), v.end());
  const size_t m = v.size() / 2;
  return v.size() % 2 ? v[m] : 0.5 * (v[m - 1] + v[m]);
}

} // namespace

OpeningRegulariseResult
regularise_openings(std::vector<core::OpeningRecord> &openings,
                    const std::vector<core::WallRecord> &walls,
                    const std::vector<core::StoreyRecord> &storeys,
                    const OpeningRegulariseOptions &opt) {
  OpeningRegulariseResult res;
  if (!opt.plan_regularise || openings.empty() || storeys.empty())
    return res;
  if (!(opt.plan_angle_deg >= 0.0 && opt.plan_angle_deg < 45.0) ||
      !(opt.plan_facade_m >= 0.0) || !(opt.plan_column_m >= 0.0) ||
      !(opt.plan_row_m >= 0.0))
    throw std::invalid_argument(
        "regularise_openings: an option is out of range");

  std::map<std::string, const core::WallRecord *> host_of;
  for (const auto &w : walls)
    host_of[w.guid] = &w;
  std::map<int, PlaneMap> map_of;
  for (const auto &st : storeys)
    map_of[st.idx] = plane_map(st, storeys.front());

  struct Measured {
    double s_start, s_end, z_sill, z_head;
  };
  std::vector<Measured> measured;
  for (const auto &o : openings)
    measured.push_back({o.s_start, o.s_end, o.z_sill, o.z_head});
  std::vector<std::vector<std::string>> rules(openings.size());
  const auto add = [&](size_t i, const char *r) {
    auto &v = rules[i];
    if (std::find(v.begin(), v.end(), r) == v.end())
      v.emplace_back(r);
  };

  std::vector<Window> wins;
  for (size_t i = 0; i < openings.size(); ++i) {
    const auto &o = openings[i];
    if (o.kind != "window" || o.locked)
      continue;
    const auto it = host_of.find(o.wall);
    if (it == host_of.end())
      continue;
    const auto *w = it->second;
    const auto mit = map_of.find(w->storey);
    if (mit == map_of.end())
      continue;
    const Eigen::Vector2d p0(w->x0, w->y0), p1(w->x1, w->y1);
    const double len = (p1 - p0).norm();
    if (len <= 1e-9)
      continue;
    const Eigen::Vector2d u = (p1 - p0) / len;
    Window win;
    win.index = i;
    win.storey = w->storey;
    win.host = w;
    win.length = len;
    win.u = mit->second.direction(u);
    win.centre = mit->second.point(p0 + 0.5 * (o.s_start + o.s_end) * u);
    wins.push_back(win);
  }

  // Would [s0, s1] on the host of opening i stay on it and off its others?
  const auto fits = [&](size_t i, double s0, double s1, double len) {
    if (s0 < -1e-9 || s1 > len + 1e-9 || s1 - s0 <= 0.0)
      return false;
    for (size_t j = 0; j < openings.size(); ++j)
      if (j != i && openings[j].wall == openings[i].wall &&
          std::min(s1, openings[j].s_end) - std::max(s0, openings[j].s_start) >
              0.0)
        return false;
    return true;
  };

  // 1. Columns: windows of different storeys over each other on one facade.
  const double cos_par = std::cos(opt.plan_angle_deg * kDeg);
  struct Candidate {
    double dist;
    size_t i, j;
  };
  std::vector<Candidate> cands;
  for (size_t i = 0; i < wins.size(); ++i)
    for (size_t j = i + 1; j < wins.size(); ++j) {
      const auto &a = wins[i], &b = wins[j];
      if (a.storey == b.storey || std::abs(a.u.dot(b.u)) < cos_par)
        continue;
      const Eigen::Vector2d dv = b.centre - a.centre;
      const double along = std::abs(a.u.dot(dv));
      const double across = std::abs(tangent_of(a.u).dot(dv));
      if (along <= opt.plan_column_m && across <= opt.plan_facade_m)
        cands.push_back({along, i, j});
    }
  std::stable_sort(
      cands.begin(), cands.end(), [](const Candidate &x, const Candidate &y) {
        return std::tie(x.dist, x.i, x.j) < std::tie(y.dist, y.i, y.j);
      });
  std::vector<size_t> owner(wins.size());
  std::iota(owner.begin(), owner.end(), size_t{0});
  std::vector<std::vector<size_t>> members(wins.size());
  for (size_t i = 0; i < wins.size(); ++i)
    members[i] = {i};
  const auto valid = [&](const std::vector<size_t> &m) {
    const auto &r = wins[m.front()];
    double amin = 1e300, amax = -1e300, cmin = 1e300, cmax = -1e300;
    for (size_t x = 0; x < m.size(); ++x) {
      for (size_t y = x + 1; y < m.size(); ++y)
        if (wins[m[x]].storey == wins[m[y]].storey ||
            std::abs(wins[m[x]].u.dot(wins[m[y]].u)) < cos_par)
          return false;
      const Eigen::Vector2d dv = wins[m[x]].centre - r.centre;
      amin = std::min(amin, r.u.dot(dv));
      amax = std::max(amax, r.u.dot(dv));
      cmin = std::min(cmin, tangent_of(r.u).dot(dv));
      cmax = std::max(cmax, tangent_of(r.u).dot(dv));
    }
    return amax - amin <= opt.plan_column_m + 1e-12 &&
           cmax - cmin <= opt.plan_facade_m + 1e-12;
  };
  for (const auto &c : cands) {
    const size_t a = owner[c.i], b = owner[c.j];
    if (a == b)
      continue;
    std::vector<size_t> merged = members[a];
    merged.insert(merged.end(), members[b].begin(), members[b].end());
    std::sort(merged.begin(), merged.end());
    if (!valid(merged))
      continue;
    for (const size_t i : members[b])
      owner[i] = a;
    members[a] = std::move(merged);
    members[b].clear();
  }
  for (const auto &m : members) {
    if (m.size() < 2)
      continue;
    const auto &r = wins[m.front()];
    double along = 0.0;
    std::vector<double> widths;
    for (const size_t x : m) {
      along += r.u.dot(wins[x].centre - r.centre);
      const auto &o = openings[wins[x].index];
      widths.push_back(o.s_end - o.s_start);
    }
    along /= static_cast<double>(m.size());
    const double width = median_of(widths);
    bool any = false;
    for (const size_t x : m) {
      const auto &w = wins[x];
      auto &o = openings[w.index];
      const double sigma = w.u.dot(r.u) >= 0.0 ? 1.0 : -1.0;
      const double moved = along - r.u.dot(w.centre - r.centre);
      const double mid = 0.5 * (o.s_start + o.s_end) + sigma * moved;
      const double s0 = mid - 0.5 * width, s1 = mid + 0.5 * width;
      if (std::abs(s0 - o.s_start) <= kMoved &&
          std::abs(s1 - o.s_end) <= kMoved)
        continue;
      if (!fits(w.index, s0, s1, w.length)) {
        ++res.skipped;
        continue;
      }
      o.s_start = s0;
      o.s_end = s1;
      add(w.index, "column");
      any = true;
    }
    res.columns += any ? 1 : 0;
  }

  // 2. Rows: the windows of one storey whose sills (heads) lie within
  //    plan_row_m of each other share the median.
  std::map<int, std::vector<size_t>> by_storey;
  for (const auto &w : wins)
    by_storey[w.storey].push_back(w.index);
  for (const auto &[st, idx] : by_storey)
    for (const bool sill : {true, false}) {
      const auto z = [&](size_t i) -> double & {
        return sill ? openings[i].z_sill : openings[i].z_head;
      };
      std::vector<size_t> order = idx;
      std::stable_sort(order.begin(), order.end(),
                       [&](size_t a, size_t b) { return z(a) < z(b); });
      size_t begin = 0;
      while (begin < order.size()) {
        size_t end = begin + 1;
        while (end < order.size() &&
               z(order[end]) - z(order[begin]) <= opt.plan_row_m + 1e-12)
          ++end;
        if (end - begin >= 2) {
          std::vector<double> zs;
          for (size_t k = begin; k < end; ++k)
            zs.push_back(z(order[k]));
          const double target = median_of(zs);
          bool any = false;
          for (size_t k = begin; k < end; ++k) {
            const size_t i = order[k];
            if (std::abs(z(i) - target) <= kMoved)
              continue;
            const double other = sill ? openings[i].z_head : openings[i].z_sill;
            if ((sill && other - target < 0.1) ||
                (!sill && target - other < 0.1)) {
              ++res.skipped;
              continue;
            }
            z(i) = target;
            add(i, "row");
            any = true;
          }
          res.rows += any ? 1 : 0;
        }
        begin = end;
      }
    }

  // 3. The measured values go to the metadata of every moved opening.
  for (size_t i = 0; i < openings.size(); ++i) {
    if (rules[i].empty())
      continue;
    auto &o = openings[i];
    const auto &m = measured[i];
    const double move = std::max(
        {std::abs(o.s_start - m.s_start), std::abs(o.s_end - m.s_end),
         std::abs(o.z_sill - m.z_sill), std::abs(o.z_head - m.z_head)});
    res.max_shift_m = std::max(res.max_shift_m, move);
    ++res.openings_moved;
    nlohmann::json meta = nlohmann::json::object();
    if (!o.metadata.empty()) {
      auto parsed = nlohmann::json::parse(o.metadata, nullptr, false);
      if (parsed.is_object())
        meta = std::move(parsed);
    }
    meta["regularised"] = {{"rules", rules[i]},
                           {"measured",
                            {{"s_start", m.s_start},
                             {"s_end", m.s_end},
                             {"z_sill", m.z_sill},
                             {"z_head", m.z_head}}}};
    o.metadata = meta.dump();
  }
  return res;
}

} // namespace reusex::geometry
