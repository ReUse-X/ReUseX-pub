// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// fit_sloped_ceilings() (reusex-546): see segmentation/spaces.hpp.

#include "reusex/core/logging.hpp"
#include "reusex/core/sloped_ceiling.hpp"
#include "reusex/geometry/level_frame.hpp"
#include "reusex/segmentation/spaces.hpp"

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <fmt/format.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <numbers>
#include <numeric>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace reusex::geometry {
namespace {

constexpr double kCell = 0.1;        ///< raster of the space polygons [m]
constexpr double kCapStep = 0.02;    ///< cap search grid [m]
constexpr double kMaxRms = 0.05;     ///< a candidate's inliers off its plane
constexpr double kMinHeadroom = 0.1; ///< ceiling above the floor at a vertex
constexpr double kInf = std::numeric_limits<double>::infinity();

bool in_ring(const std::vector<core::PlanPoint> &r, double x, double y) {
  bool in = false;
  for (std::size_t i = 0, j = r.size() - 1; i < r.size(); j = i++) {
    const auto &a = r[i], &b = r[j];
    if ((a[1] > y) != (b[1] > y) &&
        x < (b[0] - a[0]) * (y - a[1]) / (b[1] - a[1]) + a[0])
      in = !in;
  }
  return in;
}

bool in_space(const core::SpaceRecord &s, double x, double y) {
  if (s.polygon.outer.size() < 3 || !in_ring(s.polygon.outer, x, y))
    return false;
  for (const auto &h : s.polygon.holes)
    if (h.size() >= 3 && in_ring(h, x, y))
      return false;
  return true;
}

/// One ceiling observation: a raster cell's centre and its ceiling height.
struct Obs {
  double x, y, z;
};

double lowest(const std::vector<core::CeilingPlane> &planes, double x,
              double y) {
  double z = kInf;
  for (const auto &p : planes)
    z = std::min(z, p.z_at(x, y));
  return z;
}

/// L1 error of min(cap, planes) over @p obs, minimised over the cap (a 2 cm
/// grid over the observed heights, or none). Returns {error, cap}; cap is
/// +inf when no cap fits best.
std::pair<double, double> best_cap(const std::vector<Obs> &obs,
                                   const std::vector<core::CeilingPlane> &pl) {
  double lo = kInf, hi = -kInf;
  std::vector<double> under(obs.size());
  for (std::size_t i = 0; i < obs.size(); ++i) {
    lo = std::min(lo, obs[i].z);
    hi = std::max(hi, obs[i].z);
    under[i] = lowest(pl, obs[i].x, obs[i].y);
  }
  const auto cost = [&](double cap) {
    double e = 0.0;
    for (std::size_t i = 0; i < obs.size(); ++i)
      e += std::abs(obs[i].z - std::min(cap, under[i]));
    return e;
  };
  std::pair<double, double> best{kInf, kInf};
  if (!pl.empty())
    best = {cost(kInf), kInf};
  for (double cap = lo; cap <= hi + 1e-9; cap += kCapStep)
    if (const double e = cost(cap); e < best.first - 1e-12)
      best = {e, cap};
  return best;
}

/// @p meta (a JSON object, or empty) with "ceiling_fit" set.
std::string with_fit(const std::string &meta, const nlohmann::json &fit) {
  nlohmann::json j = nlohmann::json::object();
  if (!meta.empty()) {
    try {
      j = nlohmann::json::parse(meta);
    } catch (const std::exception &) {
      j = nlohmann::json::object();
    }
    if (!j.is_object())
      j = nlohmann::json::object();
  }
  j["ceiling_fit"] = fit;
  return j.dump();
}

struct Candidate {
  int label = 0;
  core::CeilingPlane plane;
  std::vector<Eigen::Vector3d> pts; ///< inliers, level frame
};

// ── Eaves (reusex-ow8p) ─────────────────────────────────────────────────────

constexpr double kNeighbourGap = 0.6; ///< spaces this close are neighbours [m]
/// Eaves with no wall (reusex-wyi0): the outline's vertices within this of
/// its lowest contour line make the eave [m] ...
constexpr double kEaveBand = 0.3;
/// ... at least this long [m] ...
constexpr double kEaveMinLength = 0.75;
/// ... unless a wall along it lies within this of it (plus its half
/// thickness) over half its length [m].
constexpr double kEaveWallReach = 0.5;
/// ... and only knee-high: an outline the slope meets higher than this
/// above the floor is not at a knee wall (a bay, a gable end) [m].
constexpr double kEaveMaxKnee = 1.2;
/// ... merging stretches of one line this far apart [m].
constexpr double kEaveMergeGap = 2.0;
constexpr double kCutMin = 0.1;    ///< a plane cuts a cell this far down [m]
constexpr int kMinCutCells = 10;   ///< ... in at least this many cells
constexpr double kAgree = 0.15;    ///< an observation this close agrees [m]
constexpr double kMaxContra = 0.2; ///< share of cut cells seen above
constexpr double kMaxContraGain = 0.35;  ///< ... when it fits much better
constexpr double kEavesReach = 2.5;      ///< facade to its sloped space [m]
constexpr double kAlongSin = 0.342;      ///< sin 20 deg: along the contours
constexpr double kBin = 0.25;            ///< wall top bins along the axis [m]
constexpr double kRise = 0.35;           ///< a dormer rises this over the knee
constexpr double kMinDormer = 0.75;      ///< shortest dormer [m]
constexpr double kDormerCeilShare = 0.2; ///< ceiling seen over the bay
constexpr int kMinCheekPoints = 15;      ///< a cheek wall's points
constexpr double kKneeShare = 0.8;       ///< wall bins at the knee line

double seg_dist(const core::PlanPoint &p, const core::PlanPoint &a,
                const core::PlanPoint &b) {
  const double dx = b[0] - a[0], dy = b[1] - a[1];
  const double l2 = dx * dx + dy * dy;
  double t = l2 > 0.0 ? ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2 : 0.0;
  t = std::clamp(t, 0.0, 1.0);
  return std::hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy);
}

/// Distance from @p p to the ring @p r (0 inside it).
double ring_dist(const std::vector<core::PlanPoint> &r,
                 const core::PlanPoint &p) {
  if (r.size() < 3)
    return kInf;
  if (in_ring(r, p[0], p[1]))
    return 0.0;
  double d = kInf;
  for (std::size_t i = 0, j = r.size() - 1; i < r.size(); j = i++)
    d = std::min(d, seg_dist(p, r[j], r[i]));
  return d;
}

/// The gap between two rings (0 when they touch or overlap).
double ring_gap(const std::vector<core::PlanPoint> &a,
                const std::vector<core::PlanPoint> &b) {
  double d = kInf;
  for (const auto &p : a)
    d = std::min(d, ring_dist(b, p));
  for (const auto &p : b)
    d = std::min(d, ring_dist(a, p));
  return d;
}

bool same_plane(const core::CeilingPlane &a, const core::CeilingPlane &b) {
  return a.at == b.at && a.slope == b.slope;
}

double median(std::vector<double> v) {
  if (v.empty())
    return std::numeric_limits<double>::quiet_NaN();
  const auto m = v.begin() + static_cast<std::ptrdiff_t>(v.size() / 2);
  std::nth_element(v.begin(), m, v.end());
  return *m;
}

/// A level-frame point and its normal, for the facade measurements.
struct LevelPoint {
  Eigen::Vector3f q, n;
};

/// The storey's points in 0.5 m buckets.
struct Buckets {
  static constexpr double kSize = 0.5;
  std::unordered_map<std::int64_t, std::vector<std::uint32_t>> map;
  static std::int64_t key(int c, int r) {
    return (static_cast<std::int64_t>(c) << 32) ^ static_cast<std::uint32_t>(r);
  }
  void add(double x, double y, std::uint32_t i) {
    map[key(static_cast<int>(std::floor(x / kSize)),
            static_cast<int>(std::floor(y / kSize)))]
        .push_back(i);
  }
  /// Every point index in buckets touching [x0, x1] x [y0, y1].
  template <class F>
  void each(double x0, double y0, double x1, double y1, F &&f) const {
    const int c0 = static_cast<int>(std::floor(x0 / kSize));
    const int c1 = static_cast<int>(std::floor(x1 / kSize));
    const int r0 = static_cast<int>(std::floor(y0 / kSize));
    const int r1 = static_cast<int>(std::floor(y1 / kSize));
    for (int c = c0; c <= c1; ++c)
      for (int r = r0; r <= r1; ++r)
        if (const auto it = map.find(key(c, r)); it != map.end())
          for (const auto i : it->second)
            f(i);
  }
};

} // namespace

// ── Roof regularisation (reusex-p3gm) ───────────────────────────────────────

namespace {

/// The value at half the total weight of @p v (value, weight); the lower
/// one on a tie, so the result is one of the values.
double weighted_median(std::vector<std::pair<double, double>> v) {
  std::sort(v.begin(), v.end());
  double total = 0.0;
  for (const auto &x : v)
    total += x.second;
  double acc = 0.0;
  for (const auto &x : v) {
    acc += x.second;
    if (acc >= 0.5 * total - 1e-12)
      return x.first;
  }
  return v.back().first;
}

/// The area and area centroid of ring @p r (vertex mean when degenerate).
std::pair<double, Eigen::Vector2d>
ring_area_centroid(const std::vector<core::PlanPoint> &r) {
  double a2 = 0.0, cx = 0.0, cy = 0.0;
  for (std::size_t i = 0; i < r.size(); ++i) {
    const auto &p = r[i], &q = r[(i + 1) % r.size()];
    const double c = p[0] * q[1] - q[0] * p[1];
    a2 += c;
    cx += (p[0] + q[0]) * c;
    cy += (p[1] + q[1]) * c;
  }
  if (std::abs(a2) < 1e-12) {
    Eigen::Vector2d m = Eigen::Vector2d::Zero();
    for (const auto &p : r)
      m += Eigen::Vector2d(p[0], p[1]);
    return {0.0,
            r.empty() ? m : Eigen::Vector2d(m / static_cast<double>(r.size()))};
  }
  return {std::abs(0.5 * a2),
          Eigen::Vector2d(cx / (3.0 * a2), cy / (3.0 * a2))};
}

} // namespace

RoofRegulariseStats regularise_roof(std::vector<core::SpaceRecord> &spaces,
                                    const std::vector<std::size_t> &which,
                                    const SpacesOptions &o) {
  if (!(o.roof_azimuth_deg >= 0.0 && o.roof_azimuth_deg < 45.0) ||
      !(o.roof_pitch_round_deg >= 0.0) || !(o.roof_pitch_tol_deg >= 0.0) ||
      !(o.roof_merge_m >= 0.0) || !(o.roof_cap_merge_m >= 0.0) ||
      !(o.roof_cap_raise_m >= 0.0))
    throw std::invalid_argument(
        "regularise_roof: an option is out of range (roof_azimuth_deg in "
        "[0, 45), the others >= 0)");
  RoofRegulariseStats st;
  if (!o.roof_regularise)
    return st;
  constexpr double kDeg = std::numbers::pi / 180.0;

  // The distinct planes, each with the area and centroid of its spaces.
  struct Plane {
    core::CeilingPlane p;
    double w = 0.0;
    Eigen::Vector2d c = Eigen::Vector2d::Zero();
    double az = 0.0, pitch = 0.0; ///< regularised [deg]
    double d = 0.0;               ///< eaves line on the shared top
    std::size_t group = 0;
  };
  std::vector<Plane> planes;
  std::vector<std::pair<double, double>> caps; // z_ceiling, area
  std::vector<std::size_t> sloped;             // into spaces
  std::vector<double> areas;                   // of sloped
  for (const auto si : which) {
    const auto &s = spaces[si];
    if (s.ceiling_planes.empty() || s.polygon.outer.size() < 3)
      continue;
    auto [area, c] = ring_area_centroid(s.polygon.outer);
    if (s.area > 0.0)
      area = s.area;
    area = std::max(area, 1e-6);
    caps.emplace_back(s.z_ceiling, area);
    sloped.push_back(si);
    areas.push_back(area);
    for (const auto &p : s.ceiling_planes) {
      auto it = std::find_if(planes.begin(), planes.end(), [&](const Plane &x) {
        return same_plane(x.p, p);
      });
      if (it == planes.end()) {
        planes.push_back({p});
        it = planes.end() - 1;
      }
      it->w += area;
      it->c += area * c;
    }
  }
  if (planes.empty())
    return st;
  st.planes_in = static_cast<int>(planes.size());
  for (auto &x : planes)
    x.c /= x.w;

  // The storey's pitch and top.
  std::vector<std::pair<double, double>> pitches;
  for (auto &x : planes) {
    x.pitch = std::atan(std::hypot(x.p.slope[0], x.p.slope[1])) / kDeg;
    x.az = std::atan2(x.p.slope[1], x.p.slope[0]) / kDeg;
    pitches.emplace_back(x.pitch, x.w);
  }
  double pitch = weighted_median(pitches);
  if (o.roof_pitch_round_deg > 0.0)
    pitch = std::round(pitch / o.roof_pitch_round_deg) * o.roof_pitch_round_deg;
  pitch = std::clamp(pitch, 1.0, core::kMaxCeilingSlopeDeg);
  st.pitch_deg = pitch;
  // The reference height the eaves lines are measured at (planes of one
  // pitch keep their spacing at any height): the storey's median top.
  const double top = weighted_median(caps);
  st.top_z = top;

  // Directions onto the heaviest plane's grid; pitches onto the storey's.
  std::size_t heaviest = 0;
  for (std::size_t k = 1; k < planes.size(); ++k)
    if (planes[k].w > planes[heaviest].w + 1e-9)
      heaviest = k;
  const double phi0 = planes[heaviest].az;
  for (auto &x : planes) {
    const double r = x.az - phi0;
    const double k = std::round(r / 90.0);
    if (std::abs(r - 90.0 * k) <= o.roof_azimuth_deg)
      x.az = phi0 + 90.0 * k;
    // One direction, one value: (-180, 180].
    x.az = std::remainder(x.az, 360.0);
    if (x.az <= -180.0)
      x.az += 360.0;
    if (std::abs(x.pitch - pitch) <= o.roof_pitch_tol_deg)
      x.pitch = pitch;
    // Same height over its spaces' centroid, now on the new direction and
    // pitch: where it reaches the top.
    const Eigen::Vector2d u(std::cos(x.az * kDeg), std::sin(x.az * kDeg));
    const double t = std::tan(x.pitch * kDeg);
    const double zc = x.p.z_at(x.c.x(), x.c.y());
    x.d = u.dot(x.c) + (top - zc) / t;
  }

  // Sides (one direction, one pitch); within a side, planes whose eaves
  // lines follow each other within roof_merge_m are one.
  std::vector<std::size_t> order(planes.size());
  std::iota(order.begin(), order.end(), std::size_t{0});
  std::stable_sort(
      order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
        const auto &x = planes[a], &y = planes[b];
        return std::tie(x.az, x.pitch, x.d) < std::tie(y.az, y.pitch, y.d);
      });
  std::vector<core::CeilingPlane> merged;
  std::size_t i = 0;
  while (i < order.size()) {
    std::size_t j = i + 1;
    const auto &first = planes[order[i]];
    while (j < order.size()) {
      const auto &prev = planes[order[j - 1]], &cur = planes[order[j]];
      if (std::abs(cur.az - first.az) > 1e-9 ||
          std::abs(cur.pitch - first.pitch) > 1e-9)
        break;
      if (cur.d - prev.d > o.roof_merge_m)
        break;
      ++j;
    }
    double sw = 0.0, sd = 0.0;
    for (std::size_t k = i; k < j; ++k) {
      sw += planes[order[k]].w;
      sd += planes[order[k]].w * planes[order[k]].d;
    }
    const double d = sd / sw;
    const Eigen::Vector2d u(std::cos(first.az * kDeg),
                            std::sin(first.az * kDeg));
    const double t = std::tan(first.pitch * kDeg);
    core::CeilingPlane p;
    p.at = {u.x() * d, u.y() * d, top};
    p.slope = {t * u.x(), t * u.y()};
    for (std::size_t k = i; k < j; ++k)
      planes[order[k]].group = merged.size();
    merged.push_back(p);
    i = j;
  }
  st.planes_out = static_cast<int>(merged.size());
  {
    std::set<std::pair<long long, long long>> sides;
    for (const auto &x : planes)
      sides.emplace(std::llround(x.az * 1e6), std::llround(x.pitch * 1e6));
    st.sides = static_cast<int>(sides.size());
  }

  // One roof: sloped spaces that share a merged plane or stand side by side
  // (within kNeighbourGap); each roof gets one flat top, the area-weighted
  // median of its spaces' tops, for those within roof_cap_merge_m of it.
  const auto group_of = [&](const core::CeilingPlane &p) {
    const auto it =
        std::find_if(planes.begin(), planes.end(),
                     [&](const Plane &x) { return same_plane(x.p, p); });
    return it->group;
  };
  std::vector<std::size_t> roof(sloped.size());
  std::iota(roof.begin(), roof.end(), std::size_t{0});
  const auto find = [&](std::size_t a) {
    while (roof[a] != a)
      a = roof[a] = roof[roof[a]];
    return a;
  };
  for (std::size_t a = 0; a < sloped.size(); ++a)
    for (std::size_t b = a + 1; b < sloped.size(); ++b) {
      const auto &sa = spaces[sloped[a]], &sb = spaces[sloped[b]];
      bool shared = false;
      for (const auto &p : sa.ceiling_planes)
        for (const auto &q : sb.ceiling_planes)
          shared |= group_of(p) == group_of(q);
      if (shared ||
          ring_gap(sa.polygon.outer, sb.polygon.outer) <= kNeighbourGap)
        roof[find(a)] = find(b);
    }
  std::map<std::size_t, std::vector<std::pair<double, double>>> roof_caps;
  for (std::size_t a = 0; a < sloped.size(); ++a)
    roof_caps[find(a)].emplace_back(spaces[sloped[a]].z_ceiling, areas[a]);
  std::map<std::size_t, double> roof_top;
  for (const auto &[r, v] : roof_caps)
    roof_top[r] = weighted_median(v);

  // Each space onto the merged planes and its roof's top, unless that would
  // bring its ceiling down to its floor.
  for (std::size_t a = 0; a < sloped.size(); ++a) {
    const auto si = sloped[a];
    auto &s = spaces[si];
    const double my_top = roof_top[find(a)];
    auto trial = s;
    trial.ceiling_planes.clear();
    for (const auto &p : s.ceiling_planes) {
      const auto &q = merged[group_of(p)];
      if (std::none_of(trial.ceiling_planes.begin(), trial.ceiling_planes.end(),
                       [&](const auto &x) { return same_plane(x, q); }))
        trial.ceiling_planes.push_back(q);
    }
    // A top above the roof's comes down to it (a peak, a kink); one below
    // it is raised only a little: a lowered ceiling (a corridor's) is seen.
    if ((s.z_ceiling >= my_top && s.z_ceiling - my_top <= o.roof_cap_merge_m) ||
        (s.z_ceiling < my_top && my_top - s.z_ceiling <= o.roof_cap_raise_m))
      trial.z_ceiling = my_top;
    if (std::any_of(s.polygon.outer.begin(), s.polygon.outer.end(),
                    [&](const auto &v) {
                      return core::ceiling_at(trial, v[0], v[1]) <
                             s.z_floor + kMinHeadroom;
                    })) {
      debug("spaces: space {}: the regularised roof would meet its floor at a "
            "polygon vertex; it keeps its fitted ceiling",
            s.guid);
      ++st.reverted;
      continue;
    }
    const bool same =
        trial.z_ceiling == s.z_ceiling &&
        trial.ceiling_planes.size() == s.ceiling_planes.size() &&
        std::equal(trial.ceiling_planes.begin(), trial.ceiling_planes.end(),
                   s.ceiling_planes.begin(), [](const auto &a, const auto &b) {
                     return same_plane(a, b);
                   });
    if (same)
      continue;
    nlohmann::json fit = nlohmann::json::object();
    try {
      if (!s.metadata.empty()) {
        const auto j = nlohmann::json::parse(s.metadata);
        if (j.is_object() && j.contains("ceiling_fit") &&
            j.at("ceiling_fit").is_object())
          fit = j.at("ceiling_fit");
      }
    } catch (const std::exception &) {
    }
    fit["fitted"] = {{"planes", core::encode_ceiling_planes(s.ceiling_planes)},
                     {"z_ceiling", s.z_ceiling}};
    trial.metadata = with_fit(s.metadata, fit);
    s = std::move(trial);
    ++st.spaces;
  }
  return st;
}

SlopedCeilingStats fit_sloped_ceilings(
    std::vector<core::SpaceRecord> &spaces,
    const std::vector<core::StoreyRecord> &storeys, const Cloud &cloud,
    const CloudN &normals, const CloudL &plane_labels,
    const std::vector<int> &ceiling_labels,
    const std::vector<int> &ceiling_storeys, const SpacesOptions &o,
    const std::vector<core::WallRecord> *walls) {
  if (cloud.size() != normals.size() || cloud.size() != plane_labels.size())
    throw std::invalid_argument(fmt::format(
        "fit_sloped_ceilings: cloud has {} points, normals {}, plane labels {}",
        cloud.size(), normals.size(), plane_labels.size()));
  if (ceiling_labels.size() != ceiling_storeys.size())
    throw std::invalid_argument(
        "fit_sloped_ceilings: ceiling_labels and ceiling_storeys differ in "
        "size");
  SlopedCeilingStats stats;
  if (!o.fit_ceiling_planes || spaces.empty() || ceiling_labels.empty())
    return stats;
  const double cos_min =
      std::cos(o.ceiling_min_tilt_deg * std::numbers::pi / 180.0);
  const double cos_max =
      std::cos(o.ceiling_max_tilt_deg * std::numbers::pi / 180.0);

  for (const auto &st : storeys) {
    std::vector<std::size_t> mine;
    for (std::size_t i = 0; i < spaces.size(); ++i)
      if (spaces[i].storey == st.idx && spaces[i].polygon.outer.size() >= 3)
        mine.push_back(i);
    if (mine.empty())
      continue;
    const LevelFrame frame = LevelFrame::from_record(st);
    const Eigen::Matrix3d rot = frame.world_to_level.linear();

    // The storey's raster: cell -> space.
    double x0 = kInf, y0 = kInf, x1 = -kInf, y1 = -kInf;
    for (const auto i : mine)
      for (const auto &p : spaces[i].polygon.outer) {
        x0 = std::min(x0, p[0]);
        y0 = std::min(y0, p[1]);
        x1 = std::max(x1, p[0]);
        y1 = std::max(y1, p[1]);
      }
    const int W = static_cast<int>(std::ceil((x1 - x0) / kCell)) + 1;
    const int H = static_cast<int>(std::ceil((y1 - y0) / kCell)) + 1;
    if (W <= 0 || H <= 0 || static_cast<double>(W) * H > 5e7)
      continue;
    std::vector<int> owner(static_cast<std::size_t>(W) * H, -1);
    std::vector<std::vector<int>> cells_of(spaces.size());
    for (const auto i : mine) {
      const auto &s = spaces[i];
      double sx0 = kInf, sy0 = kInf, sx1 = -kInf, sy1 = -kInf;
      for (const auto &p : s.polygon.outer) {
        sx0 = std::min(sx0, p[0]);
        sy0 = std::min(sy0, p[1]);
        sx1 = std::max(sx1, p[0]);
        sy1 = std::max(sy1, p[1]);
      }
      const int c0 = std::max(0, static_cast<int>((sx0 - x0) / kCell));
      const int c1 = std::min(W - 1, static_cast<int>((sx1 - x0) / kCell));
      const int r0 = std::max(0, static_cast<int>((sy0 - y0) / kCell));
      const int r1 = std::min(H - 1, static_cast<int>((sy1 - y0) / kCell));
      for (int r = r0; r <= r1; ++r)
        for (int c = c0; c <= c1; ++c) {
          const auto k = static_cast<std::size_t>(r) * W + c;
          if (owner[k] >= 0)
            continue;
          if (in_space(s, x0 + (c + 0.5) * kCell, y0 + (r + 0.5) * kCell)) {
            owner[k] = static_cast<int>(i);
            cells_of[i].push_back(static_cast<int>(k));
          }
        }
    }
    const auto cell_of = [&](double x, double y) -> long {
      const int c = static_cast<int>(std::floor((x - x0) / kCell));
      const int r = static_cast<int>(std::floor((y - y0) / kCell));
      if (c < 0 || r < 0 || c >= W || r >= H)
        return -1;
      return static_cast<long>(r) * W + c;
    };

    // Candidates of this storey, by label.
    std::unordered_map<int, std::size_t> cand_of;
    std::vector<Candidate> cands;
    for (std::size_t k = 0; k < ceiling_labels.size(); ++k)
      if (ceiling_storeys[k] < 0 || ceiling_storeys[k] == st.idx)
        if (cand_of.emplace(ceiling_labels[k], cands.size()).second)
          cands.push_back({ceiling_labels[k], {}, {}});
    if (cands.empty())
      continue;

    // One pass: candidate inliers, and the horizontal ceiling samples.
    // samples[cell] = heights observed there.
    std::unordered_map<long, std::vector<float>> samples;
    std::vector<std::pair<long, std::size_t>> inlier_cells; // (cell, cand)
    // Every point of the storey, for the facade measurements (eaves).
    std::vector<LevelPoint> lpts;
    Buckets buckets;
    for (std::size_t i = 0; i < cloud.size(); ++i) {
      const auto &p = cloud.points[i];
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
        continue;
      const Eigen::Vector3d q = frame.to_level(Eigen::Vector3d(p.x, p.y, p.z));
      if (q.z() < st.z_floor - 0.5 || q.z() > st.z_ceiling + 3.0)
        continue;
      const long cell = cell_of(q.x(), q.y());
      const int sp = cell >= 0 ? owner[static_cast<std::size_t>(cell)] : -1;
      const auto it =
          cand_of.find(static_cast<int>(plane_labels.points[i].label));
      if (it != cand_of.end()) {
        cands[it->second].pts.push_back(q);
        if (sp >= 0 &&
            q.z() > spaces[static_cast<std::size_t>(sp)].z_floor + 0.3)
          inlier_cells.emplace_back(cell, it->second);
        // A flat ceiling plane's points are horizontal samples below.
      }
      const auto &nn = normals.points[i];
      const Eigen::Vector3d nl =
          rot * Eigen::Vector3d(nn.normal_x, nn.normal_y, nn.normal_z);
      if (walls != nullptr && std::isfinite(nl.z()) &&
          q.z() > st.z_floor - 0.3) {
        buckets.add(q.x(), q.y(), static_cast<std::uint32_t>(lpts.size()));
        lpts.push_back({q.cast<float>(), nl.cast<float>()});
      }
      if (sp < 0)
        continue;
      if (!std::isfinite(nl.z()) || std::abs(nl.z()) <= 0.8)
        continue;
      if (q.z() >=
          spaces[static_cast<std::size_t>(sp)].z_floor + o.height_ceiling_min_m)
        samples[cell].push_back(static_cast<float>(q.z()));
    }

    // Fit each candidate in the level frame; keep the tilted, planar ones.
    std::vector<bool> usable(cands.size(), false);
    for (std::size_t k = 0; k < cands.size(); ++k) {
      auto &c = cands[k];
      if (c.pts.size() < 30)
        continue;
      Eigen::Vector3d mean = Eigen::Vector3d::Zero();
      for (const auto &q : c.pts)
        mean += q;
      mean /= static_cast<double>(c.pts.size());
      Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
      for (const auto &q : c.pts)
        cov += (q - mean) * (q - mean).transpose();
      cov /= static_cast<double>(c.pts.size());
      const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
      Eigen::Vector3d n = es.eigenvectors().col(0);
      if (n.z() < 0.0)
        n = -n;
      const double rms = std::sqrt(std::max(es.eigenvalues()(0), 0.0));
      if (!(n.z() > cos_max) || !(n.z() < cos_min) || rms > kMaxRms)
        continue;
      c.plane.at = {mean.x(), mean.y(), mean.z()};
      c.plane.slope = {-n.x() / n.z(), -n.y() / n.z()};
      usable[k] = true;
      ++stats.candidates;
    }

    // Inliers count as ceiling observations too.
    std::map<std::pair<int, std::size_t>, std::set<long>> support;
    for (const auto &[cell, k] : inlier_cells) {
      if (!usable[k])
        continue;
      const int sp = owner[static_cast<std::size_t>(cell)];
      support[{sp, k}].insert(cell);
    }
    for (std::size_t k = 0; k < cands.size(); ++k) {
      if (!usable[k])
        continue;
      for (const auto &q : cands[k].pts) {
        const long cell = cell_of(q.x(), q.y());
        if (cell < 0 || owner[static_cast<std::size_t>(cell)] < 0)
          continue;
        const auto &s = spaces[static_cast<std::size_t>(
            owner[static_cast<std::size_t>(cell)])];
        if (q.z() > s.z_floor + 0.3)
          samples[cell].push_back(static_cast<float>(q.z()));
      }
    }

    const auto centre = [&](int cell) {
      return std::pair<double, double>{x0 + (cell % W + 0.5) * kCell,
                                       y0 + (cell / W + 0.5) * kCell};
    };
    // A cell's ceiling observation: the 80th percentile of its samples.
    const auto observed = [&](int cell) -> std::optional<double> {
      const auto it = samples.find(cell);
      if (it == samples.end() || it->second.empty())
        return std::nullopt;
      auto &v = it->second;
      const auto q = v.begin() + static_cast<std::ptrdiff_t>(
                                     0.8 * static_cast<double>(v.size() - 1));
      std::nth_element(v.begin(), q, v.end());
      return static_cast<double>(*q);
    };
    std::vector<std::vector<int>> taken_labels(spaces.size());
    for (const auto si : mine) {
      auto &s = spaces[si];
      const auto &cells = cells_of[si];
      if (cells.empty())
        continue;
      std::vector<Obs> obs;
      for (const int cell : cells)
        if (const auto z = observed(cell)) {
          const auto [x, y] = centre(cell);
          obs.push_back({x, y, *z});
        }
      if (static_cast<int>(obs.size()) < o.height_min_cells)
        continue;
      const double min_cells =
          std::max(static_cast<double>(o.ceiling_min_cells),
                   o.height_min_share * static_cast<double>(cells.size()));
      std::vector<std::size_t> pool;
      for (std::size_t k = 0; k < cands.size(); ++k) {
        const auto it = support.find({static_cast<int>(si), k});
        if (usable[k] && it != support.end() &&
            static_cast<double>(it->second.size()) >= min_cells)
          pool.push_back(k);
      }
      if (pool.empty())
        continue;

      std::vector<core::CeilingPlane> taken;
      const auto [flat_err, flat_cap] = best_cap(obs, taken);
      (void)flat_cap;
      double err = flat_err;
      std::vector<std::size_t> taken_k;
      while (!pool.empty()) {
        std::size_t best = pool.size();
        double best_err = err;
        for (std::size_t j = 0; j < pool.size(); ++j) {
          auto trial = taken;
          trial.push_back(cands[pool[j]].plane);
          const double e = best_cap(obs, trial).first;
          if (e < best_err - 1e-12) {
            best_err = e;
            best = j;
          }
        }
        if (best == pool.size() || best_err > (1.0 - o.ceiling_min_gain) * err)
          break;
        taken.push_back(cands[pool[best]].plane);
        taken_k.push_back(pool[best]);
        pool.erase(pool.begin() + static_cast<std::ptrdiff_t>(best));
        err = best_err;
      }
      // No plane may bring the ceiling down to the floor at a vertex: drop
      // the lowest one there until none does.
      double cap = kInf;
      while (!taken.empty()) {
        cap = best_cap(obs, taken).second;
        std::size_t worst = taken.size();
        for (const auto &v : s.polygon.outer) {
          if (std::min(cap, lowest(taken, v[0], v[1])) >=
              s.z_floor + kMinHeadroom)
            continue;
          double zl = kInf;
          for (std::size_t j = 0; j < taken.size(); ++j)
            if (const double z = taken[j].z_at(v[0], v[1]); z < zl) {
              zl = z;
              worst = j;
            }
          break;
        }
        if (worst == taken.size())
          break;
        warn("spaces: space {}: ceiling plane {} would meet the floor at a "
             "polygon vertex; dropped",
             s.guid, cands[taken_k[worst]].label);
        taken.erase(taken.begin() + static_cast<std::ptrdiff_t>(worst));
        taken_k.erase(taken_k.begin() + static_cast<std::ptrdiff_t>(worst));
      }
      if (taken.empty())
        continue;
      err = best_cap(obs, taken).first;
      // No cap: the highest point of the planes over the space.
      if (!std::isfinite(cap)) {
        cap = -kInf;
        for (const int cell : cells) {
          const int c = cell % W, r = cell / W;
          cap = std::max(cap, lowest(taken, x0 + (c + 0.5) * kCell,
                                     y0 + (r + 0.5) * kCell));
        }
      }
      if (!(cap > s.z_floor + kMinHeadroom))
        continue;
      s.ceiling_planes = taken;
      s.z_ceiling = cap;
      for (const auto k : taken_k)
        taken_labels[si].push_back(cands[k].label);
      double sum = 0.0;
      for (const int cell : cells) {
        const int c = cell % W, r = cell / W;
        sum += core::ceiling_at(s, x0 + (c + 0.5) * kCell,
                                y0 + (r + 0.5) * kCell) -
               s.z_floor;
      }
      const double mean_h = sum / static_cast<double>(cells.size());
      if (s.area > 0.0)
        s.volume = s.area * mean_h;
      nlohmann::json labels = nlohmann::json::array();
      for (const auto k : taken_k)
        labels.push_back(cands[k].label);
      const auto n = static_cast<double>(obs.size());
      s.metadata = with_fit(s.metadata, {{"cells", obs.size()},
                                         {"planes", labels},
                                         {"flat_l1_m", flat_err / n},
                                         {"fit_l1_m", err / n}});
      ++stats.spaces_sloped;
      stats.planes += static_cast<int>(taken.size());
      debug("spaces: space {}: {} sloped ceiling plane(s), cap {:.2f} m; "
            "mean ceiling error {:.3f} -> {:.3f} m over {} cells",
            s.guid, taken.size(), cap, flat_err / n, err / n, obs.size());
    }

    // ── Eaves (reusex-ow8p) ─────────────────────────────────────────────
    // The volume of space @p si over its raster, its dormers raised.
    const auto revolume = [&](std::size_t si,
                              const std::vector<core::Dormer> &dormers) {
      auto &s = spaces[si];
      const auto &cells = cells_of[si];
      if (cells.empty() || !(s.area > 0.0))
        return;
      double sum = 0.0;
      for (const int cell : cells) {
        const auto [x, y] = centre(cell);
        sum += core::ceiling_at(s, dormers, x, y) - s.z_floor;
      }
      s.volume = s.area * sum / static_cast<double>(cells.size());
    };

    // (1) A slope runs on into the space beside it: the strip between a
    // knee partition and the facade, its ceiling mostly unseen.
    std::vector<std::vector<core::CeilingPlane>> fitted(spaces.size());
    for (const auto si : mine)
      fitted[si] = spaces[si].ceiling_planes;
    for (const auto si : mine) {
      auto &s = spaces[si];
      const auto &cells = cells_of[si];
      if (cells.empty())
        continue;
      std::unordered_map<int, double> seen;
      for (const int cell : cells)
        if (const auto z = observed(cell))
          seen.emplace(cell, *z);
      const bool was_flat = s.ceiling_planes.empty();
      nlohmann::json from = nlohmann::json::array();
      for (const auto ni : mine) {
        if (ni == si || fitted[ni].empty() ||
            ring_gap(s.polygon.outer, spaces[ni].polygon.outer) > kNeighbourGap)
          continue;
        for (std::size_t pk = 0; pk < fitted[ni].size(); ++pk) {
          const auto &p = fitted[ni][pk];
          if (std::any_of(s.ceiling_planes.begin(), s.ceiling_planes.end(),
                          [&](const auto &q) { return same_plane(p, q); }))
            continue;
          // Where it cuts the ceiling, what is seen must not say otherwise:
          // few observations over it, and no worse a fit than before.
          int cut = 0, agree = 0, contra = 0, n_seen = 0;
          double err_now = 0.0, err_with = 0.0;
          for (const int cell : cells) {
            const auto [x, y] = centre(cell);
            const double zp = p.z_at(x, y);
            const double now = core::ceiling_at(s, x, y);
            if (!(zp < now - kCutMin))
              continue;
            ++cut;
            const auto it = seen.find(cell);
            if (it == seen.end())
              continue;
            ++n_seen;
            err_now += std::abs(it->second - now);
            err_with += std::abs(it->second - zp);
            if (it->second > zp + kAgree)
              ++contra;
            else if (it->second >= zp - kAgree)
              ++agree;
          }
          if (cut < kMinCutCells)
            continue;
          // A plane that fits what is seen much better than the ceiling
          // there now may leave more of it above (a beam, a light well).
          const double max_contra =
              err_with <= (1.0 - o.ceiling_min_gain) * err_now ? kMaxContraGain
                                                               : kMaxContra;
          if (contra > std::max(3.0, max_contra * (agree + contra)) ||
              err_with > err_now + 0.02 * n_seen) {
            debug("spaces: space {}: not carrying the slope of space {}: of "
                  "the {} cells it cuts, {} are seen ({} on it, {} above "
                  "it; mean error {:.3f} -> {:.3f} m)",
                  s.guid, spaces[ni].guid, cut, n_seen, agree, contra,
                  err_now / std::max(n_seen, 1),
                  err_with / std::max(n_seen, 1));
            continue;
          }
          auto trial = s;
          trial.ceiling_planes.push_back(p);
          if (std::any_of(s.polygon.outer.begin(), s.polygon.outer.end(),
                          [&](const auto &v) {
                            return core::ceiling_at(trial, v[0], v[1]) <
                                   s.z_floor + kMinHeadroom;
                          })) {
            debug("spaces: space {}: the slope of space {} would meet its "
                  "floor at a polygon vertex; not carried over",
                  s.guid, spaces[ni].guid);
            continue;
          }
          s.ceiling_planes.push_back(p);
          ++stats.extended;
          const int label =
              pk < taken_labels[ni].size() ? taken_labels[ni][pk] : -1;
          from.push_back({{"space", spaces[ni].guid},
                          {"plane", label},
                          {"cells_cut", cut},
                          {"seen_agree", agree},
                          {"seen_above", contra}});
          debug("spaces: space {}: carried the slope (plane {}) of space {} "
                "over {} cells ({} seen on it, {} above it)",
                s.guid, label, spaces[ni].guid, cut, agree, contra);
        }
      }
      if (from.empty())
        continue;
      if (was_flat) {
        ++stats.spaces_sloped;
        s.metadata = with_fit(s.metadata, {{"extended_from", from}});
      } else {
        nlohmann::json fit = nlohmann::json::object();
        try {
          const auto j = nlohmann::json::parse(s.metadata);
          if (j.is_object() && j.contains("ceiling_fit"))
            fit = j.at("ceiling_fit");
        } catch (const std::exception &) {
        }
        fit["extended_from"] = from;
        s.metadata = with_fit(s.metadata, fit);
      }
      stats.planes += static_cast<int>(from.size());
      revolume(si, {});
    }

    // (1b) The roof as few planes (reusex-p3gm): one top per roof, one
    // plane per side, one pitch. After the slopes are carried on (that
    // checks the fitted planes against what is seen), before the facades
    // are measured against them.
    if (o.roof_regularise) {
      const auto rr = regularise_roof(spaces, mine, o);
      for (const auto si : mine)
        if (!spaces[si].ceiling_planes.empty())
          revolume(si, {});
      stats.roof_planes_fitted += rr.planes_in;
      stats.roof_planes += rr.planes_out;
      if (rr.planes_in > 0)
        info("spaces: storey {}: roof regularised: {} fitted plane(s) -> {} "
             "({} side(s)) at {:.1f} deg, top {:.2f} m; {} space(s) changed, "
             "{} kept as fitted (the floor would be hit)",
             st.idx, rr.planes_in, rr.planes_out, rr.sides, rr.pitch_deg,
             rr.top_z, rr.spaces, rr.reverted);
    }

    // (2) The facades under each slope, measured: knee walls and dormers.
    if (walls == nullptr)
      continue;
    std::vector<std::pair<std::string, std::pair<double, double>>> dormer_runs;
    for (const auto si : mine) {
      auto &s = spaces[si];
      core::RoofEdges edges;
      for (const auto &p : s.ceiling_planes) {
        const Eigen::Vector2d g(p.slope[0], p.slope[1]);
        if (!(g.norm() >
              std::tan(o.ceiling_min_tilt_deg * std::numbers::pi / 180.0)))
          continue;
        for (const auto &w : *walls) {
          if (w.storey != st.idx)
            continue;
          const Eigen::Vector2d p0(w.x0, w.y0), p1(w.x1, w.y1);
          const double len = (p1 - p0).norm();
          if (!(len > 0.5) || !std::isfinite(len))
            continue;
          const Eigen::Vector2d d = (p1 - p0) / len;
          if (std::abs(d.dot(g) / g.norm()) > kAlongSin)
            continue;
          Eigen::Vector2d n(-d.y(), d.x()); // uphill, into the building
          if (n.dot(g) < 0.0)
            n = -n;
          const double gn = n.dot(g);
          const double half = w.thickness > 0.0 ? 0.5 * w.thickness : 0.0;
          const Eigen::Vector2d mid = p0 + d * (0.5 * len);
          const double knee_mid = core::ceiling_at(s, mid.x(), mid.y());
          if (!(knee_mid < s.z_ceiling - 0.3) ||
              !(knee_mid > s.z_floor + kMinHeadroom) ||
              ring_dist(s.polygon.outer, {mid.x(), mid.y()}) > kEavesReach)
            continue;
          // A facade: no space of the storey outside it, and inside it this
          // space or none (a bay no room reached), not a room of its own.
          const Eigen::Vector2d out = mid - n * (half + 0.3);
          const Eigen::Vector2d in = mid + n * (half + 0.3);
          if (std::any_of(mine.begin(), mine.end(), [&](std::size_t k) {
                return in_space(spaces[k], out.x(), out.y()) ||
                       (k != si && in_space(spaces[k], in.x(), in.y()));
              }))
            continue;
          if (std::find(edges.knee_walls.begin(), edges.knee_walls.end(),
                        w.guid) != edges.knee_walls.end())
            continue;

          // The wall's top per bin: the 95th percentile of its points.
          const double tol = w.thickness > 0.0 ? half + 0.1 : 0.25;
          const int nb = std::max(1, static_cast<int>(std::ceil(len / kBin)));
          std::vector<std::vector<double>> zs(static_cast<std::size_t>(nb));
          const Eigen::Vector2d lo = p0.cwiseMin(p1).array() - tol;
          const Eigen::Vector2d hi = p0.cwiseMax(p1).array() + tol;
          buckets.each(lo.x(), lo.y(), hi.x(), hi.y(), [&](std::uint32_t i) {
            const auto &lp = lpts[i];
            const Eigen::Vector2d r(lp.q.x() - p0.x(), lp.q.y() - p0.y());
            const double sv = r.dot(d), t = r.dot(n);
            if (sv < 0.0 || sv >= len || std::abs(t) > tol ||
                std::abs(lp.n.z()) >= 0.5 || lp.q.z() < s.z_floor + 0.1 ||
                lp.q.z() > s.z_ceiling + 0.3)
              return;
            zs[static_cast<std::size_t>(
                   std::min(nb - 1, static_cast<int>(sv / kBin)))]
                .push_back(lp.q.z());
          });
          std::vector<double> top(static_cast<std::size_t>(nb), kInf);
          std::vector<double> knee(static_cast<std::size_t>(nb));
          for (int b = 0; b < nb; ++b) {
            auto &v = zs[static_cast<std::size_t>(b)];
            const Eigen::Vector2d c = p0 + d * std::min(len, (b + 0.5) * kBin);
            knee[static_cast<std::size_t>(b)] =
                core::ceiling_at(s, c.x(), c.y());
            if (v.size() < 5)
              continue;
            const auto q =
                v.begin() + static_cast<std::ptrdiff_t>(
                                0.95 * static_cast<double>(v.size() - 1));
            std::nth_element(v.begin(), q, v.end());
            top[static_cast<std::size_t>(b)] = *q;
          }
          const auto rises = [&](int b) {
            const auto k = static_cast<std::size_t>(b);
            return std::isfinite(top[k]) && top[k] > knee[k] + kRise;
          };
          // Rising runs, bridging one unseen bin.
          std::vector<bool> dormer_bin(static_cast<std::size_t>(nb), false);
          for (int b = 0; b < nb;) {
            if (!rises(b)) {
              ++b;
              continue;
            }
            int e = b;
            while (e + 1 < nb &&
                   (rises(e + 1) ||
                    (e + 2 < nb &&
                     !std::isfinite(top[static_cast<std::size_t>(e + 1)]) &&
                     rises(e + 2))))
              e += rises(e + 1) ? 1 : 2;
            const double s0 = b * kBin, s1 = std::min(len, (e + 1) * kBin);
            const int next = e + 1;
            if (s1 - s0 < kMinDormer) {
              b = next;
              continue;
            }
            // Inside the stretch: this space or none, as for the wall.
            const Eigen::Vector2d run_in =
                p0 + d * (0.5 * (s0 + s1)) + n * (half + 0.3);
            if (std::any_of(mine.begin(), mine.end(), [&](std::size_t k) {
                  return k != si && in_space(spaces[k], run_in.x(), run_in.y());
                })) {
              b = next;
              continue;
            }
            std::vector<double> tops;
            for (int k = b; k <= e; ++k)
              if (std::isfinite(top[static_cast<std::size_t>(k)]))
                tops.push_back(top[static_cast<std::size_t>(k)]);
            const double zd0 = median(tops);
            const Eigen::Vector2d a0 = p0 + d * (0.5 * (s0 + s1));
            const double depth0 = (zd0 - p.z_at(a0.x(), a0.y())) / gn;
            if (!(depth0 > 0.2) || !(depth0 < 4.0) ||
                !(zd0 <= s.z_ceiling + 0.05)) {
              b = next;
              continue;
            }
            // Evidence: its flat ceiling over the bay, or a cheek wall.
            std::vector<double> ceil_z;
            std::set<std::pair<int, int>> ceil_cells;
            int cheek = 0;
            const Eigen::Vector2d c0 = p0 + d * s0, c1 = p0 + d * s1;
            const Eigen::Vector2d far0 = c0 + n * (depth0 + 0.2),
                                  far1 = c1 + n * (depth0 + 0.2);
            const Eigen::Vector2d blo =
                c0.cwiseMin(c1).cwiseMin(far0).cwiseMin(far1).array() - 0.3;
            const Eigen::Vector2d bhi =
                c0.cwiseMax(c1).cwiseMax(far0).cwiseMax(far1).array() + 0.3;
            buckets.each(
                blo.x(), blo.y(), bhi.x(), bhi.y(), [&](std::uint32_t i) {
                  const auto &lp = lpts[i];
                  const Eigen::Vector2d r(lp.q.x() - p0.x(), lp.q.y() - p0.y());
                  const double sv = r.dot(d), t = r.dot(n);
                  const double z = lp.q.z();
                  if (t < 0.05 || t > depth0 + 0.1)
                    return;
                  const double under = p.z_at(lp.q.x(), lp.q.y());
                  if (std::abs(lp.n.z()) > 0.8 && sv >= s0 && sv <= s1 &&
                      std::abs(z - zd0) < 0.25 && z > under + 0.1) {
                    ceil_z.push_back(z);
                    ceil_cells.emplace(static_cast<int>(std::floor(sv / kCell)),
                                       static_cast<int>(std::floor(t / kCell)));
                  } else if (std::abs(lp.n.z()) < 0.4 &&
                             std::abs(lp.n.x() * d.x() + lp.n.y() * d.y()) >
                                 0.8 &&
                             (std::abs(sv - s0) < 0.2 ||
                              std::abs(sv - s1) < 0.2) &&
                             z > under + 0.05 && z < zd0 + 0.1) {
                    ++cheek;
                  }
                });
            const double bay_cells =
                std::max(1.0, (s1 - s0) / kCell *
                                  std::max(depth0 - 0.05, kCell) / kCell);
            const bool ceiling_seen = static_cast<double>(ceil_cells.size()) >=
                                      kDormerCeilShare * bay_cells;
            if (!ceiling_seen && cheek < kMinCheekPoints) {
              debug("spaces: wall {} rises over the knee line of space {} "
                    "at s {:.2f}..{:.2f} (to {:.2f} m) but shows no dormer "
                    "ceiling or cheek; not a dormer",
                    w.guid, s.guid, s0, s1, zd0);
              b = next;
              continue;
            }
            const double zd =
                ceiling_seen && ceil_z.size() >= 20 ? median(ceil_z) : zd0;
            const double depth = (zd - p.z_at(a0.x(), a0.y())) / gn;
            const bool taken = std::any_of(
                dormer_runs.begin(), dormer_runs.end(), [&](const auto &r) {
                  return r.first == w.guid && r.second.first < s1 &&
                         r.second.second > s0;
                });
            if (!taken && depth > 0.2) {
              core::Dormer dm;
              dm.wall = w.guid;
              dm.s0 = s0;
              dm.s1 = s1;
              dm.z = zd;
              const Eigen::Vector2d i0 = c0 + n * depth, i1 = c1 + n * depth;
              dm.outline = {{c0.x(), c0.y()},
                            {c1.x(), c1.y()},
                            {i1.x(), i1.y()},
                            {i0.x(), i0.y()}};
              for (int k = b; k <= e; ++k)
                dormer_bin[static_cast<std::size_t>(k)] = true;
              dormer_runs.push_back({w.guid, {s0, s1}});
              info("spaces: space {}: dormer in wall {} over s {:.2f}..{:.2f} "
                   "m, its ceiling at {:.2f} m (knee line {:.2f} m), {:.2f} m "
                   "deep; ceiling seen in {} of {:.0f} cells, {} cheek points",
                   s.guid, w.guid, s0, s1, zd, p.z_at(a0.x(), a0.y()), depth,
                   ceil_cells.size(), bay_cells, cheek);
              edges.dormers.push_back(std::move(dm));
              ++stats.dormers;
            }
            b = next;
          }
          // A knee wall: its other stretches stay at the knee line.
          int seen_bins = 0, at_knee = 0;
          std::vector<double> gaps;
          for (int b = 0; b < nb; ++b) {
            const auto k = static_cast<std::size_t>(b);
            if (dormer_bin[k] || !std::isfinite(top[k]))
              continue;
            ++seen_bins;
            if (top[k] <= knee[k] + kRise) {
              ++at_knee;
              gaps.push_back(top[k] - knee[k]);
            }
          }
          const bool any_dormer =
              std::find(dormer_bin.begin(), dormer_bin.end(), true) !=
              dormer_bin.end();
          if ((seen_bins >= 2 &&
               at_knee >= kKneeShare * static_cast<double>(seen_bins)) ||
              (seen_bins == 0 && any_dormer)) {
            edges.knee_walls.push_back(w.guid);
            ++stats.knee_walls;
            debug("spaces: space {}: wall {} is a knee wall: {} of {} seen "
                  "bins at its knee line ({:.2f} m above the floor at its "
                  "middle; tops {:+.2f} m off it, median)",
                  s.guid, w.guid, at_knee, seen_bins, knee_mid - s.z_floor,
                  gaps.empty() ? 0.0 : median(gaps));
          }
        }
      }
      s.metadata = core::with_roof_edges(s.metadata, edges);
      if (!edges.dormers.empty())
        revolume(si, edges.dormers);
    }

    // (3) Eaves with no wall (reusex-wyi0): where a slope comes down to the
    // space's outline at a facade (no space outside), knee-high, with no
    // wall along it and no dormer bay behind it -- a knee wall seen too
    // little to be a wall, or hidden -- record the stretch; build_solids()
    // draws a knee wall there, its top on the slope. The stretch: the
    // outline's lowest contour line, over the vertices within kEaveBand of
    // it.
    std::vector<core::Dormer> storey_dormers;
    for (const auto si : mine)
      for (auto &d : core::roof_edges(spaces[si]).dormers)
        storey_dormers.push_back(std::move(d));
    for (const auto si : mine) {
      auto &s = spaces[si];
      auto edges = core::roof_edges(s);
      const std::size_t before = edges.eaves.size();
      for (const auto &p : s.ceiling_planes) {
        const Eigen::Vector2d g(p.slope[0], p.slope[1]);
        if (!(g.norm() >
              std::tan(o.ceiling_min_tilt_deg * std::numbers::pi / 180.0)))
          continue;
        {
          const Eigen::Vector2d u = g.normalized(); // uphill
          const Eigen::Vector2d c(-u.y(), u.x());   // along the contour
          double umin = std::numeric_limits<double>::infinity();
          for (const auto &v : s.polygon.outer)
            umin = std::min(umin, u.x() * v[0] + u.y() * v[1]);
          // The outline's edges along the contour within kEaveBand of its
          // lowest point, merged where they follow each other on one line
          // within kEaveMergeGap (the outline recedes from the facade over
          // furniture, a raster step): each stretch at its edges'
          // length-weighted line.
          struct Stretch {
            double t0, t1, uw, w;
          };
          std::vector<Stretch> runs;
          const auto &ring = s.polygon.outer;
          for (std::size_t e = 0; e < ring.size(); ++e) {
            const Eigen::Vector2d v0(ring[e][0], ring[e][1]);
            const auto &n1 = ring[(e + 1) % ring.size()];
            const Eigen::Vector2d v1(n1[0], n1[1]);
            const double len = (v1 - v0).norm();
            if (!(len > 1e-6) || u.dot(v0) > umin + kEaveBand ||
                u.dot(v1) > umin + kEaveBand ||
                std::abs((v1 - v0).dot(u)) > kAlongSin * len)
              continue;
            runs.push_back({std::min(c.dot(v0), c.dot(v1)),
                            std::max(c.dot(v0), c.dot(v1)),
                            0.5 * u.dot(v0 + v1) * len, len});
          }
          std::sort(runs.begin(), runs.end(),
                    [](const Stretch &a, const Stretch &b) {
                      return std::tie(a.t0, a.t1) < std::tie(b.t0, b.t1);
                    });
          std::vector<Stretch> stretches;
          for (const auto &r : runs)
            if (!stretches.empty() &&
                r.t0 <= stretches.back().t1 + kEaveMergeGap &&
                std::abs(r.uw / r.w - stretches.back().uw /
                                          stretches.back().w) <= kEaveBand) {
              auto &b = stretches.back();
              b.t1 = std::max(b.t1, r.t1);
              b.uw += r.uw;
              b.w += r.w;
            } else {
              stretches.push_back(r);
            }
          for (const auto &st_run : stretches) {
            const double t0 = st_run.t0, t1 = st_run.t1;
            const double uline = st_run.uw / st_run.w;
            const Eigen::Vector2d ea = u * uline + c * t0,
                                  eb = u * uline + c * t1;
            const Eigen::Vector2d emid = 0.5 * (ea + eb);
            const Eigen::Vector2d probe = emid + u * 0.05;
            const double knee = core::ceiling_at(s, probe.x(), probe.y());
            bool eave = t1 - t0 >= kEaveMinLength && knee < s.z_ceiling - 0.3 &&
                        knee > s.z_floor + kMinHeadroom &&
                        knee <= s.z_floor + kEaveMaxKnee &&
                        std::abs(p.z_at(probe.x(), probe.y()) - knee) < 1e-6;
            // Not where a dormer bay opens off the space.
            for (const auto &d : storey_dormers)
              if (eave &&
                  ring_dist(d.outline, {emid.x(), emid.y()}) <= kEaveWallReach)
                eave = false;
            // A facade: no space of the storey just outside it.
            for (const double f : {0.2, 0.5, 0.8}) {
              const Eigen::Vector2d q = ea + (eb - ea) * f - u * 0.3;
              if (eave && std::any_of(mine.begin(), mine.end(), [&](auto k) {
                    return in_space(spaces[k], q.x(), q.y());
                  }))
                eave = false;
            }
            // No wall along it (a knee wall, the facade, a partition).
            if (eave)
              for (const auto &w : *walls) {
                if (w.storey != st.idx)
                  continue;
                const Eigen::Vector2d p0(w.x0, w.y0), p1(w.x1, w.y1);
                const double len = (p1 - p0).norm();
                if (!(len > 1e-6))
                  continue;
                const Eigen::Vector2d d = (p1 - p0) / len;
                if (std::abs(d.dot(u)) > kAlongSin)
                  continue;
                const double half = w.thickness > 0.0 ? 0.5 * w.thickness : 0.0;
                const double off = u.dot(0.5 * (p0 + p1)) - uline;
                const double w0 = std::min(c.dot(p0), c.dot(p1)),
                             w1 = std::max(c.dot(p0), c.dot(p1));
                const double ov = std::min(t1, w1) - std::max(t0, w0);
                if (std::abs(off) <= kEaveWallReach + half &&
                    ov >= 0.5 * (t1 - t0)) {
                  eave = false;
                  break;
                }
              }
            if (eave) {
              edges.eaves.push_back({{ea.x(), ea.y()}, {eb.x(), eb.y()}});
              ++stats.eaves;
              debug("spaces: space {}: an eave with no wall, ({:.2f}, {:.2f}) "
                    "-> ({:.2f}, {:.2f}), {:.2f} m above the floor",
                    s.guid, ea.x(), ea.y(), eb.x(), eb.y(), knee - s.z_floor);
            }
          }
        }
      }
      if (edges.eaves.size() != before)
        s.metadata = core::with_roof_edges(s.metadata, edges);
    }
  }
  if (stats.spaces_sloped > 0)
    info("spaces: sloped ceilings: {} space(s) took {} plane(s) of {} tilted "
         "ceiling candidate(s), {} carried on from a neighbour; {} knee "
         "wall(s), {} dormer(s), {} eave(s) with no wall",
         stats.spaces_sloped, stats.planes, stats.candidates, stats.extended,
         stats.knee_walls, stats.dormers, stats.eaves);
  return stats;
}

} // namespace reusex::geometry
