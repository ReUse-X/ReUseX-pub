// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/core/space_identity.hpp"
#include "reusex/core/sloped_ceiling.hpp"

#include "reusex/core/guid.hpp"
#include "reusex/core/space_polygon.hpp"
#include "reusex/core/storey_lookup.hpp"

#include <Eigen/Geometry>
#include <fmt/format.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace reusex::core {
namespace {

/// Rounded to 5 cm, printed with two decimals; -0.00 prints as 0.00.
std::string coord(double v) {
  double r = std::round(v / 0.05) * 0.05;
  if (std::abs(r) < 1e-9)
    r = 0.0;
  return fmt::format("{:.2f}", r);
}

double area_of(const SpaceRecord &s) {
  return s.area > 0.0 ? s.area : space_polygon_area(s.polygon);
}

Eigen::Matrix4d to_matrix(const std::array<double, 16> &a) {
  Eigen::Matrix4d m;
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      m(r, c) = a[static_cast<size_t>(r * 4 + c)];
  return m;
}

/// Every point of @p p through @p m, each taken at level height @p z.
SpacePolygon map_polygon(const SpacePolygon &p, const Eigen::Matrix4d &m,
                         double z) {
  auto ring = [&](const std::vector<PlanPoint> &in) {
    std::vector<PlanPoint> out;
    out.reserve(in.size());
    for (const auto &q : in) {
      const Eigen::Vector4d w = m * Eigen::Vector4d(q[0], q[1], z, 1.0);
      out.push_back({w.x(), w.y()});
    }
    return out;
  };
  SpacePolygon out;
  out.outer = ring(p.outer);
  for (const auto &h : p.holes)
    out.holes.push_back(ring(h));
  return out;
}

const StoreyRecord *by_idx(const std::vector<StoreyRecord> &storeys, int idx) {
  for (const auto &s : storeys)
    if (s.idx == idx)
      return &s;
  return nullptr;
}

const StoreyRecord *by_guid(const std::vector<StoreyRecord> &storeys,
                            const std::string &guid) {
  for (const auto &s : storeys)
    if (s.guid == guid)
      return &s;
  return nullptr;
}

/// A footprint as horizontal runs: row k spans y in [k dy, (k + 1) dy) and
/// is sampled at its centre; each row holds the sorted, disjoint x
/// intervals inside the polygon (even-odd over every ring).
struct Footprint {
  long first = 0;
  std::vector<std::vector<std::pair<double, double>>> rows;
  double min_x = 0.0, max_x = 0.0;
};

Footprint footprint(const SpacePolygon &p, double dy) {
  Footprint f;
  constexpr double kInf = std::numeric_limits<double>::infinity();
  double y0 = kInf, y1 = -kInf;
  f.min_x = kInf;
  f.max_x = -kInf;
  for (const auto &q : p.outer) {
    y0 = std::min(y0, q[1]);
    y1 = std::max(y1, q[1]);
    f.min_x = std::min(f.min_x, q[0]);
    f.max_x = std::max(f.max_x, q[0]);
  }
  if (!(y1 > y0) || !std::isfinite(y0) || !std::isfinite(y1))
    return f;
  f.first = static_cast<long>(std::floor(y0 / dy));
  const long last = static_cast<long>(std::floor(y1 / dy));
  f.rows.resize(static_cast<size_t>(last - f.first + 1));
  std::vector<double> xs;
  for (long k = f.first; k <= last; ++k) {
    const double y = (static_cast<double>(k) + 0.5) * dy;
    xs.clear();
    auto cross = [&](const std::vector<PlanPoint> &ring) {
      for (size_t i = 0; i < ring.size(); ++i) {
        const auto &a = ring[i];
        const auto &b = ring[(i + 1) % ring.size()];
        // Half-open in y, so a vertex on the scanline counts once.
        if ((a[1] <= y) != (b[1] <= y))
          xs.push_back(a[0] + (y - a[1]) * (b[0] - a[0]) / (b[1] - a[1]));
      }
    };
    cross(p.outer);
    for (const auto &h : p.holes)
      cross(h);
    std::sort(xs.begin(), xs.end());
    auto &row = f.rows[static_cast<size_t>(k - f.first)];
    for (size_t i = 0; i + 1 < xs.size(); i += 2)
      if (xs[i + 1] > xs[i])
        row.emplace_back(xs[i], xs[i + 1]);
  }
  return f;
}

double overlap(const Footprint &a, const Footprint &b, double dy) {
  if (a.rows.empty() || b.rows.empty() || a.max_x <= b.min_x ||
      b.max_x <= a.min_x)
    return 0.0;
  const long lo = std::max(a.first, b.first);
  const long hi = std::min(a.first + static_cast<long>(a.rows.size()),
                           b.first + static_cast<long>(b.rows.size()));
  double len = 0.0;
  for (long k = lo; k < hi; ++k) {
    const auto &ra = a.rows[static_cast<size_t>(k - a.first)];
    const auto &rb = b.rows[static_cast<size_t>(k - b.first)];
    size_t i = 0, j = 0;
    while (i < ra.size() && j < rb.size()) {
      const double l = std::max(ra[i].first, rb[j].first);
      const double r = std::min(ra[i].second, rb[j].second);
      if (r > l)
        len += r - l;
      if (ra[i].second < rb[j].second)
        ++i;
      else
        ++j;
    }
  }
  return len * dy;
}

/// @p text as a JSON object; text that is not one is kept under
/// "previous_metadata".
nlohmann::json metadata_object(const std::string &text) {
  auto md = nlohmann::json::parse(text, nullptr, false);
  if (md.is_object())
    return md;
  nlohmann::json out = nlohmann::json::object();
  if (!text.empty())
    out["previous_metadata"] = text;
  return out;
}

/// The person's part of a stored row: name, kind, source, lock, and its
/// metadata keys under the new row's own.
void inherit(SpaceRecord &to, const SpaceRecord &from) {
  to.name = from.name;
  to.kind = from.kind;
  to.source = from.source;
  to.locked = from.locked;
  nlohmann::json merged = metadata_object(from.metadata);
  const auto own = nlohmann::json::parse(to.metadata, nullptr, false);
  if (own.is_object())
    for (const auto &[k, v] : own.items())
      merged[k] = v;
  merged.erase("unmatched");
  to.metadata = merged.empty() ? std::string() : merged.dump();
}

/// Named or locked by a person: the row's name and kind are theirs.
bool named(const SpaceRecord &s) { return s.locked || s.source == "manual"; }

} // namespace

std::string space_natural_key(const SpaceRecord &space) {
  const auto c = space_polygon_centroid(space.polygon);
  return fmt::format("spaces/{}/{},{}", space.storey, coord(c[0]), coord(c[1]));
}

double space_overlap_area(const SpacePolygon &a, const SpacePolygon &b,
                          double dy) {
  if (!(dy > 0.0))
    throw std::invalid_argument("space_overlap_area: dy must be > 0");
  return overlap(footprint(a, dy), footprint(b, dy), dy);
}

SpacePolygon space_polygon_world(const SpaceRecord &s,
                                 const StoreyRecord &storey) {
  return map_polygon(s.polygon, to_matrix(storey.level_transform).inverse(),
                     s.z_floor);
}

SpaceReframeResult
reframe_spaces(std::vector<SpaceRecord> &spaces,
               const std::vector<StoreyRecord> &old_storeys,
               const std::vector<StoreyRecord> &new_storeys) {
  SpaceReframeResult r;
  for (auto &s : spaces) {
    const StoreyRecord *from = by_idx(old_storeys, s.storey);
    if (!from) {
      r.lost.push_back(s.guid);
      continue;
    }
    const Eigen::Matrix4d from_world =
        to_matrix(from->level_transform).inverse();
    const auto c = space_polygon_centroid(s.polygon);
    const StoreyRecord *to = by_guid(new_storeys, from->guid);
    if (!to) {
      // Its storey is gone: the new storey whose band holds its floor.
      const Eigen::Vector4d w =
          from_world * Eigen::Vector4d(c[0], c[1], s.z_floor, 1.0);
      if (const auto idx = which_storey(new_storeys, {w.x(), w.y(), w.z()}))
        to = by_idx(new_storeys, *idx);
    }
    if (!to) {
      r.lost.push_back(s.guid);
      continue;
    }
    if (from->level_transform == to->level_transform && s.storey == to->idx) {
      ++r.unchanged;
      continue;
    }
    const Eigen::Matrix4d m = to_matrix(to->level_transform) * from_world;
    s.polygon = map_polygon(s.polygon, m, s.z_floor);
    const double zf = (m * Eigen::Vector4d(c[0], c[1], s.z_floor, 1.0)).z();
    const double zc = (m * Eigen::Vector4d(c[0], c[1], s.z_ceiling, 1.0)).z();
    s.z_floor = zf;
    s.z_ceiling = zc;
    // Sloped ceilings (reusex-546) move with the space.
    std::array<double, 16> rm{};
    for (int i = 0; i < 4; ++i)
      for (int j = 0; j < 4; ++j)
        rm[static_cast<std::size_t>(4 * i + j)] = m(i, j);
    for (auto &pl : s.ceiling_planes)
      pl = map_ceiling_plane(pl, rm);
    s.storey = to->idx;
    ++r.moved;
  }
  return r;
}

SpaceIdentityResult
assign_space_guids(std::vector<SpaceRecord> &fresh,
                   std::vector<SpaceBoundaryRecord> &boundaries,
                   const std::vector<SpaceRecord> &stored,
                   const std::vector<StoreyRecord> &storeys,
                   std::string_view project_guid,
                   const SpaceIdentityOptions &opt) {
  // Validates the namespace even when there is nothing to mint.
  (void)deterministic_guid(project_guid, "spaces");
  SpaceIdentityResult result;
  std::set<std::string> placeholders;
  for (const auto &s : fresh)
    if (!placeholders.insert(s.guid).second)
      throw std::invalid_argument(
          "assign_space_guids: two spaces share the placeholder " + s.guid);

  // Footprints and centroids in world x/y (in the level frame when the
  // storey is unknown).
  constexpr double kDy = 0.02; // [m]
  auto world = [&](const SpaceRecord &s) {
    const StoreyRecord *st = by_idx(storeys, s.storey);
    return st ? space_polygon_world(s, *st) : s.polygon;
  };
  std::vector<Footprint> ff(fresh.size()), sf(stored.size());
  std::vector<PlanPoint> fc(fresh.size()), sc(stored.size());
  std::vector<double> fa(fresh.size()), sa(stored.size());
  for (size_t i = 0; i < fresh.size(); ++i) {
    const auto w = world(fresh[i]);
    ff[i] = footprint(w, kDy);
    fc[i] = space_polygon_centroid(w);
    fa[i] = area_of(fresh[i]);
  }
  for (size_t j = 0; j < stored.size(); ++j) {
    const auto w = world(stored[j]);
    sf[j] = footprint(w, kDy);
    sc[j] = space_polygon_centroid(w);
    sa[j] = area_of(stored[j]);
  }

  struct Pair {
    double inter, dist;
    size_t i, j;
  };
  std::vector<Pair> pairs;
  for (size_t i = 0; i < fresh.size(); ++i)
    for (size_t j = 0; j < stored.size(); ++j) {
      if (fresh[i].storey != stored[j].storey)
        continue;
      const double d = std::hypot(fc[i][0] - sc[j][0], fc[i][1] - sc[j][1]);
      const double inter = overlap(ff[i], sf[j], kDy);
      if (inter <= 0.0 && d > opt.match_m)
        continue;
      pairs.push_back({inter, d, i, j});
    }

  std::map<std::string, std::string> renamed; // placeholder -> GUID
  std::vector<bool> fresh_done(fresh.size(), false);
  std::vector<bool> stored_used(stored.size(), false);
  auto take = [&](size_t i, size_t j) {
    renamed[fresh[i].guid] = stored[j].guid;
    fresh[i].guid = stored[j].guid;
    if (named(stored[j])) {
      inherit(fresh[i], stored[j]);
      ++result.named_matched;
    }
    fresh_done[i] = stored_used[j] = true;
    ++result.matched;
    if (stored[j].locked)
      ++result.locked_matched;
  };
  // Round 1 measures the overlap against the larger footprint (the same
  // room), round 2 against the smaller one (a split or merged room's
  // remnant). Greedy: largest share first, then nearest, then index.
  for (const bool against_larger : {true, false}) {
    std::vector<std::tuple<double, double, size_t, size_t>> order;
    for (const auto &p : pairs) {
      if (fresh_done[p.i] || stored_used[p.j])
        continue;
      const double a = fa[p.i], b = sa[p.j];
      const double hi = std::max(a, b), lo = std::min(a, b);
      if (!(lo > 0.0))
        continue;
      const double share = std::min(p.inter / (against_larger ? hi : lo), 1.0);
      const bool same_by_centroid = against_larger && p.dist <= opt.match_m &&
                                    lo >= opt.min_area_ratio * hi;
      if (share >= opt.min_overlap || same_by_centroid)
        order.emplace_back(-share, p.dist, p.i, p.j);
    }
    std::sort(order.begin(), order.end());
    for (const auto &[neg_share, d, i, j] : order) {
      (void)neg_share;
      (void)d;
      if (!fresh_done[i] && !stored_used[j])
        take(i, j);
    }
  }

  std::set<std::string> taken;
  for (const auto &s : stored)
    taken.insert(s.guid);
  for (size_t i = 0; i < fresh.size(); ++i) {
    if (fresh_done[i])
      continue;
    const std::string key = space_natural_key(fresh[i]);
    std::string guid = deterministic_guid(project_guid, key);
    for (int k = 2; taken.count(guid); ++k)
      guid = deterministic_guid(project_guid, fmt::format("{}#{}", key, k));
    renamed[fresh[i].guid] = guid;
    fresh[i].guid = guid;
    taken.insert(guid);
    ++result.minted;
  }
  for (auto &b : boundaries)
    if (const auto it = renamed.find(b.space); it != renamed.end())
      b.space = it->second;

  // A locked stored space nothing matched stays as stored, flagged.
  for (size_t j = 0; j < stored.size(); ++j) {
    if (stored_used[j] || !stored[j].locked)
      continue;
    SpaceRecord k = stored[j];
    auto md = metadata_object(k.metadata);
    md["unmatched"] = true;
    k.metadata = md.dump();
    result.kept.push_back(std::move(k));
  }
  return result;
}

} // namespace reusex::core
