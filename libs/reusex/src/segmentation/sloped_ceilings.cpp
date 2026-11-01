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
#include <set>
#include <stdexcept>
#include <string>
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

} // namespace

SlopedCeilingStats fit_sloped_ceilings(
    std::vector<core::SpaceRecord> &spaces,
    const std::vector<core::StoreyRecord> &storeys, const Cloud &cloud,
    const CloudN &normals, const CloudL &plane_labels,
    const std::vector<int> &ceiling_labels,
    const std::vector<int> &ceiling_storeys, const SpacesOptions &o) {
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
      if (spaces[i].storey == st.idx && !spaces[i].locked &&
          spaces[i].polygon.outer.size() >= 3)
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
      if (sp < 0)
        continue;
      const auto &nn = normals.points[i];
      const Eigen::Vector3d nl =
          rot * Eigen::Vector3d(nn.normal_x, nn.normal_y, nn.normal_z);
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

    for (const auto si : mine) {
      auto &s = spaces[si];
      const auto &cells = cells_of[si];
      if (cells.empty())
        continue;
      std::vector<Obs> obs;
      for (const int cell : cells) {
        const auto it = samples.find(cell);
        if (it == samples.end() || it->second.empty())
          continue;
        auto &v = it->second;
        const auto q = v.begin() + static_cast<std::ptrdiff_t>(
                                       0.8 * static_cast<double>(v.size() - 1));
        std::nth_element(v.begin(), q, v.end());
        const int c = cell % W, r = cell / W;
        obs.push_back({x0 + (c + 0.5) * kCell, y0 + (r + 0.5) * kCell,
                       static_cast<double>(*q)});
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
  }
  if (stats.spaces_sloped > 0)
    info("spaces: sloped ceilings: {} space(s) took {} plane(s) of {} tilted "
         "ceiling candidate(s)",
         stats.spaces_sloped, stats.planes, stats.candidates);
  return stats;
}

} // namespace reusex::geometry
