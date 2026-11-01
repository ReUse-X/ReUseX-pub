// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/core/sloped_ceiling.hpp"

#include <fmt/format.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

namespace reusex::core {
namespace {

using json = nlohmann::json;

bool finite(double v) { return std::isfinite(v); }

double number(const json &v, const char *what) {
  if (!v.is_number() || !std::isfinite(v.get<double>()))
    throw std::runtime_error(fmt::format("{} must be a finite number", what));
  return v.get<double>();
}

} // namespace

double ceiling_at(const SpaceRecord &space, double x, double y) {
  double z = space.z_ceiling;
  for (const auto &p : space.ceiling_planes)
    z = std::min(z, p.z_at(x, y));
  return z;
}

double wall_top_at(const WallRecord &wall, double s) {
  const double cap = wall.z_base + wall.height;
  const auto &t = wall.top;
  if (t.empty())
    return cap;
  double z = t.front()[1];
  if (s >= t.back()[0]) {
    z = t.back()[1];
  } else if (s > t.front()[0]) {
    const auto hi =
        std::upper_bound(t.begin(), t.end(), s,
                         [](double v, const PlanPoint &p) { return v < p[0]; });
    const auto lo = hi - 1;
    const double w = ((*hi)[0] - (*lo)[0]) > 0.0
                         ? (s - (*lo)[0]) / ((*hi)[0] - (*lo)[0])
                         : 0.0;
    z = (*lo)[1] + w * ((*hi)[1] - (*lo)[1]);
  }
  return std::min(cap, z);
}

std::string check_ceiling_planes(const SpaceRecord &space) {
  const double max_grad =
      std::tan(kMaxCeilingSlopeDeg * std::numbers::pi / 180.0);
  for (std::size_t i = 0; i < space.ceiling_planes.size(); ++i) {
    const auto &p = space.ceiling_planes[i];
    if (!finite(p.at[0]) || !finite(p.at[1]) || !finite(p.at[2]) ||
        !finite(p.slope[0]) || !finite(p.slope[1]))
      return fmt::format("ceiling_planes[{}] has a non-finite number", i);
    if (std::hypot(p.slope[0], p.slope[1]) > max_grad)
      return fmt::format("ceiling_planes[{}] is steeper than {} deg (slope "
                         "[{}, {}])",
                         i, kMaxCeilingSlopeDeg, p.slope[0], p.slope[1]);
  }
  if (space.ceiling_planes.empty())
    return {};
  for (const auto &v : space.polygon.outer) {
    const double z = ceiling_at(space, v[0], v[1]);
    if (z < space.z_floor - 1e-9)
      return fmt::format("the ceiling is at {:.3f} m at vertex ({}, {}), "
                         "below z_floor {:.3f} m",
                         z, v[0], v[1], space.z_floor);
  }
  return {};
}

std::string check_wall_top(const WallRecord &wall) {
  const auto &t = wall.top;
  if (t.empty())
    return {};
  if (t.size() < 2)
    return "'top' needs at least 2 points [s, z]";
  for (std::size_t i = 0; i < t.size(); ++i) {
    if (!finite(t[i][0]) || !finite(t[i][1]))
      return fmt::format("top[{}] has a non-finite number", i);
    if (i > 0 && !(t[i][0] > t[i - 1][0]))
      return fmt::format("top[{}]: s ({}) must exceed the previous s ({})", i,
                         t[i][0], t[i - 1][0]);
    if (!(t[i][1] > wall.z_base))
      return fmt::format("top[{}]: z ({}) must be above z_base ({})", i,
                         t[i][1], wall.z_base);
  }
  return {};
}

std::string encode_ceiling_planes(const std::vector<CeilingPlane> &planes) {
  if (planes.empty())
    return {};
  json arr = json::array();
  for (const auto &p : planes)
    arr.push_back({{"at", p.at}, {"slope", p.slope}});
  return arr.dump();
}

std::vector<CeilingPlane> decode_ceiling_planes(const std::string &text) {
  std::vector<CeilingPlane> out;
  if (text.empty())
    return out;
  json arr;
  try {
    arr = json::parse(text);
  } catch (const std::exception &e) {
    throw std::runtime_error(std::string("ceiling_planes: ") + e.what());
  }
  if (!arr.is_array())
    throw std::runtime_error("ceiling_planes: expected a JSON array");
  for (const auto &o : arr) {
    if (!o.is_object() || !o.contains("at") || !o.contains("slope") ||
        !o["at"].is_array() || o["at"].size() != 3 || !o["slope"].is_array() ||
        o["slope"].size() != 2)
      throw std::runtime_error(
          "ceiling_planes: each plane is {\"at\": [x, y, z], \"slope\": "
          "[dzdx, dzdy]}");
    CeilingPlane p;
    for (std::size_t k = 0; k < 3; ++k)
      p.at[k] = number(o["at"][k], "ceiling_planes: at");
    for (std::size_t k = 0; k < 2; ++k)
      p.slope[k] = number(o["slope"][k], "ceiling_planes: slope");
    out.push_back(p);
  }
  return out;
}

std::string encode_wall_top(const std::vector<PlanPoint> &top) {
  if (top.empty())
    return {};
  return json(top).dump();
}

std::vector<PlanPoint> decode_wall_top(const std::string &text) {
  std::vector<PlanPoint> out;
  if (text.empty())
    return out;
  json arr;
  try {
    arr = json::parse(text);
  } catch (const std::exception &e) {
    throw std::runtime_error(std::string("top_profile: ") + e.what());
  }
  if (!arr.is_array())
    throw std::runtime_error("top_profile: expected a JSON array");
  for (const auto &p : arr) {
    if (!p.is_array() || p.size() != 2)
      throw std::runtime_error("top_profile: each point is [s, z]");
    out.push_back(
        {number(p[0], "top_profile: s"), number(p[1], "top_profile: z")});
  }
  return out;
}

} // namespace reusex::core
