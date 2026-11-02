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

RoofEdges roof_edges(const SpaceRecord &space) {
  RoofEdges out;
  if (space.metadata.empty())
    return out;
  try {
    const json j = json::parse(space.metadata);
    if (!j.is_object() || !j.contains("roof") || !j.at("roof").is_object())
      return out;
    const json &r = j.at("roof");
    if (r.contains("knee_walls"))
      for (const auto &g : r.at("knee_walls"))
        out.knee_walls.push_back(g.get<std::string>());
    if (r.contains("dormers"))
      for (const auto &d : r.at("dormers")) {
        Dormer x;
        x.wall = d.at("wall").get<std::string>();
        x.s0 = number(d.at("s").at(0), "a dormer's s0");
        x.s1 = number(d.at("s").at(1), "a dormer's s1");
        x.z = number(d.at("z"), "a dormer's z");
        for (const auto &p : d.at("outline"))
          x.outline.push_back({number(p.at(0), "a dormer outline x"),
                               number(p.at(1), "a dormer outline y")});
        if (x.outline.size() >= 3 && x.s1 > x.s0)
          out.dormers.push_back(std::move(x));
      }
  } catch (const std::exception &) {
    return {};
  }
  return out;
}

std::string with_roof_edges(const std::string &metadata,
                            const RoofEdges &edges) {
  json j = json::object();
  if (!metadata.empty()) {
    try {
      j = json::parse(metadata);
    } catch (const std::exception &) {
      j = json::object();
    }
    if (!j.is_object())
      j = json::object();
  }
  if (edges.empty()) {
    j.erase("roof");
    return j.empty() && metadata.empty() ? std::string() : j.dump();
  }
  json dormers = json::array();
  for (const auto &d : edges.dormers) {
    json outline = json::array();
    for (const auto &p : d.outline)
      outline.push_back({p[0], p[1]});
    dormers.push_back({{"wall", d.wall},
                       {"s", {d.s0, d.s1}},
                       {"z", d.z},
                       {"outline", outline}});
  }
  j["roof"] = {{"knee_walls", edges.knee_walls}, {"dormers", dormers}};
  return j.dump();
}

double ceiling_at(const SpaceRecord &space, const std::vector<Dormer> &dormers,
                  double x, double y) {
  double z = ceiling_at(space, x, y);
  for (const auto &d : dormers) {
    if (!(d.z > z))
      continue;
    bool in = false;
    const auto &r = d.outline;
    for (std::size_t i = 0, k = r.size() - 1; i < r.size(); k = i++)
      if ((r[i][1] > y) != (r[k][1] > y) &&
          x < (r[k][0] - r[i][0]) * (y - r[i][1]) / (r[k][1] - r[i][1]) +
                  r[i][0])
        in = !in;
    if (in)
      z = d.z;
  }
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

CeilingPlane map_ceiling_plane(const CeilingPlane &plane,
                               const std::array<double, 16> &m) {
  const auto on = [&](double dx, double dy) {
    const double x = plane.at[0] + dx, y = plane.at[1] + dy;
    const double z = plane.z_at(x, y);
    std::array<double, 3> q{};
    for (std::size_t r = 0; r < 3; ++r)
      q[r] = m[4 * r] * x + m[4 * r + 1] * y + m[4 * r + 2] * z + m[4 * r + 3];
    return q;
  };
  const auto p0 = on(0, 0), p1 = on(1, 0), p2 = on(0, 1);
  const std::array<double, 3> u{p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]};
  const std::array<double, 3> v{p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]};
  const double nx = u[1] * v[2] - u[2] * v[1];
  const double ny = u[2] * v[0] - u[0] * v[2];
  const double nz = u[0] * v[1] - u[1] * v[0];
  CeilingPlane out = plane;
  out.at = p0;
  if (std::abs(nz) > 1e-12)
    out.slope = {-nx / nz, -ny / nz};
  return out;
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
