// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/core/stairs.hpp"

#include <fmt/format.h>
#include <nlohmann/json.hpp>

#include <cmath>
#include <stdexcept>

namespace reusex::core {

using json = nlohmann::json;

namespace {

double run_length(const StairRun &r) {
  return std::hypot(r.end[0] - r.start[0], r.end[1] - r.start[1]);
}

bool finite2(const PlanPoint &p) {
  return std::isfinite(p[0]) && std::isfinite(p[1]);
}

json ring_json(const std::vector<PlanPoint> &r) {
  json a = json::array();
  for (const auto &p : r)
    a.push_back({p[0], p[1]});
  return a;
}

std::vector<PlanPoint> ring_from(const json &a) {
  std::vector<PlanPoint> r;
  for (const auto &p : a)
    r.push_back({p.at(0).get<double>(), p.at(1).get<double>()});
  return r;
}

} // namespace

int stair_riser_count(const StairRecord &stair) {
  int n = 0;
  for (const auto &r : stair.runs)
    n += r.risers;
  return n;
}

double stair_riser_height(const StairRecord &stair) {
  const int n = stair_riser_count(stair);
  return n > 0 ? (stair.z_top - stair.z_base) / n : -1.0;
}

double stair_going(const StairRecord &stair, std::size_t i) {
  if (i >= stair.runs.size())
    return -1.0;
  if (stair.tread_depth > 0.0)
    return stair.tread_depth;
  const auto &r = stair.runs[i];
  return r.risers > 1 ? run_length(r) / (r.risers - 1) : -1.0;
}

double stair_run_base(const StairRecord &stair, std::size_t i) {
  const double h = stair_riser_height(stair);
  int below = 0;
  for (std::size_t k = 0; k < i && k < stair.runs.size(); ++k)
    below += stair.runs[k].risers;
  return stair.z_base + below * std::max(0.0, h);
}

std::string check_stair(const StairRecord &s) {
  if (s.kind != "straight" && s.kind != "L" && s.kind != "U" &&
      s.kind != "spiral")
    return "kind must be straight | L | U | spiral, not '" + s.kind + "'";
  if (s.location != "left" && s.location != "centre" && s.location != "right")
    return "location must be left | centre | right, not '" + s.location + "'";
  if (s.runs.empty())
    return "a stair needs at least one run";
  for (std::size_t i = 0; i < s.runs.size(); ++i) {
    const auto &r = s.runs[i];
    if (r.risers < 1)
      return fmt::format("run {} has {} risers (>= 1)", i, r.risers);
    if (!(r.width > 0.0) || !std::isfinite(r.width))
      return fmt::format("run {} has width {} (> 0)", i, r.width);
    if (!finite2(r.start) || !finite2(r.end) ||
        (r.risers > 1 && !(run_length(r) > 1e-6)))
      return fmt::format("run {} has no walking line", i);
  }
  if (!std::isfinite(s.z_base) || !std::isfinite(s.z_top) ||
      !(s.z_top > s.z_base))
    return fmt::format("z_top ({}) must be above z_base ({})", s.z_top,
                       s.z_base);
  if (!std::isfinite(s.tread_depth) || !std::isfinite(s.nosing))
    return "tread_depth and nosing must be finite";
  for (std::size_t i = 0; i < s.landings.size(); ++i) {
    if (s.landings[i].outer.size() < 3)
      return fmt::format("landing {} has fewer than 3 points", i);
    for (const auto &p : s.landings[i].outer)
      if (!finite2(p))
        return fmt::format("landing {} has a coordinate that is not finite", i);
  }
  return {};
}

std::vector<std::string> stair_advice(const StairRecord &s) {
  std::vector<std::string> out;
  if (!check_stair(s).empty())
    return out;
  const double h = stair_riser_height(s);
  if (h > 0.20)
    out.push_back(fmt::format("riser {:.3f} m is above 0.20 m; add risers", h));
  else if (h > 0.18)
    out.push_back(fmt::format(
        "riser {:.3f} m is above 0.18 m (fine only inside a dwelling)", h));
  for (std::size_t i = 0; i < s.runs.size(); ++i) {
    const double g = stair_going(s, i);
    if (g < 0.0)
      continue;
    const double rule = 2.0 * h + g;
    if (rule < 0.59 || rule > 0.65)
      out.push_back(fmt::format("run {}: 2 x riser + going = {:.3f} m, "
                                "outside 0.59 .. 0.65 m",
                                i, rule));
    if (g < 0.25)
      out.push_back(
          fmt::format("run {}: going {:.3f} m is under 0.25 m", i, g));
    if (s.runs[i].width < 0.80)
      out.push_back(fmt::format("run {}: width {:.2f} m is under 0.80 m", i,
                                s.runs[i].width));
  }
  return out;
}

std::string encode_stair_runs(const std::vector<StairRun> &runs) {
  if (runs.empty())
    return {};
  json a = json::array();
  for (const auto &r : runs)
    a.push_back({{"start", {r.start[0], r.start[1]}},
                 {"end", {r.end[0], r.end[1]}},
                 {"width", r.width},
                 {"risers", r.risers}});
  return a.dump();
}

std::vector<StairRun> decode_stair_runs(const std::string &text) {
  std::vector<StairRun> out;
  if (text.empty())
    return out;
  try {
    for (const auto &j : json::parse(text)) {
      StairRun r;
      r.start = {j.at("start").at(0).get<double>(),
                 j.at("start").at(1).get<double>()};
      r.end = {j.at("end").at(0).get<double>(),
               j.at("end").at(1).get<double>()};
      r.width = j.at("width").get<double>();
      r.risers = j.at("risers").get<int>();
      out.push_back(r);
    }
  } catch (const json::exception &e) {
    throw std::runtime_error(std::string("stair runs: ") + e.what());
  }
  return out;
}

std::string encode_stair_landings(const std::vector<SpacePolygon> &landings) {
  if (landings.empty())
    return {};
  json a = json::array();
  for (const auto &l : landings) {
    json holes = json::array();
    for (const auto &h : l.holes)
      holes.push_back(ring_json(h));
    a.push_back({{"outer", ring_json(l.outer)}, {"holes", holes}});
  }
  return a.dump();
}

std::vector<SpacePolygon> decode_stair_landings(const std::string &text) {
  std::vector<SpacePolygon> out;
  if (text.empty())
    return out;
  try {
    for (const auto &j : json::parse(text)) {
      SpacePolygon p;
      p.outer = ring_from(j.at("outer"));
      for (const auto &h : j.value("holes", json::array()))
        p.holes.push_back(ring_from(h));
      out.push_back(std::move(p));
    }
  } catch (const json::exception &e) {
    throw std::runtime_error(std::string("stair landings: ") + e.what());
  }
  return out;
}

} // namespace reusex::core
