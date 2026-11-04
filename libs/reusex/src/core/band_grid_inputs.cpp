// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/band_grid_inputs.hpp"

#include "core/ProjectDB.hpp"
#include "core/model_edits.hpp"

#include <fmt/format.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cstdint>

namespace reusex::core {
namespace {

using json = nlohmann::json;

/// FNV-1a 64 over the rows' grid fields, sorted by guid.
template <typename R>
std::string fingerprint(std::vector<R> rows,
                        std::initializer_list<const char *> keys) {
  std::sort(rows.begin(), rows.end(),
            [](const R &a, const R &b) { return a.guid < b.guid; });
  json all = json::array();
  for (const auto &r : rows) {
    const json full = model_record_json(r);
    json kept = json::object();
    for (const char *k : keys)
      if (const auto it = full.find(k); it != full.end())
        kept[k] = *it;
    all.push_back(std::move(kept));
  }
  const std::string text = all.dump();
  uint64_t h = 1469598103934665603ull;
  for (const unsigned char c : text) {
    h ^= c;
    h *= 1099511628211ull;
  }
  return fmt::format("{:016x}", h);
}

} // namespace

std::map<std::string, std::string>
band_grid_input_fingerprints(const ProjectDB &db, bool walls, bool openings) {
  std::map<std::string, std::string> out;
  out["storeys"] = fingerprint(
      db.storeys(), {"guid", "idx", "z_floor", "z_ceiling", "level_transform"});
  if (walls)
    out["walls"] = fingerprint(db.walls(), {"guid", "storey", "x0", "y0", "x1",
                                            "y1", "z_base", "height",
                                            "thickness", "seen_share", "top"});
  if (openings)
    out["openings"] =
        fingerprint(db.openings(), {"guid", "wall", "kind", "s_start", "s_end",
                                    "z_sill", "z_head", "state"});
  return out;
}

std::vector<std::string> stale_band_grid_inputs(const ProjectDB &db) {
  std::vector<std::string> out;
  if (!db.has_point_cloud("rooms"))
    return out;
  const auto params = db.point_cloud_provenance("rooms").second;
  const auto j = json::parse(params, nullptr, false);
  if (!j.is_object())
    return out;
  const auto it = j.find(kBandGridInputsKey);
  if (it == j.end() || !it->is_object())
    return out;
  const auto now = band_grid_input_fingerprints(db, it->contains("walls"),
                                                it->contains("openings"));
  for (const char *k : {"storeys", "walls", "openings"}) {
    const auto stamped = it->find(k);
    if (stamped == it->end() || !stamped->is_string())
      continue;
    if (const auto n = now.find(k);
        n == now.end() || n->second != stamped->get<std::string>())
      out.emplace_back(k);
  }
  return out;
}

} // namespace reusex::core
