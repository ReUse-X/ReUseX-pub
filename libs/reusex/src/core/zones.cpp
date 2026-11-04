// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/zones.hpp"

#include "core/space_polygon.hpp"

#include <algorithm>
#include <map>
#include <set>

namespace reusex::core {

ZoneMeasures zone_measures(const ZoneRecord &zone,
                           const std::vector<SpaceRecord> &spaces) {
  std::map<std::string, const SpaceRecord *, std::less<>> by_guid;
  for (const auto &s : spaces)
    by_guid.emplace(s.guid, &s);
  ZoneMeasures m;
  std::set<int> storeys;
  for (const auto &g : zone.spaces) {
    const auto it = by_guid.find(g);
    if (it == by_guid.end()) {
      m.missing.push_back(g);
      continue;
    }
    const SpaceRecord &s = *it->second;
    const double area = s.area >= 0.0 ? s.area : space_polygon_area(s.polygon);
    m.area += area;
    m.volume += s.volume >= 0.0 ? s.volume
                                : area * std::max(0.0, s.z_ceiling - s.z_floor);
    storeys.insert(s.storey);
  }
  m.storeys.assign(storeys.begin(), storeys.end());
  return m;
}

} // namespace reusex::core
