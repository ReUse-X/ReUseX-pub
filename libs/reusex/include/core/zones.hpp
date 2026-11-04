// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// @file
/// Zones (reusex-pccw): what a zone measures, from its member spaces.

#include <reusex/core/solid_model.hpp>

#include <string>
#include <vector>

namespace reusex::core {

struct ZoneMeasures {
  double area = 0.0;   ///< sum of the members' floor areas [m^2]
  double volume = 0.0; ///< sum of the members' volumes [m^3]
  /// The members' storey idxs, ascending, each once.
  std::vector<int> storeys;
  /// Member guids @p spaces does not hold.
  std::vector<std::string> missing;
};

/// A member's area is its stored `area`, or its polygon's when that is not
/// computed (-1); its volume the stored `volume`, or area x (z_ceiling -
/// z_floor).
ZoneMeasures zone_measures(const ZoneRecord &zone,
                           const std::vector<SpaceRecord> &spaces);

} // namespace reusex::core
