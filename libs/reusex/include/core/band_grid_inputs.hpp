// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// @file band_grid_inputs.hpp
/// @brief Detecting a `rooms` cloud made from other storeys / walls /
/// openings than the project has now (reusex-14aj).
///
/// The band-rooms grid (`rooms --method band`) reads the storeys table and,
/// with its wall graph / door cuts on, the walls and openings tables. The
/// spaces stage rebuilds that grid and refuses when it no longer gives back
/// the `rooms` cloud's labels, which happens after any edit or re-detection
/// of those tables that moves a room. The rooms stage stamps a fingerprint
/// of each table it read into the cloud's `parameters` JSON
/// (`"band_grid_inputs": {"storeys": ..., "walls": ..., "openings": ...}`),
/// so `rux validate --stage spaces` can say "re-run rooms" before the spaces
/// stage spends a grid rebuild on finding out.
///
/// Only the fields the grid reads are fingerprinted (storeys: guid, idx,
/// z_floor, z_ceiling, level_transform; walls: guid, storey, axis, z_base,
/// height, thickness, seen_share, top; openings: guid, wall, kind, s range,
/// sill, head, state), so renaming, (un)locking or re-running a stage with
/// the same result leaves it unchanged. A changed fingerprint does not prove
/// the grid moves (a 1 mm edit may change no cell), so it is a warning; the
/// spaces stage's own label check decides.

#include <map>
#include <string>
#include <string_view>
#include <vector>

namespace reusex {
class ProjectDB;
}

namespace reusex::core {

/// The `parameters` JSON key the rooms stage stamps the fingerprints under.
inline constexpr std::string_view kBandGridInputsKey = "band_grid_inputs";

/// FNV-1a fingerprints (16 hex digits) of the grid's input tables, keyed
/// "storeys", and "walls" / "openings" when @p walls / @p openings.
std::map<std::string, std::string>
band_grid_input_fingerprints(const ProjectDB &db, bool walls, bool openings);

/// The tables ("storeys", "walls", "openings") whose fingerprint stamped in
/// the `rooms` cloud differs from today's, in that order. Empty when they
/// all match, and when there is no `rooms` cloud or no stamp (a leiden
/// cloud, or one made before the stamp existed: unknown, not stale).
std::vector<std::string> stale_band_grid_inputs(const ProjectDB &db);

} // namespace reusex::core
