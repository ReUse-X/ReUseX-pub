// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// @file
/// Stable identity of spaces across re-runs of the spaces and storeys stages
/// (solid-model WP4, reusex-z1v.6.3; names and locks across a pose change,
/// reusex-djf).
///
/// A re-detected space takes the GUID of the stored space of the same storey
/// whose footprint it overlaps, compared in WORLD coordinates (each polygon
/// taken out of its storey's level frame): first pairs whose overlap is at
/// least `min_overlap` of the LARGER footprint (the same room), then, among
/// what is left, pairs whose overlap is at least `min_overlap` of the
/// SMALLER one (one room split or merged: the remnant keeps the identity).
/// The centroid rule of z1v.6.3 (centroids within `match_m`, areas within
/// `min_area_ratio`) still makes a pair in the first round. Pairs are taken
/// greedily, largest share first, so no stored space is matched twice and
/// no new space takes two identities. A space matched to a stored row that
/// a person named or locked (locked, or source manual) also inherits its
/// name, kind, source, lock and metadata keys; its outline is the new one.
/// A locked stored space that nothing matches is kept as stored, flagged
/// `"unmatched": true` in its metadata. Only an unmatched space gets a fresh
/// GUID, uuid5(project_guid, space_natural_key()), suffixed "#n" until it
/// collides with no stored or assigned GUID.
///
/// reframe_spaces() moves stored spaces into a new storeys table's level
/// frames through world coordinates: the storeys stage calls it, because a
/// re-detected storey's frame moves (its origin is a floor centroid).
///
/// Pure functions over core::SpaceRecord; no ProjectDB.

#include "reusex/core/solid_model.hpp"

#include <string>
#include <string_view>
#include <vector>

namespace reusex::core {

struct SpaceIdentityOptions {
  /// A stored space is the same space as a new one of the same storey when
  /// their centroids are within this distance [m]...
  double match_m = 1.0;
  /// ...and the smaller area is at least this share of the larger one.
  double min_area_ratio = 0.5;
  /// Or: their footprints, in world coordinates, overlap by at least this
  /// share of the larger one (first round) or, among the spaces left, of
  /// the smaller one (second round).
  double min_overlap = 0.5;
};

struct SpaceIdentityResult {
  int matched = 0;        ///< new spaces that took a stored space's GUID
  int minted = 0;         ///< new spaces that got a fresh GUID
  int locked_matched = 0; ///< of the matched, those whose stored row is locked
  int named_matched = 0;  ///< of the matched, those that took a stored name
  /// Locked stored spaces no new space matched: the stored rows, flagged
  /// `"unmatched": true` in their metadata, to be written back as they are.
  std::vector<SpaceRecord> kept;
};

/// "spaces/<storey>/<cx>,<cy>": the polygon's area centroid rounded to 5 cm,
/// two decimals.
std::string space_natural_key(const SpaceRecord &space);

/// Set the `guid` of every space in @p fresh. On input each guid is a
/// placeholder unique within @p fresh (polygonise_spaces' "space-<storey>-
/// <room>"); on output a UUID: the GUID of the stored space it matches
/// (same storey idx, footprint overlap in world coordinates through
/// @p storeys' level frames, see the file comment), else
/// uuid5(@p project_guid, space_natural_key()). A space matched to a named
/// or locked stored row takes its name, kind, source, lock and metadata
/// keys too. Each SpaceBoundaryRecord::space of @p boundaries that names a
/// placeholder of @p fresh is rewritten to that space's GUID.
/// Deterministic: the same inputs give the same GUIDs. A space whose storey
/// idx is not in @p storeys is compared in its level frame as is.
/// @throws std::invalid_argument when @p project_guid is not a UUID, or two
///         spaces of @p fresh share a placeholder.
SpaceIdentityResult
assign_space_guids(std::vector<SpaceRecord> &fresh,
                   std::vector<SpaceBoundaryRecord> &boundaries,
                   const std::vector<SpaceRecord> &stored,
                   const std::vector<StoreyRecord> &storeys,
                   std::string_view project_guid,
                   const SpaceIdentityOptions &opt = {});

/// Area [m^2] of the intersection of two footprints given in the same 2D
/// frame (outer ring minus holes; even-odd, so orientation does not
/// matter). Computed on horizontal scanlines @p dy apart, exact along x.
double space_overlap_area(const SpacePolygon &a, const SpacePolygon &b,
                          double dy = 0.02);

/// @p s's footprint in world x/y: its rings taken out of the level frame of
/// @p storey at the space's floor height (level z = z_floor).
SpacePolygon space_polygon_world(const SpaceRecord &s,
                                 const StoreyRecord &storey);

struct SpaceReframeResult {
  int moved = 0;     ///< spaces whose polygon or storey idx changed
  int unchanged = 0; ///< same frame and idx: left as they were
  /// Spaces left in their old frame: their storey is in neither table, or
  /// no new storey holds them (guids).
  std::vector<std::string> lost;
};

/// Re-express each space of @p spaces, stored in the level frame of the row
/// of @p old_storeys with its `storey` idx, in the frame of the row of
/// @p new_storeys with that storey's GUID (else of the new storey whose
/// height band holds the space's floor centroid, core::which_storey), and
/// set its `storey` to that row's idx. Through world coordinates: each
/// polygon point at the space's floor height goes level(old) -> world ->
/// level(new); z_floor / z_ceiling are carried at the polygon centroid.
/// Area, perimeter and volume stay (the map is rigid). A space that finds
/// no storey stays as it is and is listed in `lost`.
SpaceReframeResult reframe_spaces(std::vector<SpaceRecord> &spaces,
                                  const std::vector<StoreyRecord> &old_storeys,
                                  const std::vector<StoreyRecord> &new_storeys);

} // namespace reusex::core
