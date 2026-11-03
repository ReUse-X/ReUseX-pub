// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// @file
/// Geometry the Rediger editor asks the library for (reusex-yuja), beside
/// the wall topology edits of core/wall_edit_ops.hpp:
///
///  - rebuild_spaces_from_walls(): a storey's space polygons from its
///    current walls alone (no cloud, no band grid; "Genopbyg rum fra
///    vægge"): a CGAL arrangement of every wall's faces (axis +- thickness
///    / 2, a one-sided wall's axis itself), each run extended past both
///    ends so corners and tees close, plus a cap across each thick wall's
///    ends; dangling stubs removed; every bounded face that is not a wall
///    body and not a sliver (min_area_m2, min_width_m) is a space. A new
///    polygon takes the guid, name, kind, heights and ceiling planes of the
///    stored space it overlaps most (one-to-one), so names survive the rebuild.
///    Boundaries are the polygon edges lying on a wall face. A stored space no
///    polygon takes (its walls do not close it) is reported, not dropped:
///    detected walls often leave rooms open, and a rebuild must not delete what
///    it cannot see. A preview: nothing is written; rebuild_spaces_ops() turns
///    the result into one edit batch.
///  - fit_ceiling_plane(): a ceiling plane (core::CeilingPlane) fitted to
///    the cloud's points above a plan region within a height band ("fit fra
///    punkter"): least squares z = a + b x + c y, trimmed to the points
///    within `inlier_m` of the fit until the inlier set settles. Also a
///    preview: the client sends the plane back as ceiling.set.
///
/// Coordinates are in the storey's level frame (StoreyRecord::
/// level_transform maps world -> level), as every solid-model record.
/// Deterministic (STANDARDS §6): exact constructions, faces visited in
/// arrangement order, rings start at their lexicographically smallest
/// vertex, GUIDs minted with core::deterministic_guid.

#include <reusex/core/model_edits.hpp>
#include <reusex/core/solid_model.hpp>
#include <reusex/types/point_types.hpp>

#include <string>
#include <string_view>
#include <vector>

namespace reusex::segmentation {

struct RebuildSpacesOptions {
  /// Each wall face run is extended this far past both ends [m], so the
  /// faces of walls that meet at a corner or a tee cross and close it.
  double join_extension_m = 0.30;
  /// A face smaller than this is not a space [m^2] (an overshoot, a niche).
  double min_area_m2 = 1.0;
  /// Nor one narrower than this [m], measured as 2 area / perimeter (the
  /// width of a long strip): the gap between two parallel walls, a doubled
  /// wall's two sheets.
  double min_width_m = 0.6;
  /// A stored space hands its guid and name to the new polygon that covers
  /// at least this share of the smaller of the two.
  double min_name_overlap = 0.5;
  /// A polygon edge within this distance of a wall face, parallel to it,
  /// is a boundary of that wall [m].
  double boundary_tolerance_m = 0.02;
};

struct RebuildSpacesResult {
  /// The storey's spaces after the rebuild (matched ones keep their guid).
  std::vector<core::SpaceRecord> spaces;
  std::vector<core::SpaceBoundaryRecord> boundaries;
  /// Stored spaces of the storey no new polygon took (their walls do not
  /// close them, or another polygon took the overlap).
  std::vector<std::string> unmatched;
  int walls = 0;       ///< walls of the storey inserted
  int faces = 0;       ///< bounded faces of the arrangement
  int wall_bodies = 0; ///< faces inside a wall
  int too_small = 0;   ///< faces below min_area_m2 or min_width_m
  int matched = 0;     ///< spaces that took a stored space's guid
};

/// @p walls / @p stored may hold other storeys' rows (ignored).
/// @p project_guid names the fresh guids (core::deterministic_guid).
/// @throws std::invalid_argument for a project_guid that is not a UUID.
RebuildSpacesResult
rebuild_spaces_from_walls(const std::vector<core::WallRecord> &walls,
                          const std::vector<core::SpaceRecord> &stored,
                          const core::StoreyRecord &storey,
                          std::string_view project_guid,
                          const RebuildSpacesOptions &opt = {});

/// One edit batch applying @p result: put every space with its boundaries;
/// with @p remove_unmatched also remove the stored spaces no polygon took
/// (the user asked for exactly the walls' rooms), else they stay.
std::vector<core::ModelEditOp>
rebuild_spaces_ops(const RebuildSpacesResult &result,
                   bool remove_unmatched = false);

struct CeilingFitOptions {
  double inlier_m = 0.05;     ///< points farther from the fit are dropped
  int min_points = 30;        ///< fewer (after trimming): no fit
  int max_rounds = 8;         ///< trimming rounds
  double max_slope_deg = 75.; ///< steeper: not a ceiling, no fit
};

struct CeilingFitResult {
  bool ok = false;
  std::string reason;       ///< why not, when !ok
  core::CeilingPlane plane; ///< `at` at the inliers' centroid
  double rms_m = -1.0;      ///< of the inliers
  double slope_deg = 0.0;
  int points = 0;  ///< in the region and band
  int inliers = 0; ///< used by the final fit
};

/// Fit a ceiling plane to the points of @p cloud (world coordinates) that
/// fall, in @p storey's level frame, inside @p region (outer ring minus
/// holes) with z in [@p z_lo, @p z_hi].
CeilingFitResult fit_ceiling_plane(const Cloud &cloud,
                                   const core::StoreyRecord &storey,
                                   const core::SpacePolygon &region,
                                   double z_lo, double z_hi,
                                   const CeilingFitOptions &opt = {});

} // namespace reusex::segmentation
