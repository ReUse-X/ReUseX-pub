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
///  - place_space_at() / merge_spaces() / split_space(): the room tools
///    (place a room by a click in a closed area, merge two rooms across
///    their wall, split one along a line), previews again.
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

// --- Room tools (Rediger slice S6, reusex-pccw) ------------------------------

/// "Placér i lukket område" (Revit: click inside a closed area): the room
/// rebuild_spaces_from_walls() finds around @p seed. A preview like the
/// rebuild; place_space_ops() makes it one edit batch.
struct PlaceSpaceResult {
  bool ok = false;
  std::string reason; ///< why not, when !ok
  core::SpaceRecord space;
  std::vector<core::SpaceBoundaryRecord> boundaries;
  /// The region is (mostly) a stored space, whose guid, name, kind and
  /// heights it took: applying it re-fits that space to its walls.
  bool existing = false;
};

/// @p seed in @p storey's level frame. Not ok when the point lies in a wall
/// body or in no bounded face the walls close.
PlaceSpaceResult place_space_at(const std::vector<core::WallRecord> &walls,
                                const std::vector<core::SpaceRecord> &stored,
                                const core::StoreyRecord &storey,
                                std::string_view project_guid,
                                const core::PlanPoint &seed,
                                const RebuildSpacesOptions &opt = {});
/// One put of the space with its boundaries (empty when !ok).
std::vector<core::ModelEditOp> place_space_ops(const PlaceSpaceResult &r);

struct RoomToolOptions {
  /// An edge of a space within this distance of a wall face, parallel to
  /// it, bounds the space on that wall [m] (rebuild's boundary_tolerance_m).
  double boundary_tolerance_m = 0.02;
  /// merge: two spaces on opposite faces of a wall, overlapping along it at
  /// least this much, are merged across that wall's body [m].
  double min_shared_m = 0.10;
  /// split: the line is extended this far past both given points [m], so a
  /// line snapped to the room's edges still crosses them.
  double split_extension_m = 0.05;
  /// Vertices closer than this to the line through their neighbours are
  /// dropped from a result ring [m] (collinear runs, slivers of the union).
  double simplify_m = 1e-4;
};

/// "Flet rum": @p b joined into @p a across the walls between them (the
/// wall body is part of the result; Revit: delete the room separation).
/// The result keeps @p a's guid, name, kind, heights and ceiling planes;
/// metrics are measured again, boundaries found again from @p walls (the
/// storey's), @p b is removed.
struct MergeSpacesResult {
  bool ok = false;
  std::string reason;
  core::SpaceRecord space;
  std::vector<core::SpaceBoundaryRecord> boundaries;
  std::string removed;                    ///< b's guid
  std::vector<std::string> bridged_walls; ///< walls whose body joined them
};
/// Not ok for spaces on different storeys, or whose union (with the bodies
/// of the walls they share) is not one polygon.
MergeSpacesResult merge_spaces(const core::SpaceRecord &a,
                               const core::SpaceRecord &b,
                               const std::vector<core::WallRecord> &walls,
                               const RoomToolOptions &opt = {});
/// Put a (with its boundaries), remove b.
std::vector<core::ModelEditOp> merge_spaces_ops(const MergeSpacesResult &r);

/// "Del rum": @p space cut along the segment p0 -> p1 (level frame; extended
/// by split_extension_m). The largest piece keeps the space's guid; the
/// others copy its name (with " (2)", " (3)" ...), kind, heights and
/// ceiling planes under fresh guids (core::deterministic_guid of
/// @p project_guid and "spaces/split/<guid>/<k>", skipping @p taken).
struct SplitSpaceResult {
  bool ok = false;
  std::string reason;
  std::vector<core::SpaceRecord> spaces; ///< largest first
  std::vector<core::SpaceBoundaryRecord> boundaries;
};
/// Not ok when the segment does not cut the space in two.
SplitSpaceResult split_space(const core::SpaceRecord &space,
                             const core::PlanPoint &p0,
                             const core::PlanPoint &p1,
                             const std::vector<core::WallRecord> &walls,
                             std::string_view project_guid,
                             const std::vector<std::string> &taken = {},
                             const RoomToolOptions &opt = {});
/// A put per piece, with its boundaries.
std::vector<core::ModelEditOp> split_space_ops(const SplitSpaceResult &r);

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
