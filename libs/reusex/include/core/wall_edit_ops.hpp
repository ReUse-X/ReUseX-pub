// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// @file
/// Wall topology edits for the Rediger editor (reusex-yuja): split a wall,
/// merge two collinear walls, join two walls at their axes' intersection
/// (extend / trim), extend / trim one wall to a boundary wall, and move /
/// rotate / copy a selection (reusex-so4t). Pure functions over core
/// records in a storey's level frame; they return the records to write,
/// which the caller applies as one batch (core/model_edits.hpp:
/// ModelEditOp::put / remove, or the wire ops wall.split / wall.merge /
/// wall.join / wall.extend_to / selection.transform / wall.offset that
/// model_edit_ops_from_json() expands into such a batch).
///
/// An opening's extent is measured along its host's axis from (x0, y0), so
/// whenever a wall's start moves or a wall is cut, its openings are
/// re-measured; they keep their guid, width and heights. A sloped top
/// profile (WallRecord::top) is re-measured the same way.

#include <reusex/core/solid_model.hpp>

#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace reusex::core {

/// What split_wall() made.
struct WallSplitResult {
  WallRecord a; ///< (x0, y0) .. the cut; keeps the wall's guid
  WallRecord b; ///< the cut .. (x1, y1); a new guid
  /// The wall's openings, each on the part holding its centre and
  /// re-measured on it; one that crossed the cut is clipped to its part.
  std::vector<OpeningRecord> openings;
  std::vector<std::string> clipped; ///< guids of the clipped openings
  /// The joins of the wall, moved to the part they touch, plus a
  /// collinear join between a's end and b's start.
  std::vector<WallJoinRecord> joins;
};

/// Split @p wall at @p s metres along its axis from (x0, y0).
/// @p openings / @p joins may hold other walls' rows (ignored); @p others
/// are the storey's walls, to place a "path" join (a tee onto the wall) on
/// the part it touches. @p new_guid names part b (empty: a fresh guid).
/// @throws std::invalid_argument when @p s is not inside the wall by at
///         least @p min_part_m at both ends.
WallSplitResult split_wall(const WallRecord &wall, double s,
                           const std::vector<OpeningRecord> &openings,
                           const std::vector<WallJoinRecord> &joins = {},
                           const std::vector<WallRecord> &others = {},
                           const std::string &new_guid = "",
                           double min_part_m = 0.05);

struct WallMergeOptions {
  double max_angle_deg = 10.0; ///< the two axes are parallel within this
  double max_offset_m = 0.30;  ///< ...and each lies this close to the other
};

/// What merge_walls() made.
struct WallMergeResult {
  /// One wall: the length-weighted axis fit through both (direction and
  /// position), from the outermost projected end to the other; thickness
  /// the length-weighted mean of the known ones, z_base the lower, height
  /// up to the higher top; guid / source / metadata of @p a.
  WallRecord wall;
  std::string removed; ///< @p b's guid, to delete
  /// Both walls' openings, re-hosted on the merged wall and re-measured.
  std::vector<OpeningRecord> openings;
  /// Both walls' joins (but the one between them), on the merged wall,
  /// their ends re-read: a join whose end is now inside the wall is a tee
  /// onto its path.
  std::vector<WallJoinRecord> joins;
};

/// @throws std::invalid_argument when the walls are on different storeys,
///         are the same wall, or are not collinear within @p opt.
WallMergeResult merge_walls(const WallRecord &a, const WallRecord &b,
                            const std::vector<OpeningRecord> &openings,
                            const std::vector<WallJoinRecord> &joins = {},
                            const WallMergeOptions &opt = {});

/// What join_walls() made.
struct WallJoinResult {
  WallRecord a, b; ///< extended / trimmed to the axes' intersection
  /// corner (both ends meet there) or tee (the intersection lies inside
  /// one wall, which is left as it is: the other ends on its path).
  WallJoinRecord join;
  std::vector<OpeningRecord> openings; ///< both walls' openings, re-measured
};

/// Extend or trim @p a and @p b to the intersection of their axes: each
/// moves the end nearer to it (extended when the intersection lies beyond
/// it, trimmed when it lies at most @p max_trim_m inside), unless the
/// intersection lies deeper inside the wall than @p max_trim_m from both
/// its ends: a tee onto that wall, which stays as it is.
/// @throws std::invalid_argument when the axes are parallel within
///         @p min_angle_deg, the walls are on different storeys, or the move
///         would leave a wall shorter than 5 cm.
WallJoinResult join_walls(const WallRecord &a, const WallRecord &b,
                          const std::vector<OpeningRecord> &openings,
                          double min_angle_deg = 5.0, double max_trim_m = 0.5);

/// What extend_wall_to() made.
struct WallExtendResult {
  WallRecord wall; ///< its @p end moved
  /// Set when the join is a corner and the target's nearer end had to move
  /// to the axes' intersection too.
  std::optional<WallRecord> target;
  /// {wall, target, corner | tee, end, start | end | path}.
  WallJoinRecord join;
  /// The openings of whichever wall's start moved, re-measured.
  std::vector<OpeningRecord> openings;
};

/// Extend or trim the @p end ("start" | "end") of @p wall to @p target
/// (Revit's Trim/Extend to a boundary), by the join rules the solids follow
/// (reconstruction/solids.hpp, reusex-6x9g):
///  - a tee, when the axes meet inside the target farther than its
///    thickness (at least 0.1 m) from both its ends: the end goes to the
///    target's near face (the side the wall's other end is on; the axis
///    when the target's thickness is unknown) and the target stays;
///  - a corner otherwise: the end goes to the axes' intersection and so does
///    the target's nearer end (the solids mitre or butt it there).
/// The wall's joins at @p end are the caller's to drop (the end left them).
/// @throws std::invalid_argument when the walls are the same, on different
///         storeys, parallel within @p min_angle_deg, or a moved end would
///         leave its wall shorter than 5 cm or turn it around.
WallExtendResult extend_wall_to(const WallRecord &wall, const std::string &end,
                                const WallRecord &target,
                                const std::vector<OpeningRecord> &openings,
                                double min_angle_deg = 5.0);

/// A rigid plan motion: rotate by rotate_deg (counter-clockwise) about
/// pivot, then translate by (dx, dy).
struct PlanTransform {
  double dx = 0.0, dy = 0.0;
  double rotate_deg = 0.0;
  PlanPoint pivot{0.0, 0.0};
  [[nodiscard]] PlanPoint apply(PlanPoint p) const;
};

/// What transform_selection() made: the rows to put.
struct SelectionTransformResult {
  /// The moved walls (or their copies), and unmoved walls whose end a
  /// re-solved join moved.
  std::vector<WallRecord> walls;
  /// Openings that moved: copies of the copied walls' openings, selected
  /// openings slid along an unmoved host, openings re-measured after an end
  /// moved. (A moved wall's openings move with it as they are.)
  std::vector<OpeningRecord> openings;
  std::vector<SpaceRecord> spaces; ///< moved spaces (or their copies)
  /// Every join after the edit: the input's, re-solved / dropped, plus the
  /// copies' joins among themselves.
  std::vector<WallJoinRecord> joins;
  /// The copied spaces' boundaries (on the copied walls; others dropped).
  std::vector<SpaceBoundaryRecord> boundaries;
  std::vector<std::pair<std::string, std::string>> copies; ///< old -> new
  std::vector<std::string> warnings; ///< e.g. a join that had to be dropped
};

/// Move, rotate or copy the walls, openings and spaces named by @p guids as
/// one rigid body. A wall's hosted openings go with it (copied with it); an
/// opening selected without its host slides along the host to the point
/// nearest its moved centre. Joins among the moved walls stay; a join
/// between a moved and an unmoved wall is re-solved like a dragged Revit
/// wall (a corner moves both ends to the new intersection, a tee's stem is
/// extended / trimmed to the through wall's near face) or, when it no
/// longer can be (parallel, missed, a wall would collapse), dropped with a
/// warning. A copy leaves the originals and their joins alone; the copies
/// keep only the joins among themselves. @p new_guids names copies (old ->
/// new); any other copy gets a fresh guid. A copy is a new, hand-drawn row:
/// its metadata loses `edited` / `edited_from`.
/// @throws std::invalid_argument for a guid that is no wall, opening or
///         space.
SelectionTransformResult
transform_selection(const std::vector<std::string> &guids,
                    const PlanTransform &t, bool copy,
                    const std::vector<WallRecord> &walls,
                    const std::vector<OpeningRecord> &openings,
                    const std::vector<SpaceRecord> &spaces,
                    const std::vector<WallJoinRecord> &joins,
                    const std::vector<SpaceBoundaryRecord> &boundaries,
                    const std::map<std::string, std::string> &new_guids = {});

/// The wall's axis length [m].
double wall_length(const WallRecord &wall);

} // namespace reusex::core
