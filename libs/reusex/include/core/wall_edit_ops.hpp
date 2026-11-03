// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// @file
/// Wall topology edits for the Rediger editor (reusex-yuja): split a wall,
/// merge two collinear walls, join two walls at their axes' intersection
/// (extend / trim). Pure functions over core records in a storey's level
/// frame; they return the records to write, which the caller applies as one
/// batch (core/model_edits.hpp: ModelEditOp::put / remove, or the wire ops
/// wall.split / wall.merge / wall.join that model_edit_ops_from_json()
/// expands into such a batch).
///
/// An opening's extent is measured along its host's axis from (x0, y0), so
/// whenever a wall's start moves or a wall is cut, its openings are
/// re-measured; they keep their guid, width and heights. A sloped top
/// profile (WallRecord::top) is re-measured the same way.

#include <reusex/core/solid_model.hpp>

#include <string>
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

/// The wall's axis length [m].
double wall_length(const WallRecord &wall);

} // namespace reusex::core
