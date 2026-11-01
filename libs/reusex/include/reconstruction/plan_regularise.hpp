// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// @file
/// Plan regularisation (reusex-z1v.12): prefer a clean, architect-like plan
/// over following every few centimetres of scan noise. Runs on the wall graph
/// before the walls are stored (and on the openings before they are), so
/// spaces, solids and every export inherit it:
///
///  - regularise_walls(): wall lines onto the Manhattan grid (angles), walls
///    that continue each other on one storey onto one shared line
///    ("collinear"), walls that stand over each other on adjacent storeys
///    onto one shared line ("stack"), optional dimension rounding, and the
///    corner / tee ends re-solved on the moved lines;
///  - regularise_openings(): windows of one facade column (the same facade
///    line, one per storey, centres within column_m) share their horizontal
///    position and width across storeys, and the windows of one storey whose
///    sills (heads) lie within row_m share one sill (head) height.
///
/// Every snap keeps the measured value and the delta (WallSegment::
/// regularised, the opening's metadata "regularised"), so a measured-vs-
/// regularised quantity stays available.
///
/// What must never happen: two genuinely different walls merged. Two walls of
/// one storey that overlap along their axis by more than max_overlap_m are
/// different walls (the two sides of a corridor, two partitions side by
/// side) and never share a line, however close; walls share a line only
/// through a face seen from the same side, so a one-sided wall and one seen
/// only from the other side are never pulled together.
///
/// Deterministic (STANDARDS §6): no randomness, inputs visited in index
/// order, candidate pairs in a fixed order.

#include "reusex/core/solid_model.hpp"
#include "reusex/reconstruction/walls.hpp"

#include <vector>

namespace reusex::geometry {

/// The tunables of regularise_walls() (STANDARDS §4).
struct PlanRegulariseOptions {
  /// Master switch: off leaves the measured wall graph untouched.
  bool plan_regularise = true;
  /// Two walls share a line only if their axes are parallel within this
  /// [deg] (after the grid snap).
  double plan_angle_deg = 2.0;
  /// Walls of one storey that continue each other (overlap along the axis
  /// by at most plan_max_overlap_m, gap at most plan_collinear_gap_m) share
  /// a line when a face seen from the same side lies within this of the
  /// other's [m]. Every face of a line lies within this of every other, so a
  /// chain of small steps never adds up to a big one. (0.10 / 2.0 by the
  /// solid bench, 2026-11-01: 0.15 / 5.0 cost house1 a wall match and +20 %
  /// wall rms against references that follow the scan's drift.)
  double plan_collinear_m = 0.10;
  double plan_collinear_gap_m = 2.0;
  /// Two walls of one storey overlapping along the axis by more than this
  /// are different walls and never share a line [m].
  double plan_max_overlap_m = 0.3;
  /// Walls of adjacent storeys overlapping along the axis by at least
  /// plan_stack_min_overlap_m with a same-side face within plan_stack_m
  /// share a line (facades stack) [m].
  bool plan_stack = true;
  double plan_stack_m = 0.15;
  double plan_stack_min_overlap_m = 0.5;
  /// Turn the walls within this of the project's Manhattan grid (the
  /// orthogonal pair with the most wall length, over all storeys) onto it
  /// [deg]; 0 = off. Non-orthogonal families farther than this stay. Within
  /// one storey the wall families are already an exact orthogonal pair, so
  /// this only turns whole storeys onto each other. Off by default: on
  /// house2 it turned storey 1 by 1.26 deg, +26 % wall rms and one opening
  /// lost against a reference that follows the scan (2026-11-01).
  double plan_grid_deg = 0.0;
  /// Round each line's offset (its reference face) to a multiple of this in
  /// the grid frame [m] (0.01 / 0.05); 0 = off.
  double plan_round_m = 0.0;
};

/// What regularise_walls() did, for the stage's log.
struct PlanRegulariseResult {
  int lines = 0;               ///< groups of >= 2 walls put on one line
  int stacked_lines = 0;       ///< ...of which span more than one storey
  int walls_moved = 0;         ///< walls whose axis moved (shift or turn)
  int walls_turned = 0;        ///< walls turned onto the grid
  int ends_rejoined = 0;       ///< corner / tee ends re-solved on moved lines
  double grid_angle_deg = 0.0; ///< the grid's first axis in storey 0's frame
  double max_shift_m = 0.0;    ///< largest axis shift
  double rms_shift_m = 0.0;    ///< over the moved walls
  double max_turn_deg = 0.0;   ///< largest turn
};

/// Regularise the wall graphs of all storeys together. @p graphs and
/// @p storeys are index-aligned (one graph per storey, in its level frame,
/// as detect_walls() made it). The walls keep their order, family and
/// joins; their axes (WallSegment::d, axis_offset, face_lo / face_hi, s0 /
/// s1) move and WallSegment::regularised records the measured axis.
///
/// Storeys whose level frames differ (a tilted storey with its own frame)
/// are compared through the world frame, in storey 0's level frame.
///
/// @throws std::invalid_argument when the sizes differ or an option is out
///         of range.
PlanRegulariseResult
regularise_walls(std::vector<WallGraph> &graphs,
                 const std::vector<core::StoreyRecord> &storeys,
                 const PlanRegulariseOptions &opt = {});

/// The walls as they were measured: a record whose metadata carries
/// "regularised" (wall_records()) gets its measured axis back. Openings are
/// found against these -- the evidence lies on the measured face, and a
/// regularised axis a few centimetres off it would read solid wall as open
/// -- and then put onto the regularised walls (openings_onto_walls()).
std::vector<core::WallRecord>
measured_walls(const std::vector<core::WallRecord> &walls);

/// Re-express @p openings, whose s_start / s_end run along the axes of
/// @p measured, along the axes of @p walls (matched by guid): each end goes
/// to its projection on the regularised axis, clamped to the wall. Openings
/// whose host is in neither are left alone.
void openings_onto_walls(std::vector<core::OpeningRecord> &openings,
                         const std::vector<core::WallRecord> &measured,
                         const std::vector<core::WallRecord> &walls);

/// The tunables of regularise_openings() (STANDARDS §4).
struct OpeningRegulariseOptions {
  /// Master switch.
  bool plan_regularise = true;
  /// Windows of different storeys are one facade column when their hosts
  /// are parallel within plan_angle_deg, their centres lie within
  /// plan_facade_m of one facade line and within plan_column_m of each
  /// other along it (at most one per storey) [m]. The column shares the
  /// mean centre and the median width.
  double plan_angle_deg = 2.0;
  double plan_facade_m = 0.3;
  double plan_column_m = 0.2;
  /// Windows of one storey whose sills (heads) lie within this of each other
  /// share the median sill (head) [m].
  double plan_row_m = 0.1;
};

struct OpeningRegulariseResult {
  int columns = 0; ///< columns of >= 2 windows aligned
  int rows = 0;    ///< sill / head rows of >= 2 windows aligned
  int openings_moved = 0;
  int skipped = 0;          ///< a snap that would leave the host or overlap
  double max_shift_m = 0.0; ///< largest horizontal or vertical move
};

/// Align the windows of @p openings (kind "window") on @p walls (their
/// hosts, by guid) across and within storeys. A snap that would push a
/// window past its host's ends, or onto another opening of the host, is
/// skipped for that window. Each moved opening's metadata gets
/// "regularised": {"rules": ["column"|"row", ...], "measured": {"s_start",
/// "s_end", "z_sill", "z_head"}}.
OpeningRegulariseResult
regularise_openings(std::vector<core::OpeningRecord> &openings,
                    const std::vector<core::WallRecord> &walls,
                    const std::vector<core::StoreyRecord> &storeys,
                    const OpeningRegulariseOptions &opt = {});

} // namespace reusex::geometry
