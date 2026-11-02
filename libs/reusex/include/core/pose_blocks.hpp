// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Trajectory blocks (reusex-y2lg.1): the unit every scan-repair tool acts on.
// The trajectory of each session (scan) is cut into runs of consecutive
// frames; the web editor draws each block as a bone and each cut as a joint
// (rux-frontend frontend-aev.5/.6).
//
// suggest_block_splits() proposes the cuts the Sygehusgrunden repair
// (reusex-d9h steps 5, 8, 9, 15) found by hand: session boundaries, time gaps
// over 100 s, tracking gaps over 5 s, a change of the camera's storey band
// (stairs) and odometry jumps; a single frame that jumps away and back is a
// glitch, reported but not a cut. The user accepts or ignores them and adds
// own cuts; the block list is stored per project (`pose_blocks`, v40).
//
// Detach cuts a block from the one before it: a reject_edge edit for the
// odometry link at the joint goes into the working edits
// (core/working_pose_edits.hpp), because optimize bridges a merely pruned
// link from the initial poses (reusex-i9p), and the stored odometry row at
// the joint is rejected so 'rux graph init' does not put it back.
//
// Core-only (ProjectDB + Eigen) so rux, the Qt client and ruxd's light API
// library share it.

#include <reusex/core/ProjectDB.hpp>

#include <Eigen/Core>
#include <nlohmann/json_fwd.hpp>

#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace reusex::core {

/// One posed frame of the trajectory.
struct TrajectorySample {
  int node_id = 0;
  int scan_id = 0;
  double timestamp = -1.0; ///< epoch seconds; < 0 when unknown
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
};

/// The project's posed frames in node order, positioned by their current
/// `transform` (or their `initial_transform` when @p initial and present).
std::vector<TrajectorySample> trajectory_samples(const ProjectDB &db,
                                                 bool initial = false);

/// Why a cut is suggested, strongest first.
enum class SplitReason {
  session,       ///< the frame starts another session (scan)
  time_gap,      ///< > time_gap_s since the previous frame
  tracking_gap,  ///< > tracking_gap_s since the previous frame
  storey_change, ///< the camera moved to another storey band and stayed
  odometry_jump, ///< the camera jumped > odometry_jump_m and did not return
};
std::string_view to_string(SplitReason reason);

struct BlockSuggestOptions {
  double time_gap_s = 100.0;
  double tracking_gap_s = 5.0;
  double odometry_jump_m = 1.0;
  /// Storey bands: camera-height levels at least this far apart.
  double storey_separation_m = 1.8;
  /// A band needs this many frames to count, and a camera this many frames
  /// in the new band before a change is suggested.
  int storey_min_frames = 20;
};

/// A suggested cut: a new block starts at `node_id`.
struct BlockSplitSuggestion {
  int node_id = 0;
  int prev_node_id = 0;
  SplitReason reason = SplitReason::session; ///< the strongest reason
  std::vector<SplitReason> reasons;          ///< every reason at this node
  double score = 0.0;                        ///< 0..1, how sure the cut is
  double gap_s = 0.0;  ///< time since the previous frame (0 when unknown)
  double jump_m = 0.0; ///< camera distance from the previous frame
  std::string detail;  ///< one line for a tooltip
};

/// A single frame that jumps > odometry_jump_m away from both neighbours,
/// which are close to each other (y2lg.17): exclude it, do not cut.
struct TrajectoryGlitch {
  int node_id = 0;
  double jump_m = 0.0;
};

struct BlockSuggestions {
  std::vector<BlockSplitSuggestion> splits; ///< in node order
  std::vector<TrajectoryGlitch> glitches;
  std::vector<double> storey_levels; ///< camera-height bands found, ascending
};

BlockSuggestions suggest_block_splits(std::span<const TrajectorySample> traj,
                                      const BlockSuggestOptions &options = {});
BlockSuggestions suggest_block_splits(const ProjectDB &db,
                                      const BlockSuggestOptions &options = {});

using PoseBlock = ProjectDB::PoseBlockRecord;

/// One block per session, named s0, s1, ... in node order. A block is
/// `detached` when the working pose edits cut the odometry link at its
/// start (a detach, or a session imported with --append, reusex-y2lg.30);
/// every list below derives the flag that way, so undoing a detach
/// re-attaches the block.
std::vector<PoseBlock> default_pose_blocks(const ProjectDB &db);

/// The stored blocks, plus a default block for every session none of them
/// covers (a session imported after the blocks were stored); the defaults
/// alone when nothing is stored.
std::vector<PoseBlock> pose_blocks(const ProjectDB &db);

/// Check @p blocks against the project and store them. Every posed frame of
/// each session must lie in exactly one block of that session, and a block
/// may not reach into another session. Empty names become "<session>.<k>".
/// `detached` is not taken from @p blocks (use detach_pose_block()).
/// @returns the stored list (ids 1..n).
/// @throws std::invalid_argument naming the block and the problem.
std::vector<PoseBlock> set_pose_blocks(ProjectDB &db,
                                       std::vector<PoseBlock> blocks);

/// Blocks cut at every session start plus @p split_nodes (each the first
/// node of a new block). Splits keep the detached flag of default blocks.
std::vector<PoseBlock> blocks_from_splits(const ProjectDB &db,
                                          std::vector<int> split_nodes);

/// Cut the block containing @p node_id so a new block starts there.
/// @throws std::invalid_argument when node_id is no posed frame or already
///         starts a block.
std::vector<PoseBlock> split_pose_block(ProjectDB &db, int node_id,
                                        std::string name = "");

/// Merge block @p second into the block before it (same session, adjacent).
/// @throws std::invalid_argument otherwise.
std::vector<PoseBlock> merge_pose_blocks(ProjectDB &db, int first, int second);

std::vector<PoseBlock> rename_pose_block(ProjectDB &db, int block_id,
                                         std::string name);

struct DetachResult {
  std::vector<PoseBlock> blocks;
  int prev_node_id = 0;       ///< the joint: last frame of the block before
  int node_id = 0;            ///< first frame of the detached block
  std::string edit_id;        ///< the reject_edge edit in the working edits
  bool edge_rejected = false; ///< a stored odometry row was rejected
  /// Loop edges (not rejected) from the detached block's session part to
  /// the rest; 0 means nothing holds it (warn).
  int holding_loops = 0;
  /// The pose version ("detach" / "attach") the cut was recorded as: it
  /// changes the working edits and an edge status, so it can be undone.
  int64_t version_id = 0;
};

/// Detach (or with @p attach, re-attach) block @p block_id from the block
/// before it. One transaction; records a pose version.
/// @throws std::invalid_argument for an unknown id or the first frame.
DetachResult detach_pose_block(ProjectDB &db, int block_id, bool attach = false,
                               std::string_view author = "");

/// The node ids of a block list's members that are posed frames, in order.
std::vector<int> block_nodes(const ProjectDB &db, const PoseBlock &block);

nlohmann::json to_json(const PoseBlock &block);
/// The block list for an API response: every block of pose_blocks() with
/// its `frame_count`, its session ordinal `session` (0, 1, ... in node
/// order) and the camera positions of its first and last frames.
nlohmann::json pose_blocks_json(const ProjectDB &db);
nlohmann::json to_json(const BlockSplitSuggestion &split);
nlohmann::json to_json(const BlockSuggestions &suggestions);
nlohmann::json to_json(const DetachResult &result);
/// @throws std::invalid_argument for a malformed block object.
PoseBlock pose_block_from_json(const nlohmann::json &j);

} // namespace reusex::core
