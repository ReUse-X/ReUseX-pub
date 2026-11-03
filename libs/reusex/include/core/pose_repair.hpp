// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Pose-repair operations of the scan-repair editor (reusex-y2lg.29), each
// with a preview that writes nothing and an apply that writes a pose version
// (core/pose_versions.hpp):
//
//  * rebuild_odometry() (reusex-y2lg.16): RTAB-Map's stored odometry is raw
//    VIO while its poses are loop-corrected (reusex-33j), so plain optimize
//    re-introduces the drift. This replaces the odometry of every pair of
//    consecutive frames of a session by the relative pose of their initial
//    poses -- including the pairs at tracking gaps, whose stale measurement
//    once lifted a session 2.8 m after a re-seed (reusex-xmt).
//  * apply_block_transform() (reusex-y2lg.9, reusex-om3): move / rotate a
//    node set as a RE-SEED. A soft move edit is not honoured for a large
//    move, because plane landmarks are associated from the seed; what worked
//    in reusex-d9h step 11 was to write the moved poses into `transform` AND
//    `initial_transform` (seeding every other frame from its current pose),
//    re-derive the odometry, and hold the block. The move fades out over
//    `blend_frames` at a joint to an attached neighbour (smoothstep weights,
//    slerp), and is rigid at a session boundary or a detached joint.
//
// Core-only (ProjectDB + Eigen): the solve itself (GTSAM) stays in slam.

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/pose_edits.hpp>

#include <nlohmann/json_fwd.hpp>

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace reusex::core {

struct RebuildOdometryOptions {
  /// Snapshot every initial pose from the current pose first (seed :=
  /// current), so the odometry follows the current poses.
  bool from_current = false;
  /// Only this session (scan id); nullopt = every session.
  std::optional<int> scan_id;
  /// Record a pose version ("odometry") of the result.
  bool record_version = true;
  std::string author;
};

struct RebuildOdometryResult {
  int pairs = 0;               ///< consecutive in-session pairs considered
  int written = 0;             ///< odometry_init rows written
  int changed = 0;             ///< of those, whose measurement changed
  int replaced_imported = 0;   ///< imported (rtabmap) rows replaced
  int kept_confirmed = 0;      ///< confirmed rows left as they are
  int kept_rejected = 0;       ///< user-rejected rows left as they are
  int cut_detached = 0;        ///< pairs at a detached joint (no odometry)
  int snapshots_written = 0;   ///< initial poses re-snapshotted (from_current)
  double max_change_m = 0.0;   ///< largest measurement change (translation)
  double max_change_deg = 0.0; ///< largest measurement change (rotation)
  std::optional<int64_t> version_id;
};

/// Re-derive the odometry from the initial poses (see the file comment).
/// Imported rows are replaced even when rejected (the rejection only hid the
/// raw measurement); confirmed rows and rows a user rejected are kept. One
/// transaction.
RebuildOdometryResult
rebuild_odometry(ProjectDB &db, const RebuildOdometryOptions &options = {});

enum class BlockTransformMode {
  reseed, ///< seed := current everywhere, move transform + initial, rebuild
          ///< odometry (reusex-om3)
  local,  ///< move only the selection's transform (reusex-y2lg.12)
};

struct BlockTransformHold {
  std::uint8_t dofs = Dof::all;
  double sigma_trans_m = 0.05;
  double sigma_rot_deg = 1.0;
};

struct BlockTransformRequest {
  /// The node set: these blocks (core/pose_blocks.hpp) ...
  std::vector<int> block_ids;
  /// ... plus these inclusive node ranges.
  std::vector<std::pair<int, int>> node_ranges;
  /// The rigid delta, in world coordinates (yaw / pitch / roll about
  /// `delta.pivot`, by default the centroid of the selection's cameras, or a
  /// full `matrix`; core/pose_edits.hpp).
  RigidTransform delta;
  BlockTransformMode mode = BlockTransformMode::reseed;
  /// Frames over which the move fades out at a joint to an attached
  /// neighbour; 0 = rigid. Never applied at a session boundary or a
  /// detached joint.
  int blend_frames = 10;
  /// Add a hold for the moved node set to the working edits.
  std::optional<BlockTransformHold> hold;
  bool preview = false;
  std::string label;
  std::string author;
};

/// A frame the transform moves.
struct MovedFrame {
  int node_id = 0;
  double weight = 1.0; ///< share of the delta (< 1 in a blend zone)
  std::array<double, 16> before{};
  std::array<double, 16> after{};
  double shift_m = 0.0;
};

/// One end of a moved run of frames.
struct JointReport {
  int inside_node = 0;  ///< last moved frame at this end
  int outside_node = 0; ///< its unmoved neighbour (0: none, session end)
  bool session_boundary = false;
  bool detached = false;
  int blend_frames = 0;      ///< frames blended at this end
  double gap_before_m = 0.0; ///< camera distance across the joint, before
  double gap_after_m = 0.0;  ///< ... and after
};

struct BlockTransformResult {
  bool preview = true;
  std::array<double, 16> delta{}; ///< the full delta, row-major
  std::array<double, 3> pivot{};
  int frames_moved = 0;   ///< frames with weight 1
  int frames_blended = 0; ///< frames with 0 < weight < 1
  double max_shift_m = 0.0;
  std::vector<MovedFrame> frames;
  std::vector<JointReport> joints;
  /// Active loop edges with exactly one endpoint in the selection, and how
  /// many of them disagree with the moved seed by > 0.3 m or 4 deg.
  int loops_touching = 0;
  int loops_disagreeing = 0;
  std::optional<RebuildOdometryResult> odometry; ///< apply, reseed
  std::optional<std::string> hold_edit_id;
  std::optional<int64_t> version_id; ///< apply
};

/// Preview or apply @p request (see the file comment). An apply is one
/// transaction and records a pose version ("apply").
/// @throws std::invalid_argument for an empty or unknown selection, an
///         unknown block id or a non-rigid matrix.
BlockTransformResult
apply_block_transform(ProjectDB &db, const BlockTransformRequest &request);

/// The smoothstep blend weight of the k-th frame (1-based) from a joint,
/// over n blend frames: 0 < w < 1 for k <= n, 1 beyond.
double blend_weight(int k, int n);

/// The fraction @p w of the rigid delta @p D about @p pivot: rotation
/// slerped from identity, pivot translation scaled.
std::array<double, 16> partial_delta(const std::array<double, 16> &D,
                                     const std::array<double, 3> &pivot,
                                     double w);

/// Request from the JSON the web editor sends (ruxd-64g.5):
/// {blocks?: [id], nodes?: [[a, b], ...], delta: {xyz, yaw_deg, pitch_deg,
///  roll_deg, pivot?, matrix?}, mode?: "reseed"|"local", blend_frames?,
///  hold?: {dofs?: ["x", ...], sigma_trans_m?, sigma_rot_deg?}, preview?,
///  label?}
/// @throws std::invalid_argument naming the bad field.
BlockTransformRequest
block_transform_request_from_json(const nlohmann::json &j);

nlohmann::json to_json(const RebuildOdometryResult &result);
/// @p with_frames: include the moved frames' poses (preview).
nlohmann::json to_json(const BlockTransformResult &result,
                       bool with_frames = true);

} // namespace reusex::core
