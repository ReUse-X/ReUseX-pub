// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Pose versions (reusex-y2lg.21): every scan-repair step (an import, a block
// transform, an odometry rebuild, ...) stores the pose state it produced as a
// version in `pose_versions` (schema v40), so any step can be undone and any
// earlier state restored. The repair log reusex-d9h kept 11 versions as
// separate .rux copies; this keeps them in the project.
//
// A version's state is everything a solve depends on that the editor
// changes: each frame's `transform` and `initial_transform`, every
// `pose_graph_edges` row (measurement, status, metrics, id), and the working
// pose-edits document (core/working_pose_edits.hpp). Blocks are not part of
// it: splitting the trajectory moves no pose.
//
// The versions form a tree: each one's parent is the version that was
// current when it was recorded. Undo moves to the parent, redo to the newest
// child, restore to any version; none of them deletes a version. The current
// version is kept in project_settings (kSettingPoseVersionCurrent).
//
// Core-only (ProjectDB + Eigen), like core/pose_graph.hpp, so rux, the Qt
// client and ruxd's light API library share it.

#include <reusex/core/ProjectDB.hpp>

#include <nlohmann/json_fwd.hpp>

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace reusex::core {

/// project_settings key holding the current version's id.
inline constexpr std::string_view kSettingPoseVersionCurrent =
    "pose_versions.current";

/// One frame's poses in a version.
struct FramePoseState {
  int node_id = 0;
  std::array<double, 16> transform{};
  std::optional<std::array<double, 16>> initial;
  bool operator==(const FramePoseState &) const = default;
};

/// The pose state a version holds.
struct PoseState {
  std::vector<FramePoseState> frames;          ///< ordered by node_id
  std::vector<ProjectDB::PoseGraphEdge> edges; ///< ordered by id
  std::optional<std::string> edits;            ///< working pose-edits document
};

/// The live state of @p db. Frames without a usable `transform` are left out.
PoseState capture_pose_state(const ProjectDB &db);

/// Write @p state into @p db: poses of every frame it lists, the whole
/// `pose_graph_edges` table, and the working edits. Frames the state does not
/// list (imported after it was taken) keep their poses, and so do the edges
/// whose two endpoints are both such frames. Opens no transaction: run it
/// inside a ProjectDB::Transaction.
void restore_pose_state(ProjectDB &db, const PoseState &state);

/// Every this many links a version is stored in full (a keyframe); the
/// versions in between store only what differs from their parent
/// (reusex-md4a), so a restore reads at most this many snapshots.
inline constexpr int kPoseVersionKeyframeInterval = 16;

/// Deterministic binary packing of a full keyframe; equal states pack to
/// equal bytes.
///
/// Storage (reusex-md4a): "RXF2" / "RXE2" blobs hold a base version id (0 for
/// a keyframe), varint ids, poses as 12 doubles when their bottom row is
/// exactly (0, 0, 0, 1), information matrices as their diagonal or upper
/// triangle when bitwise diagonal / symmetric, and an edge's strings through
/// a per-blob table. A delta lists the frames whose transform or initial
/// pose changed, the edges that changed (a metrics-only record when only
/// residual, weight, seed disagreement or deviation did) and the frames /
/// edges removed. v40's fixed-width "RXF1" / "RXE1" snapshots are still read.
/// Versions are never deleted: a delta's base must stay (a future pruning
/// has to re-base the children first).
ProjectDB::PoseVersionSnapshot pack_pose_state(const PoseState &state);
/// Unpack a full snapshot (a keyframe or a v40 one).
/// @throws std::runtime_error for a blob this build cannot read, or a delta
///         (it needs its base: use pose_version_state()).
PoseState unpack_pose_state(const ProjectDB::PoseVersionSnapshot &snapshot);

/// The state version @p id holds, its delta chain applied.
/// @throws std::invalid_argument for an unknown id; std::runtime_error for
///         an unreadable blob.
PoseState pose_version_state(const ProjectDB &db, int64_t id);

/// Whether version @p id is stored in full.
/// @throws std::invalid_argument for an unknown id.
bool pose_version_is_keyframe(const ProjectDB &db, int64_t id);

/// The current version, or nullopt when none is recorded.
std::optional<int64_t> current_pose_version(const ProjectDB &db);

struct PoseVersionOptions {
  std::string kind = "apply"; ///< import | baseline | external | apply | ...
  std::string label;
  std::string author;
  std::string metrics_json = "{}";
};

/// Record the live state as a new version, a child of the current one, and
/// make it current. Opens no transaction (call it inside the caller's).
/// @returns the new version's id.
int64_t record_pose_version(ProjectDB &db, const PoseVersionOptions &options);

/// Make sure the live state is a version before a step changes it: records
/// it as "baseline" when no version exists, or as "external" when the live
/// state differs from the current version (a `rux optimize` run since, say).
/// Opens no transaction. @returns the current version id afterwards.
int64_t ensure_pose_version(ProjectDB &db, std::string_view author = "");

/// What a restore / undo / redo changed.
struct PoseVersionChange {
  int64_t from_version = 0; ///< current before (0: none)
  int64_t to_version = 0;   ///< current after
  int frames_changed = 0;   ///< frames whose transform moved
  double max_shift_m = 0.0; ///< largest transform translation change
  int edges_changed = 0;    ///< edge rows added, removed or changed
};

/// Restore version @p id and make it current. Records the live state first
/// when it is not a version yet (ensure_pose_version), so nothing is lost.
/// One transaction. @throws std::invalid_argument for an unknown id.
PoseVersionChange restore_pose_version(ProjectDB &db, int64_t id,
                                       std::string_view author = "");

/// Restore the current version's parent.
/// @throws std::runtime_error when there is nothing to undo.
PoseVersionChange undo_pose_version(ProjectDB &db,
                                    std::string_view author = "");

/// Restore the newest child of the current version.
/// @throws std::runtime_error when there is nothing to redo.
PoseVersionChange redo_pose_version(ProjectDB &db,
                                    std::string_view author = "");

/// One frame that differs between two versions.
struct FrameMove {
  int node_id = 0;
  double shift_m = 0.0;      ///< transform translation change
  double rotation_deg = 0.0; ///< transform rotation change
};

/// Frames whose transform differs by more than the thresholds between
/// versions @p a and @p b (only frames both versions hold), largest shift
/// first. @throws std::invalid_argument for an unknown id.
std::vector<FrameMove> diff_pose_versions(const ProjectDB &db, int64_t a,
                                          int64_t b, double min_shift_m = 1e-4,
                                          double min_rotation_deg = 1e-3);

/// {id, parent_id, kind, label, author, created_at, metrics, frame_count,
///  edge_count, current}
nlohmann::json to_json(const ProjectDB::PoseVersionRecord &record,
                       bool current);
nlohmann::json to_json(const PoseVersionChange &change);
nlohmann::json to_json(const FrameMove &move);

} // namespace reusex::core
