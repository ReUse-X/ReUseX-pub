// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>

namespace reusex {
class ProjectDB;
}

namespace reusex::io {

/// import_rtabmap() in append mode refused a session the project already
/// holds (same node count and first / last timestamps). ruxd maps it to 409.
class DuplicateSessionError : public std::runtime_error {
    public:
  using std::runtime_error::runtime_error;
};

struct ImportRtabmapOptions {
  /// Append the database as one more DETACHED session (reusex-y2lg.30):
  /// refuse a session the project already holds (DuplicateSessionError),
  /// replace RTAB-Map's raw odometry by edges from the imported poses
  /// (core::rebuild_odometry, reusex-y2lg.16), cut the odometry link to the
  /// previous session in the working pose edits (a reject_edge, so optimize
  /// does not bridge it), and record a pose version ("import").
  bool append_session = false;
  /// Recorded on the pose version.
  std::string author;
};

struct ImportRtabmapResult {
  int scan_id = 0;
  int id_offset = 0;
  int frames = 0;
  int first_node = 0; ///< 0 when no frame was imported
  int last_node = 0;
  int odometry_edges = 0; ///< RTAB-Map odometry links stored
  int loop_edges = 0;     ///< RTAB-Map loop links stored
  /// Append mode: odometry_init edges written from the imported poses.
  int odometry_rebuilt = 0;
  /// Append mode: the reject_edge edit cutting the previous session off.
  std::string detach_edit_id;
  std::optional<int64_t> version_id;
};

/// Import raw sensor data from an RTABMap database into a ProjectDB.
///
/// For each node the function extracts color, depth, confidence, the optimized
/// world pose, and camera intrinsics in their original RTABMap format and
/// stores everything in the project database. Images are stored in their native
/// orientation without rotation.
///
/// Also stores RTABMap's graph links in `pose_graph_edges`
/// (Neighbor/NeighborMerged → odometry, Global/LocalSpace/LocalTime/User
/// closures → loop_closure, source `rtabmap`) and each imported pose in
/// `sensor_frames.initial_transform`.
///
/// No heavy processing (voxelization, normal estimation, etc.) is performed.
/// Point cloud generation is handled separately by
/// reusex::geometry::reconstruct_point_clouds().
///
/// @param db  Target project database (must be open in write mode).
/// @param rtabmap_db_path  Path to the source RTABMap .db file.
void import_rtabmap(ProjectDB &db,
                    const std::filesystem::path &rtabmap_db_path);

/// import_rtabmap() with options; see ImportRtabmapOptions.
/// @throws DuplicateSessionError (append mode) before writing anything.
ImportRtabmapResult import_rtabmap(ProjectDB &db,
                                   const std::filesystem::path &rtabmap_db_path,
                                   const ImportRtabmapOptions &options);
} // namespace reusex::io
