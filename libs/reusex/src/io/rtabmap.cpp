// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "io/rtabmap.hpp"
#include "core/ProjectDB.hpp"
#include "core/SensorIntrinsics.hpp"
#include "core/file_fingerprint.hpp"
#include "core/logging.hpp"
#include "core/pose_blocks.hpp"
#include "core/pose_graph.hpp"
#include "core/pose_repair.hpp"
#include "core/pose_versions.hpp"
#include "core/processing_observer.hpp"
#include "core/working_pose_edits.hpp"
#include "io/rtabmap_links.hpp"

#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/Graph.h>
#include <rtabmap/core/Memory.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/util3d.h>

#include <algorithm>
#include <map>
#include <set>
#include <string>
#include <tuple>

static_assert(reusex::io::rtabmap_link_type::neighbor ==
              rtabmap::Link::kNeighbor);
static_assert(reusex::io::rtabmap_link_type::global_closure ==
              rtabmap::Link::kGlobalClosure);
static_assert(reusex::io::rtabmap_link_type::local_space_closure ==
              rtabmap::Link::kLocalSpaceClosure);
static_assert(reusex::io::rtabmap_link_type::local_time_closure ==
              rtabmap::Link::kLocalTimeClosure);
static_assert(reusex::io::rtabmap_link_type::user_closure ==
              rtabmap::Link::kUserClosure);
static_assert(reusex::io::rtabmap_link_type::virtual_closure ==
              rtabmap::Link::kVirtualClosure);
static_assert(reusex::io::rtabmap_link_type::neighbor_merged ==
              rtabmap::Link::kNeighborMerged);
static_assert(reusex::io::rtabmap_link_type::pose_prior ==
              rtabmap::Link::kPosePrior);
static_assert(reusex::io::rtabmap_link_type::landmark ==
              rtabmap::Link::kLandmark);
static_assert(reusex::io::rtabmap_link_type::gravity ==
              rtabmap::Link::kGravity);

namespace reusex::io {

namespace {

/// What identifies a session regardless of its file name: node count and
/// first / last stamp (an upload lands under a new name).
nlohmann::json
session_fingerprint(const std::map<int, rtabmap::Transform> &poses,
                    const std::map<int, rtabmap::Signature> &nodes) {
  double first = -1.0, last = -1.0;
  for (const auto &[id, sig] : nodes) {
    const double s = sig.getStamp();
    if (s <= 0.0)
      continue;
    first = first < 0.0 ? s : std::min(first, s);
    last = std::max(last, s);
  }
  return {
      {"nodes", poses.size()}, {"first_stamp", first}, {"last_stamp", last}};
}

/// The scan already holding a session with @p fp, or -1.
int find_session(const ProjectDB &db, const nlohmann::json &fp) {
  for (const auto &s : db.scans()) {
    try {
      const auto p = nlohmann::json::parse(s.provenance_json);
      if (p.contains("fingerprint") && p["fingerprint"] == fp)
        return s.id;
    } catch (const nlohmann::json::exception &) {
    }
  }
  return -1;
}

} // namespace

void import_rtabmap(ProjectDB &db,
                    const std::filesystem::path &rtabmap_db_path) {
  import_rtabmap(db, rtabmap_db_path, ImportRtabmapOptions{});
}

ImportRtabmapResult import_rtabmap(ProjectDB &db,
                                   const std::filesystem::path &rtabmap_db_path,
                                   const ImportRtabmapOptions &options) {
  core::info("Importing RTABMap database: {}{}", rtabmap_db_path,
             options.append_session ? " (append as a detached session)" : "");
  core::stopwatch timer;
  ImportRtabmapResult result;

  // ── Initialise RTABMap and load the optimised graph ──────────────
  rtabmap::ParametersMap params;
  rtabmap::Rtabmap rtabmap;
  rtabmap.init(params, rtabmap_db_path.c_str());
  // NOTE: RTABMap's working directory is deliberately left unset (issue #245).
  // It is only ever used for optional side outputs — the LogF.txt/LogI.txt
  // statistics logs, gated on `Rtabmap/StatisticLogged` (default false), and a
  // debug-only global scan-map PCD dump from createGlobalScanMap(). This import
  // passes an empty ParametersMap, never calls process(), and getGraph() does
  // not touch it, so the previous setWorkingDirectory() call — which pointed
  // at the process working directory — wrote nothing; it only implied a
  // dependency we do not have. Do not reintroduce it: if statistics logging
  // (or the scan-map dump) is ever enabled here,
  // pass an explicit directory derived from the destination project instead.
  core::debug("RTABMap initialized in {:.3f}s", timer);

  timer.reset();
  std::map<int, rtabmap::Transform> poses;
  std::multimap<int, rtabmap::Link> graph_links; // unused: see link import
  std::map<int, rtabmap::Signature> nodes;
  rtabmap.getGraph(poses, graph_links,
                   /*optimized=*/true, /*global=*/true, &nodes,
                   /*withImages=*/true, /*withScan=*/true,
                   /*withUserData=*/true, /*withGrid=*/true);
  core::debug("Graph loaded in {:.3f}s ({} nodes)", timer, poses.size());

  // Create a scan record and get the ID offset for this session.
  // id_offset = MAX(existing node_id), ensuring new IDs never collide with
  // previously imported sessions. FK-constrained tables (segmentation_images,
  // panoramic_images, glass_confidence_images) cannot hold references above
  // the offset because SQLite FK enforcement is on. pose_graph_edges has no
  // FK; this importer writes RTABMap links for its own (offset) id range only.
  const auto fingerprint = session_fingerprint(poses, nodes);
  const std::string file_fp = core::file_fingerprint(rtabmap_db_path);
  if (options.append_session) {
    int dup = find_session(db, fingerprint);
    if (dup < 0)
      dup = core::scan_with_fingerprint(db, file_fp).value_or(-1);
    if (dup >= 0) {
      rtabmap.close();
      throw DuplicateSessionError(fmt::format(
          "{} is already imported as session (scan) {} ({} nodes, same "
          "content or first and last timestamps); not importing it twice",
          rtabmap_db_path.filename().string(), dup, poses.size()));
    }
  }
  int previous_last = 0; // the last posed frame before this session
  for (const int id : db.sensor_frame_ids())
    if (db.has_sensor_frame_pose(id))
      previous_last = std::max(previous_last, id);
  auto scan = db.create_scan(
      rtabmap_db_path.string(),
      nlohmann::json{{"importer", "rtabmap"},
                     {"append_session", options.append_session},
                     {"fingerprint", fingerprint},
                     {std::string(core::kProvenanceFileFingerprint), file_fp}}
          .dump());
  core::info("Multi-session import: scan id={} id_offset={}", scan.id,
             scan.id_offset);
  result.scan_id = scan.id;
  result.id_offset = scan.id_offset;

  // ── Import each node ─────────────────────────────────────────────
  auto observer =
      core::ProgressObserver(core::Stage::importing_data, poses.size());

  int logId = db.log_pipeline_start(
      "import", fmt::format(R"({{"source":"{}"}})", rtabmap_db_path.string()));

  int imported_count = 0;
  std::map<int, std::array<double, 16>> imported; // RTABMap id -> stored pose
  try {
    for (const auto &[id, pose] : poses) {
      auto nodeIt = nodes.find(id);
      if (nodeIt == nodes.end()) {
        core::warn("Node {} present in poses but missing from signatures, "
                   "skipping",
                   id);
        ++observer;
        continue;
      }

      rtabmap::SensorData data = nodeIt->second.sensorData();
      data.uncompressData();

      // ── Extract raw sensor data ──────────────────────────────────
      cv::Mat color = data.imageRaw();
      cv::Mat depth = data.depthOrRightRaw();
      cv::Mat confidence = data.depthConfidenceRaw();

      if (color.empty()) {
        core::debug("Node {} has no color image, skipping", id);
        ++observer;
        continue;
      }

      // Store images in original RTABMap orientation (no rotation).
      // Any rotation for display should be handled at visualization time.

      // ── Camera intrinsics ────────────────────────────────────────
      core::SensorIntrinsics intrinsics;
      if (!data.cameraModels().empty()) {
        const auto &cm = data.cameraModels().front();
        // Store intrinsics as-is from RTABMap
        intrinsics.fx = cm.fx();
        intrinsics.fy = cm.fy();
        intrinsics.cx = cm.cx();
        intrinsics.cy = cm.cy();
        intrinsics.width = cm.imageWidth();
        intrinsics.height = cm.imageHeight();

        // Convert local transform to row-major array
        const auto &lt = cm.localTransform();
        if (lt.isNull()) {
          core::warn("Node {}: localTransform is null, using identity. "
                     "Per-frame orientation may be incorrect.",
                     id);
        }
        if (!lt.isNull()) {
          const float *d = lt.data();
          for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 4; ++c)
              intrinsics.local_transform[r * 4 + c] =
                  static_cast<double>(d[r * 4 + c]);
          intrinsics.local_transform[12] = 0;
          intrinsics.local_transform[13] = 0;
          intrinsics.local_transform[14] = 0;
          intrinsics.local_transform[15] = 1;
        }
      }

      // ── World pose as row-major 4x4 ─────────────────────────────
      // The RTABMap BASE-frame pose, verbatim (the camera's localTransform
      // lives in the intrinsics). RTABMap links are relative poses between
      // the same base frames, so they need no conversion either.
      std::array<double, 16> worldPose =
          pose.isNull() ? std::array<double, 16>{1, 0, 0, 0, 0, 1, 0, 0,
                                                 0, 0, 1, 0, 0, 0, 0, 1}
                        : rtabmap_transform_to_array(pose.data());

      // ── Timestamp ────────────────────────────────────────────────
      // Try SensorData stamp first, fall back to Signature stamp.
      // RTABMap returns 0.0 when no timestamp is set; treat as "unknown".
      double rawStamp = data.stamp();
      if (rawStamp <= 0.0)
        rawStamp = nodeIt->second.getStamp();
      double stamp = rawStamp > 0.0 ? rawStamp : -1.0;

      // ── Store in ProjectDB ───────────────────────────────────────
      db.save_sensor_frame(id + scan.id_offset, color, depth, confidence,
                           worldPose, intrinsics, stamp, scan.id);
      db.set_initial_pose(id + scan.id_offset, worldPose);
      imported.emplace(id, worldPose);
      const int node = id + scan.id_offset;
      result.first_node =
          result.first_node == 0 ? node : std::min(result.first_node, node);
      result.last_node = std::max(result.last_node, node);

      core::trace("Imported node {} ({}x{}, depth {}x{})", id, color.cols,
                  color.rows, depth.cols, depth.rows);
      ++imported_count;
      ++observer;
    }

    // ── Links -> pose_graph_edges (spec §4.2) ─────────────────────
    // Every link stored in the database, not getGraph()'s links: getGraph
    // returns only the links of its optimized graph and drops stored loop
    // closures it did not keep. Null links are included so the converter
    // counts them (invalid_transform) instead of RTABMap dropping them.
    std::multimap<int, rtabmap::Link> links;
    if (const rtabmap::Memory *memory = rtabmap.getMemory())
      links = memory->getAllLinks(/*lookInDatabase=*/true,
                                  /*ignoreNullLinks=*/false,
                                  /*withLandmarks=*/false);
    else
      core::warn("RTABMap memory unavailable: no links imported, so the "
                 "pose graph has no RTABMap edges. Check that {} is a valid "
                 "RTABMap database",
                 rtabmap_db_path);
    int odometry = 0, loops = 0, duplicates = 0;
    std::map<RtabmapLinkSkip, int> skipped;
    std::map<int, int> unsupported_by_type; // rtabmap link type -> count
    std::set<std::tuple<int, int, std::string>> seen;
    {
      ProjectDB::Transaction tx(db);
      for (const auto &[key, l] : links) {
        RtabmapLink in;
        in.from = l.from();
        in.to = l.to();
        in.type = static_cast<int>(l.type());
        in.transform = l.transform().isNull()
                           ? std::array<double, 16>{}
                           : rtabmap_transform_to_array(l.transform().data());
        const cv::Mat &inf = l.infMatrix();
        if (inf.rows == 6 && inf.cols == 6 && inf.type() == CV_64FC1)
          for (int r = 0; r < 6; ++r)
            for (int c = 0; c < 6; ++c)
              in.information[static_cast<size_t>(r * 6 + c)] =
                  inf.at<double>(r, c);
        const auto conv = convert_rtabmap_link(in, scan.id_offset, imported);
        if (!conv.edge) {
          ++skipped[conv.skip];
          if (conv.skip == RtabmapLinkSkip::unsupported_type)
            ++unsupported_by_type[in.type];
          continue;
        }
        const auto &e = *conv.edge;
        if (!seen.emplace(std::min(e.from_node_id, e.to_node_id),
                          std::max(e.from_node_id, e.to_node_id), e.edge_type)
                 .second) {
          ++duplicates; // the same link listed in both directions
          continue;
        }
        db.upsert_pose_graph_edge(e);
        (e.edge_type == core::pose_graph::kTypeOdometry ? odometry : loops)++;
      }
      tx.commit();
    }
    // Unsupported types (gravity, pose prior, landmark, virtual, ...) and
    // the reverse copy of a link are expected in every RTABMap database.
    const int unsupported = skipped[RtabmapLinkSkip::unsupported_type];
    core::info("RTABMap links: {} odometry + {} loop-closure edges stored "
               "in pose_graph_edges; {} of an unused type (gravity, prior, "
               "landmark, virtual) and {} reverse-direction duplicates "
               "ignored",
               odometry, loops, unsupported, duplicates);
    for (const auto &[type, count] : unsupported_by_type)
      core::debug("RTABMap links of type {} ignored: {}", type, count);
    const int self = skipped[RtabmapLinkSkip::self_link];
    const int dangling = skipped[RtabmapLinkSkip::endpoint_not_imported];
    const int invalid = skipped[RtabmapLinkSkip::invalid_transform];
    if (self + dangling + invalid > 0)
      core::warn("RTABMap links skipped: {} self links, {} touching a node "
                 "that was not imported, {} with a null/non-rigid transform. "
                 "Links to nodes without colour data are expected to be "
                 "dropped; re-export the RTABMap database if frames or "
                 "links are missing",
                 self, dangling, invalid);

    result.odometry_edges = odometry;
    result.loop_edges = loops;
    db.log_pipeline_end(logId, true);
  } catch (...) {
    db.log_pipeline_end(logId, false, "import_rtabmap failed");
    rtabmap.close();
    throw;
  }

  rtabmap.close();
  result.frames = imported_count;
  core::info("Import complete: scan {} ({} frames, id_offset={}), {} total "
             "sensor frames in project",
             scan.id, imported_count, scan.id_offset,
             db.sensor_frame_ids().size());

  if (options.append_session && imported_count > 0) {
    // RTAB-Map's odometry is raw VIO; the poses are loop-corrected
    // (reusex-33j). Replace this session's odometry by the imported poses'.
    const auto odo = core::rebuild_odometry(
        db, {.scan_id = scan.id, .record_version = false});
    result.odometry_rebuilt = odo.written;
    ProjectDB::Transaction tx(db);
    // The session starts detached: optimize must not bridge the gap to the
    // previous session from the initial poses (reusex-y2lg.30).
    if (previous_last > 0) {
      core::put_working_pose_edit(
          db, core::reject_odometry_edit(
                  previous_last, result.first_node,
                  fmt::format("session {} imported detached ({})", scan.id,
                              rtabmap_db_path.filename().string())));
      result.detach_edit_id =
          core::detach_edit_id(previous_last, result.first_node);
    }
    result.version_id = core::record_pose_version(
        db, {.kind = "import",
             .label = fmt::format("import {} as session {}",
                                  rtabmap_db_path.filename().string(), scan.id),
             .author = options.author,
             .metrics_json =
                 nlohmann::json{{"scan_id", scan.id},
                                {"frames", imported_count},
                                {"first_node", result.first_node},
                                {"last_node", result.last_node},
                                {"odometry_rebuilt", odo.written},
                                {"odometry_replaced", odo.replaced_imported},
                                {"loop_edges", result.loop_edges}}
                     .dump()});
    tx.commit();
    core::info("Session {} appended detached: nodes {}..{}, {} odometry "
               "edges from the imported poses, pose version {}",
               scan.id, result.first_node, result.last_node, odo.written,
               *result.version_id);
  }
  return result;
}

} // namespace reusex::io
