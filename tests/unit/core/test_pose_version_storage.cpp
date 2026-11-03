// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// How pose versions are stored (reusex-md4a): a version keeps only what
// differs from its parent (frames whose poses changed, edge rows that
// changed, metrics-only edge changes as a short record), with a full
// keyframe every core::kPoseVersionKeyframeInterval links so a restore walks
// a bounded chain. Restoring any version must still give back the captured
// state byte for byte, and v40's full snapshots ("RXF1" / "RXE1") stay
// readable.

#include <catch2/catch_test_macros.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/SensorIntrinsics.hpp>
#include <reusex/core/pose_graph.hpp>
#include <reusex/core/pose_versions.hpp>
#include <reusex/core/working_pose_edits.hpp>

#include "../../support/temp_path.hpp"

#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <random>
#include <string>
#include <vector>

using reusex::ProjectDB;
using reusex::test_support::TempPath;
namespace core = reusex::core;
namespace pg = reusex::core::pose_graph;

namespace {

std::array<double, 16> pose_at(double x, double y, double z,
                               double yaw_deg = 0.0) {
  const double c = std::cos(yaw_deg * M_PI / 180.0);
  const double s = std::sin(yaw_deg * M_PI / 180.0);
  return {c, -s, 0, x, s, c, 0, y, 0, 0, 1, z, 0, 0, 0, 1};
}

core::SensorIntrinsics intrinsics() {
  core::SensorIntrinsics i;
  i.fx = i.fy = 100.0;
  i.cx = i.cy = 50.0;
  i.width = i.height = 100;
  return i;
}

void add_frame(ProjectDB &db, int id, double t,
               const std::array<double, 16> &pose) {
  db.save_sensor_frame(id, cv::Mat(4, 4, CV_8UC3, cv::Scalar(90, 90, 90)),
                       cv::Mat(), cv::Mat(), pose, intrinsics(), t, 1);
  db.set_initial_pose(id, pose);
}

/// @p n frames walking +x with one odometry edge per consecutive pair, each
/// carrying a full (non-diagonal) information matrix and seed metrics.
void chain(ProjectDB &db, int n) {
  db.create_scan("a.db");
  for (int i = 1; i <= n; ++i)
    add_frame(db, i, 1000.0 + i, pose_at(0.1 * i, 0.0, 0.0, 0.01 * i));
  for (int i = 1; i < n; ++i) {
    ProjectDB::PoseGraphEdge e;
    e.from_node_id = i;
    e.to_node_id = i + 1;
    e.edge_type = std::string(pg::kTypeOdometry);
    e.transform = pose_at(0.1, 0.0, 0.0, 0.01);
    std::array<double, 36> info{};
    for (int k = 0; k < 6; ++k)
      info[static_cast<size_t>(k * 7)] = 100.0 + i;
    info[1] = info[6] = 0.5; // symmetric off-diagonal term
    e.information = info;
    e.source = std::string(pg::kSourceRtabmap);
    e.seed_disagreement_m = 0.001 * i;
    db.upsert_pose_graph_edge(e);
  }
}

/// The live state packed in full: the reference a restore is checked with.
ProjectDB::PoseVersionSnapshot live(const ProjectDB &db) {
  return core::pack_pose_state(core::capture_pose_state(db));
}

void check_same(const ProjectDB::PoseVersionSnapshot &a,
                const ProjectDB::PoseVersionSnapshot &b) {
  CHECK(a.frames == b.frames);
  CHECK(a.edges == b.edges);
  CHECK(a.edits == b.edits);
}

size_t stored_bytes(const ProjectDB &db, int64_t id) {
  const auto s = db.pose_version_snapshot(id);
  REQUIRE(s);
  return s->frames.size() + s->edges.size();
}

/// What `rux optimize` writes into every edge: new metrics, nothing else.
void write_metrics(ProjectDB &db, double scale) {
  std::vector<ProjectDB::PoseGraphEdgeMetrics> m;
  for (const auto &e : db.list_pose_graph_edges())
    m.push_back({e.from_node_id, e.to_node_id, e.edge_type,
                 0.01 * scale * e.from_node_id, 0.002 * scale, 0.03 * scale,
                 std::nullopt, std::nullopt});
  db.update_pose_graph_edge_metrics(m);
}

} // namespace

TEST_CASE("PoseVersionStorage_RandomSteps_EveryVersionRestoresByteIdentical",
          "[core][pose_versions]") {
  TempPath tmp("test_pose_version_storage_random");
  ProjectDB db(tmp.path);
  chain(db, 30);

  std::mt19937 rng(7);
  std::uniform_int_distribution<int> node(1, 30);
  std::uniform_real_distribution<double> d(-0.5, 0.5);
  std::vector<std::pair<int64_t, ProjectDB::PoseVersionSnapshot>> taken;

  // More steps than a keyframe interval, so the chain crosses keyframes.
  const int steps = 3 * core::kPoseVersionKeyframeInterval + 5;
  for (int step = 0; step < steps; ++step) {
    switch (step % 7) {
    case 0: // move a few frames
      for (int k = 0; k < 3; ++k)
        db.update_sensor_frame_pose(node(rng), pose_at(d(rng), d(rng), d(rng)));
      break;
    case 1: // re-seed one frame
      db.set_initial_pose(node(rng), pose_at(d(rng), 1.0, 0.0, 5.0));
      break;
    case 2: // optimize-style metrics on every edge
      write_metrics(db, 1.0 + step);
      break;
    case 3: { // a loop edge appears, with a full (non-affine) transform
      ProjectDB::PoseGraphEdge e;
      e.from_node_id = node(rng);
      e.to_node_id = e.from_node_id < 30 ? 30 : 1;
      e.edge_type = std::string(pg::kTypeLoopClosure);
      auto T = pose_at(d(rng), d(rng), 0.0);
      T[12] = 1e-9; // keep a non-rigid bottom row exactly
      e.transform = T;
      e.source = std::string(pg::kSourceManual);
      db.upsert_pose_graph_edge(e);
      break;
    }
    case 4: // a status change
      if (!db.list_pose_graph_edges().empty()) {
        const auto e = db.list_pose_graph_edges().front();
        db.set_pose_graph_edge_status(e.from_node_id, e.to_node_id, e.edge_type,
                                      step % 2 ? pg::kStatusRejected
                                               : pg::kStatusConfirmed);
      }
      break;
    case 5: { // edges dropped (a restore-style replace of the table)
      auto edges = db.list_pose_graph_edges();
      if (edges.size() > 5)
        edges.erase(edges.begin() + 2);
      db.replace_pose_graph_edges(edges);
      break;
    }
    case 6: // a working edit; one frame loses its initial pose
      core::put_working_pose_edit(
          db, core::reject_odometry_edit(step % 29 + 1, step % 29 + 2, ""));
      db.clear_initial_pose(node(rng));
      break;
    }
    const int64_t id = core::record_pose_version(db, {.kind = "apply"});
    taken.emplace_back(id, live(db));
  }

  // Restore in a scrambled order; each must reproduce its capture exactly.
  std::vector<size_t> order(taken.size());
  for (size_t i = 0; i < order.size(); ++i)
    order[i] = i;
  std::shuffle(order.begin(), order.end(), rng);
  for (const size_t i : order) {
    INFO("version " << taken[i].first);
    core::restore_pose_version(db, taken[i].first);
    check_same(live(db), taken[i].second);
    check_same(
        core::pack_pose_state(core::pose_version_state(db, taken[i].first)),
        taken[i].second);
  }
}

TEST_CASE("PoseVersionStorage_SmallStep_StoresOnlyTheDifference",
          "[core][pose_versions]") {
  TempPath tmp("test_pose_version_storage_small");
  ProjectDB db(tmp.path);
  chain(db, 400);
  const int64_t base = core::record_pose_version(db, {.kind = "baseline"});

  db.update_sensor_frame_pose(17, pose_at(9.0, 9.0, 9.0));
  const int64_t step = core::record_pose_version(db, {.kind = "apply"});
  INFO("baseline " << stored_bytes(db, base) << " B, one-frame step "
                   << stored_bytes(db, step) << " B");
  CHECK(stored_bytes(db, step) * 100 < stored_bytes(db, base));

  // An optimize run rewrites every pose and every edge's metrics: still
  // well under a full snapshot, because the edges carry only their metrics.
  for (int i = 1; i <= 400; ++i)
    db.update_sensor_frame_pose(i, pose_at(0.1 * i + 0.01, 0.02, 0.0));
  write_metrics(db, 2.0);
  const int64_t opt = core::record_pose_version(db, {.kind = "external"});
  INFO("optimize-style step " << stored_bytes(db, opt) << " B");
  CHECK(stored_bytes(db, opt) * 2 < stored_bytes(db, base));

  // The full keyframe itself is smaller than v40's fixed-width packing
  // (frames 4+4 + 400 * 265 B, edges 4+4 + 399 * ~520 B).
  CHECK(stored_bytes(db, base) < (8 + 400 * 265 + 8 + 399 * 520) * 3 / 4);
}

TEST_CASE("PoseVersionStorage_KeyframeInterval_BoundsTheChain",
          "[core][pose_versions]") {
  TempPath tmp("test_pose_version_storage_keyframes");
  ProjectDB db(tmp.path);
  chain(db, 20);
  std::vector<int64_t> ids;
  for (int i = 0; i < 2 * core::kPoseVersionKeyframeInterval + 1; ++i) {
    db.update_sensor_frame_pose(1 + i % 20, pose_at(i, 0, 0));
    ids.push_back(core::record_pose_version(db, {.kind = "apply"}));
  }
  int keyframes = 0;
  for (const auto id : ids)
    keyframes += core::pose_version_is_keyframe(db, id) ? 1 : 0;
  CHECK(core::pose_version_is_keyframe(db, ids.front()));
  CHECK(keyframes == 3);
}

namespace {

/// A v40 snapshot ("RXF1" / "RXE1", fixed width), written byte by byte the
/// way schema v40's packer did.
struct V1Writer {
  std::vector<std::uint8_t> out;
  template <typename T> void put(const T &v) {
    const auto *p = reinterpret_cast<const std::uint8_t *>(&v);
    out.insert(out.end(), p, p + sizeof(T));
  }
  void put_str(const std::string &s) {
    put(static_cast<std::uint32_t>(s.size()));
    out.insert(out.end(), s.begin(), s.end());
  }
};

} // namespace

TEST_CASE("PoseVersionStorage_V40FullSnapshot_StillRestores",
          "[core][pose_versions]") {
  TempPath tmp("test_pose_version_storage_v1");
  ProjectDB db(tmp.path);
  chain(db, 3);
  const auto edges = db.list_pose_graph_edges();
  const auto frames = db.posed_frames();

  ProjectDB::PoseVersionSnapshot snap;
  V1Writer f;
  f.put(std::uint32_t{0x31465852});
  f.put(static_cast<std::uint32_t>(frames.size()));
  for (const auto &p : frames) {
    f.put(static_cast<std::int32_t>(p.node_id));
    f.put(p.transform);
    f.put(static_cast<std::uint8_t>(p.initial ? 1 : 0));
    if (p.initial)
      f.put(*p.initial);
  }
  snap.frames = f.out;
  V1Writer e;
  e.put(std::uint32_t{0x31455852});
  e.put(static_cast<std::uint32_t>(edges.size()));
  auto opt = [&](const std::optional<double> &v) {
    e.put(static_cast<std::uint8_t>(v ? 1 : 0));
    e.put(v.value_or(0.0));
  };
  for (const auto &x : edges) {
    e.put(static_cast<std::int64_t>(x.id));
    e.put(static_cast<std::int32_t>(x.from_node_id));
    e.put(static_cast<std::int32_t>(x.to_node_id));
    e.put_str(x.edge_type);
    e.put(x.residual);
    e.put(x.weight);
    e.put(static_cast<std::uint8_t>((x.transform ? 1 : 0) |
                                    (x.information ? 2 : 0)));
    if (x.transform)
      e.put(*x.transform);
    if (x.information)
      e.put(*x.information);
    e.put_str(x.source);
    e.put_str(x.status);
    e.put_str(x.created_at);
    opt(x.seed_disagreement_m);
    opt(x.seed_disagreement_deg);
    opt(x.deviation_m);
    opt(x.deviation_deg);
  }
  snap.edges = e.out;
  ProjectDB::PoseVersionRecord rec;
  rec.kind = "baseline";
  const int64_t v1 = db.add_pose_version(rec, snap);
  db.set_setting(core::kSettingPoseVersionCurrent, std::to_string(v1));
  const auto before = live(db);

  // A delta on top of the v40 snapshot, then back to it.
  db.update_sensor_frame_pose(2, pose_at(5, 5, 5));
  const int64_t v2 = core::record_pose_version(db, {.kind = "apply"});
  CHECK_FALSE(core::pose_version_is_keyframe(db, v2));
  core::restore_pose_version(db, v1);
  check_same(live(db), before);
  CHECK(core::unpack_pose_state(snap).frames.size() == 3);
}
