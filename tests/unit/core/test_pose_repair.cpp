// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Scan-repair editor, first slice (reusex-y2lg): schema v40 (pose_blocks,
// pose_versions), block suggestion and storage (y2lg.1), odometry rebuild
// (y2lg.16, reusex-xmt), block transform as a re-seed with blend
// (y2lg.9, reusex-om3) and pose versions with undo (y2lg.21).

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/SensorIntrinsics.hpp>
#include <reusex/core/file_fingerprint.hpp>
#include <reusex/core/pose_blocks.hpp>
#include <reusex/core/pose_graph.hpp>
#include <reusex/core/pose_repair.hpp>
#include <reusex/core/pose_versions.hpp>
#include <reusex/core/working_pose_edits.hpp>

#include "../../support/temp_path.hpp"

#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <sqlite3.h>

#include <cmath>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

using Catch::Approx;
using reusex::ProjectDB;
using reusex::test_support::TempPath;
namespace core = reusex::core;
namespace pg = reusex::core::pose_graph;
using json = nlohmann::json;

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

void add_frame(ProjectDB &db, int id, int scan, double t,
               const std::array<double, 16> &pose) {
  db.save_sensor_frame(id, cv::Mat(4, 4, CV_8UC3, cv::Scalar(90, 90, 90)),
                       cv::Mat(), cv::Mat(), pose, intrinsics(), t, scan);
  db.set_initial_pose(id, pose);
}

/// Two sessions: scan 1 = nodes 1..40 walking +x at z = 0 (1 Hz, 0.1 m
/// steps), scan 2 = nodes 41..70 walking +y at z = 0, 600 s later.
/// Odometry edges as an RTAB-Map import stores them (source rtabmap), with
/// a raw measurement that disagrees with the poses by 5 cm.
void two_sessions(ProjectDB &db) {
  REQUIRE(db.create_scan("session-a.db").id == 1);
  REQUIRE(db.create_scan("session-b.db").id == 2);
  for (int i = 1; i <= 40; ++i)
    add_frame(db, i, 1, 1000.0 + i, pose_at(0.1 * i, 0.0, 0.0));
  for (int i = 41; i <= 70; ++i)
    add_frame(db, i, 2, 1600.0 + i, pose_at(4.0, 0.1 * (i - 40), 0.0));
  for (int i = 1; i < 70; ++i) {
    if (i == 40)
      continue; // sessions are imported detached
    ProjectDB::PoseGraphEdge e;
    e.from_node_id = i;
    e.to_node_id = i + 1;
    e.edge_type = std::string(pg::kTypeOdometry);
    e.transform = pose_at(i < 40 ? 0.15 : 0.0, i < 40 ? 0.0 : 0.15, 0.0);
    e.source = std::string(pg::kSourceRtabmap);
    db.upsert_pose_graph_edge(e);
  }
}

std::string file_bytes(const std::filesystem::path &p) {
  std::ifstream in(p, std::ios::binary);
  return {std::istreambuf_iterator<char>(in), {}};
}

void exec_raw(const std::filesystem::path &path, const std::string &sql) {
  sqlite3 *raw = nullptr;
  REQUIRE(sqlite3_open(path.string().c_str(), &raw) == SQLITE_OK);
  char *err = nullptr;
  const int rc = sqlite3_exec(raw, sql.c_str(), nullptr, nullptr, &err);
  INFO((err ? err : ""));
  sqlite3_free(err);
  sqlite3_close(raw);
  REQUIRE(rc == SQLITE_OK);
}

Eigen::Vector3d pos(const std::array<double, 16> &p) {
  return {p[3], p[7], p[11]};
}

} // namespace

// ── Schema v40 ──────────────────────────────────────────────────────────────

TEST_CASE("MigrationV40_V39Project_GetsBlocksAndVersions",
          "[projectdb][migration][pose_repair]") {
  TempPath tmp("test_migration_v40");
  {
    ProjectDB db(tmp.path);
    two_sessions(db);
  }
  exec_raw(tmp.path, R"(
    DROP TABLE pose_blocks;
    DROP TABLE pose_versions;
    DELETE FROM schema_version WHERE version >= 40;
  )");
  {
    ProjectDB ro(tmp.path, /*readOnly=*/true);
    CHECK(ro.schema_version() == 39);
    CHECK(ro.pose_blocks().empty());
    CHECK(ro.pose_versions().empty());
    // Blocks fall back to one per session without the table.
    CHECK(core::pose_blocks(ro).size() == 2);
  }
  ProjectDB db(tmp.path);
  CHECK(db.schema_version() == 40);
  CHECK(ProjectDB::latest_schema_version() >= 40);
  db.replace_pose_blocks(
      {{0, "a", 1, 40, 1, false}, {0, "b", 41, 70, 2, true}});
  REQUIRE(db.pose_blocks().size() == 2);
  CHECK(db.pose_blocks()[1].detached);
  CHECK(db.pose_blocks()[1].id == 2);
  CHECK_THROWS_AS(db.replace_pose_blocks({{0, "x", 9, 3, 1, false}}),
                  std::invalid_argument);
}

// ── Suggestions ─────────────────────────────────────────────────────────────

TEST_CASE("SuggestBlockSplits_SyntheticThreeSessions_FindsEveryReason",
          "[core][pose_repair]") {
  std::vector<core::TrajectorySample> t;
  int id = 0;
  double time = 0.0;
  auto push = [&](int scan, double dt, Eigen::Vector3d p) {
    time += dt;
    t.push_back({++id, scan, time, p});
  };
  // Session A: 60 frames on the ground floor, a 7 s tracking gap at 31,
  // a 150 s pause at 46; frame 20 is a single-frame glitch.
  for (int i = 0; i < 60; ++i) {
    const double dt = i == 30 ? 7.0 : (i == 45 ? 150.0 : 1.0);
    Eigen::Vector3d p(0.1 * i, 0.0, 1.4);
    if (i == 19)
      p.x() += 3.0; // node 20 jumps away and back
    push(1, dt, p);
  }
  // Session B: 40 frames on the ground floor, a stair, then 40 frames
  // 3 m up.
  for (int i = 0; i < 40; ++i)
    push(2, 1.0, {0.1 * i, 2.0, 1.4});
  for (int i = 1; i <= 10; ++i)
    push(2, 1.0, {4.0, 2.0 + 0.2 * i, 1.4 + 0.3 * i});
  for (int i = 0; i < 40; ++i)
    push(2, 1.0, {4.0 - 0.1 * i, 4.0, 4.4});
  // Session C: an odometry step of 2.5 m that does not come back.
  for (int i = 0; i < 30; ++i)
    push(3, 1.0, {0.1 * i + (i >= 15 ? 2.5 : 0.0), -3.0, 1.4});

  const auto s =
      core::suggest_block_splits(std::span<const core::TrajectorySample>(t));
  auto find = [&](core::SplitReason r) {
    std::vector<int> out;
    for (const auto &x : s.splits)
      if (x.reason == r)
        out.push_back(x.node_id);
    return out;
  };
  CHECK(find(core::SplitReason::session) == std::vector<int>{61, 151});
  CHECK(find(core::SplitReason::tracking_gap) == std::vector<int>{31});
  CHECK(find(core::SplitReason::time_gap) == std::vector<int>{46});
  CHECK(find(core::SplitReason::odometry_jump) == std::vector<int>{166});
  const auto storey = find(core::SplitReason::storey_change);
  REQUIRE(storey.size() == 1);
  CHECK(storey[0] > 100);
  CHECK(storey[0] <= 111); // inside the stair run
  REQUIRE(s.glitches.size() == 1);
  CHECK(s.glitches[0].node_id == 20);
  REQUIRE(s.storey_levels.size() == 2);
  CHECK(s.storey_levels[0] == Approx(1.4).margin(0.15));
  CHECK(s.storey_levels[1] == Approx(4.4).margin(0.15));
  for (const auto &x : s.splits) {
    CHECK(x.score > 0.0);
    CHECK(x.score <= 1.0);
    CHECK_FALSE(x.detail.empty());
  }
  const json j = core::to_json(s);
  CHECK(j["splits"].size() == s.splits.size());
  CHECK(j["splits"][0]["reason"] == "tracking_gap");
}

// ── Stored blocks ──────────────────────────────────────────────────────────

TEST_CASE("PoseBlocks_SplitMergeRenameDetach", "[core][pose_repair]") {
  TempPath tmp("test_pose_blocks");
  ProjectDB db(tmp.path);
  two_sessions(db);

  auto blocks = core::pose_blocks(db);
  REQUIRE(blocks.size() == 2);
  CHECK(blocks[0].name == "s0");
  CHECK_FALSE(blocks[0].detached);
  CHECK(blocks[1].name == "s1");
  CHECK(blocks[1].detached); // sessions start detached

  blocks = core::split_pose_block(db, 21, "far end");
  REQUIRE(blocks.size() == 3);
  CHECK(blocks[0].last_node == 20);
  CHECK(blocks[1].first_node == 21);
  CHECK(blocks[1].name == "far end");
  CHECK_THROWS_AS(core::split_pose_block(db, 21), std::invalid_argument);
  CHECK_THROWS_AS(core::split_pose_block(db, 999), std::invalid_argument);

  blocks = core::rename_pose_block(db, 1, "near end");
  CHECK(blocks[0].name == "near end");

  // A block reaching into the other session is refused, and so is a gap.
  auto bad = blocks;
  bad[1].last_node = 45;
  CHECK_THROWS_AS(core::set_pose_blocks(db, bad), std::invalid_argument);
  bad = blocks;
  bad[1].first_node = 25;
  CHECK_THROWS_AS(core::set_pose_blocks(db, bad), std::invalid_argument);

  // Detach the far end: a reject_edge edit and a rejected odometry row.
  const auto d = core::detach_pose_block(db, 2);
  CHECK(d.prev_node_id == 20);
  CHECK(d.node_id == 21);
  CHECK(d.edge_rejected);
  CHECK(d.holding_loops == 0);
  CHECK(d.blocks[1].detached);
  const auto edits = json::parse(core::working_pose_edits(db));
  REQUIRE(edits["edits"].size() == 1);
  CHECK(edits["edits"][0]["id"] == "detach-20-21");
  CHECK(edits["edits"][0]["kind"] == "reject_edge");
  CHECK(db.pose_graph_edge(20, 21, "odometry")->status == "rejected");

  // Re-attach puts the odometry back from the initial poses.
  core::detach_pose_block(db, 2, /*attach=*/true);
  CHECK(json::parse(core::working_pose_edits(db))["edits"].empty());
  const auto e = db.pose_graph_edge(20, 21, "odometry");
  REQUIRE(e);
  CHECK(e->status == "auto");
  CHECK(e->source == "odometry_init");
  CHECK((*e->transform)[3] == Approx(0.1));

  blocks = core::merge_pose_blocks(db, 1, 2);
  REQUIRE(blocks.size() == 2);
  CHECK(blocks[0].last_node == 40);
  CHECK_THROWS_AS(core::merge_pose_blocks(db, 1, 2), std::invalid_argument);

  // Splits from suggestions.
  const auto from = core::blocks_from_splits(db, {10, 50});
  REQUIRE(from.size() == 4);
  CHECK(from[1].first_node == 10);
  CHECK(from[2].first_node == 41);
  CHECK(from[2].detached);
  CHECK(from[3].first_node == 50);
}

TEST_CASE("WorkingPoseEdits_RoundTripAndShapeCheck", "[core][pose_repair]") {
  TempPath tmp("test_working_edits");
  ProjectDB db(tmp.path);
  CHECK(json::parse(core::working_pose_edits(db))["edits"].empty());
  core::put_working_pose_edit(
      db,
      core::hold_edit("h1", 3, 9, core::Dof::x | core::Dof::y, 0.05, 1.0, ""));
  auto doc = json::parse(core::working_pose_edits(db));
  REQUIRE(doc["edits"].size() == 1);
  CHECK(doc["edits"][0]["dofs"] == json::array({"x", "y"}));
  core::set_working_pose_edits(db, doc.dump());
  CHECK(json::parse(core::working_pose_edits(db)) == doc);
  CHECK_THROWS_AS(core::set_working_pose_edits(db, "{\"edits\": []}"),
                  std::invalid_argument); // no schema
  doc["edits"].push_back(doc["edits"][0]);
  CHECK_THROWS_AS(core::set_working_pose_edits(db, doc.dump()),
                  std::invalid_argument); // duplicate id
  CHECK(core::remove_working_pose_edit(db, "h1"));
  CHECK_FALSE(core::remove_working_pose_edit(db, "h1"));
}

// ── Odometry rebuild ────────────────────────────────────────────────────────

TEST_CASE("RebuildOdometry_ReplacesRawImportedOdometry",
          "[core][pose_repair]") {
  TempPath tmp("test_rebuild_odometry");
  ProjectDB db(tmp.path);
  two_sessions(db);
  db.set_pose_graph_edge_status(5, 6, "odometry", "confirmed");

  const auto r = core::rebuild_odometry(db);
  CHECK(r.pairs == 68); // 39 + 29, nothing across the sessions
  CHECK(r.kept_confirmed == 1);
  CHECK(r.written == 67);
  CHECK(r.replaced_imported == 67);
  CHECK(r.max_change_m == Approx(0.05).margin(1e-9));
  REQUIRE(r.version_id);
  CHECK(db.pose_version(*r.version_id)->kind == "odometry");
  const auto e = db.pose_graph_edge(10, 11, "odometry");
  CHECK(e->source == "odometry_init");
  CHECK((*e->transform)[3] == Approx(0.1));
  CHECK(db.pose_graph_edge(5, 6, "odometry")->source == "rtabmap");
  CHECK_FALSE(db.pose_graph_edge(40, 41, "odometry"));

  // reusex-xmt: a new seed for one frame re-derives the edges around it.
  db.set_initial_pose(30, pose_at(3.0, 0.5, 0.0));
  const auto again = core::rebuild_odometry(db, {.record_version = false});
  CHECK(again.changed == 2);
  CHECK(again.max_change_m == Approx(0.5).margin(1e-9));
  CHECK((*db.pose_graph_edge(29, 30, "odometry")->transform)[7] == Approx(0.5));
}

// ── Block transform ─────────────────────────────────────────────────────────

TEST_CASE("BlockTransform_PreviewWritesNothing", "[core][pose_repair]") {
  TempPath tmp("test_block_transform_preview");
  {
    ProjectDB db(tmp.path);
    two_sessions(db);
  }
  {
    ProjectDB db(tmp.path); // a no-op open/close, so the bytes settle
  }
  const std::string before = file_bytes(tmp.path);
  {
    ProjectDB db(tmp.path);
    core::BlockTransformRequest req;
    req.block_ids = {2};
    req.delta.xyz = {0.0, 0.0, -2.5};
    req.preview = true;
    const auto r = core::apply_block_transform(db, req);
    CHECK(r.preview);
    CHECK(r.frames_moved == 30);
    CHECK_FALSE(r.version_id);
    REQUIRE(r.frames.size() == 30);
    CHECK(pos(r.frames[0].after).z() == Approx(-2.5));
    CHECK(db.pose_versions().empty());
    CHECK(pos(db.sensor_frame_pose(50)).z() == Approx(0.0));
  }
  CHECK(file_bytes(tmp.path) == before);
}

TEST_CASE("BlockTransform_MoveToStoreyAndRotate90_ExactInsideBlendAtJoint",
          "[core][pose_repair]") {
  TempPath tmp("test_block_transform_apply");
  ProjectDB db(tmp.path);
  two_sessions(db);
  core::split_pose_block(db, 21); // blocks: 1..20, 21..40, 41..70

  // Session 2 (detached, a session boundary on both ends): a storey down
  // plus 90 deg about its first camera. Rigid, no blend.
  core::BlockTransformRequest req;
  req.block_ids = {3};
  req.delta.xyz = {0.0, 0.0, -2.8};
  req.delta.yaw_deg = 90.0;
  req.delta.pivot = std::array<double, 3>{4.0, 0.1, 0.0};
  req.hold = core::BlockTransformHold{};
  const auto r = core::apply_block_transform(db, req);
  CHECK(r.frames_moved == 30);
  CHECK(r.frames_blended == 0);
  REQUIRE(r.version_id);
  for (int id : {41, 55, 70}) {
    const auto p = db.sensor_frame_pose(id);
    // y-walk turned by +90 deg about (4, 0.1): (4, 0.1 + d) -> (4 - d, 0.1)
    const double d = 0.1 * (id - 40) - 0.1;
    CHECK(p[3] == Approx(4.0 - d).margin(1e-3));
    CHECK(p[7] == Approx(0.1).margin(1e-3));
    CHECK(p[11] == Approx(-2.8).margin(1e-3));
    CHECK(std::atan2(p[4], p[0]) * 180.0 / M_PI == Approx(90.0).margin(0.01));
    CHECK(*db.initial_pose(id) == p); // re-seeded
  }
  CHECK(r.hold_edit_id == std::optional<std::string>("hold-41-70"));
  CHECK(json::parse(core::working_pose_edits(db))["edits"][0]["kind"] ==
        "move");
  // Odometry re-derived from the new seed: the moved session's edges are
  // seed-consistent, nothing bridges the sessions.
  REQUIRE(r.odometry);
  const auto e = db.pose_graph_edge(50, 51, "odometry");
  CHECK(e->source == "odometry_init");
  CHECK((*e->transform)[7] == Approx(0.1).margin(1e-9)); // still body +y
  CHECK_FALSE(db.pose_graph_edge(40, 41, "odometry"));

  // The middle block of session 1, lifted 1 m with a 5-frame blend at the
  // attached joint to block 1; its other end is the session's end.
  core::BlockTransformRequest up;
  up.block_ids = {2};
  up.delta.xyz = {0.0, 0.0, 1.0};
  up.blend_frames = 5;
  const auto u = core::apply_block_transform(db, up);
  CHECK(u.frames_blended == 5);
  CHECK(u.frames_moved == 15);
  double prev_z = 0.0;
  for (int id = 21; id <= 26; ++id) {
    const double z = pos(db.sensor_frame_pose(id)).z();
    CHECK(z > prev_z); // smooth ramp from the joint
    CHECK(z <= 1.0 + 1e-9);
    prev_z = z;
  }
  CHECK(pos(db.sensor_frame_pose(26)).z() == Approx(1.0));
  CHECK(pos(db.sensor_frame_pose(20)).z() == Approx(0.0)); // not moved
  REQUIRE(u.joints.size() == 2);
  CHECK(u.joints[0].blend_frames == 5);
  CHECK(u.joints[0].gap_after_m < 0.3); // the joint stays together
  CHECK(u.joints[1].session_boundary);
}

TEST_CASE("BlockTransform_DetachedJoint_IsRigid", "[core][pose_repair]") {
  TempPath tmp("test_block_transform_detached");
  ProjectDB db(tmp.path);
  two_sessions(db);
  core::split_pose_block(db, 21);
  core::detach_pose_block(db, 2);
  core::BlockTransformRequest req;
  req.block_ids = {2};
  req.delta.xyz = {0.5, 0.0, 0.0};
  const auto r = core::apply_block_transform(db, req);
  CHECK(r.frames_moved == 20);
  CHECK(r.frames_blended == 0);
  CHECK(r.joints[0].detached);
  // The cut stays cut through the odometry rebuild.
  CHECK(db.pose_graph_edge(20, 21, "odometry")->status == "rejected");
  CHECK(r.odometry->cut_detached == 1);
}

TEST_CASE("BlockTransform_RequestFromJson", "[core][pose_repair]") {
  const auto req = core::block_transform_request_from_json(json::parse(R"({
    "blocks": [3], "nodes": [[10, 20], 33],
    "delta": {"xyz": [0, 0, -2.8], "yaw_deg": 90, "pivot": [4, 0.1, 0]},
    "mode": "local", "blend_frames": 0,
    "hold": {"dofs": ["x", "y", "yaw"], "sigma_trans_m": 0.05},
    "preview": true, "label": "s2 down"
  })"));
  CHECK(req.block_ids == std::vector<int>{3});
  REQUIRE(req.node_ranges.size() == 2);
  CHECK(req.node_ranges[1] == std::pair<int, int>{33, 33});
  CHECK(req.delta.yaw_deg == 90.0);
  CHECK(req.mode == core::BlockTransformMode::local);
  REQUIRE(req.hold);
  CHECK(req.hold->dofs == (core::Dof::x | core::Dof::y | core::Dof::yaw));
  CHECK(req.preview);
  CHECK_THROWS_AS(core::block_transform_request_from_json(
                      json::parse(R"({"delta": {"xyz": [1, 2]}})")),
                  std::invalid_argument);
  CHECK_THROWS_AS(core::block_transform_request_from_json(
                      json::parse(R"({"delta": {}, "bogus": 1})")),
                  std::invalid_argument);
}

TEST_CASE("BlendWeight_SmoothstepRamp", "[core][pose_repair]") {
  CHECK(core::blend_weight(1, 0) == 1.0);
  CHECK(core::blend_weight(6, 5) == 1.0);
  CHECK(core::blend_weight(1, 5) > 0.0);
  CHECK(core::blend_weight(1, 5) < core::blend_weight(2, 5));
  CHECK(core::blend_weight(5, 5) < 1.0);
  const auto D = pose_at(1.0, 0.0, 0.0, 90.0);
  const auto half = core::partial_delta(D, {0, 0, 0}, 0.5);
  CHECK(std::atan2(half[4], half[0]) * 180.0 / M_PI == Approx(45.0));
  CHECK(half[3] == Approx(0.5));
  const auto full = core::partial_delta(D, {2, 1, 0}, 1.0);
  for (size_t k = 0; k < 16; ++k)
    CHECK(full[k] == Approx(D[k]).margin(1e-12));
}

// ── Pose versions ───────────────────────────────────────────────────────────

TEST_CASE("PoseVersions_UndoRedoRestore_ByteIdentical", "[core][pose_repair]") {
  TempPath tmp("test_pose_versions");
  ProjectDB db(tmp.path);
  two_sessions(db);
  const auto before = core::pack_pose_state(core::capture_pose_state(db));

  core::BlockTransformRequest req;
  req.block_ids = {2};
  req.delta.xyz = {1.0, 2.0, -2.5};
  req.delta.yaw_deg = 30.0;
  req.hold = core::BlockTransformHold{};
  const auto r = core::apply_block_transform(db, req);
  auto versions = db.pose_versions();
  REQUIRE(versions.size() == 2);
  CHECK(versions[0].kind == "baseline");
  CHECK(versions[1].kind == "apply");
  CHECK(versions[1].parent_id == versions[0].id);
  CHECK(core::current_pose_version(db) == r.version_id);
  const auto after = core::pack_pose_state(core::capture_pose_state(db));
  CHECK(after.frames != before.frames);

  const auto moved =
      core::diff_pose_versions(db, versions[0].id, versions[1].id);
  CHECK(moved.size() == 30);
  CHECK(moved.front().node_id >= 41);

  const auto undo = core::undo_pose_version(db);
  CHECK(undo.to_version == versions[0].id);
  CHECK(undo.frames_changed == 30);
  const auto restored = core::pack_pose_state(core::capture_pose_state(db));
  CHECK(restored.frames == before.frames);
  CHECK(restored.edges == before.edges);
  CHECK(restored.edits == before.edits);
  CHECK_THROWS_AS(core::undo_pose_version(db), std::runtime_error);

  const auto redo = core::redo_pose_version(db);
  CHECK(redo.to_version == versions[1].id);
  const auto again = core::pack_pose_state(core::capture_pose_state(db));
  CHECK(again.frames == after.frames);
  CHECK(again.edges == after.edges);
  CHECK(again.edits == after.edits);
  CHECK_THROWS_AS(core::redo_pose_version(db), std::runtime_error);

  // A change made outside the editor becomes a version before an undo, so
  // the undo goes back to the last editor version and nothing is lost.
  db.update_sensor_frame_pose(3, pose_at(9.0, 9.0, 9.0));
  const auto u2 = core::undo_pose_version(db);
  versions = db.pose_versions();
  REQUIRE(versions.size() == 3);
  CHECK(versions[2].kind == "external");
  CHECK(u2.to_version == versions[1].id);
  CHECK(pos(db.sensor_frame_pose(3)).x() == Approx(0.3));
  core::restore_pose_version(db, versions[2].id);
  CHECK(pos(db.sensor_frame_pose(3)).x() == Approx(9.0));
  CHECK_THROWS_AS(core::restore_pose_version(db, 999), std::invalid_argument);

  const json j = core::to_json(versions[1], true);
  CHECK(j["kind"] == "apply");
  CHECK(j["metrics"]["frames_moved"] == 30);
}

TEST_CASE("PoseVersions_RestoreKeepsLaterSessions", "[core][pose_repair]") {
  TempPath tmp("test_pose_versions_later");
  ProjectDB db(tmp.path);
  db.create_scan("a.db");
  for (int i = 1; i <= 10; ++i)
    add_frame(db, i, 1, i, pose_at(0.1 * i, 0, 0));
  core::rebuild_odometry(db); // baseline + odometry version
  const auto v = *core::current_pose_version(db);
  // A second session imported after that version.
  db.create_scan("b.db");
  for (int i = 11; i <= 20; ++i)
    add_frame(db, i, 2, 100 + i, pose_at(0.1 * i, 5, 0));
  core::rebuild_odometry(db, {.scan_id = 2});
  const auto edges = db.list_pose_graph_edges().size();
  CHECK(edges == 18);
  core::restore_pose_version(db, v);
  // The later session's frames and edges are untouched by the restore.
  CHECK(db.list_pose_graph_edges().size() == edges);
  CHECK(db.pose_graph_edge(15, 16, "odometry"));
}

TEST_CASE("FileFingerprint_ContentNotName_FindsTheScan",
          "[core][pose_repair]") {
  TempPath a("test_fingerprint_a", ".db");
  TempPath b("test_fingerprint_b", ".db");
  {
    std::ofstream(a.path, std::ios::binary) << "SQLite format 3 session";
  }
  {
    std::ofstream(b.path, std::ios::binary) << "SQLite format 3 session";
  }
  const auto fa = core::file_fingerprint(a.path);
  CHECK(fa.rfind("fnv1a64:", 0) == 0);
  CHECK(fa == core::file_fingerprint(b.path)); // same bytes, other name
  {
    std::ofstream(b.path, std::ios::binary) << "SQLite format 3 other";
  }
  CHECK(fa != core::file_fingerprint(b.path));
  CHECK_THROWS_AS(core::file_fingerprint("/nonexistent/x.db"),
                  std::runtime_error);

  TempPath tmp("test_fingerprint_db");
  ProjectDB db(tmp.path);
  CHECK_FALSE(core::scan_with_fingerprint(db, fa));
  const auto scan =
      db.create_scan("x.db", json{{"file_fingerprint", fa}}.dump());
  CHECK(core::scan_with_fingerprint(db, fa) == scan.id);
}
