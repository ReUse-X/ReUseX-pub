// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Wall topology edits for the Rediger editor (reusex-yuja): split, merge,
// join (extend / trim), and their wire ops through core::apply_model_edit.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/model_edits.hpp>
#include <reusex/core/wall_edit_ops.hpp>

#include "../../support/temp_path.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <string>
#include <vector>

using Catch::Matchers::WithinAbs;
using namespace reusex;
using reusex::test_support::TempPath;

namespace {

core::WallRecord wall(const std::string &guid, double x0, double y0, double x1,
                      double y1, double thickness = 0.2) {
  core::WallRecord w;
  w.guid = guid;
  w.x0 = x0;
  w.y0 = y0;
  w.x1 = x1;
  w.y1 = y1;
  w.height = 2.5;
  w.thickness = thickness;
  return w;
}

core::OpeningRecord opening(const std::string &guid, const std::string &host,
                            double s0, double s1) {
  core::OpeningRecord o;
  o.guid = guid;
  o.wall = host;
  o.kind = "window";
  o.s_start = s0;
  o.s_end = s1;
  o.z_sill = 0.9;
  o.z_head = 2.1;
  return o;
}

const core::OpeningRecord &find(const std::vector<core::OpeningRecord> &os,
                                const std::string &guid) {
  const auto it = std::find_if(os.begin(), os.end(),
                               [&](const auto &o) { return o.guid == guid; });
  REQUIRE(it != os.end());
  return *it;
}

} // namespace

TEST_CASE("WallEditOps_Split_RehostsOpeningsAndJoins",
          "[core][wall_edit_ops]") {
  const auto south = wall("south", 0, 0, 4, 0);
  const auto west = wall("west", 0, 0, 0, 3);
  const auto east = wall("east", 4, 0, 4, 3);
  const auto mid = wall("mid", 3, 0.1, 3, 3); // a tee onto the south wall
  const std::vector<core::OpeningRecord> openings = {
      opening("door", "south", 1.0, 1.9), opening("window", "south", 3.0, 3.5),
      opening("across", "south", 1.8, 2.3), opening("other", "west", 1, 2)};
  const std::vector<core::WallJoinRecord> joins = {
      {"south", "west", "corner", "start", "start"},
      {"east", "south", "corner", "start", "end"},
      {"mid", "south", "tee", "start", "path"}};

  const auto r = core::split_wall(south, 2.0, openings, joins,
                                  {south, west, east, mid}, "south-b");
  CHECK(r.a.guid == "south");
  CHECK(r.b.guid == "south-b");
  CHECK(r.a.x1 == 2.0);
  CHECK(r.b.x0 == 2.0);
  CHECK(r.b.x1 == 4.0);
  CHECK(r.b.thickness == 0.2);
  REQUIRE(r.openings.size() == 3); // not the west wall's
  CHECK(find(r.openings, "door").wall == "south");
  CHECK_THAT(find(r.openings, "door").s_start, WithinAbs(1.0, 1e-12));
  CHECK(find(r.openings, "window").wall == "south-b");
  CHECK_THAT(find(r.openings, "window").s_start, WithinAbs(1.0, 1e-12));
  CHECK_THAT(find(r.openings, "window").s_end, WithinAbs(1.5, 1e-12));
  // Centre 2.05 is on b; its left 0.2 m is cut off.
  CHECK(find(r.openings, "across").wall == "south-b");
  CHECK_THAT(find(r.openings, "across").s_start, WithinAbs(0.0, 1e-12));
  CHECK_THAT(find(r.openings, "across").s_end, WithinAbs(0.3, 1e-12));
  CHECK(r.clipped == std::vector<std::string>{"across"});

  REQUIRE(r.joins.size() == 4);
  const auto has = [&](const std::string &a, const std::string &b) {
    return std::any_of(r.joins.begin(), r.joins.end(), [&](const auto &j) {
      return j.wall_a == a && j.wall_b == b;
    });
  };
  CHECK(has("south", "west"));    // start stays on a
  CHECK(has("east", "south-b"));  // end moves to b
  CHECK(has("mid", "south-b"));   // the tee at x = 3 is on b
  CHECK(has("south", "south-b")); // the cut itself

  CHECK_THROWS_AS(core::split_wall(south, 0.01, openings),
                  std::invalid_argument);
  CHECK_THROWS_AS(core::split_wall(south, 4.5, openings),
                  std::invalid_argument);
}

TEST_CASE("WallEditOps_Split_CutsTheTopProfile", "[core][wall_edit_ops]") {
  auto gable = wall("gable", 0, 0, 4, 0);
  gable.top = {{0.0, 1.0}, {2.0, 3.0}, {4.0, 1.0}};
  const auto r = core::split_wall(gable, 1.0, {});
  REQUIRE(r.a.top.size() == 2);
  CHECK(r.a.top[1][0] == 1.0);
  CHECK_THAT(r.a.top[1][1], WithinAbs(2.0, 1e-12));
  REQUIRE(r.b.top.size() == 3);
  CHECK(r.b.top.front()[0] == 0.0);
  CHECK_THAT(r.b.top.front()[1], WithinAbs(2.0, 1e-12));
  CHECK_THAT(r.b.top[1][0], WithinAbs(1.0, 1e-12)); // the ridge, re-measured
  CHECK_THAT(r.b.top[1][1], WithinAbs(3.0, 1e-12));
  CHECK_THAT(r.b.top.back()[0], WithinAbs(3.0, 1e-12));
  CHECK_THAT(r.b.top.back()[1], WithinAbs(1.0, 1e-12));
}

TEST_CASE("WallEditOps_Merge_RefitsTheAxis", "[core][wall_edit_ops]") {
  const auto a = wall("a", 0, 0, 2, 0, 0.2);
  const auto b = wall("b", 2.1, 0.02, 4, 0.02, 0.3);
  const std::vector<core::OpeningRecord> openings = {
      opening("w1", "b", 0.5, 1.0), opening("w0", "a", 0.2, 0.6)};
  const std::vector<core::WallJoinRecord> joins = {
      {"a", "b", "collinear", "end", "start"},
      {"west", "a", "corner", "start", "start"},
      {"b", "east", "corner", "end", "start"}};
  const auto r = core::merge_walls(a, b, openings, joins);
  CHECK(r.wall.guid == "a");
  CHECK(r.removed == "b");
  CHECK_THAT(r.wall.x0, WithinAbs(0.0, 1e-3));
  CHECK_THAT(r.wall.x1, WithinAbs(4.0, 1e-3));
  CHECK(r.wall.y0 > 0.0);
  CHECK(r.wall.y0 < 0.02);
  CHECK_THAT(r.wall.thickness, WithinAbs((0.2 * 2.0 + 0.3 * 1.9) / 3.9, 1e-4));
  REQUIRE(r.openings.size() == 2);
  // b's window centre was at x = 2.85.
  const auto w1 = find(r.openings, "w1");
  CHECK(w1.wall == "a");
  CHECK_THAT(0.5 * (w1.s_start + w1.s_end), WithinAbs(2.85, 2e-3));
  CHECK_THAT(w1.s_end - w1.s_start, WithinAbs(0.5, 1e-9));
  REQUIRE(r.joins.size() == 2); // the a-b join is gone
  CHECK(r.joins[0].wall_b == "a");
  CHECK(r.joins[0].at_b == "start");
  CHECK(r.joins[1].wall_a == "a");
  CHECK(r.joins[1].at_a == "end");

  CHECK_THROWS_AS(core::merge_walls(a, wall("c", 2, 0, 2, 3), {}),
                  std::invalid_argument); // 90 deg
  CHECK_THROWS_AS(core::merge_walls(a, wall("d", 2, 0.8, 4, 0.8), {}),
                  std::invalid_argument); // 0.8 m off
}

TEST_CASE("WallEditOps_Join_CornerTeeAndTrim", "[core][wall_edit_ops]") {
  SECTION("a corner: both ends meet at the axes' intersection") {
    const auto a = wall("a", 0, 0, 3, 0);
    const auto b = wall("b", 4, 0.5, 4, 3);
    const auto r = core::join_walls(a, b, {opening("w", "b", 1.0, 1.5)});
    CHECK(r.a.x1 == 4.0);
    CHECK(r.a.y1 == 0.0);
    CHECK(r.b.x0 == 4.0);
    CHECK(r.b.y0 == 0.0);
    CHECK(r.join.kind == "corner");
    CHECK(r.join.at_a == "end");
    CHECK(r.join.at_b == "start");
    // b's start moved 0.5 m back: its window keeps its place in the plan.
    REQUIRE(r.openings.size() == 1);
    CHECK_THAT(r.openings[0].s_start, WithinAbs(1.5, 1e-12));
  }
  SECTION("a tee: the intersection is deep inside a") {
    const auto a = wall("a", 0, 0, 4, 0);
    const auto b = wall("b", 2, 0.4, 2, 3);
    const auto r = core::join_walls(a, b, {});
    CHECK(r.a.x0 == 0.0);
    CHECK(r.a.x1 == 4.0);
    CHECK(r.b.y0 == 0.0);
    CHECK(r.join.kind == "tee");
    CHECK(r.join.at_a == "path");
    CHECK(r.join.at_b == "start");
  }
  SECTION("an overshoot is trimmed") {
    const auto a = wall("a", 0, 0, 4.3, 0);
    const auto b = wall("b", 4, 0, 4, 3);
    const auto r = core::join_walls(a, b, {});
    CHECK_THAT(r.a.x1, WithinAbs(4.0, 1e-12));
    CHECK(r.join.kind == "corner");
  }
  SECTION("parallel or crossing walls are refused") {
    CHECK_THROWS_AS(
        core::join_walls(wall("a", 0, 0, 4, 0), wall("b", 0, 1, 4, 1), {}),
        std::invalid_argument);
    CHECK_THROWS_AS(
        core::join_walls(wall("a", 0, 0, 4, 0), wall("b", 2, -2, 2, 2), {}),
        std::invalid_argument);
  }
}

TEST_CASE("WallEditOps_WireOps_SplitMergeJoin_AndUndo",
          "[core][wall_edit_ops][model_edits]") {
  TempPath project("test_wall_edit_ops_wire");
  ProjectDB db(project.path);
  const auto south = wall("south", 0, 0, 4, 0);
  const auto west = wall("west", 0, 0, 0, 3);
  db.save_walls({south, west}, {{"south", "west", "corner", "start", "start"}});
  db.save_openings({opening("win", "south", 3.0, 3.5)});
  using json = nlohmann::json;

  // Split the south wall at 2 m.
  auto ops = core::model_edit_ops_from_json(db, json::parse(R"([
    {"op": "wall.split", "guid": "south", "s": 2.0, "new_guid": "south-b"}
  ])"));
  core::apply_model_edit(db, ops);
  REQUIRE(db.wall_by_guid("south-b"));
  CHECK(db.wall_by_guid("south")->x1 == 2.0);
  CHECK(db.wall_by_guid("south")->locked);
  CHECK(db.opening_by_guid("win")->wall == "south-b");
  CHECK(db.wall_joins().size() == 2); // corner + the cut

  // Merge them back.
  ops = core::model_edit_ops_from_json(
      db,
      json::parse(R"([{"op": "wall.merge", "guids": ["south", "south-b"]}])"));
  core::apply_model_edit(db, ops);
  CHECK_FALSE(db.wall_by_guid("south-b"));
  CHECK_THAT(db.wall_by_guid("south")->x1, WithinAbs(4.0, 1e-9));
  CHECK(db.opening_by_guid("win")->wall == "south");
  CHECK_THAT(db.opening_by_guid("win")->s_start, WithinAbs(3.0, 1e-9));

  // Undo the merge, then the split: the detected walls are back.
  core::undo_model_edit(db);
  CHECK(db.wall_by_guid("south-b"));
  core::undo_model_edit(db);
  CHECK_FALSE(db.wall_by_guid("south-b"));
  const auto s = db.wall_by_guid("south");
  CHECK(s->x1 == 4.0);
  CHECK(s->source == "detected");
  CHECK_FALSE(s->locked);
  CHECK(db.opening_by_guid("win")->wall == "south");
  CHECK(db.wall_joins().size() == 1);

  // Join a short wall to the west wall.
  db.save_walls({south, west, wall("stub", 0.4, 2.0, 2.0, 2.0)}, {});
  ops = core::model_edit_ops_from_json(
      db, json::parse(R"([{"op": "wall.join", "guids": ["stub", "west"]}])"));
  core::apply_model_edit(db, ops);
  CHECK(db.wall_by_guid("stub")->x0 == 0.0);
  CHECK_FALSE(db.wall_by_guid("west")->locked); // the tee host did not move
  const auto joins = db.wall_joins();
  CHECK(std::any_of(joins.begin(), joins.end(), [](const auto &j) {
    return j.wall_a == "stub" && j.wall_b == "west" && j.kind == "tee";
  }));
}

TEST_CASE("WallEditOps_WireSplit_LeavesUnmovedOpeningsUnedited",
          "[core][wall_edit_ops][model_edits]") {
  TempPath project("test_wall_edit_ops_unmoved");
  ProjectDB db(project.path);
  db.save_walls({wall("south", 0, 0, 4, 0)}, {});
  db.save_openings(
      {opening("near", "south", 0.5, 1.0), opening("far", "south", 3.0, 3.5)});
  const auto ops = core::model_edit_ops_from_json(
      db, nlohmann::json::parse(
              R"([{"op": "wall.split", "guid": "south", "s": 2.0}])"));
  const auto r = core::apply_model_edit(db, ops);
  CHECK(r.changes.size() == 3); // a, b and the far opening
  CHECK_FALSE(db.opening_by_guid("near")->locked);
  CHECK(db.opening_by_guid("far")->locked);
}
