// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// The Rediger CAD editor's wall tools (reusex-so4t): extend / trim a wall
// end to a boundary wall, move / rotate / copy a selection as one body, and
// their wire ops (wall.extend_to, selection.transform, wall.offset) through
// core::apply_model_edit.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/model_edits.hpp>
#include <reusex/core/wall_edit_ops.hpp>

#include "../../support/temp_path.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <string>
#include <string_view>
#include <vector>

using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;
using namespace reusex;
using reusex::test_support::TempPath;
using json = nlohmann::json;

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
  o.kind = "door";
  o.s_start = s0;
  o.s_end = s1;
  o.z_head = 2.1;
  return o;
}

template <typename R>
const R &find(const std::vector<R> &rows, std::string_view guid) {
  const auto it = std::find_if(rows.begin(), rows.end(),
                               [&](const auto &r) { return r.guid == guid; });
  REQUIRE(it != rows.end());
  return *it;
}

template <typename R>
bool has(const std::vector<R> &rows, const std::string &guid) {
  return std::any_of(rows.begin(), rows.end(),
                     [&](const auto &r) { return r.guid == guid; });
}

bool has_join(const std::vector<core::WallJoinRecord> &joins,
              const std::string &a, const std::string &b,
              const std::string &kind) {
  return std::any_of(joins.begin(), joins.end(), [&](const auto &j) {
    return ((j.wall_a == a && j.wall_b == b) ||
            (j.wall_a == b && j.wall_b == a)) &&
           j.kind == kind;
  });
}

} // namespace

TEST_CASE("WallTools_ExtendTo_TeeEndsOnTheNearFace", "[core][wall_tools]") {
  const auto target = wall("t", 0, 0, 6, 0); // faces at y = -0.1 / +0.1

  SECTION("an end short of the target is extended") {
    const auto w = wall("w", 3, 2, 3, 1);
    const auto r = core::extend_wall_to(w, "end", target, {});
    CHECK_THAT(r.wall.x1, WithinAbs(3.0, 1e-12));
    CHECK_THAT(r.wall.y1, WithinAbs(0.1, 1e-12));
    CHECK(r.wall.x0 == 3.0); // the other end stays
    CHECK(r.wall.y0 == 2.0);
    CHECK_FALSE(r.target);
    CHECK(r.join.wall_a == "w");
    CHECK(r.join.wall_b == "t");
    CHECK(r.join.kind == "tee");
    CHECK(r.join.at_a == "end");
    CHECK(r.join.at_b == "path");
  }
  SECTION("an end through the target is trimmed back to the near face") {
    const auto w = wall("w", 3, 2, 3, -0.6);
    const auto r = core::extend_wall_to(w, "end", target, {});
    CHECK_THAT(r.wall.y1, WithinAbs(0.1, 1e-12));
  }
  SECTION("the near face is the side the other end is on") {
    const auto w = wall("w", 3, -2, 3, -1);
    const auto r = core::extend_wall_to(w, "end", target, {});
    CHECK_THAT(r.wall.y1, WithinAbs(-0.1, 1e-12));
  }
  SECTION("a slanted wall meets the face, not the axis") {
    const auto w = wall("w", 2, 2, 3, 1); // 45 deg down-right
    const auto r = core::extend_wall_to(w, "end", target, {});
    CHECK_THAT(r.wall.x1, WithinAbs(3.9, 1e-12));
    CHECK_THAT(r.wall.y1, WithinAbs(0.1, 1e-12));
  }
  SECTION("unknown thickness: the axis") {
    const auto w = wall("w", 3, 2, 3, 1);
    const auto r =
        core::extend_wall_to(w, "end", wall("t", 0, 0, 6, 0, -1.0), {});
    CHECK_THAT(r.wall.y1, WithinAbs(0.0, 1e-12));
  }
  SECTION("a moved start re-measures the wall's openings") {
    const auto w = wall("w", 3, 1, 3, 4); // start nearest the target
    const auto r =
        core::extend_wall_to(w, "start", target, {opening("d", "w", 1.0, 1.9)});
    CHECK_THAT(r.wall.y0, WithinAbs(0.1, 1e-12));
    REQUIRE(r.openings.size() == 1);
    // The start moved 0.9 m back: the door keeps its place in the plan.
    CHECK_THAT(r.openings[0].s_start, WithinAbs(1.9, 1e-12));
    CHECK_THAT(r.openings[0].s_end, WithinAbs(2.8, 1e-12));
  }
}

TEST_CASE("WallTools_ExtendTo_CornerMovesBothEnds", "[core][wall_tools]") {
  SECTION("beyond the target's end: both reach the intersection") {
    const auto target = wall("t", 0, 0, 4, 0);
    const auto w = wall("w", 5, 0.5, 5, 3);
    const auto r = core::extend_wall_to(
        w, "start", target,
        {opening("d", "w", 1.0, 1.5), opening("x", "t", 1.0, 2.0)});
    CHECK_THAT(r.wall.x0, WithinAbs(5.0, 1e-12));
    CHECK_THAT(r.wall.y0, WithinAbs(0.0, 1e-12));
    REQUIRE(r.target);
    CHECK_THAT(r.target->x1, WithinAbs(5.0, 1e-12));
    CHECK(r.target->x0 == 0.0);
    CHECK(r.join.kind == "corner");
    CHECK(r.join.at_a == "start");
    CHECK(r.join.at_b == "end");
    // w's door shifted with its start; t's start did not move.
    REQUIRE(r.openings.size() == 1);
    CHECK(r.openings[0].guid == "d");
    CHECK_THAT(r.openings[0].s_start, WithinAbs(1.5, 1e-12));
  }
  SECTION("within the target's thickness of its end: a corner, trimmed") {
    const auto target = wall("t", 0, 0, 4, 0);
    const auto w = wall("w", 3.9, 1, 3.9, 3);
    const auto r = core::extend_wall_to(w, "start", target, {});
    CHECK(r.join.kind == "corner");
    REQUIRE(r.target);
    CHECK_THAT(r.target->x1, WithinAbs(3.9, 1e-12));
    CHECK_THAT(r.wall.y0, WithinAbs(0.0, 1e-12));
  }
}

TEST_CASE("WallTools_ExtendTo_Refusals", "[core][wall_tools]") {
  const auto target = wall("t", 0, 0, 6, 0);
  CHECK_THROWS_AS(
      core::extend_wall_to(wall("w", 0, 1, 6, 1), "end", target, {}),
      std::invalid_argument); // parallel
  CHECK_THROWS_AS(core::extend_wall_to(target, "end", target, {}),
                  std::invalid_argument); // itself
  CHECK_THROWS_AS(
      core::extend_wall_to(wall("w", 3, 2, 3, 1), "middle", target, {}),
      std::invalid_argument);
  // Moving the start (the far end) onto the face would turn the wall round.
  CHECK_THROWS_AS(
      core::extend_wall_to(wall("w", 3, 2, 3, 1), "start", target, {}),
      std::invalid_argument);
  auto other = wall("w", 3, 2, 3, 1);
  other.storey = 1;
  CHECK_THROWS_AS(core::extend_wall_to(other, "end", target, {}),
                  std::invalid_argument);
}

TEST_CASE("WallTools_PlanTransform_RotatesAboutThePivotThenTranslates",
          "[core][wall_tools]") {
  core::PlanTransform t;
  t.rotate_deg = 90.0;
  t.pivot = {1.0, 0.0};
  t.dx = 10.0;
  const auto p = t.apply({2.0, 0.0});
  CHECK_THAT(p[0], WithinAbs(11.0, 1e-12));
  CHECK_THAT(p[1], WithinAbs(1.0, 1e-12));
}

TEST_CASE("WallTools_Transform_MovesWallsAndResolvesJoins",
          "[core][wall_tools]") {
  // An L of a (south) and b (east), c teed onto a from the north.
  const auto a = wall("a", 0, 0, 4, 0);
  const auto b = wall("b", 4, 0, 4, 3);
  const auto c = wall("c", 2, 0.1, 2, 3);
  const std::vector<core::WallRecord> walls{a, b, c};
  const std::vector<core::WallJoinRecord> joins{
      {"a", "b", "corner", "end", "start"}, {"c", "a", "tee", "start", "path"}};
  const std::vector<core::OpeningRecord> openings{opening("d", "b", 1.0, 1.9)};

  SECTION("moving b east drags a's end along (the corner follows)") {
    core::PlanTransform t;
    t.dx = 1.0;
    const auto r = core::transform_selection({"b"}, t, false, walls, openings,
                                             {}, joins, {});
    CHECK(r.warnings.empty());
    const auto &nb = find(r.walls, "b");
    CHECK(nb.x0 == 5.0);
    CHECK(nb.x1 == 5.0);
    const auto &na = find(r.walls, "a");
    CHECK_THAT(na.x1, WithinAbs(5.0, 1e-12));
    CHECK(na.x0 == 0.0);
    CHECK_FALSE(has(r.walls, "c")); // untouched
    CHECK(has_join(r.joins, "a", "b", "corner"));
    CHECK(has_join(r.joins, "c", "a", "tee"));
    // b's door moved with b as it is.
    CHECK_FALSE(has(r.openings, "d"));
  }
  SECTION("moving the stem along the host keeps it teed on the face") {
    core::PlanTransform t;
    t.dx = 1.0;
    t.dy = 1.0; // also away from a: the stem is extended back to it
    const auto r = core::transform_selection({"c"}, t, false, walls, openings,
                                             {}, joins, {});
    const auto &nc = find(r.walls, "c");
    CHECK_THAT(nc.x0, WithinAbs(3.0, 1e-12));
    CHECK_THAT(nc.y0, WithinAbs(0.1, 1e-12));
    CHECK_THAT(nc.y1, WithinAbs(4.0, 1e-12));
    CHECK(has_join(r.joins, "c", "a", "tee"));
    CHECK_FALSE(has(r.walls, "a"));
  }
  SECTION("moving the host drags the stem's end to its new face") {
    core::PlanTransform t;
    t.dy = -1.0;
    // a alone: its corner with b drags b's start down too.
    const auto r = core::transform_selection({"a"}, t, false, walls, openings,
                                             {}, joins, {});
    const auto &nc = find(r.walls, "c");
    CHECK_THAT(nc.y0, WithinAbs(-0.9, 1e-12));
    const auto &nb = find(r.walls, "b");
    CHECK_THAT(nb.y0, WithinAbs(-1.0, 1e-12));
    // b's start moved 1 m back: its door keeps its place in the plan.
    CHECK_THAT(find(r.openings, "d").s_start, WithinAbs(2.0, 1e-12));
  }
  SECTION("a stem moved off the host's end loses the join, with a warning") {
    core::PlanTransform t;
    t.dx = -5.0;
    const auto r = core::transform_selection({"c"}, t, false, walls, openings,
                                             {}, joins, {});
    CHECK_FALSE(has_join(r.joins, "c", "a", "tee"));
    REQUIRE(r.warnings.size() == 1);
    CHECK_THAT(r.warnings[0], ContainsSubstring("c"));
  }
  SECTION("moving walls together keeps their joins as they are") {
    core::PlanTransform t;
    t.dx = 2.0;
    t.dy = 2.0;
    const auto r = core::transform_selection({"a", "b", "c"}, t, false, walls,
                                             openings, {}, joins, {});
    CHECK(r.walls.size() == 3);
    CHECK(r.joins.size() == 2);
    CHECK(r.warnings.empty());
    CHECK(find(r.walls, "c").y0 == 2.1);
  }
  SECTION("rotation about a pivot") {
    core::PlanTransform t;
    t.rotate_deg = 90.0;
    const auto r = core::transform_selection({"a", "b", "c"}, t, false, walls,
                                             openings, {}, joins, {});
    const auto &na = find(r.walls, "a");
    CHECK_THAT(na.x1, WithinAbs(0.0, 1e-12));
    CHECK_THAT(na.y1, WithinAbs(4.0, 1e-12));
  }
  SECTION("unknown guids are refused") {
    CHECK_THROWS_AS(core::transform_selection({"nope"}, {}, false, walls,
                                              openings, {}, joins, {}),
                    std::invalid_argument);
  }
}

TEST_CASE("WallTools_Transform_CopiesWallsOpeningsAndSpaces",
          "[core][wall_tools]") {
  const auto a = wall("a", 0, 0, 4, 0);
  const auto b = wall("b", 4, 0, 4, 3);
  const std::vector<core::WallRecord> walls{a, b};
  const std::vector<core::WallJoinRecord> joins{
      {"a", "b", "corner", "end", "start"}};
  const std::vector<core::OpeningRecord> openings{opening("d", "a", 1.0, 1.9)};
  core::SpaceRecord s;
  s.guid = "s";
  s.polygon.outer = {{0, 0}, {4, 0}, {4, 3}, {0, 3}};
  s.ceiling_planes.push_back({{0.0, 0.0, 2.5}, {0.1, 0.0}});
  s.room_label = 3;
  s.metadata = R"({"edited":true,"edited_from":{"guid":"s"}})";
  const std::vector<core::SpaceRecord> spaces{s};
  const std::vector<core::SpaceBoundaryRecord> bounds{
      {"s", "a", "a", 0.0, 4.0}, {"s", "z", "b", 0.0, 3.0}};

  core::PlanTransform t;
  t.dx = 10.0;
  const auto r = core::transform_selection({"a", "b", "s"}, t, true, walls,
                                           openings, spaces, joins, bounds,
                                           {{"a", "a2"}, {"s", "s2"}});
  REQUIRE(r.walls.size() == 2);
  CHECK(has(r.walls, "a2"));
  CHECK_FALSE(has(r.walls, "a")); // the originals stay as they are
  const auto &na = find(r.walls, "a2");
  CHECK(na.x0 == 10.0);
  CHECK(na.id == -1);
  std::string b2;
  for (const auto &[from, to] : r.copies)
    if (from == "b")
      b2 = to;
  REQUIRE_FALSE(b2.empty());
  // The copies are joined to each other, the originals as before.
  CHECK(has_join(r.joins, "a2", b2, "corner"));
  CHECK(has_join(r.joins, "a", "b", "corner"));
  CHECK(r.joins.size() == 2);
  // a's door is copied onto a2.
  REQUIRE(r.openings.size() == 1);
  CHECK(r.openings[0].wall == "a2");
  CHECK(r.openings[0].guid != "d");
  CHECK(r.openings[0].s_start == 1.0);
  // The space copy: moved polygon and ceiling, a fresh row.
  const auto &ns = find(r.spaces, "s2");
  CHECK(ns.polygon.outer[0][0] == 10.0);
  CHECK(ns.ceiling_planes[0].at[0] == 10.0);
  CHECK_THAT(ns.ceiling_planes[0].z_at(11.0, 0.0), WithinAbs(2.6, 1e-12));
  CHECK(ns.room_label == -1);
  CHECK(ns.metadata.find("edited_from") == std::string::npos);
  // Its boundary on a moved to a2; the one on an uncopied wall is dropped.
  REQUIRE(r.boundaries.size() == 1);
  CHECK(r.boundaries[0].space == "s2");
  CHECK(r.boundaries[0].wall == "a2");
}

TEST_CASE("WallTools_Transform_SlidesALoneOpeningAlongItsHost",
          "[core][wall_tools]") {
  const std::vector<core::WallRecord> walls{wall("a", 0, 0, 4, 0)};
  const std::vector<core::OpeningRecord> openings{opening("d", "a", 1.0, 1.9)};
  core::PlanTransform t;
  t.dx = 1.0;
  t.dy = 0.7; // across the wall: ignored
  auto r =
      core::transform_selection({"d"}, t, false, walls, openings, {}, {}, {});
  REQUIRE(r.openings.size() == 1);
  CHECK(r.openings[0].guid == "d");
  CHECK_THAT(r.openings[0].s_start, WithinAbs(2.0, 1e-12));
  CHECK(r.walls.empty());
  // A copy lands beside it.
  r = core::transform_selection({"d"}, t, true, walls, openings, {}, {}, {},
                                {{"d", "d2"}});
  REQUIRE(r.openings.size() == 1);
  CHECK(r.openings[0].guid == "d2");
  CHECK(r.openings[0].wall == "a");
  CHECK_THAT(r.openings[0].s_end, WithinAbs(2.9, 1e-12));
}

TEST_CASE("WallTools_WireOps_ExtendTransformOffset_OneUndoStepEach",
          "[core][wall_tools][model_edits]") {
  TempPath project("test_wall_tools_wire");
  ProjectDB db(project.path);
  db.save_walls(
      {wall("a", 0, 0, 4, 0), wall("b", 4, 0, 4, 3), wall("c", 2, 1, 2, 3)},
      {{"a", "b", "corner", "end", "start"}});
  db.save_openings({opening("d", "a", 1.0, 1.9)});
  const auto history0 = core::model_edit_history(db).size();

  SECTION("wall.extend_to") {
    const auto ops = core::model_edit_ops_from_json(db, json::parse(R"([
      {"op": "wall.extend_to", "guid": "c", "end": "start", "target": "a"}
    ])"));
    const auto r = core::apply_model_edit(db, ops);
    CHECK(r.changes.size() == 1); // a tee: only c moved
    const auto c = db.wall_by_guid("c");
    CHECK_THAT(c->y0, WithinAbs(0.1, 1e-12));
    CHECK(c->locked);
    CHECK(c->metadata.find("edited_from") != std::string::npos);
    CHECK_FALSE(db.wall_by_guid("a")->locked);
    CHECK(has_join(db.wall_joins(), "c", "a", "tee"));
    CHECK(core::model_edit_history(db).size() == history0 + 1);
    core::undo_model_edit(db);
    CHECK(db.wall_by_guid("c")->y0 == 1.0);
    CHECK_FALSE(has_join(db.wall_joins(), "c", "a", "tee"));
  }
  SECTION("selection.transform: copy") {
    const auto ops = core::model_edit_ops_from_json(db, json::parse(R"([
      {"op": "selection.transform", "guids": ["a", "b"],
       "translate": [0, 5], "copy": true, "new_guids": {"a": "a2", "b": "b2"}}
    ])"));
    const auto r = core::apply_model_edit(db, ops);
    const auto a2 = db.wall_by_guid("a2");
    REQUIRE(a2);
    CHECK(a2->y0 == 5.0);
    CHECK(a2->source == "manual");
    CHECK(a2->locked);
    CHECK(a2->metadata.find("edited_from") == std::string::npos);
    CHECK_FALSE(db.wall_by_guid("a")->locked); // the original is untouched
    CHECK(has_join(db.wall_joins(), "a2", "b2", "corner"));
    CHECK(has_join(db.wall_joins(), "a", "b", "corner"));
    CHECK(db.openings().size() == 2);
    CHECK(r.changes.size() == 3); // a2, b2, the door's copy
    CHECK(core::model_edit_history(db).size() == history0 + 1);
    core::undo_model_edit(db);
    CHECK_FALSE(db.wall_by_guid("a2"));
    CHECK(db.openings().size() == 1);
  }
  SECTION("selection.transform: move and rotate, dropped joins warn") {
    const auto ops = core::model_edit_ops_from_json(db, json::parse(R"([
      {"op": "selection.transform", "guids": ["b"], "rotate_deg": 90,
       "pivot": [4, 0]}
    ])"));
    const auto r = core::apply_model_edit(db, ops);
    const auto b = db.wall_by_guid("b");
    CHECK_THAT(b->x1, WithinAbs(1.0, 1e-12)); // now along a: parallel
    CHECK_FALSE(has_join(db.wall_joins(), "a", "b", "corner"));
    REQUIRE(r.warnings.size() == 1);
    CHECK_THAT(r.warnings[0], ContainsSubstring("dropped"));
  }
  SECTION("wall.offset: a copy beside it, on its left") {
    const auto ops = core::model_edit_ops_from_json(db, json::parse(R"([
      {"op": "wall.offset", "guid": "a", "distance": 1.5, "new_guid": "a2"}
    ])"));
    core::apply_model_edit(db, ops);
    const auto a2 = db.wall_by_guid("a2");
    REQUIRE(a2);
    CHECK(a2->y0 == 1.5);
    CHECK(a2->y1 == 1.5);
    CHECK(db.wall_by_guid("a")->y0 == 0.0);
  }
  SECTION("wall.offset without copy moves it") {
    const auto ops = core::model_edit_ops_from_json(db, json::parse(R"([
      {"op": "wall.offset", "guid": "c", "distance": 0.5, "copy": false}
    ])"));
    core::apply_model_edit(db, ops);
    CHECK_THAT(db.wall_by_guid("c")->x0, WithinAbs(1.5, 1e-12));
  }
  SECTION("bad ops are refused") {
    CHECK_THROWS_AS(core::model_edit_ops_from_json(db, json::parse(R"([
      {"op": "wall.extend_to", "guid": "c", "end": "start"}])")),
                    std::invalid_argument);
    CHECK_THROWS_AS(core::model_edit_ops_from_json(db, json::parse(R"([
      {"op": "selection.transform", "guids": []}])")),
                    std::invalid_argument);
    CHECK_THROWS_AS(core::model_edit_ops_from_json(db, json::parse(R"([
      {"op": "selection.transform", "guids": ["a"], "translate": [1]}])")),
                    std::invalid_argument);
    CHECK_THROWS_AS(core::model_edit_ops_from_json(db, json::parse(R"([
      {"op": "wall.offset", "guid": "a"}])")),
                    std::invalid_argument);
  }
}
