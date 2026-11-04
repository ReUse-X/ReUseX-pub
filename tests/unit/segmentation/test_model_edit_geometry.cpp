// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Rediger editor geometry (reusex-yuja): spaces rebuilt from the walls
// alone, and a ceiling plane fitted to cloud points.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/guid.hpp>
#include <reusex/core/model_edits.hpp>
#include <reusex/core/space_polygon.hpp>
#include <reusex/segmentation/model_edit_geometry.hpp>

#include "../../support/temp_path.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <set>
#include <string>
#include <vector>

using Catch::Matchers::WithinAbs;
using namespace reusex;
using reusex::test_support::TempPath;

namespace {

const std::string kProject = "6f1c2a7e-3b9d-4c1e-9a55-2d7f8e0b4c11";

core::WallRecord wall(const std::string &guid, double x0, double y0, double x1,
                      double y1, double t = 0.2) {
  core::WallRecord w;
  w.guid = guid;
  w.x0 = x0;
  w.y0 = y0;
  w.x1 = x1;
  w.y1 = y1;
  w.height = 2.5;
  w.thickness = t;
  return w;
}

/// A 6 x 4 box (axes on the outline) with a partition at x = 3.
std::vector<core::WallRecord> box_walls() {
  return {wall("s", 0, 0, 6, 0), wall("e", 6, 0, 6, 4), wall("n", 6, 4, 0, 4),
          wall("w", 0, 4, 0, 0), wall("p", 3, 0, 3, 4, 0.1)};
}

core::StoreyRecord storey() {
  core::StoreyRecord s;
  s.guid = "storey-0";
  s.z_floor = 0.0;
  s.z_ceiling = 2.5;
  return s;
}

core::SpaceRecord stored_space(const std::string &guid, double x0, double x1,
                               const std::string &name) {
  core::SpaceRecord s;
  s.guid = guid;
  s.name = name;
  s.kind = "room";
  s.polygon.outer = {{x0, 0.1}, {x1, 0.1}, {x1, 3.9}, {x0, 3.9}};
  s.z_ceiling = 2.6;
  return s;
}

} // namespace

TEST_CASE("RebuildSpaces_FromWalls_TwoRoomsKeepNames",
          "[segmentation][model_edit_geometry]") {
  const auto walls = box_walls();
  const std::vector<core::SpaceRecord> stored = {
      stored_space("kitchen", 0.1, 2.9, "Køkken"),
      stored_space("gone", 10.0, 12.0, "Skur")};
  const auto r = segmentation::rebuild_spaces_from_walls(walls, stored,
                                                         storey(), kProject);
  INFO(r.faces << " faces, " << r.wall_bodies << " wall bodies, " << r.too_small
               << " too small");
  REQUIRE(r.spaces.size() == 2);
  CHECK(r.walls == 5);
  CHECK(r.matched == 1);
  CHECK(r.unmatched == std::vector<std::string>{"gone"});

  const auto left =
      std::find_if(r.spaces.begin(), r.spaces.end(),
                   [](const auto &s) { return s.guid == "kitchen"; });
  REQUIRE(left != r.spaces.end());
  CHECK(left->name == "Køkken");
  CHECK(left->z_ceiling == 2.6);
  // Inner faces: x 0.1 .. 2.95, y 0.1 .. 3.9.
  CHECK_THAT(left->area, WithinAbs(2.85 * 3.8, 1e-6));
  CHECK_THAT(left->volume, WithinAbs(left->area * 2.6, 1e-6));
  const auto right =
      std::find_if(r.spaces.begin(), r.spaces.end(),
                   [](const auto &s) { return s.guid != "kitchen"; });
  CHECK(right->guid.size() == 36);
  CHECK(right->name.empty());
  CHECK(right->z_ceiling == 2.5);
  CHECK_THAT(right->area, WithinAbs(2.85 * 3.8, 1e-6));

  // Each room is bounded by four walls; the left room lies left of every
  // axis (side a), the partition included (it runs northwards at x = 3).
  std::set<std::string> left_walls;
  for (const auto &b : r.boundaries)
    if (b.space == "kitchen")
      left_walls.insert(b.wall + "/" + b.side);
  CHECK(left_walls == std::set<std::string>{"s/a", "n/a", "w/a", "p/a"});
  for (const auto &b : r.boundaries)
    CHECK(b.s_end > b.s_start);

  // Deterministic.
  const auto again = segmentation::rebuild_spaces_from_walls(
      walls, stored, storey(), kProject);
  REQUIRE(again.spaces.size() == 2);
  CHECK(again.spaces[0].guid == r.spaces[0].guid);
  CHECK(again.spaces[1].guid == r.spaces[1].guid);
}

TEST_CASE("RebuildSpaces_GapBetweenParallelWalls_IsNoSpace",
          "[segmentation][model_edit_geometry]") {
  // A doubled partition: two walls 0.25 m apart leave a 0.15 m x 3.8 m
  // strip (0.57 m^2 would pass a lower area limit; its width does not).
  auto walls = box_walls();
  walls.pop_back();
  walls.push_back(wall("p1", 2.9, 0, 2.9, 4, 0.1));
  walls.push_back(wall("p2", 3.15, 0, 3.15, 4, 0.1));
  segmentation::RebuildSpacesOptions opt;
  opt.min_area_m2 = 0.1;
  const auto r = segmentation::rebuild_spaces_from_walls(walls, {}, storey(),
                                                         kProject, opt);
  CHECK(r.spaces.size() == 2);
  CHECK(r.too_small >= 1);
}

TEST_CASE("RebuildSpaces_OpenWalls_NoSpace_AndOps",
          "[segmentation][model_edit_geometry]") {
  // Three walls of a box: nothing closes. The stored space is reported,
  // and only removed when asked for.
  auto walls = box_walls();
  walls.erase(walls.begin() + 1);
  walls.pop_back();
  const std::vector<core::SpaceRecord> kept = {
      stored_space("open", 0.1, 5.9, "Stue")};
  const auto r =
      segmentation::rebuild_spaces_from_walls(walls, kept, storey(), kProject);
  CHECK(r.spaces.empty());
  CHECK(r.unmatched == std::vector<std::string>{"open"});
  CHECK(segmentation::rebuild_spaces_ops(r).empty());
  REQUIRE(segmentation::rebuild_spaces_ops(r, true).size() == 1);

  // Applied through one edit batch, then undone.
  TempPath project("test_rebuild_spaces_ops");
  ProjectDB db(project.path);
  db.save_storeys({storey()});
  db.save_walls(box_walls(), {});
  db.save_spaces({stored_space("old", 0.1, 5.9, "Stue")}, {});
  const auto rebuilt = segmentation::rebuild_spaces_from_walls(
      db.walls(), db.spaces(), storey(), db.project_guid());
  REQUIRE(rebuilt.spaces.size() == 2);
  core::apply_model_edit(db, segmentation::rebuild_spaces_ops(rebuilt));
  const auto rows = db.spaces();
  REQUIRE(rows.size() == 2);
  CHECK(rebuilt.unmatched.empty());
  CHECK(db.space_by_guid("old")); // the larger share kept the old guid
  CHECK_FALSE(db.space_boundaries().empty());
  core::undo_model_edit(db);
  REQUIRE(db.spaces().size() == 1);
  CHECK(db.spaces()[0].polygon.outer[1][0] == 5.9);
}

TEST_CASE("FitCeilingPlane_SlopedCeilingWithClutter",
          "[segmentation][model_edit_geometry]") {
  Cloud cloud;
  auto add = [&](double x, double y, double z) {
    PointT p;
    p.x = static_cast<float>(x);
    p.y = static_cast<float>(y);
    p.z = static_cast<float>(z);
    cloud.push_back(p);
  };
  int k = 0;
  for (double x = 0.05; x < 3.0; x += 0.1)
    for (double y = 0.05; y < 3.0; y += 0.1) {
      const double noise = 0.01 * ((k++ % 5) - 2) / 2.0;
      add(x, y, 2.4 + 0.2 * y + noise); // the ceiling, rising northwards
      add(x, y, 0.0);                   // the floor, outside the band
    }
  for (int i = 0; i < 40; ++i) // a lamp hanging 0.3 m under it
    add(1.0 + 0.01 * i, 1.5, 2.4 + 0.2 * 1.5 - 0.3);
  add(10.0, 10.0, 2.5); // outside the region

  core::SpacePolygon region;
  region.outer = {{0, 0}, {3, 0}, {3, 3}, {0, 3}};
  const auto r =
      segmentation::fit_ceiling_plane(cloud, storey(), region, 2.0, 3.5);
  INFO(r.reason);
  REQUIRE(r.ok);
  CHECK(r.points == 900 + 40);
  CHECK(r.inliers == 900);
  CHECK_THAT(r.plane.slope[0], WithinAbs(0.0, 2e-3));
  CHECK_THAT(r.plane.slope[1], WithinAbs(0.2, 2e-3));
  CHECK_THAT(r.plane.z_at(1.5, 0.0), WithinAbs(2.4, 5e-3));
  CHECK(r.rms_m < 0.01);
  CHECK_THAT(r.slope_deg,
             WithinAbs(std::atan(0.2) * 180.0 / std::numbers::pi, 0.2));

  // Through a level frame: world = level shifted up by 10 m.
  auto lifted = storey();
  lifted.level_transform[11] = -10.0; // world -> level
  Cloud up = cloud;
  for (auto &p : up.points)
    p.z += 10.0f;
  const auto r2 = segmentation::fit_ceiling_plane(up, lifted, region, 2.0, 3.5);
  REQUIRE(r2.ok);
  CHECK_THAT(r2.plane.z_at(1.5, 0.0), WithinAbs(2.4, 5e-3));

  // Too few points: no fit, and the reason says why.
  core::SpacePolygon tiny;
  tiny.outer = {{0, 0}, {0.2, 0}, {0.2, 0.2}, {0, 0.2}};
  const auto r3 =
      segmentation::fit_ceiling_plane(cloud, storey(), tiny, 2.0, 3.5);
  CHECK_FALSE(r3.ok);
  CHECK(r3.reason.find("point") != std::string::npos);
}

// --- Ceiling fit plausibility (reusex-y2ed) ---------------------------------

namespace {

void add_point(Cloud &cloud, double x, double y, double z) {
  PointT p;
  p.x = static_cast<float>(x);
  p.y = static_cast<float>(y);
  p.z = static_cast<float>(z);
  cloud.push_back(p);
}

core::StoreyRecord storey_at(int idx, double z_floor, double z_ceiling) {
  core::StoreyRecord s;
  s.guid = "storey-" + std::to_string(idx);
  s.idx = idx;
  s.z_floor = z_floor;
  s.z_ceiling = z_ceiling;
  return s;
}

core::SpacePolygon square_region(double side) {
  core::SpacePolygon r;
  r.outer = {{0, 0}, {side, 0}, {side, side}, {0, side}};
  return r;
}

} // namespace

TEST_CASE("CeilingFitBand_StopsBelowTheNextStoreysFloor",
          "[segmentation][model_edit_geometry]") {
  const std::vector<core::StoreyRecord> storeys = {storey_at(0, 0.0, 2.4),
                                                   storey_at(1, 2.7, 5.1)};
  // The lower storey: halfway up the room, capped under the upper floor.
  const auto lo = segmentation::default_ceiling_fit_band(0.0, 2.4, storeys);
  CHECK_THAT(lo.z_lo, WithinAbs(1.2, 1e-9));
  CHECK_THAT(lo.z_hi, WithinAbs(2.7 - 0.15, 1e-9));
  CHECK(lo.capped);
  // The top storey: nothing above, so the band reaches 3 m over the
  // ceiling (a sloped roof's high side).
  const auto hi = segmentation::default_ceiling_fit_band(2.7, 5.1, storeys);
  CHECK_THAT(hi.z_lo, WithinAbs(3.9, 1e-9));
  CHECK_THAT(hi.z_hi, WithinAbs(8.1, 1e-9));
  CHECK_FALSE(hi.capped);
  // No storeys at all: the same uncapped band.
  const auto none = segmentation::default_ceiling_fit_band(0.0, 2.4, {});
  CHECK_THAT(none.z_hi, WithinAbs(5.4, 1e-9));
  // A storey floor under this ceiling (split level) does not cut the band.
  const auto split = segmentation::default_ceiling_fit_band(
      0.0, 2.4, {storey_at(0, 0.0, 2.4), storey_at(1, 1.25, 3.0)});
  CHECK_THAT(split.z_hi, WithinAbs(5.4, 1e-9));
  CHECK_FALSE(split.capped);
  // A slab thinner than the margin: the band still holds the ceiling.
  const auto thin = segmentation::default_ceiling_fit_band(
      0.0, 2.4, {storey_at(0, 0.0, 2.4), storey_at(1, 2.45, 5.0)});
  CHECK_THAT(thin.z_hi, WithinAbs(2.45, 1e-9));
}

TEST_CASE("FitCeilingPlane_TwoStoreys_DefaultBandFindsTheRoomsOwnCeiling",
          "[segmentation][model_edit_geometry]") {
  // Storey 0: floor 0, flat ceiling 2.4. Storey 1 above: floor 2.7 (twice
  // as dense, it is scanned from above), a sloped roof rising from 3.0.
  Cloud cloud;
  for (double x = 0.05; x < 4.0; x += 0.1)
    for (double y = 0.05; y < 4.0; y += 0.1) {
      add_point(cloud, x, y, 0.0);
      add_point(cloud, x, y, 2.4);
      add_point(cloud, x, y, 2.7);
      add_point(cloud, x + 0.05, y, 2.7);
      add_point(cloud, x, y, 3.0 + 0.5 * x);
    }
  const std::vector<core::StoreyRecord> storeys = {storey_at(0, 0.0, 2.4),
                                                   storey_at(1, 2.7, 5.1)};
  const auto region = square_region(4.0);
  segmentation::CeilingFitOptions opt;
  opt.z_floor = 0.0;

  const auto band = segmentation::default_ceiling_fit_band(0.0, 2.4, storeys);
  const auto r = segmentation::fit_ceiling_plane(cloud, storeys[0], region,
                                                 band.z_lo, band.z_hi, opt);
  INFO(r.reason);
  REQUIRE(r.ok);
  CHECK(r.code.empty());
  CHECK_THAT(r.plane.z_at(2.0, 2.0), WithinAbs(2.4, 1e-3));
  CHECK(r.slope_deg < 0.5);

  // The old default band (to 3 m over the ceiling) reaches the storey above
  // and fits its denser floor instead: plausible on its own, so only the
  // band cap keeps it out.
  const auto old =
      segmentation::fit_ceiling_plane(cloud, storeys[0], region, 1.2, 5.4, opt);
  CHECK(old.points > 2 * r.points);
  CHECK_FALSE((old.ok && std::abs(old.plane.z_at(2.0, 2.0) - 2.4) < 0.05));
}

TEST_CASE("FitCeilingPlane_RefusesALowInlierShare",
          "[segmentation][model_edit_geometry]") {
  // Five equal horizontal layers: the trimmed fit settles on one of them,
  // 20 % of the band's points, under the default 25 % share.
  Cloud cloud;
  for (double x = 0.05; x < 3.0; x += 0.1)
    for (double y = 0.05; y < 3.0; y += 0.1)
      for (int l = 0; l < 5; ++l)
        add_point(cloud, x, y, 2.4 + 0.4 * l);
  const auto r = segmentation::fit_ceiling_plane(cloud, storey_at(0, 0, 2.4),
                                                 square_region(3.0), 2.0, 4.5);
  INFO(r.reason);
  CHECK_FALSE(r.ok);
  CHECK(r.code == "low_inlier_share");
  CHECK(r.inliers == 900);
  CHECK(r.points == 4500);
  CHECK(r.reason.find("20") != std::string::npos);

  segmentation::CeilingFitOptions lax;
  lax.min_inlier_share = 0.1;
  const auto r2 = segmentation::fit_ceiling_plane(
      cloud, storey_at(0, 0, 2.4), square_region(3.0), 2.0, 4.5, lax);
  CHECK(r2.ok);
}

TEST_CASE("FitCeilingPlane_RefusesAPlaneTooCloseToTheFloor",
          "[segmentation][model_edit_geometry]") {
  // A clean plane, 26.6 deg, 1.75 m over the floor at the room's middle
  // (x = 1.5): a stair flight, not this room's ceiling.
  Cloud cloud;
  for (double x = 0.05; x < 3.0; x += 0.1)
    for (double y = 0.05; y < 3.0; y += 0.1)
      add_point(cloud, x, y, 1.0 + 0.5 * x);
  segmentation::CeilingFitOptions opt;
  opt.z_floor = 0.0;
  const auto r = segmentation::fit_ceiling_plane(
      cloud, storey_at(0, 0, 2.4), square_region(3.0), 0.5, 4.0, opt);
  INFO(r.reason);
  CHECK_FALSE(r.ok);
  CHECK(r.code == "too_low");
  CHECK(r.inliers == 900);
  CHECK_THAT(r.clearance_m, WithinAbs(1.75, 1e-3));

  // Without a floor to measure from, the same plane passes.
  const auto r2 = segmentation::fit_ceiling_plane(cloud, storey_at(0, 0, 2.4),
                                                  square_region(3.0), 0.5, 4.0);
  CHECK(r2.ok);
  // A floor far enough below lets it pass too, though the plane's low
  // edge (1.0 m over it at x = 0) is under the clearance: an attic's
  // sloped ceiling over a knee wall.
  opt.z_floor = -0.3;
  const auto r3 = segmentation::fit_ceiling_plane(
      cloud, storey_at(0, 0, 2.4), square_region(3.0), 0.5, 4.0, opt);
  CHECK(r3.ok);
  CHECK_THAT(r3.clearance_m, WithinAbs(2.05, 1e-3));
}

TEST_CASE("FitCeilingPlane_ReasonCodes",
          "[segmentation][model_edit_geometry]") {
  Cloud cloud;
  for (double x = 0.05; x < 3.0; x += 0.1)
    for (double y = 0.05; y < 3.0; y += 0.1)
      add_point(cloud, x, y, 2.4);
  core::SpacePolygon line;
  line.outer = {{0, 0}, {1, 0}};
  CHECK(segmentation::fit_ceiling_plane(cloud, storey_at(0, 0, 2.4), line, 2.0,
                                        3.0)
            .code == "bad_region");
  CHECK(segmentation::fit_ceiling_plane(cloud, storey_at(0, 0, 2.4),
                                        square_region(3.0), 2.5, 3.0)
            .code == "too_few_points");
  Cloud wall;
  for (double y = 0.05; y < 3.0; y += 0.1)
    for (double z = 2.0; z < 3.0; z += 0.02)
      add_point(wall, 1.0 + 0.01 * (z - 2.0), y, z); // near vertical
  segmentation::CeilingFitOptions opt;
  opt.inlier_m = 0.2;
  CHECK(segmentation::fit_ceiling_plane(wall, storey_at(0, 0, 2.4),
                                        square_region(3.0), 1.9, 3.1, opt)
            .code == "too_steep");
}

// --- Room tools of the Rediger editor's slice S6 (reusex-pccw) ---------------

TEST_CASE("PlaceSpace_ClickInAClosedArea_GivesThatRoom",
          "[segmentation][model_edit_geometry]") {
  const auto walls = box_walls();
  // The right room: nothing stored there yet.
  const auto r = segmentation::place_space_at(
      walls, {stored_space("kitchen", 0.1, 2.9, "Køkken")}, storey(), kProject,
      {4.5, 2.0});
  INFO(r.reason);
  REQUIRE(r.ok);
  CHECK_FALSE(r.existing);
  CHECK(r.space.guid.size() == 36);
  CHECK(r.space.storey == 0);
  CHECK_THAT(r.space.area, WithinAbs(2.85 * 3.8, 1e-6));
  std::set<std::string> ws;
  for (const auto &b : r.boundaries) {
    CHECK(b.space == r.space.guid);
    ws.insert(b.wall);
  }
  CHECK(ws == std::set<std::string>{"s", "e", "n", "p"});
  const auto ops = segmentation::place_space_ops(r);
  REQUIRE(ops.size() == 1);
  CHECK(ops[0].boundaries);

  // A click in the stored room re-fits that room to its walls.
  const auto again = segmentation::place_space_at(
      walls, {stored_space("kitchen", 0.5, 2.0, "Køkken")}, storey(), kProject,
      {1.0, 1.0});
  REQUIRE(again.ok);
  CHECK(again.existing);
  CHECK(again.space.guid == "kitchen");
  CHECK(again.space.name == "Køkken");

  // In a wall body, or outside the walls: no room.
  CHECK_FALSE(
      segmentation::place_space_at(walls, {}, storey(), kProject, {3.0, 2.0})
          .ok);
  const auto outside =
      segmentation::place_space_at(walls, {}, storey(), kProject, {20.0, 2.0});
  CHECK_FALSE(outside.ok);
  CHECK_FALSE(outside.reason.empty());
}

TEST_CASE("MergeSpaces_AcrossTheirPartition_OneRoomTheOtherRemoved",
          "[segmentation][model_edit_geometry]") {
  const auto walls = box_walls();
  const auto rebuilt =
      segmentation::rebuild_spaces_from_walls(walls, {}, storey(), kProject);
  REQUIRE(rebuilt.spaces.size() == 2);
  auto a = rebuilt.spaces[0];
  auto b = rebuilt.spaces[1];
  if (a.polygon.outer[0][0] > b.polygon.outer[0][0])
    std::swap(a, b);
  a.name = "Stue";
  const auto r = segmentation::merge_spaces(a, b, walls);
  INFO(r.reason);
  REQUIRE(r.ok);
  CHECK(r.space.guid == a.guid);
  CHECK(r.space.name == "Stue");
  CHECK(r.removed == b.guid);
  CHECK(r.bridged_walls == std::vector<std::string>{"p"});
  // Both rooms plus the partition's 0.1 x 3.8 m.
  CHECK_THAT(r.space.area, WithinAbs(2 * 2.85 * 3.8 + 0.1 * 3.8, 1e-6));
  CHECK(r.space.polygon.outer.size() == 4); // a clean rectangle
  CHECK(r.space.polygon.holes.empty());
  for (const auto &bd : r.boundaries)
    CHECK(bd.wall != "p");
  const auto ops = segmentation::merge_spaces_ops(r);
  REQUIRE(ops.size() == 2);
  CHECK(ops[1].action == core::ModelEditOp::Action::remove);

  // Two rooms apart (another room between them): refused.
  auto far = b;
  for (auto &p : far.polygon.outer)
    p[0] += 10.0;
  const auto no = segmentation::merge_spaces(a, far, walls);
  CHECK_FALSE(no.ok);
  CHECK_FALSE(no.reason.empty());
}

TEST_CASE("SplitSpace_ByALine_TwoRoomsWithBoundaries",
          "[segmentation][model_edit_geometry]") {
  auto walls = box_walls();
  walls.pop_back(); // no partition: one 5.8 x 3.8 room
  const auto rebuilt =
      segmentation::rebuild_spaces_from_walls(walls, {}, storey(), kProject);
  REQUIRE(rebuilt.spaces.size() == 1);
  auto s = rebuilt.spaces[0];
  s.name = "Stue";
  // A line across the room at x = 2 (drawn from just inside the faces).
  const auto r = segmentation::split_space(s, {2.0, 0.12}, {2.0, 3.88}, walls,
                                           kProject, {s.guid});
  INFO(r.reason);
  REQUIRE(r.ok);
  REQUIRE(r.spaces.size() == 2);
  CHECK(r.spaces[0].guid == s.guid); // the larger keeps the guid
  CHECK(r.spaces[0].name == "Stue");
  CHECK_THAT(r.spaces[0].area, WithinAbs(3.9 * 3.8, 1e-6));
  CHECK_THAT(r.spaces[1].area, WithinAbs(1.9 * 3.8, 1e-6));
  CHECK(r.spaces[1].guid != s.guid);
  CHECK(r.spaces[1].guid.size() == 36);
  CHECK(r.spaces[1].kind == s.kind);
  std::set<std::string> small;
  for (const auto &b : r.boundaries)
    if (b.space == r.spaces[1].guid)
      small.insert(b.wall);
  CHECK(small == std::set<std::string>{"s", "n", "w"});
  CHECK(segmentation::split_space_ops(r).size() == 2);

  // A line that does not cross the room: refused.
  const auto no = segmentation::split_space(s, {2.0, 1.0}, {2.0, 2.0}, walls,
                                            kProject, {s.guid});
  CHECK_FALSE(no.ok);
  CHECK_FALSE(no.reason.empty());
}
