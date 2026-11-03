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
