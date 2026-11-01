// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Schema v38 -> v39 (reusex-546): spaces.ceiling_planes and
// walls.top_profile (sloped ceilings and wall tops), and the core helpers
// that evaluate, check and encode them (core/sloped_ceiling.hpp).

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/guid.hpp>
#include <reusex/core/sloped_ceiling.hpp>

#include "../../support/temp_path.hpp"

#include <sqlite3.h>

#include <filesystem>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using Catch::Approx;
using reusex::ProjectDB;
using reusex::core::CeilingPlane;
using reusex::core::SpaceRecord;
using reusex::core::WallRecord;
using reusex::test_support::TempPath;
namespace fs = std::filesystem;

namespace {

void exec_raw(const fs::path &path, const std::string &sql) {
  sqlite3 *raw = nullptr;
  REQUIRE(sqlite3_open(path.string().c_str(), &raw) == SQLITE_OK);
  char *err = nullptr;
  const int rc = sqlite3_exec(raw, sql.c_str(), nullptr, nullptr, &err);
  INFO((err ? err : ""));
  sqlite3_free(err);
  sqlite3_close(raw);
  REQUIRE(rc == SQLITE_OK);
}

/// A 4 x 4 m attic room: ridge along x at y = 2, 35 deg pitch, knee walls
/// 1.15 m high at y = 0 and y = 4, ridge capped flat at 2.5 m.
SpaceRecord attic(const ProjectDB &db) {
  SpaceRecord s;
  s.guid = reusex::core::deterministic_guid(db.project_guid(), "attic");
  s.kind = "room";
  s.polygon.outer = {{0, 0}, {4, 0}, {4, 4}, {0, 4}};
  s.z_floor = 0.0;
  s.z_ceiling = 2.5;
  const double t = 0.7002075382097097; // tan(35 deg)
  s.ceiling_planes = {CeilingPlane{{0, 0, 1.15}, {0, t}},
                      CeilingPlane{{0, 4, 1.15}, {0, -t}}};
  return s;
}

WallRecord gable(const ProjectDB &db) {
  WallRecord w;
  w.guid = reusex::core::deterministic_guid(db.project_guid(), "gable");
  w.x0 = 0;
  w.y0 = 0;
  w.x1 = 0;
  w.y1 = 4;
  w.height = 2.5;
  w.thickness = 0.2;
  w.top = {{0.0, 1.15}, {2.0, 2.5}, {4.0, 1.15}};
  return w;
}

} // namespace

TEST_CASE("SlopedCeiling_CeilingAt_IsTheLowestOfCapAndPlanes",
          "[core][sloped]") {
  TempPath tmp("test_sloped_ceiling_at");
  ProjectDB db(tmp.path);
  const auto s = attic(db);
  CHECK(reusex::core::ceiling_at(s, 2.0, 0.0) == Approx(1.15));
  CHECK(reusex::core::ceiling_at(s, 2.0, 1.0) == Approx(1.15 + 0.7002075));
  CHECK(reusex::core::ceiling_at(s, 2.0, 3.0) == Approx(1.15 + 0.7002075));
  // The planes meet at 2.55 m over the ridge; the cap holds it at 2.5.
  CHECK(reusex::core::ceiling_at(s, 2.0, 2.0) == Approx(2.5));
  SpaceRecord flat = s;
  flat.ceiling_planes.clear();
  CHECK(reusex::core::ceiling_at(flat, 1.0, 1.0) == 2.5);
}

TEST_CASE("SlopedCeiling_WallTopAt_InterpolatesAndCaps", "[core][sloped]") {
  TempPath tmp("test_sloped_wall_top");
  ProjectDB db(tmp.path);
  auto w = gable(db);
  CHECK(reusex::core::wall_top_at(w, 0.0) == Approx(1.15));
  CHECK(reusex::core::wall_top_at(w, 1.0) == Approx(1.825));
  CHECK(reusex::core::wall_top_at(w, 2.0) == Approx(2.5));
  CHECK(reusex::core::wall_top_at(w, -1.0) == Approx(1.15)); // held past ends
  CHECK(reusex::core::wall_top_at(w, 5.0) == Approx(1.15));
  w.height = 2.0; // the cap cuts the peak
  CHECK(reusex::core::wall_top_at(w, 2.0) == Approx(2.0));
  w.top.clear();
  CHECK(reusex::core::wall_top_at(w, 1.0) == Approx(2.0));
}

TEST_CASE("SlopedCeiling_Checks_RejectBadPlanesAndProfiles", "[core][sloped]") {
  TempPath tmp("test_sloped_checks");
  ProjectDB db(tmp.path);
  auto s = attic(db);
  CHECK(reusex::core::check_ceiling_planes(s).empty());
  auto steep = s;
  steep.ceiling_planes[0].slope = {0.0, 6.0}; // ~80.5 deg
  CHECK_FALSE(reusex::core::check_ceiling_planes(steep).empty());
  auto low = s;
  low.ceiling_planes[0].at[2] = -0.5; // below the floor at y = 0
  CHECK_FALSE(reusex::core::check_ceiling_planes(low).empty());
  auto nan = s;
  nan.ceiling_planes[1].at[0] = std::numeric_limits<double>::quiet_NaN();
  CHECK_FALSE(reusex::core::check_ceiling_planes(nan).empty());

  auto w = gable(db);
  CHECK(reusex::core::check_wall_top(w).empty());
  auto one = w;
  one.top.resize(1);
  CHECK_FALSE(reusex::core::check_wall_top(one).empty());
  auto back = w;
  back.top[2][0] = 1.0; // s goes back
  CHECK_FALSE(reusex::core::check_wall_top(back).empty());
  auto under = w;
  under.top[0][1] = 0.0; // at z_base
  CHECK_FALSE(reusex::core::check_wall_top(under).empty());
}

TEST_CASE("SlopedCeiling_Codec_RoundTrips", "[core][sloped]") {
  TempPath tmp("test_sloped_codec");
  ProjectDB db(tmp.path);
  const auto s = attic(db);
  CHECK(reusex::core::encode_ceiling_planes({}).empty());
  CHECK(reusex::core::decode_ceiling_planes("").empty());
  const auto planes = reusex::core::decode_ceiling_planes(
      reusex::core::encode_ceiling_planes(s.ceiling_planes));
  REQUIRE(planes.size() == 2);
  CHECK(planes[1].at == s.ceiling_planes[1].at);
  CHECK(planes[1].slope == s.ceiling_planes[1].slope);
  const auto w = gable(db);
  CHECK(reusex::core::decode_wall_top(reusex::core::encode_wall_top(w.top)) ==
        w.top);
  CHECK_THROWS_AS(reusex::core::decode_wall_top("[[1]]"), std::runtime_error);
  CHECK_THROWS_AS(reusex::core::decode_ceiling_planes("{}"),
                  std::runtime_error);
}

TEST_CASE("MigrationV39_FreshProject_RoundTripsSlopes",
          "[projectdb][migration][sloped]") {
  TempPath tmp("test_migration_v39_fresh");
  ProjectDB db(tmp.path);
  CHECK(ProjectDB::latest_schema_version() >= 39);
  const auto s = attic(db);
  const auto w = gable(db);
  REQUIRE(db.save_walls({w}, {}, -1) == 1);
  REQUIRE(db.save_spaces({s}, {}, -1) == 1);
  const auto spaces = db.spaces();
  REQUIRE(spaces.size() == 1);
  REQUIRE(spaces[0].ceiling_planes.size() == 2);
  CHECK(spaces[0].ceiling_planes[0].slope == s.ceiling_planes[0].slope);
  CHECK(spaces[0].ceiling_planes[1].at == s.ceiling_planes[1].at);
  const auto walls = db.walls();
  REQUIRE(walls.size() == 1);
  CHECK(walls[0].top == w.top);
  REQUIRE(db.space_by_guid(s.guid));
  CHECK(db.space_by_guid(s.guid)->ceiling_planes.size() == 2);
  REQUIRE(db.wall_by_guid(w.guid));
  CHECK(db.wall_by_guid(w.guid)->top.size() == 3);
}

TEST_CASE("MigrationV39_Save_RejectsInvalidSlopes",
          "[projectdb][migration][sloped]") {
  TempPath tmp("test_migration_v39_reject");
  ProjectDB db(tmp.path);
  auto s = attic(db);
  s.ceiling_planes[0].at[2] = -1.0;
  CHECK_THROWS_AS(db.save_spaces({s}, {}, -1), std::invalid_argument);
  auto w = gable(db);
  w.top[1][0] = 5.0; // s not increasing
  CHECK_THROWS_AS(db.save_walls({w}, {}, -1), std::invalid_argument);
}

TEST_CASE("MigrationV39_V38Project_KeepsRowsFlat",
          "[projectdb][migration][sloped]") {
  TempPath tmp("test_migration_v39_from_v38");
  std::string space_guid, wall_guid;
  {
    ProjectDB db(tmp.path);
    auto s = attic(db);
    s.ceiling_planes.clear();
    auto w = gable(db);
    w.top.clear();
    space_guid = s.guid;
    wall_guid = w.guid;
    db.save_walls({w}, {}, -1);
    db.save_spaces({s}, {}, -1);
  }
  // A v38 project: no slope columns.
  exec_raw(tmp.path, R"(
    ALTER TABLE spaces DROP COLUMN ceiling_planes;
    ALTER TABLE walls DROP COLUMN top_profile;
    DELETE FROM schema_version WHERE version >= 39;
  )");
  {
    // Read-only: not migrated, read without the columns.
    ProjectDB ro(tmp.path, /*readOnly=*/true);
    CHECK(ro.schema_version() == 38);
    REQUIRE(ro.spaces().size() == 1);
    CHECK(ro.spaces()[0].ceiling_planes.empty());
    REQUIRE(ro.walls().size() == 1);
    CHECK(ro.walls()[0].top.empty());
    CHECK(ro.space_by_guid(space_guid).has_value());
    CHECK(ro.wall_by_guid(wall_guid).has_value());
  }
  ProjectDB db(tmp.path);
  CHECK(db.schema_version() == 39);
  REQUIRE(db.spaces().size() == 1);
  CHECK(db.spaces()[0].ceiling_planes.empty());
  CHECK(db.spaces()[0].z_ceiling == 2.5);
  REQUIRE(db.walls().size() == 1);
  CHECK(db.walls()[0].top.empty());
  // And the migrated project stores slopes.
  auto s = db.spaces()[0];
  s.ceiling_planes = attic(db).ceiling_planes;
  db.save_spaces({s}, {}, -1);
  CHECK(db.spaces()[0].ceiling_planes.size() == 2);
}

TEST_CASE("SlopedCeiling_MapCeilingPlane_FollowsAFrameChange",
          "[core][sloped]") {
  // 90 deg yaw about z plus a shift (1, 2, 0.5): a plane rising along +y
  // rises along -x afterwards, and every point of it maps onto the new one.
  const std::array<double, 16> m{0, -1, 0, 1,   1, 0, 0, 2,
                                 0, 0,  1, 0.5, 0, 0, 0, 1};
  const CeilingPlane p{{0.0, 0.0, 1.15}, {0.0, 0.7}};
  const CeilingPlane q = reusex::core::map_ceiling_plane(p, m);
  CHECK(q.slope[0] == Approx(-0.7));
  CHECK(q.slope[1] == Approx(0.0).margin(1e-12));
  for (const auto &[x, y] :
       std::vector<std::pair<double, double>>{{0, 0}, {2, 3}, {-1, 4}}) {
    const double z = p.z_at(x, y);
    // (x, y, z) -> (-y + 1, x + 2, z + 0.5)
    CHECK(q.z_at(-y + 1, x + 2) == Approx(z + 0.5));
  }
}
