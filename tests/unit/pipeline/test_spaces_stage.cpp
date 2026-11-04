// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// The `spaces` pipeline stage (reusex-z1v.6.3): the band rooms of `rooms`
// polygonised against the walls table -> the spaces and space_boundaries
// tables, through pipeline::run_stage (the path `rux create spaces` and the
// Qt Pipeline page share).

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/guid.hpp>
#include <reusex/core/model_edits.hpp>
#include <reusex/core/sloped_ceiling.hpp>
#include <reusex/core/space_identity.hpp>
#include <reusex/core/space_polygon.hpp>
#include <reusex/core/stage_contract.hpp>
#include <reusex/core/validate.hpp>
#include <reusex/io/solid_model_json.hpp>
#include <reusex/pipeline/space_parameters.hpp>
#include <reusex/pipeline/stage_parameters.hpp>
#include <reusex/pipeline/stages.hpp>
#include <reusex/types/point_types.hpp>

#include <Eigen/Core>

#include "../../support/temp_path.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <set>
#include <string>
#include <vector>

using Catch::Matchers::WithinAbs;
using namespace reusex;
using namespace reusex::pipeline;
using reusex::test_support::TempPath;

namespace {

/// The openings stage test's two rooms (copied: tests share no fixtures
/// header yet). The walls stage test's two rooms (floor z = 0, ceiling z = 2.5
/// over [0, 4] x [0, 3], a 10 cm partition at x = 2), with a door (y 1.0
/// .. 1.9, floor to 2.1 m) through the partition and a window (x 2.5 .. 3.5, z
/// 0.9
/// .. 2.0) in the south wall. Every face is its own plane, as `create planes`
/// leaves them; the plane classes are written as `create classify` would.
/// @p door_y0 .. @p door_y1 move the door's jambs and @p door_head its head
/// (RoomsStage_AfterWallsAndOpenings_CutsAtAWideDoor: a doorway wider than
/// the band's constriction cut, its lintel above the wall band).
void seed_rooms(ProjectDB &db, double door_y0 = 1.0, double door_y1 = 1.9,
                double door_head = 2.1) {
  const bool oriented = true;
  Cloud cloud;
  CloudL planes;
  CloudN normals;
  CloudN point_normals; // per point: its plane's normal (the band reads it)
  pcl::PointCloud<pcl::PointXYZ> centroids;
  std::vector<core::PlaneClassRecord> classes;
  uint32_t label = 0;
  const double step = 0.05;
  auto plane = [&](const char *orientation, const char *role, double nx,
                   double ny, double nz, auto &&points) {
    ++label;
    int n = 0;
    NormalT nn;
    nn.normal_x = float(nx);
    nn.normal_y = float(ny);
    nn.normal_z = float(nz);
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    points([&](double x, double y, double z) {
      PointT p;
      p.x = float(x);
      p.y = float(y);
      p.z = float(z);
      cloud.push_back(p);
      LabelT l;
      l.label = label;
      planes.push_back(l);
      point_normals.push_back(nn);
      sum += Eigen::Vector3d(x, y, z);
      ++n;
    });
    normals.push_back(nn);
    const Eigen::Vector3d mean = sum / std::max(n, 1);
    centroids.push_back(
        pcl::PointXYZ(float(mean.x()), float(mean.y()), float(mean.z())));
    core::PlaneClassRecord c;
    c.plane_label = static_cast<int>(label);
    c.guid = core::deterministic_guid(
        db.project_guid(), "plane_classes/planes/" + std::to_string(label));
    c.orientation = orientation;
    c.role = role;
    c.role_source = "rules";
    c.storey = 0;
    c.point_count = n;
    classes.push_back(c);
  };
  auto horizontal = [&](double z) {
    return [=](auto &&emit) {
      for (double x = 0.5 * step; x < 4.0; x += step)
        for (double y = 0.5 * step; y < 3.0; y += step)
          emit(x, y, z);
    };
  };
  // A vertical face from (x0, y0) to (x1, y1).
  auto face = [&](double x0, double y0, double x1, double y1) {
    return [=](auto &&emit) {
      const double len = std::hypot(x1 - x0, y1 - y0);
      for (double s = 0.5 * step; s < len; s += step)
        for (double z = 0.5 * step; z < 2.5; z += step) {
          const double x = x0 + (x1 - x0) * s / len;
          const double y = y0 + (y1 - y0) * s / len;
          const bool door = std::abs(x - 2.0) < 0.1 && y > door_y0 &&
                            y < door_y1 && z < door_head;
          const bool window =
              y < 0.01 && x > 2.5 && x < 3.5 && z > 0.9 && z < 2.0;
          if (!door && !window)
            emit(x, y, z);
        }
    };
  };
  plane("horizontal", "floor", 0, 0, 1, horizontal(0.0));
  plane("horizontal", "ceiling", 0, 0, -1, horizontal(2.5));
  plane("vertical", "wall", 0, 1, 0, face(0, 0, 4, 0));        // south, seen +y
  plane("vertical", "wall", 0, -1, 0, face(0, 3, 4, 3));       // north, seen -y
  plane("vertical", "wall", 1, 0, 0, face(0, 0, 0, 3));        // west, seen +x
  plane("vertical", "wall", -1, 0, 0, face(4, 0, 4, 3));       // east, seen -x
  plane("vertical", "wall", -1, 0, 0, face(1.95, 0, 1.95, 3)); // partition
  plane("vertical", "wall", 1, 0, 0, face(2.05, 0, 2.05, 3));  // partition

  const std::string marker =
      oriented ? R"({"normal_orientation":"sensor"})" : "{}";
  db.save_point_cloud("cloud", cloud, "reconstruct", marker);
  db.save_point_cloud("planes", planes, "segment_planes", "{}");
  db.save_point_cloud("plane_normals", normals, "segment_planes", marker);
  db.save_point_cloud("normals", point_normals, "reconstruct", marker);
  db.save_point_cloud("plane_centroids", centroids, "segment_planes", "{}");

  core::StoreyRecord s;
  s.guid = core::deterministic_guid(db.project_guid(), "storeys/z0");
  s.name = "Etage 0";
  s.z_floor = 0.0;
  s.z_ceiling = 2.5;
  db.save_storeys({s});
  db.save_plane_classes("planes", classes);
}

StageResult run(ProjectDB &db, const TempPath &project, JobStage stage,
                const std::string &params = "{}") {
  StageContext ctx;
  ctx.project = project.path;
  ctx.stage = stage;
  ctx.parameters = params;
  return run_stage(db, ctx);
}

/// walls -> openings -> rooms on the two rooms.
void seed_band_rooms(ProjectDB &db, const TempPath &project) {
  seed_rooms(db);
  for (auto stage : {JobStage::walls, JobStage::openings, JobStage::rooms}) {
    const auto r = run(db, project, stage);
    INFO(to_string(stage) << ": " << r.message);
    REQUIRE(r.ok);
  }
}

std::vector<std::string> guids(const ProjectDB &db) {
  std::vector<std::string> out;
  for (const auto &s : db.spaces())
    out.push_back(s.guid);
  std::sort(out.begin(), out.end());
  return out;
}

} // namespace

TEST_CASE("SpacesStage_Names_AndContract", "[pipeline][stages][spaces]") {
  CHECK(to_string(JobStage::spaces) == "spaces");
  CHECK(pipeline_log_name(JobStage::spaces) == "spaces");
  CHECK(parse_job_stage("spaces") == JobStage::spaces);
  const auto &c = core::stage_contract(core::PipelineStage::spaces);
  CHECK(c.command == "rux create spaces");
  REQUIRE(c.outputs.size() == 2);
  CHECK(c.outputs[0] == "spaces");
  CHECK(c.outputs[1] == "space_boundaries");
}

TEST_CASE("SpacesStage_Parameters_AreSpacesOptionsDefaults",
          "[pipeline][stages][spaces]") {
  const SpaceStageOptions d{};
  const auto &params = stage_parameters(JobStage::spaces);
  REQUIRE(params.size() == space_parameter_table().size());
  auto value = [&](const char *key) -> ParameterDefault {
    for (const auto &p : params)
      if (p.key == key)
        return p.default_value;
    FAIL("no parameter " << key);
    return {};
  };
  CHECK(std::get<bool>(value("use_walls")) == d.spaces.use_walls);
  CHECK(std::get<bool>(value("axes_only")) == d.spaces.axes_only);
  CHECK(std::get<double>(value("join_extension_m")) ==
        d.spaces.join_extension_m);
  CHECK(std::get<long long>(value("open_plan_min_holes")) ==
        d.spaces.open_plan_min_holes);
  CHECK(std::get<double>(value("guid_match_m")) == d.identity.match_m);
  CHECK(std::get<double>(value("guid_min_area_ratio")) ==
        d.identity.min_area_ratio);
  CHECK(std::get<double>(value("guid_min_overlap")) == d.identity.min_overlap);

  const auto again =
      space_options_from_parameters(space_parameters_json(SpaceStageOptions{}));
  CHECK(space_parameters_json(again) == space_parameters_json(d));
  const auto o = space_options_from_parameters(
      R"({"join_extension_m": 0.5, "axes_only": true, "use_walls": false})");
  CHECK(o.spaces.join_extension_m == 0.5);
  CHECK(o.spaces.axes_only);
  CHECK_FALSE(o.spaces.use_walls);
  CHECK_THROWS_AS(space_options_from_parameters(R"({"pure_min_share": 2})"),
                  std::invalid_argument);
  CHECK_THROWS_AS(space_options_from_parameters("[1]"), std::invalid_argument);
}

TEST_CASE("SpacesStage_NoRooms_IsRefusedAsInvalidInput",
          "[pipeline][stages][spaces]") {
  TempPath project("test_spaces_stage_norooms");
  ProjectDB db(project.path);
  seed_rooms(db);
  const auto r = run(db, project, JobStage::spaces);
  CHECK_FALSE(r.ok);
  CHECK(r.invalid_input);
  CHECK_FALSE(db.has_spaces());
}

TEST_CASE("SpacesStage_TwoRooms_TwoSpacesBoundedByTheWalls",
          "[pipeline][stages][spaces]") {
  TempPath project("test_spaces_stage_rooms");
  ProjectDB db(project.path);
  seed_band_rooms(db, project);
  const auto r = run(db, project, JobStage::spaces);
  INFO(r.message);
  REQUIRE(r.ok);
  REQUIRE(r.outputs.size() == 2);
  CHECK(r.outputs[0].name == "spaces");
  CHECK(r.outputs[1].name == "space_boundaries");

  const auto rows = db.spaces();
  REQUIRE(rows.size() == 2);
  CHECK(r.outputs[0].count == 2);
  const auto labels = db.point_cloud_label("rooms");
  std::set<int> room_ids;
  for (const auto &l : labels->points)
    if (l.label > 0)
      room_ids.insert(static_cast<int>(l.label));
  const auto walls = db.walls();
  std::set<std::string> wall_guids;
  for (const auto &w : walls)
    wall_guids.insert(w.guid);
  for (const auto &s : rows) {
    INFO("space " << s.guid << " area " << s.area);
    CHECK(s.guid.size() == 36);
    CHECK(s.storey == 0);
    CHECK(s.kind == "room");
    CHECK(s.source == "detected");
    CHECK(room_ids.count(s.room_label) == 1);
    // Each half of 4 x 3 minus the 10 cm partition: 1.95 x 3 (faces of
    // the walls, within a cell).
    CHECK_THAT(s.area, WithinAbs(1.95 * 3.0, 0.3));
    CHECK_THAT(core::space_polygon_area(s.polygon), WithinAbs(s.area, 1e-6));
    CHECK_THAT(s.volume, WithinAbs(s.area * 2.5, 1e-6));
    CHECK(s.confidence > 0.5); // mostly wall-bounded faces
  }
  CHECK(rows[0].room_label != rows[1].room_label);
  const auto c0 = core::space_polygon_centroid(rows[0].polygon);
  const auto c1 = core::space_polygon_centroid(rows[1].polygon);
  CHECK(std::abs(c0[0] - c1[0]) > 1.5); // one each side of x = 2

  const auto bounds = db.space_boundaries();
  CHECK(r.outputs[1].count == static_cast<int64_t>(bounds.size()));
  CHECK(bounds.size() >= 6); // 4 walls around each, the partition shared
  for (const auto &b : bounds) {
    CHECK(wall_guids.count(b.wall) == 1);
    CHECK(b.s_end > b.s_start);
  }
  const auto log = db.pipeline_log(1);
  REQUIRE(log.size() == 1);
  CHECK(log[0].stage == "spaces");

  // GUIDs stable across a re-run, and the model reads them.
  const auto first = guids(db);
  REQUIRE(run(db, project, JobStage::spaces).ok);
  CHECK(guids(db) == first);
  const auto m = io::load_solid_model(db);
  CHECK(m.spaces.size() == 2);
  CHECK(m.space_boundaries.size() == bounds.size());
  CHECK(m.has_spaces());
}

TEST_CASE("SpacesStage_LockedRow_SurvivesARerun",
          "[pipeline][stages][spaces]") {
  TempPath project("test_spaces_stage_locked");
  ProjectDB db(project.path);
  seed_band_rooms(db, project);
  REQUIRE(run(db, project, JobStage::spaces).ok);
  auto rows = db.spaces();
  REQUIRE(rows.size() == 2);
  rows.front().name = "Køkken"; // a hand fix
  rows.front().source = "manual";
  rows.front().locked = true;
  db.save_spaces(rows, db.space_boundaries());

  const auto r = run(db, project, JobStage::spaces);
  INFO(r.message);
  REQUIRE(r.ok);
  CHECK(r.message.find("1 locked") != std::string::npos);
  CHECK(db.spaces().size() == 2);
  const auto kept = db.space_by_guid(rows.front().guid);
  REQUIRE(kept.has_value());
  CHECK(kept->locked);
  CHECK(kept->name == "Køkken");
}

TEST_CASE("SpacesStage_LockedRow_KeepsItsGeometry_AfterAWallEdit",
          "[pipeline][stages][spaces][model_edits]") {
  // reusex-s4mm: a space named and locked by hand (`rux set`, no editor
  // marks) whose outline, heights and ceiling plane differ from what
  // detection finds; then a wall edit in the editor and the chain walls ->
  // openings -> rooms -> spaces. The locked row comes back as stored.
  TempPath project("test_spaces_stage_locked_geometry");
  ProjectDB db(project.path);
  seed_band_rooms(db, project);
  REQUIRE(run(db, project, JobStage::spaces).ok);
  auto rows = db.spaces();
  REQUIRE(rows.size() == 2);
  auto locked = rows.front();
  locked.name = "S0-01 Køkken";
  locked.source = "manual";
  locked.locked = true;
  // A hand-made outline: the room's bounding box shrunk by 0.2 m.
  double x0 = 1e9, y0 = 1e9, x1 = -1e9, y1 = -1e9;
  for (const auto &p : locked.polygon.outer) {
    x0 = std::min(x0, p[0]);
    y0 = std::min(y0, p[1]);
    x1 = std::max(x1, p[0]);
    y1 = std::max(y1, p[1]);
  }
  locked.polygon.outer = {{x0 + 0.2, y0 + 0.2},
                          {x1 - 0.2, y0 + 0.2},
                          {x1 - 0.2, y1 - 0.2},
                          {x0 + 0.2, y1 - 0.2}};
  locked.polygon.holes.clear();
  locked.z_ceiling = 2.3;
  locked.ceiling_planes = {
      core::CeilingPlane{{0.5 * (x0 + x1), 0.5 * (y0 + y1), 2.2}, {0.05, 0.0}}};
  db.update_spaces({locked});
  const auto before = *db.space_by_guid(locked.guid);

  // One wall edit in the editor (written back as it is: manual, locked).
  const auto walls = db.walls();
  REQUIRE_FALSE(walls.empty());
  core::apply_model_edit(db, {core::ModelEditOp::put(walls.front())});

  for (auto stage : {JobStage::walls, JobStage::openings, JobStage::rooms,
                     JobStage::spaces}) {
    const auto r = run(db, project, stage);
    INFO(to_string(stage) << ": " << r.message);
    REQUIRE(r.ok);
  }
  const auto after = db.space_by_guid(locked.guid);
  REQUIRE(after.has_value());
  CHECK(after->name == before.name);
  CHECK(after->locked);
  CHECK(after->source == "manual");
  CHECK(after->polygon.outer == before.polygon.outer);
  CHECK(after->polygon.holes == before.polygon.holes);
  CHECK(after->z_floor == before.z_floor);
  CHECK(after->z_ceiling == before.z_ceiling);
  CHECK(core::encode_ceiling_planes(after->ceiling_planes) ==
        core::encode_ceiling_planes(before.ceiling_planes));
  CHECK(after->area == before.area);
  // Still two spaces: the detection of the locked room was dropped, not
  // added next to it.
  CHECK(db.spaces().size() == 2);
  // Its room_label names a room of today's rooms cloud.
  const auto labels = db.point_cloud_label("rooms");
  std::set<int> room_ids;
  for (const auto &l : labels->points)
    if (l.label > 0)
      room_ids.insert(static_cast<int>(l.label));
  CHECK(room_ids.count(after->room_label) == 1);
  // Its boundaries are the stored ones on walls that still exist.
  std::set<std::string> wall_guids;
  for (const auto &w : db.walls())
    wall_guids.insert(w.guid);
  for (const auto &b : db.space_boundaries())
    CHECK(wall_guids.count(b.wall) == 1);
  CHECK(db.dangling_space_boundaries() == 0);
}

TEST_CASE("SpacesStage_StaleRooms_IsRefused", "[pipeline][stages][spaces]") {
  // The band grid is rebuilt from today's tables: without the walls and
  // openings nothing cuts the wide doorway, so the grid has one room where
  // 'rooms' has two.
  TempPath project("test_spaces_stage_stale");
  ProjectDB db(project.path);
  seed_rooms(db, 0.8, 2.2, 2.35); // a wide doorway only the door cut closes
  for (auto stage : {JobStage::walls, JobStage::openings, JobStage::rooms})
    REQUIRE(run(db, project, stage).ok);
  db.delete_openings();
  db.delete_walls();
  const auto r = run(db, project, JobStage::spaces);
  INFO(r.message);
  CHECK_FALSE(r.ok);
  CHECK(r.invalid_input);
  CHECK(r.message.find("re-run `rux create rooms`") != std::string::npos);
  CHECK_FALSE(db.has_spaces());
}

TEST_CASE("SpacesStage_WallEditWithoutRooms_NamesTheRoomsStage",
          "[pipeline][stages][spaces][model_edits]") {
  // reusex-14aj: after a wall edit the rooms cloud's grid is stale. `rux
  // validate --stage spaces` says so up front and names the rooms stage; the
  // spaces stage refuses with the changed table; walls -> rooms -> spaces
  // runs.
  TempPath project("test_spaces_stage_stale_walls");
  ProjectDB db(project.path);
  seed_band_rooms(db, project);
  auto stale = [&] {
    std::vector<core::ValidationIssue> issues;
    for (const auto &i :
         core::validate_stage(db, core::PipelineStage::spaces).issues)
      if (i.check == "stale_rooms_grid")
        issues.push_back(i);
    return issues;
  };
  CHECK(stale().empty());
  REQUIRE(run(db, project, JobStage::spaces).ok);
  CHECK(stale().empty()); // writing spaces does not touch the grid's inputs

  // Move the partition 0.4 m east.
  auto walls = db.walls();
  auto partition = std::find_if(walls.begin(), walls.end(), [](const auto &w) {
    return std::abs(w.x0 - 2.0) < 0.2 && std::abs(w.x1 - 2.0) < 0.2;
  });
  REQUIRE(partition != walls.end());
  partition->x0 += 0.4;
  partition->x1 += 0.4;
  core::apply_model_edit(db, {core::ModelEditOp::put(*partition)});

  const auto issues = stale();
  REQUIRE(issues.size() == 1);
  CHECK(issues[0].severity == core::ValidationSeverity::warning);
  CHECK(issues[0].artifact == "rooms");
  CHECK(issues[0].message.find("walls") != std::string::npos);
  CHECK(issues[0].commands == std::vector<std::string>{"rux create rooms"});

  const auto refused = run(db, project, JobStage::spaces);
  INFO(refused.message);
  CHECK_FALSE(refused.ok);
  CHECK(refused.invalid_input);
  CHECK(refused.message.find("changed since rooms ran: walls") !=
        std::string::npos);
  CHECK(refused.message.find("re-run `rux create rooms`") != std::string::npos);

  // The walls stage keeps the edited wall and finds the rest again: same
  // inputs, so rooms then spaces run and the warning is gone.
  for (auto stage : {JobStage::walls, JobStage::rooms, JobStage::spaces}) {
    const auto r = run(db, project, stage);
    INFO(to_string(stage) << ": " << r.message);
    REQUIRE(r.ok);
  }
  CHECK(stale().empty());
  REQUIRE(run(db, project, JobStage::walls).ok);
  CHECK(stale().empty()); // re-detected the same: still fresh
}

TEST_CASE("SpacesStage_LeidenRooms_IsRefused", "[pipeline][stages][spaces]") {
  TempPath project("test_spaces_stage_leiden");
  ProjectDB db(project.path);
  seed_rooms(db);
  const auto rooms =
      run(db, project, JobStage::rooms, R"({"method":"leiden"})");
  INFO(rooms.message);
  REQUIRE(rooms.ok);
  const auto r = run(db, project, JobStage::spaces);
  INFO(r.message);
  CHECK_FALSE(r.ok);
  CHECK(r.invalid_input);
  CHECK(r.message.find("leiden") != std::string::npos);
}

TEST_CASE("SpacesStage_RenamedThenStoreysRedetected_NamesSurviveNoDuplicates",
          "[pipeline][stages][spaces]") {
  // reusex-djf: name a space by hand, re-detect the storeys (the seeded
  // storey has the identity frame; the detected one puts its origin at the
  // floor centroid, so the frame moves by ~(2, 1.5) m), re-run the chain
  // walls -> openings -> rooms -> spaces: the name, kind and lock stay on
  // the same room, in the same place in the world, and no row is doubled.
  TempPath project("test_spaces_stage_reframe");
  ProjectDB db(project.path);
  seed_band_rooms(db, project);
  REQUIRE(run(db, project, JobStage::spaces).ok);
  auto rows = db.spaces();
  REQUIRE(rows.size() == 2);
  const auto old_storey = db.storeys().at(0);
  auto named = rows.front();
  named.name = "S0-01 Køkken";
  named.kind = "corridor";
  named.source = "manual";
  named.locked = true;
  db.update_spaces({named}); // what `rux set spaces.<guid>.name` writes
  const auto world_before = core::space_polygon_centroid(
      core::space_polygon_world(named, old_storey));

  const auto st = run(db, project, JobStage::storeys);
  INFO(st.message);
  REQUIRE(st.ok);
  const auto new_storey = db.storeys().at(0);
  REQUIRE(new_storey.guid == old_storey.guid);
  REQUIRE(new_storey.level_transform != old_storey.level_transform);
  for (auto stage : {JobStage::walls, JobStage::openings, JobStage::rooms,
                     JobStage::spaces}) {
    const auto r = run(db, project, stage);
    INFO(to_string(stage) << ": " << r.message);
    REQUIRE(r.ok);
  }

  rows = db.spaces();
  CHECK(rows.size() == 2);
  int named_rows = 0;
  std::set<std::string> ids;
  for (const auto &s : rows) {
    ids.insert(s.guid);
    CHECK(s.metadata.find("unmatched") == std::string::npos);
    if (s.name != "S0-01 Køkken")
      continue;
    ++named_rows;
    CHECK(s.guid == named.guid);
    CHECK(s.kind == "corridor");
    CHECK(s.locked);
    CHECK(s.source == "manual");
    const auto c =
        core::space_polygon_centroid(core::space_polygon_world(s, new_storey));
    CHECK_THAT(c[0], WithinAbs(world_before[0], 0.1));
    CHECK_THAT(c[1], WithinAbs(world_before[1], 0.1));
  }
  CHECK(named_rows == 1);
  CHECK(ids.size() == rows.size());
  // Every boundary belongs to a stored space and names a stored wall.
  std::set<std::string> walls;
  for (const auto &w : db.walls())
    walls.insert(w.guid);
  for (const auto &b : db.space_boundaries()) {
    CHECK(ids.count(b.space) == 1);
    CHECK(walls.count(b.wall) == 1);
  }
  CHECK(db.dangling_space_boundaries() == 0);
}

TEST_CASE("SpacesStage_UnmatchedLockedSpace_IsKeptFlagged",
          "[pipeline][stages][spaces]") {
  // A locked space no detected room covers (drawn by hand outside the
  // scan) stays, flagged, and is not doubled on the next run.
  TempPath project("test_spaces_stage_unmatched");
  ProjectDB db(project.path);
  seed_band_rooms(db, project);
  REQUIRE(run(db, project, JobStage::spaces).ok);
  core::SpaceRecord drawn;
  drawn.guid = core::deterministic_guid(db.project_guid(), "spaces/drawn");
  drawn.name = "S0-99 Udeareal";
  drawn.source = "manual";
  drawn.locked = true;
  drawn.polygon.outer = {{10, 10}, {14, 10}, {14, 13}, {10, 13}};
  drawn.area = 12.0;
  auto rows = db.spaces();
  rows.push_back(drawn);
  db.save_spaces(rows, db.space_boundaries());
  REQUIRE(db.spaces().size() == 3);

  for (int pass = 0; pass < 2; ++pass) {
    const auto r = run(db, project, JobStage::spaces);
    INFO(r.message);
    REQUIRE(r.ok);
    CHECK(r.message.find("1 unmatched locked") != std::string::npos);
    CHECK(db.spaces().size() == 3);
    const auto kept = db.space_by_guid(drawn.guid);
    REQUIRE(kept.has_value());
    CHECK(kept->name == "S0-99 Udeareal");
    CHECK(kept->polygon.outer == drawn.polygon.outer);
    CHECK(kept->metadata.find("\"unmatched\":true") != std::string::npos);
  }
}

TEST_CASE("SpacesStage_Rerun_KeepsHandEditedOutlines_AndDeletedSpaces",
          "[pipeline][stages][spaces][model_edits]") {
  TempPath project("test_spaces_stage_edits");
  ProjectDB db(project.path);
  seed_band_rooms(db, project);
  REQUIRE(run(db, project, JobStage::spaces).ok);
  const auto rows = db.spaces();
  REQUIRE(rows.size() == 2);
  // Redraw the first space as a plain rectangle and delete the second.
  auto redrawn = rows[0];
  const auto c = core::space_polygon_centroid(redrawn.polygon);
  redrawn.polygon.outer = {{c[0] - 0.5, c[1] - 0.5},
                           {c[0] + 0.5, c[1] - 0.5},
                           {c[0] + 0.5, c[1] + 0.5},
                           {c[0] - 0.5, c[1] + 0.5}};
  redrawn.polygon.holes.clear();
  redrawn.name = "Bad";
  core::apply_model_edit(
      db,
      {core::ModelEditOp::put(redrawn),
       core::ModelEditOp::remove(core::ModelElementKind::space, rows[1].guid)});
  const auto edited = *db.space_by_guid(rows[0].guid);

  const auto r = run(db, project, JobStage::spaces);
  INFO(r.message);
  REQUIRE(r.ok);
  const auto after = db.spaces();
  REQUIRE(after.size() == 1);
  CHECK(after[0].guid == rows[0].guid);
  CHECK(core::model_record_json(after[0]) == core::model_record_json(edited));
  CHECK_FALSE(db.space_by_guid(rows[1].guid));
}
