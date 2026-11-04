// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Slab rows and storey edits (schema v44, reusex-d5us): slabs stored and
// edited as kind 'slab', the ground storey flag, and storey delete / split
// / merge through the model edit wire ops.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/guid.hpp>
#include <reusex/core/model_edits.hpp>
#include <reusex/core/solid_model.hpp>

#include "../../support/temp_path.hpp"

#include <nlohmann/json.hpp>
#include <sqlite3.h>

#include <algorithm>
#include <filesystem>
#include <string>
#include <vector>

using Catch::Matchers::WithinAbs;
using namespace reusex;
using core::ModelEditOp;
using core::ModelElementKind;
using reusex::test_support::TempPath;
using json = nlohmann::json;

namespace {

core::StoreyRecord storey(int idx, double z0, double z1) {
  core::StoreyRecord s;
  s.guid = "storey-" + std::to_string(idx);
  s.idx = idx;
  s.name = "Etage " + std::to_string(idx);
  s.z_floor = z0;
  s.z_ceiling = z1;
  return s;
}

core::WallRecord wall(const std::string &guid, int st, double z_base) {
  core::WallRecord w;
  w.guid = guid;
  w.storey = st;
  w.x1 = 4.0;
  w.z_base = z_base;
  w.height = 2.4;
  w.thickness = 0.2;
  return w;
}

core::SpaceRecord space(const std::string &guid, int st, double z_floor) {
  core::SpaceRecord s;
  s.guid = guid;
  s.storey = st;
  s.kind = "room";
  s.polygon.outer = {{0, 0}, {4, 0}, {4, 3}, {0, 3}};
  s.z_floor = z_floor;
  s.z_ceiling = z_floor + 2.4;
  return s;
}

core::SlabRecord slab(const std::string &guid, int st, double z_top) {
  core::SlabRecord s;
  s.guid = guid;
  s.storey = st;
  s.kind = "floor";
  s.outline.outer = {{0, 0}, {4, 0}, {4, 3}, {0, 3}};
  s.z_top = z_top;
  s.thickness = 0.25;
  return s;
}

/// Three storeys: 0 [0, 2.6), 1 [2.6, 5.2), 2 [5.2, 7.8), one wall and one
/// space on each, and a detected floor slab on 1.
void seed(ProjectDB &db) {
  db.save_storeys(
      {storey(0, 0.0, 2.6), storey(1, 2.6, 5.2), storey(2, 5.2, 7.8)});
  db.save_walls({wall("w0", 0, 0.0), wall("w1", 1, 2.6), wall("w2", 2, 5.2)},
                {});
  db.save_spaces(
      {space("s0", 0, 0.0), space("s1", 1, 2.6), space("s2", 2, 5.2)}, {});
  db.save_slabs({slab("slab1", 1, 2.6)});
}

core::ModelEditResult edit(ProjectDB &db, const json &ops) {
  return core::apply_model_edit(db, core::model_edit_ops_from_json(db, ops));
}

int storey_of_wall(const ProjectDB &db, const std::string &g) {
  return db.wall_by_guid(g)->storey;
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

} // namespace

TEST_CASE("Slabs_SaveKeepsLockedAndHonoursTombstones", "[core][slabs]") {
  TempPath project("test_slabs_save");
  ProjectDB db(project.path);
  seed(db);
  REQUIRE(db.slabs().size() == 1);
  const auto stored = db.slab_by_guid("slab1");
  REQUIRE(stored);
  CHECK(stored->outline.outer.size() == 4);
  CHECK_THAT(stored->thickness, WithinAbs(0.25, 1e-12));

  // Edit it: thicker, a hole (a stair opening). A re-derivation keeps it.
  edit(db,
       json::array({{{"op", "slab.update"},
                     {"guid", "slab1"},
                     {"set",
                      {{"thickness", 0.3},
                       {"outline",
                        {{"outer", {{0, 0}, {4, 0}, {4, 3}, {0, 3}}},
                         {"holes", {{{1, 1}, {1, 2}, {2, 2}, {2, 1}}}}}}}}}}));
  auto e = db.slab_by_guid("slab1");
  REQUIRE(e);
  CHECK(e->locked);
  CHECK(e->source == "manual");
  CHECK(e->outline.holes.size() == 1);
  db.save_slabs({slab("slab1", 1, 2.6), slab("slab-new", 2, 5.2)});
  CHECK(db.slab_by_guid("slab1")->thickness == 0.3);
  CHECK(db.slab_by_guid("slab-new"));

  // A deleted detected slab stays deleted across a re-derivation.
  edit(db, json::array({{{"op", "slab.delete"}, {"guid", "slab-new"}}}));
  CHECK(db.dismissed_model_elements("slab").size() == 1);
  db.save_slabs({slab("slab-new", 2, 5.2)});
  CHECK_FALSE(db.slab_by_guid("slab-new"));
  CHECK(core::model_holds(db, ModelElementKind::slab).holds("slab1"));

  // Revert gives back the derived record (and unlocks it).
  edit(db, json::array({{{"op", "slab.revert"}, {"guid", "slab1"}}}));
  CHECK(db.slab_by_guid("slab1")->thickness == 0.25);
  CHECK_FALSE(db.slab_by_guid("slab1")->locked);

  // A drawn slab.
  edit(db, json::array({{{"op", "slab.create"},
                         {"record",
                          {{"guid", "drawn"},
                           {"storey", 0},
                           {"kind", "floor"},
                           {"outline", {{"outer", {{0, 0}, {2, 0}, {2, 2}}}}},
                           {"z_top", 0.0},
                           {"thickness", 0.2}}}}}));
  CHECK(db.slab_by_guid("drawn")->source == "manual");

  // Bad records are refused.
  CHECK_THROWS_AS(edit(db, json::array({{{"op", "slab.update"},
                                         {"guid", "drawn"},
                                         {"set", {{"thickness", 0.0}}}}})),
                  std::invalid_argument);
  CHECK_THROWS_AS(edit(db, json::array({{{"op", "slab.update"},
                                         {"guid", "drawn"},
                                         {"set", {{"kind", "wall"}}}}})),
                  std::invalid_argument);
}

TEST_CASE("Storeys_GroundFlag_OneStoreyAtATime", "[core][storeys]") {
  TempPath project("test_storeys_ground");
  ProjectDB db(project.path);
  seed(db);
  CHECK_FALSE(db.storey_by_guid("storey-0")->is_ground);
  edit(db, json::array({{{"op", "storey.update"},
                         {"guid", "storey-1"},
                         {"set", {{"is_ground", true}}}}}));
  CHECK(db.storey_by_guid("storey-1")->is_ground);
  const auto r = edit(db, json::array({{{"op", "storey.update"},
                                        {"guid", "storey-0"},
                                        {"set", {{"is_ground", true}}}}}));
  CHECK(db.storey_by_guid("storey-0")->is_ground);
  CHECK_FALSE(db.storey_by_guid("storey-1")->is_ground);
  CHECK(r.changes.size() == 2);
  core::undo_model_edit(db);
  CHECK(db.storey_by_guid("storey-1")->is_ground);
  CHECK_FALSE(db.storey_by_guid("storey-0")->is_ground);
  // A detection re-run keeps the (locked) flag.
  db.save_storeys(db.storeys());
  CHECK(db.storey_by_guid("storey-1")->is_ground);
}

TEST_CASE("Storeys_Delete_RefusedWhileItHoldsElements", "[core][storeys]") {
  TempPath project("test_storeys_delete");
  ProjectDB db(project.path);
  seed(db);
  try {
    edit(db, json::array({{{"op", "storey.delete"}, {"guid", "storey-2"}}}));
    FAIL("expected the delete to be refused");
  } catch (const std::invalid_argument &e) {
    const std::string what = e.what();
    CHECK(what.find("1 wall(s), 1 space(s)") != std::string::npos);
  }
  REQUIRE(db.storey_by_guid("storey-2"));
  // Move / delete them in the same batch: accepted.
  edit(db,
       json::array(
           {{{"op", "wall.update"}, {"guid", "w2"}, {"set", {{"storey", 1}}}},
            {{"op", "space.delete"}, {"guid", "s2"}},
            {{"op", "storey.delete"}, {"guid", "storey-2"}}}));
  CHECK_FALSE(db.storey_by_guid("storey-2"));
  CHECK(storey_of_wall(db, "w2") == 1);
}

TEST_CASE("Storeys_Split_RenumbersAboveAndMovesElementsByHeight",
          "[core][storeys]") {
  TempPath project("test_storeys_split");
  ProjectDB db(project.path);
  seed(db);
  // A wall standing on a mezzanine of storey 1 at 4.0.
  auto mezz = wall("w1m", 1, 4.0);
  edit(db, json::array({{{"op", "wall.create"},
                         {"record", core::model_record_json(mezz)}}}));
  const auto r = edit(db, json::array({{{"op", "storey.split"},
                                        {"guid", "storey-1"},
                                        {"z", 3.9},
                                        {"new_guid", "storey-1b"},
                                        {"name", "Hems"}}}));
  INFO(r.changes.size() << " changes");
  const auto rows = db.storeys();
  REQUIRE(rows.size() == 4);
  const auto a = db.storey_by_guid("storey-1");
  const auto b = db.storey_by_guid("storey-1b");
  REQUIRE(b);
  CHECK(a->idx == 1);
  CHECK(b->idx == 2);
  CHECK(b->name == "Hems");
  CHECK_THAT(a->z_ceiling, WithinAbs(3.9, 1e-12));
  CHECK_THAT(b->z_floor, WithinAbs(3.9, 1e-12));
  CHECK_THAT(b->z_ceiling, WithinAbs(5.2, 1e-12));
  CHECK(db.storey_by_guid("storey-2")->idx == 3);
  // The storey above moved up with its elements; detected rows stay
  // detected (renumbering is bookkeeping).
  CHECK(storey_of_wall(db, "w2") == 3);
  CHECK(db.space_by_guid("s2")->storey == 3);
  CHECK_FALSE(db.wall_by_guid("w2")->locked);
  // Storey 1's elements: by height.
  CHECK(storey_of_wall(db, "w1") == 1);
  CHECK(storey_of_wall(db, "w1m") == 2);
  CHECK(db.slab_by_guid("slab1")->storey == 1);

  // One undo step puts everything back.
  core::undo_model_edit(db);
  CHECK(db.storeys().size() == 3);
  CHECK(db.storey_by_guid("storey-2")->idx == 2);
  CHECK(storey_of_wall(db, "w2") == 2);
  CHECK(storey_of_wall(db, "w1m") == 1);

  CHECK_THROWS_AS(edit(db, json::array({{{"op", "storey.split"},
                                         {"guid", "storey-1"},
                                         {"z", 7.0}}})),
                  std::invalid_argument);
}

TEST_CASE("Storeys_Merge_AdjacentStoreys", "[core][storeys]") {
  TempPath project("test_storeys_merge");
  ProjectDB db(project.path);
  seed(db);
  SECTION("upper into lower") {
    edit(db, json::array({{{"op", "storey.merge"},
                           {"guids", {"storey-0", "storey-1"}}}}));
    REQUIRE(db.storeys().size() == 2);
    const auto a = db.storey_by_guid("storey-0");
    CHECK(a->idx == 0);
    CHECK_THAT(a->z_ceiling, WithinAbs(5.2, 1e-12));
    CHECK(storey_of_wall(db, "w1") == 0);
    CHECK(db.slab_by_guid("slab1")->storey == 0);
    CHECK(db.storey_by_guid("storey-2")->idx == 1);
    CHECK(storey_of_wall(db, "w2") == 1);
  }
  SECTION("lower into upper") {
    edit(db, json::array({{{"op", "storey.merge"},
                           {"guids", {"storey-1", "storey-0"}}}}));
    REQUIRE(db.storeys().size() == 2);
    const auto a = db.storey_by_guid("storey-1");
    CHECK(a->idx == 0);
    CHECK_THAT(a->z_floor, WithinAbs(0.0, 1e-12));
    CHECK(storey_of_wall(db, "w0") == 0);
    CHECK(storey_of_wall(db, "w1") == 0);
    CHECK(db.storey_by_guid("storey-2")->idx == 1);
    CHECK(storey_of_wall(db, "w2") == 1);
  }
  SECTION("refusals") {
    CHECK_THROWS_AS(
        edit(db, json::array({{{"op", "storey.merge"},
                               {"guids", {"storey-0", "storey-2"}}}})),
        std::invalid_argument);
    auto shifted = *db.storey_by_guid("storey-1");
    shifted.level_transform[3] = 1.0;
    db.put_storey_row(shifted);
    CHECK_THROWS_AS(
        edit(db, json::array({{{"op", "storey.merge"},
                               {"guids", {"storey-0", "storey-1"}}}})),
        std::invalid_argument);
  }
}

TEST_CASE("MigrationV44_V43Project_GetsSlabsAndGroundFlag",
          "[projectdb][migration][slabs]") {
  TempPath tmp("test_migration_v44");
  {
    ProjectDB db(tmp.path);
    seed(db);
  }
  exec_raw(tmp.path, R"(
    DROP TABLE slabs;
    ALTER TABLE storeys DROP COLUMN is_ground;
    DELETE FROM schema_version WHERE version >= 44;
  )");
  {
    ProjectDB ro(tmp.path, /*readOnly=*/true);
    CHECK(ro.schema_version() == 43);
    CHECK(ro.slabs().empty());
    REQUIRE(ro.storeys().size() == 3);
    CHECK_FALSE(ro.storeys()[0].is_ground);
  }
  ProjectDB db(tmp.path);
  CHECK(db.schema_version() == ProjectDB::latest_schema_version());
  db.save_slabs({slab("x", 0, 0.0)});
  CHECK(db.slabs().size() == 1);
}
