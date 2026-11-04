// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Zones (schema v43, reusex-pccw): named groups of spaces, stored in zones /
// zone_members, edited through core/model_edits.hpp as kind 'zone', their
// members following the spaces (a deleted space leaves its zones).

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/guid.hpp>
#include <reusex/core/model_edits.hpp>
#include <reusex/core/solid_model.hpp>
#include <reusex/core/zones.hpp>

#include "../../support/temp_path.hpp"

#include <nlohmann/json.hpp>
#include <sqlite3.h>

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

core::SpaceRecord room(const ProjectDB &db, const std::string &key, double x0,
                       int storey = 0) {
  core::SpaceRecord s;
  s.guid = core::deterministic_guid(db.project_guid(), key);
  s.storey = storey;
  s.name = key;
  s.kind = "room";
  s.polygon.outer = {{x0, 0}, {x0 + 4, 0}, {x0 + 4, 3}, {x0, 3}};
  s.z_ceiling = 2.5;
  s.area = 12.0;
  s.volume = 30.0;
  return s;
}

struct Seeded {
  core::SpaceRecord a, b, c;
};

Seeded seed(ProjectDB &db) {
  core::StoreyRecord st;
  st.guid = core::deterministic_guid(db.project_guid(), "storeys/0");
  st.z_ceiling = 2.5;
  core::StoreyRecord st1 = st;
  st1.guid = core::deterministic_guid(db.project_guid(), "storeys/1");
  st1.idx = 1;
  st1.z_floor = 3.0;
  st1.z_ceiling = 5.5;
  db.save_storeys({st, st1});
  Seeded m{room(db, "a", 0), room(db, "b", 5), room(db, "c", 0, 1)};
  db.save_spaces({m.a, m.b, m.c}, {});
  return m;
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

/// Apply wire ops; @returns the result.
core::ModelEditResult edit(ProjectDB &db, const json &ops) {
  return core::apply_model_edit(db, core::model_edit_ops_from_json(db, ops));
}

} // namespace

TEST_CASE("Zones_RecordJson_RoundTripsAndRefusesUnknownFields",
          "[core][zones]") {
  core::ZoneRecord z;
  z.guid = "z";
  z.name = "Bolig A";
  z.use = "bolig";
  z.spaces = {"s1", "s2"};
  z.metadata = R"({"note":1})";
  const json j = core::model_record_json(z);
  CHECK(j.at("spaces") == json::array({"s1", "s2"}));
  CHECK(j.at("use") == "bolig");
  const auto back = core::zone_from_json(j);
  CHECK(back.name == z.name);
  CHECK(back.use == z.use);
  CHECK(back.spaces == z.spaces);
  CHECK(back.metadata == z.metadata);
  CHECK_THROWS_AS(core::zone_from_json(json{{"bogus", 1}}),
                  std::invalid_argument);
  CHECK_THROWS_AS(core::zone_from_json(json{{"spaces", "s1"}}),
                  std::invalid_argument);
  CHECK(core::parse_model_element_kind("zone") == ModelElementKind::zone);
  CHECK(core::to_string(ModelElementKind::zone) == "zone");
}

TEST_CASE("Zones_CreateUpdateDelete_ThroughWireOps_WithUndo",
          "[core][zones][model_edits]") {
  TempPath project("test_zones_crud");
  ProjectDB db(project.path);
  const auto m = seed(db);
  CHECK(db.zones().empty());

  const auto r0 = db.model_revision();
  const auto created =
      edit(db, json::array({{{"op", "zone.create"},
                             {"record",
                              {{"guid", "zone-a"},
                               {"name", "Bolig A"},
                               {"use", "bolig"},
                               {"spaces", {m.a.guid, m.c.guid}}}}}}));
  CHECK(created.revision == r0 + 1);
  REQUIRE(created.changes.size() == 1);
  CHECK(created.changes[0].kind == ModelElementKind::zone);
  CHECK(created.changes[0].action == "created");
  auto z = db.zone_by_guid("zone-a");
  REQUIRE(z);
  CHECK(z->name == "Bolig A");
  CHECK(z->use == "bolig");
  CHECK(z->spaces == std::vector<std::string>{m.a.guid, m.c.guid});
  CHECK(z->source == "manual");
  CHECK(z->locked);
  CHECK(json::parse(z->metadata).at("edited") == true);

  // Area / volume / storeys come from the members.
  const auto q = core::zone_measures(*z, db.spaces());
  CHECK_THAT(q.area, WithinAbs(24.0, 1e-9));
  CHECK_THAT(q.volume, WithinAbs(60.0, 1e-9));
  CHECK(q.storeys == std::vector<int>{0, 1});
  CHECK(q.missing.empty());

  // A partial update replaces the member list.
  edit(db, json::array({{{"op", "zone.update"},
                         {"guid", "zone-a"},
                         {"set", {{"spaces", {m.b.guid}}, {"name", "B"}}}}}));
  z = db.zone_by_guid("zone-a");
  REQUIRE(z);
  CHECK(z->name == "B");
  CHECK(z->use == "bolig");
  CHECK(z->spaces == std::vector<std::string>{m.b.guid});

  core::undo_model_edit(db);
  CHECK(db.zone_by_guid("zone-a")->spaces ==
        std::vector<std::string>{m.a.guid, m.c.guid});
  core::redo_model_edit(db);
  CHECK(db.zone_by_guid("zone-a")->name == "B");

  // A drawn zone leaves no tombstone.
  edit(db, json::array({{{"op", "zone.delete"}, {"guid", "zone-a"}}}));
  CHECK_FALSE(db.zone_by_guid("zone-a"));
  CHECK(db.dismissed_model_elements().empty());
  core::undo_model_edit(db);
  REQUIRE(db.zone_by_guid("zone-a"));
  CHECK(db.zone_by_guid("zone-a")->spaces ==
        std::vector<std::string>{m.b.guid});
}

TEST_CASE("Zones_MemberMustBeAStoredSpace", "[core][zones][model_edits]") {
  TempPath project("test_zones_unknown_member");
  ProjectDB db(project.path);
  const auto m = seed(db);
  CHECK_THROWS_AS(
      edit(db,
           json::array(
               {{{"op", "zone.create"},
                 {"record", {{"name", "Z"}, {"spaces", {"no-such-space"}}}}}})),
      std::invalid_argument);
  CHECK(db.zones().empty());
  // A space created earlier in the same batch counts.
  edit(db,
       json::array({{{"op", "space.create"},
                     {"record",
                      {{"guid", "new-room"},
                       {"polygon", {{"outer", {{0, 5}, {2, 5}, {2, 7}}}}}}}},
                    {{"op", "zone.create"},
                     {"record",
                      {{"guid", "z"}, {"spaces", {m.a.guid, "new-room"}}}}}}));
  CHECK(db.zone_by_guid("z")->spaces.size() == 2);
  // The same space twice is refused.
  CHECK_THROWS_AS(
      edit(db, json::array({{{"op", "zone.update"},
                             {"guid", "z"},
                             {"set", {{"spaces", {m.a.guid, m.a.guid}}}}}})),
      std::invalid_argument);
}

TEST_CASE("Zones_DeletingASpace_DropsItFromItsZones_UndoBringsItBack",
          "[core][zones][model_edits]") {
  TempPath project("test_zones_space_delete");
  ProjectDB db(project.path);
  const auto m = seed(db);
  edit(db,
       json::array(
           {{{"op", "zone.create"},
             {"record", {{"guid", "z"}, {"spaces", {m.a.guid, m.b.guid}}}}}}));

  const auto r =
      edit(db, json::array({{{"op", "space.delete"}, {"guid", m.a.guid}}}));
  CHECK(db.zone_by_guid("z")->spaces == std::vector<std::string>{m.b.guid});
  bool zone_changed = false;
  for (const auto &c : r.changes)
    zone_changed |= c.kind == ModelElementKind::zone && c.guid == "z" &&
                    c.action == "updated";
  CHECK(zone_changed);

  core::undo_model_edit(db);
  REQUIRE(db.space_by_guid(m.a.guid));
  CHECK(db.zone_by_guid("z")->spaces ==
        std::vector<std::string>{m.a.guid, m.b.guid});
}

TEST_CASE("Zones_SpacesRerunKeepsMembers_DeleteSpacesPrunesThem",
          "[core][zones]") {
  TempPath project("test_zones_rerun");
  ProjectDB db(project.path);
  const auto m = seed(db);
  edit(db,
       json::array(
           {{{"op", "zone.create"},
             {"record", {{"guid", "z"}, {"spaces", {m.a.guid, m.b.guid}}}}}}));
  // A detection re-run that gives back a (same guid) but not b.
  db.save_spaces({m.a}, {});
  CHECK(db.zone_by_guid("z")->spaces == std::vector<std::string>{m.a.guid});
  db.delete_spaces();
  REQUIRE(db.zone_by_guid("z"));
  CHECK(db.zone_by_guid("z")->spaces.empty());
  const auto q = core::zone_measures(*db.zone_by_guid("z"), db.spaces());
  CHECK(q.area == 0.0);
}

TEST_CASE("Zones_Measures_ReportMissingAndMeasureUnmeasuredSpaces",
          "[core][zones]") {
  core::SpaceRecord s;
  s.guid = "s";
  s.polygon.outer = {{0, 0}, {2, 0}, {2, 2}, {0, 2}};
  s.z_ceiling = 3.0; // area / volume not computed (-1)
  core::ZoneRecord z;
  z.spaces = {"s", "gone"};
  const auto q = core::zone_measures(z, {s});
  CHECK_THAT(q.area, WithinAbs(4.0, 1e-9));
  CHECK_THAT(q.volume, WithinAbs(12.0, 1e-9));
  CHECK(q.missing == std::vector<std::string>{"gone"});
}

TEST_CASE("MigrationV43_V42Project_GetsZoneTables",
          "[projectdb][migration][zones]") {
  TempPath tmp("test_migration_v43");
  std::string a;
  {
    ProjectDB db(tmp.path);
    a = seed(db).a.guid;
  }
  exec_raw(tmp.path, R"(
    DROP TABLE zone_members;
    DROP TABLE zones;
    DELETE FROM schema_version WHERE version >= 43;
  )");
  {
    ProjectDB ro(tmp.path, /*readOnly=*/true);
    CHECK(ro.schema_version() == 42);
    CHECK(ro.zones().empty());
    CHECK_FALSE(ro.zone_by_guid("z"));
  }
  ProjectDB db(tmp.path);
  CHECK(db.schema_version() == ProjectDB::latest_schema_version());
  edit(db, json::array({{{"op", "zone.create"},
                         {"record", {{"guid", "z"}, {"spaces", {a}}}}}}));
  CHECK(db.zones().size() == 1);
  // The tombstone table takes the editor's newer kinds.
  db.dismiss_model_element("slab", "some-slab", "{}");
  db.dismiss_model_element("stair", "some-stair", "{}");
  CHECK(db.dismissed_model_elements("slab").size() == 1);
}
