// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Solid-model row edits (schema v41, reusex-yuja): per-row put / remove /
// revert that mark rows manual + locked, tombstones the detection writers
// honour, the edit log with undo / redo and the model revision.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/guid.hpp>
#include <reusex/core/model_edits.hpp>
#include <reusex/core/solid_model.hpp>
#include <reusex/core/wall_identity.hpp>

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

core::WallRecord wall(const ProjectDB &db, const std::string &key, double x0,
                      double y0, double x1, double y1) {
  core::WallRecord w;
  w.guid = core::deterministic_guid(db.project_guid(), key);
  w.x0 = x0;
  w.y0 = y0;
  w.x1 = x1;
  w.y1 = y1;
  w.height = 2.5;
  w.thickness = 0.2;
  w.metadata = R"({"seen":0.8})";
  return w;
}

core::OpeningRecord opening(const ProjectDB &db, const std::string &key,
                            const std::string &host) {
  core::OpeningRecord o;
  o.guid = core::deterministic_guid(db.project_guid(), key);
  o.wall = host;
  o.kind = "door";
  o.s_start = 1.0;
  o.s_end = 1.9;
  o.z_sill = 0.0;
  o.z_head = 2.1;
  return o;
}

core::SpaceRecord space(const ProjectDB &db, const std::string &key) {
  core::SpaceRecord s;
  s.guid = core::deterministic_guid(db.project_guid(), key);
  s.name = "Stue";
  s.kind = "room";
  s.polygon.outer = {{0, 0}, {4, 0}, {4, 3}, {0, 3}};
  s.z_ceiling = 2.5;
  return s;
}

core::StoreyRecord storey(const ProjectDB &db) {
  core::StoreyRecord s;
  s.guid = core::deterministic_guid(db.project_guid(), "storeys/0");
  s.name = "Stue";
  s.z_ceiling = 2.5;
  return s;
}

/// A detection run's model: one storey, a corner of two walls (south,
/// west) joined, a phantom wall, a door in the south wall, one space.
struct Seeded {
  core::WallRecord south, west, phantom;
  core::OpeningRecord door;
  core::SpaceRecord room;
};

Seeded seed(ProjectDB &db) {
  Seeded m;
  m.south = wall(db, "walls/south", 0, 0, 4, 0);
  m.west = wall(db, "walls/west", 0, 0, 0, 3);
  m.phantom = wall(db, "walls/phantom", 1, 1.5, 3, 1.5);
  m.door = opening(db, "openings/door", m.south.guid);
  m.room = space(db, "spaces/room");
  db.save_storeys({storey(db)});
  db.save_walls({m.south, m.west, m.phantom},
                {{m.south.guid, m.west.guid, "corner", "start", "start"}});
  db.save_openings({m.door});
  db.save_spaces({m.room}, {{m.room.guid, m.south.guid, "a", 0.0, 4.0}});
  return m;
}

core::ModelEditOptions options(std::string label, std::string author = "") {
  core::ModelEditOptions o;
  o.label = std::move(label);
  o.author = std::move(author);
  return o;
}

core::ModelEditOptions against(int64_t revision) {
  core::ModelEditOptions o;
  o.base_revision = revision;
  return o;
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

json meta(const std::string &text) {
  return text.empty() ? json::object() : json::parse(text);
}

} // namespace

TEST_CASE("ModelEdits_Schema_TablesAndRevision", "[core][model_edits]") {
  TempPath project("test_model_edits_schema");
  ProjectDB db(project.path);
  CHECK(db.schema_version() >= 41);
  CHECK(db.model_revision() == 0);
  CHECK(db.dismissed_model_elements().empty());
  CHECK(db.model_edits().empty());
  CHECK_FALSE(core::can_undo_model_edit(db));
  CHECK_FALSE(core::can_redo_model_edit(db));

  // Every whole-run write raises the revision.
  seed(db);
  const auto r0 = db.model_revision();
  CHECK(r0 >= 4);
  db.save_walls(db.walls(), db.wall_joins());
  CHECK(db.model_revision() == r0 + 1);
  db.delete_openings();
  CHECK(db.model_revision() == r0 + 2);
}

TEST_CASE("MigrationV41_V40Project_GetsTombstonesAndTheEditLog",
          "[projectdb][migration][model_edits]") {
  TempPath tmp("test_migration_v41");
  std::string south;
  {
    ProjectDB db(tmp.path);
    south = seed(db).south.guid;
  }
  exec_raw(tmp.path, R"(
    DROP TABLE model_dismissed;
    DROP TABLE model_edits;
    DELETE FROM schema_version WHERE version >= 41;
  )");
  {
    ProjectDB ro(tmp.path, /*readOnly=*/true);
    CHECK(ro.schema_version() == 40);
    CHECK(ro.dismissed_model_elements().empty());
    CHECK(ro.model_edits().empty());
    CHECK_FALSE(core::can_undo_model_edit(ro));
    CHECK(core::model_holds(ro, ModelElementKind::wall).dismissed.empty());
  }
  ProjectDB db(tmp.path);
  CHECK(db.schema_version() == ProjectDB::latest_schema_version());
  CHECK(db.walls().size() == 3);
  core::apply_model_edit(db,
                         {ModelEditOp::remove(ModelElementKind::wall, south)});
  CHECK(db.dismissed_model_element(south));
  CHECK(db.model_edits().size() == 1);
}

TEST_CASE("ModelEdits_PutDetectedWall_IsManualLockedAndRemembersTheOriginal",
          "[core][model_edits]") {
  TempPath project("test_model_edits_put");
  ProjectDB db(project.path);
  const auto m = seed(db);
  const auto id = db.wall_by_guid(m.south.guid)->id;

  auto moved = *db.wall_by_guid(m.south.guid);
  moved.y0 = moved.y1 = -0.3;
  moved.thickness = 0.3;
  const auto rev = db.model_revision();
  const auto r = core::apply_model_edit(db, {ModelEditOp::put(moved)},
                                        options("Flyt væg", "pfs"));
  CHECK(r.revision == rev + 1);
  CHECK(db.model_revision() == rev + 1);
  REQUIRE(r.changes.size() == 1);
  CHECK(r.changes[0].guid == m.south.guid);
  CHECK(r.changes[0].action == "updated");

  const auto row = db.wall_by_guid(m.south.guid);
  REQUIRE(row);
  CHECK(row->id == id); // updated in place: joins kept
  CHECK(row->source == "manual");
  CHECK(row->locked);
  CHECK(row->y0 == -0.3);
  CHECK(row->thickness == 0.3);
  const auto md = meta(row->metadata);
  CHECK(md.at("edited") == true);
  CHECK(md.at("seen") == 0.8);
  const auto &from = md.at("edited_from");
  CHECK(from.at("source") == "detected");
  CHECK(from.at("locked") == false);
  CHECK(from.at("y0") == 0.0);
  CHECK(db.wall_joins().size() == 1);

  // A second edit keeps the first original, not the edited state.
  auto again = *row;
  again.height = 2.7;
  core::apply_model_edit(db, {ModelEditOp::put(again)});
  CHECK(meta(db.wall_by_guid(m.south.guid)->metadata)
            .at("edited_from")
            .at("y0") == 0.0);

  // An imported row edited keeps its provenance the same way.
  auto imp = wall(db, "walls/imported", 5, 0, 5, 3);
  imp.source = "imported";
  imp.locked = true;
  core::apply_model_edit(db, {ModelEditOp::put(imp)}); // created: manual
  CHECK(db.wall_by_guid(imp.guid)->source == "manual");
}

TEST_CASE("ModelEdits_CreateWithoutGuid_MintsOne", "[core][model_edits]") {
  TempPath project("test_model_edits_create");
  ProjectDB db(project.path);
  seed(db);
  core::WallRecord drawn;
  drawn.x1 = 2.0;
  drawn.height = 2.5;
  const auto r = core::apply_model_edit(db, {ModelEditOp::put(drawn)});
  REQUIRE(r.changes.size() == 1);
  CHECK(r.changes[0].action == "created");
  const auto row = db.wall_by_guid(r.changes[0].guid);
  REQUIRE(row);
  CHECK(row->guid.size() == 36);
  CHECK(row->source == "manual");
  CHECK(row->locked);
  CHECK_FALSE(meta(row->metadata).contains("edited_from"));
}

TEST_CASE("ModelEdits_RemoveDetectedWall_TombstonesIt_AndDetectionSkipsIt",
          "[core][model_edits]") {
  TempPath project("test_model_edits_remove");
  ProjectDB db(project.path);
  const auto m = seed(db);

  const auto r = core::apply_model_edit(
      db, {ModelEditOp::remove(ModelElementKind::wall, m.south.guid)});
  CHECK_FALSE(db.wall_by_guid(m.south.guid));
  CHECK(db.wall_joins().empty()); // the join went with it
  const auto tomb = db.dismissed_model_element(m.south.guid);
  REQUIRE(tomb);
  CHECK(tomb->kind == "wall");
  CHECK(json::parse(tomb->record).at("x1") == 4.0);
  // The door on it is left for the caller, and said so.
  REQUIRE(r.warnings.size() == 1);
  CHECK(r.warnings[0].find(m.door.guid) != std::string::npos);

  // A detection run that finds it again does not write it, nor a join to it.
  db.save_walls({m.south, m.west, m.phantom},
                {{m.south.guid, m.west.guid, "corner", "start", "start"}});
  CHECK_FALSE(db.wall_by_guid(m.south.guid));
  CHECK(db.walls().size() == 2);
  CHECK(db.wall_joins().empty());

  // An import brings it back and drops the tombstone.
  core::SolidModel imported;
  auto w = m.south;
  w.source = "imported";
  w.locked = true;
  imported.walls = {w};
  db.replace_imported_solid_model(imported);
  CHECK(db.wall_by_guid(m.south.guid));
  CHECK_FALSE(db.dismissed_model_element(m.south.guid));
}

TEST_CASE("ModelEdits_RemoveHandDrawnWall_LeavesNoTombstone",
          "[core][model_edits]") {
  TempPath project("test_model_edits_remove_manual");
  ProjectDB db(project.path);
  seed(db);
  core::WallRecord drawn;
  drawn.x1 = 2.0;
  const auto g =
      core::apply_model_edit(db, {ModelEditOp::put(drawn)}).changes[0].guid;
  core::apply_model_edit(db, {ModelEditOp::remove(ModelElementKind::wall, g)});
  CHECK_FALSE(db.wall_by_guid(g));
  CHECK_FALSE(db.dismissed_model_element(g));
}

TEST_CASE("ModelEdits_Revert", "[core][model_edits]") {
  TempPath project("test_model_edits_revert");
  ProjectDB db(project.path);
  const auto m = seed(db);
  const auto original = *db.wall_by_guid(m.south.guid);

  SECTION("an edited detected row gets its detected record back, unlocked") {
    auto moved = original;
    moved.x1 = 5.0;
    core::apply_model_edit(db, {ModelEditOp::put(moved)});
    const auto r = core::apply_model_edit(
        db, {ModelEditOp::revert(ModelElementKind::wall, m.south.guid)});
    REQUIRE(r.changes.size() == 1);
    CHECK(r.changes[0].action == "reverted");
    const auto row = db.wall_by_guid(m.south.guid);
    CHECK(row->x1 == 4.0);
    CHECK(row->source == "detected");
    CHECK_FALSE(row->locked);
    CHECK(row->metadata == original.metadata);
    CHECK(row->id == original.id);
  }
  SECTION("a hand-drawn row is unlocked") {
    core::WallRecord drawn;
    drawn.x1 = 2.0;
    const auto g =
        core::apply_model_edit(db, {ModelEditOp::put(drawn)}).changes[0].guid;
    core::apply_model_edit(db,
                           {ModelEditOp::revert(ModelElementKind::wall, g)});
    const auto row = db.wall_by_guid(g);
    CHECK_FALSE(row->locked);
    CHECK(row->source == "manual");
    CHECK(row->metadata.empty());
    // ...so the next detection run replaces it.
    db.save_walls({m.south, m.west}, {});
    CHECK_FALSE(db.wall_by_guid(g));
  }
  SECTION("a deleted row comes back") {
    core::apply_model_edit(
        db, {ModelEditOp::remove(ModelElementKind::wall, m.south.guid)});
    const auto r = core::apply_model_edit(
        db, {ModelEditOp::revert(ModelElementKind::wall, m.south.guid)});
    CHECK(r.changes[0].action == "restored");
    CHECK(db.wall_by_guid(m.south.guid)->x1 == 4.0);
    CHECK_FALSE(db.dismissed_model_element(m.south.guid));
  }
  SECTION("an unknown guid is refused") {
    CHECK_THROWS_AS(
        core::apply_model_edit(
            db, {ModelEditOp::revert(ModelElementKind::wall, "no-such-wall")}),
        std::invalid_argument);
  }
}

TEST_CASE("ModelEdits_UndoRedo_RestoresRowsJoinsAndTombstones",
          "[core][model_edits]") {
  TempPath project("test_model_edits_undo");
  ProjectDB db(project.path);
  const auto m = seed(db);
  const auto south0 = *db.wall_by_guid(m.south.guid);

  // One batch: move the west wall, delete the south wall, re-host the door
  // onto the west wall.
  auto west = *db.wall_by_guid(m.west.guid);
  west.x0 = west.x1 = -0.1;
  auto door = *db.opening_by_guid(m.door.guid);
  door.wall = m.west.guid;
  const auto applied = core::apply_model_edit(
      db,
      {ModelEditOp::put(west),
       ModelEditOp::remove(ModelElementKind::wall, m.south.guid),
       ModelEditOp::put(door)},
      options("Ret hjørne"));
  CHECK(applied.changes.size() == 3);
  CHECK(applied.warnings.empty()); // the door was re-hosted in the batch
  CHECK(core::can_undo_model_edit(db));

  const auto undone = core::undo_model_edit(db);
  CHECK(undone.edit_id == applied.edit_id);
  CHECK(undone.revision == applied.revision + 1);
  const auto south = db.wall_by_guid(m.south.guid);
  REQUIRE(south);
  CHECK(south->source == "detected");
  CHECK_FALSE(south->locked);
  CHECK(south->x1 == south0.x1);
  CHECK(south->metadata == south0.metadata);
  CHECK_FALSE(db.dismissed_model_element(m.south.guid));
  CHECK(db.wall_joins().size() == 1); // the corner join is back
  CHECK(db.wall_by_guid(m.west.guid)->x0 == 0.0);
  CHECK_FALSE(db.wall_by_guid(m.west.guid)->locked);
  CHECK(db.opening_by_guid(m.door.guid)->wall == m.south.guid);
  CHECK(core::can_redo_model_edit(db));
  CHECK_FALSE(core::can_undo_model_edit(db));

  const auto redone = core::redo_model_edit(db);
  CHECK(redone.edit_id == applied.edit_id);
  CHECK_FALSE(db.wall_by_guid(m.south.guid));
  CHECK(db.dismissed_model_element(m.south.guid));
  CHECK(db.wall_by_guid(m.west.guid)->x0 == -0.1);
  CHECK(db.opening_by_guid(m.door.guid)->wall == m.west.guid);

  // A new edit after an undo drops the redo branch.
  core::undo_model_edit(db);
  auto phantom = *db.wall_by_guid(m.phantom.guid);
  phantom.height = 1.0;
  core::apply_model_edit(db, {ModelEditOp::put(phantom)});
  CHECK_FALSE(core::can_redo_model_edit(db));
  const auto history = core::model_edit_history(db);
  REQUIRE(history.size() == 1);
  CHECK(history[0].changes.size() == 1);
  CHECK(history[0].changes[0].guid == m.phantom.guid);
  CHECK_THROWS_AS(core::redo_model_edit(db), std::runtime_error);
}

TEST_CASE("ModelEdits_Conflicts", "[core][model_edits]") {
  TempPath project("test_model_edits_conflict");
  ProjectDB db(project.path);
  const auto m = seed(db);
  const auto rev = db.model_revision();

  auto moved = *db.wall_by_guid(m.south.guid);
  moved.x1 = 5.0;
  CHECK_THROWS_AS(
      core::apply_model_edit(db, {ModelEditOp::put(moved)}, against(rev - 1)),
      core::ModelEditConflict);
  CHECK(db.wall_by_guid(m.south.guid)->x1 == 4.0); // nothing written
  try {
    core::apply_model_edit(db, {ModelEditOp::put(moved)}, against(rev - 1));
  } catch (const core::ModelEditConflict &e) {
    CHECK(e.current_revision() == rev);
  }
  core::apply_model_edit(db, {ModelEditOp::put(moved)}, against(rev));

  // Revert unlocks the hand-drawn row; a detection run then replaces it,
  // so the revert can no longer be undone.
  core::WallRecord drawn;
  drawn.x1 = 2.0;
  const auto g =
      core::apply_model_edit(db, {ModelEditOp::put(drawn)}).changes[0].guid;
  core::apply_model_edit(db, {ModelEditOp::revert(ModelElementKind::wall, g)});
  db.save_walls({m.west}, {});
  CHECK_THROWS_AS(core::undo_model_edit(db), core::ModelEditConflict);

  // A failed op rolls the whole batch back.
  auto west = *db.wall_by_guid(m.west.guid);
  west.height = 9.0;
  const auto before = db.model_edits().size();
  CHECK_THROWS_AS(
      core::apply_model_edit(
          db, {ModelEditOp::put(west),
               ModelEditOp::remove(ModelElementKind::space, "no-such")}),
      std::invalid_argument);
  CHECK(db.wall_by_guid(m.west.guid)->height == 2.5);
  CHECK_FALSE(db.wall_by_guid(m.west.guid)->locked);
  CHECK(db.model_edits().size() == before);
}

TEST_CASE("ModelEdits_SpacesAndStoreys", "[core][model_edits]") {
  TempPath project("test_model_edits_spaces");
  ProjectDB db(project.path);
  const auto m = seed(db);

  // ceiling.set + a partial space update through the wire ops.
  const auto ops = core::model_edit_ops_from_json(db, json::parse(R"([
    {"op": "space.update", "guid": ")" + m.room.guid + R"(",
     "set": {"name": "Køkken", "polygon": {"outer": [[0,0],[4,0],[4,3.5],[0,3.5]]}}},
    {"op": "ceiling.set", "guid": ")" + m.room.guid + R"(",
     "planes": [{"at": [0, 0, 2.4], "slope": [0, 0.2]}]},
    {"op": "storey.update", "guid": ")" + storey(db).guid + R"(",
     "set": {"z_ceiling": 2.6}}
  ])"));
  REQUIRE(ops.size() == 3);
  core::apply_model_edit(db, ops);
  const auto room = db.space_by_guid(m.room.guid);
  REQUIRE(room);
  CHECK(room->name == "Køkken");
  CHECK(room->polygon.outer[2][1] == 3.5);
  REQUIRE(room->ceiling_planes.size() == 1);
  CHECK(room->ceiling_planes[0].slope[1] == 0.2);
  CHECK(room->locked);
  CHECK(room->source == "manual");
  CHECK(db.space_boundaries().size() == 1); // kept
  const auto st = db.storey_by_guid(storey(db).guid);
  CHECK(st->z_ceiling == 2.6);
  CHECK(st->locked);

  // A detection run keeps both (locked); the edited space keeps its outline.
  auto redetected = m.room;
  redetected.polygon.outer = {{0, 0}, {4, 0}, {4, 3}, {0, 3}};
  const auto holds = core::model_holds(db, ModelElementKind::space);
  CHECK(holds.edited.count(m.room.guid) == 1);

  // Delete the space: tombstoned; save_spaces skips it.
  core::apply_model_edit(
      db, {ModelEditOp::remove(ModelElementKind::space, m.room.guid)});
  CHECK(db.dismissed_model_element(m.room.guid));
  db.save_spaces({redetected}, {{m.room.guid, m.south.guid, "a", 0.0, 4.0}});
  CHECK(db.spaces().empty());
}

TEST_CASE("ModelEdits_WireOps", "[core][model_edits]") {
  TempPath project("test_model_edits_wire");
  ProjectDB db(project.path);
  const auto m = seed(db);

  SECTION("create then update in one list patches the created record") {
    const auto ops = core::model_edit_ops_from_json(db, json::parse(R"([
      {"op": "wall.create", "record": {"guid": "w-new", "x1": 3, "height": 2.4}},
      {"op": "wall.update", "guid": "w-new", "set": {"thickness": 0.15}}
    ])"));
    REQUIRE(ops.size() == 2);
    const auto r = core::apply_model_edit(db, ops);
    CHECK(r.changes.size() == 1);
    const auto w = db.wall_by_guid("w-new");
    REQUIRE(w);
    CHECK(w->thickness == 0.15);
    CHECK(w->height == 2.4);
  }
  SECTION("partial update of a stored wall keeps the other fields") {
    const auto ops = core::model_edit_ops_from_json(
        db, json::parse(R"([{"op": "wall.update", "guid": ")" + m.west.guid +
                        R"(", "set": {"is_external": true}}])"));
    core::apply_model_edit(db, ops);
    const auto w = db.wall_by_guid(m.west.guid);
    CHECK(w->is_external);
    CHECK(w->y1 == 3.0);
    CHECK(w->thickness == 0.2);
  }
  SECTION("bad ops are refused") {
    CHECK_THROWS_AS(
        core::model_edit_ops_from_json(
            db, json::parse(R"([{"op": "wall.fly", "guid": "x"}])")),
        std::invalid_argument);
    CHECK_THROWS_AS(
        core::model_edit_ops_from_json(
            db, json::parse(R"([{"op": "wall.update", "guid": "nope"}])")),
        std::invalid_argument);
    CHECK_THROWS_AS(
        core::model_edit_ops_from_json(
            db, json::parse(R"([{"op": "wall.update", "guid": ")" +
                            m.west.guid + R"(", "set": {"colour": 3}}])")),
        std::invalid_argument);
    CHECK_THROWS_AS(core::model_edit_ops_from_json(
                        db, json::parse(R"([{"op": "wall.delete"}])")),
                    std::invalid_argument);
  }
}

TEST_CASE("ModelEdits_RecordJson_RoundTrips", "[core][model_edits]") {
  TempPath project("test_model_edits_json");
  ProjectDB db(project.path);
  auto w = wall(db, "walls/x", 0.1, 0.2, 3.3, 0.2);
  w.top = {{0.0, 2.0}, {3.2, 2.8}};
  w.lean_deg = 0.7;
  const auto back = core::wall_from_json(core::model_record_json(w));
  CHECK(core::model_record_json(back) == core::model_record_json(w));
  CHECK(back.top == w.top);

  auto s = space(db, "spaces/x");
  s.polygon.holes = {{{1, 1}, {2, 1}, {2, 2}}};
  s.ceiling_planes = {{{0, 0, 2.4}, {0.1, 0.0}}};
  s.room_label = 3;
  CHECK(core::model_record_json(core::space_from_json(
            core::model_record_json(s))) == core::model_record_json(s));
  const auto o = opening(db, "openings/x", w.guid);
  CHECK(core::model_record_json(core::opening_from_json(
            core::model_record_json(o))) == core::model_record_json(o));
  const auto st = storey(db);
  CHECK(core::model_record_json(core::storey_from_json(
            core::model_record_json(st))) == core::model_record_json(st));
}

TEST_CASE("ModelEdits_IdentityCandidates_KeepEditsAcrossARedetection",
          "[core][model_edits]") {
  TempPath project("test_model_edits_identity");
  ProjectDB db(project.path);
  const auto m = seed(db);

  // Move the south wall 0.6 m (beyond the 0.25 m identity match) and delete
  // the phantom.
  auto moved = *db.wall_by_guid(m.south.guid);
  moved.y0 = moved.y1 = -0.6;
  core::apply_model_edit(
      db, {ModelEditOp::put(moved),
           ModelEditOp::remove(ModelElementKind::wall, m.phantom.guid)});

  // Detection finds the same three walls where they were, with fresh keys.
  std::vector<core::WallRecord> fresh = {m.south, m.west, m.phantom};
  fresh[0].guid = "walls/0/0.00,0.00,4.00,0.00";
  fresh[1].guid = "walls/0/0.00,0.00,0.00,3.00";
  fresh[2].guid = "walls/0/1.00,1.50,3.00,1.50";
  fresh[2].x0 += 0.04; // a little noise
  std::vector<core::WallJoinRecord> joins = {
      {fresh[0].guid, fresh[1].guid, "corner", "start", "start"},
      {fresh[2].guid, fresh[1].guid, "tee", "start", "path"}};
  core::assign_wall_guids(fresh, joins, core::wall_identity_candidates(db),
                          db.project_guid());
  CHECK(fresh[0].guid == m.south.guid);   // matched the edit's original
  CHECK(fresh[2].guid == m.phantom.guid); // matched the tombstone
  const auto held = core::drop_held_walls(
      fresh, joins, core::model_holds(db, ModelElementKind::wall));
  CHECK(held == 2);
  REQUIRE(fresh.size() == 1);
  CHECK(fresh[0].guid == m.west.guid);
  REQUIRE(joins.size() == 1); // the tee to the phantom is gone
  db.save_walls(fresh, joins);

  const auto rows = db.walls();
  CHECK(rows.size() == 2); // west + the edited south; no phantom, no copy
  const auto south = db.wall_by_guid(m.south.guid);
  REQUIRE(south);
  CHECK(south->y0 == -0.6);
  CHECK(south->locked);
  CHECK_FALSE(db.wall_by_guid(m.phantom.guid));
}
