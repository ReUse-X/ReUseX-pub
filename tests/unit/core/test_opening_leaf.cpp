// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Door / window leaf fields on OpeningRecord (schema v42, reusex-w6jc):
// swing_side, hinge, operation, type_name, frame_depth -- stored, edited
// through the model-edit wire ops, kept across a detection re-run -- and the
// fit check that warns when an edit leaves an opening outside its host wall
// or overlapping another one (core/opening_fit.hpp).

#include <catch2/catch_test_macros.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/guid.hpp>
#include <reusex/core/model_edits.hpp>
#include <reusex/core/opening_fit.hpp>
#include <reusex/core/solid_model.hpp>

#include "../../support/temp_path.hpp"

#include <nlohmann/json.hpp>
#include <sqlite3.h>

#include <algorithm>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

using namespace reusex;
using core::ModelEditOp;
using core::ModelElementKind;
using reusex::test_support::TempPath;
using json = nlohmann::json;

namespace {

core::WallRecord wall(const ProjectDB &db, const std::string &key, double x1) {
  core::WallRecord w;
  w.guid = core::deterministic_guid(db.project_guid(), key);
  w.x1 = x1;
  w.height = 2.5;
  w.thickness = 0.2;
  return w;
}

core::OpeningRecord door(const ProjectDB &db, const std::string &key,
                         const std::string &host, double s0, double s1) {
  core::OpeningRecord o;
  o.guid = core::deterministic_guid(db.project_guid(), key);
  o.wall = host;
  o.kind = "door";
  o.s_start = s0;
  o.s_end = s1;
  o.z_sill = 0.0;
  o.z_head = 2.1;
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

bool any_contains(const std::vector<std::string> &v, const std::string &s) {
  return std::any_of(v.begin(), v.end(), [&](const std::string &w) {
    return w.find(s) != std::string::npos;
  });
}

std::vector<core::ModelEditOp> wire(const ProjectDB &db,
                                    const std::string &text) {
  return core::model_edit_ops_from_json(db, json::parse(text));
}

} // namespace

TEST_CASE("OpeningLeaf_Defaults_AreUnknown", "[core][openings][leaf]") {
  const core::OpeningRecord o;
  CHECK(o.swing_side == "unknown");
  CHECK(o.hinge == "unknown");
  CHECK(o.operation == "unknown");
  CHECK(o.type_name.empty());
  CHECK(o.frame_depth == -1.0);
}

TEST_CASE("OpeningLeaf_SaveAndPut_RoundTripTheFields",
          "[projectdb][openings][leaf]") {
  TempPath tmp("test_opening_leaf_roundtrip");
  ProjectDB db(tmp.path);
  CHECK(db.schema_version() >= 42);
  const auto w = wall(db, "walls/a", 5.0);
  db.save_walls({w}, {});

  auto a = door(db, "openings/a", w.guid, 1.0, 1.9);
  a.swing_side = "b";
  a.hinge = "end";
  a.operation = "single_swing";
  a.type_name = "Enkeltdør 90";
  a.frame_depth = 0.12;
  auto plain = door(db, "openings/b", w.guid, 3.0, 4.0);
  db.save_openings({a, plain});

  const auto back = db.opening_by_guid(a.guid);
  REQUIRE(back);
  CHECK(back->swing_side == "b");
  CHECK(back->hinge == "end");
  CHECK(back->operation == "single_swing");
  CHECK(back->type_name == "Enkeltdør 90");
  CHECK(back->frame_depth == 0.12);
  const auto p = db.opening_by_guid(plain.guid);
  REQUIRE(p);
  CHECK(p->swing_side == "unknown");
  CHECK(p->hinge == "unknown");
  CHECK(p->operation == "unknown");
  CHECK(p->type_name.empty());
  CHECK(p->frame_depth == -1.0);

  auto win = door(db, "openings/c", w.guid, 2.0, 2.8);
  win.kind = "window";
  win.z_sill = 0.9;
  win.operation = "tilt_turn";
  win.hinge = "start";
  win.frame_depth = 0.08;
  db.put_opening_row(win);
  const auto wb = db.opening_by_guid(win.guid);
  REQUIRE(wb);
  CHECK(wb->operation == "tilt_turn");
  CHECK(wb->hinge == "start");
  CHECK(wb->swing_side == "unknown");
  CHECK(wb->frame_depth == 0.08);
}

TEST_CASE("OpeningLeaf_BadValues_AreRefused", "[projectdb][openings][leaf]") {
  TempPath tmp("test_opening_leaf_bad");
  ProjectDB db(tmp.path);
  const auto w = wall(db, "walls/a", 5.0);
  db.save_walls({w}, {});
  auto o = door(db, "openings/a", w.guid, 1.0, 1.9);

  SECTION("swing_side") { o.swing_side = "left"; }
  SECTION("hinge") { o.hinge = "middle"; }
  SECTION("operation") { o.operation = "revolving_door_of_doom"; }
  SECTION("frame_depth zero") { o.frame_depth = 0.0; }
  SECTION("frame_depth negative") { o.frame_depth = -0.5; }
  CHECK_THROWS_AS(db.save_openings({o}), std::invalid_argument);
  CHECK_THROWS_AS(db.put_opening_row(o), std::invalid_argument);
  CHECK(db.openings().empty());
}

TEST_CASE("MigrationV42_V41Project_OpeningsGetUnknownLeafFields",
          "[projectdb][migration][openings][leaf]") {
  TempPath tmp("test_migration_v42");
  std::string guid;
  {
    ProjectDB db(tmp.path);
    const auto w = wall(db, "walls/a", 5.0);
    db.save_walls({w}, {});
    const auto o = door(db, "openings/a", w.guid, 1.0, 1.9);
    guid = o.guid;
    db.save_openings({o});
  }
  exec_raw(tmp.path, R"(
    ALTER TABLE openings DROP COLUMN swing_side;
    ALTER TABLE openings DROP COLUMN hinge;
    ALTER TABLE openings DROP COLUMN operation;
    ALTER TABLE openings DROP COLUMN type_name;
    ALTER TABLE openings DROP COLUMN frame_depth;
    DELETE FROM schema_version WHERE version >= 42;
  )");
  {
    ProjectDB ro(tmp.path, /*readOnly=*/true);
    CHECK(ro.schema_version() == 41);
    const auto o = ro.opening_by_guid(guid);
    REQUIRE(o);
    CHECK(o->swing_side == "unknown");
    CHECK(o->frame_depth == -1.0);
    CHECK(ro.openings().size() == 1);
  }
  ProjectDB db(tmp.path);
  CHECK(db.schema_version() == ProjectDB::latest_schema_version());
  auto o = db.opening_by_guid(guid);
  REQUIRE(o);
  CHECK(o->hinge == "unknown");
  o->hinge = "start";
  db.put_opening_row(*o);
  CHECK(db.opening_by_guid(guid)->hinge == "start");
}

TEST_CASE("OpeningLeaf_WireOps_UpdateRevertAndJson",
          "[core][model_edits][leaf]") {
  TempPath tmp("test_opening_leaf_wire");
  ProjectDB db(tmp.path);
  const auto w = wall(db, "walls/a", 5.0);
  db.save_walls({w}, {});
  const auto d = door(db, "openings/a", w.guid, 1.0, 1.9);
  db.save_openings({d});

  core::apply_model_edit(
      db, wire(db, R"([{"op": "opening.update", "guid": ")" + d.guid +
                       R"(", "set": {"swing_side": "a", "hinge": "end",
                       "operation": "single_swing",
                       "type_name": "Enkeltdør 90", "frame_depth": 0.1}}])"));
  auto e = db.opening_by_guid(d.guid);
  REQUIRE(e);
  CHECK(e->swing_side == "a");
  CHECK(e->hinge == "end");
  CHECK(e->operation == "single_swing");
  CHECK(e->type_name == "Enkeltdør 90");
  CHECK(e->frame_depth == 0.1);
  CHECK(e->locked);
  // The original (unknown) is what edited_from remembers.
  const json from = json::parse(e->metadata).at("edited_from");
  CHECK(from.at("swing_side") == "unknown");
  CHECK(from.at("hinge") == "unknown");

  // A re-detection keeps the edited row (locked) as it is.
  db.save_openings({d});
  CHECK(db.opening_by_guid(d.guid)->hinge == "end");

  // Flip facing / hand again, then undo back to the first edit.
  core::apply_model_edit(
      db, wire(db, R"([{"op": "opening.update", "guid": ")" + d.guid +
                       R"(", "set": {"swing_side": "b", "hinge": "start"}}])"));
  CHECK(db.opening_by_guid(d.guid)->swing_side == "b");
  core::undo_model_edit(db);
  CHECK(db.opening_by_guid(d.guid)->swing_side == "a");

  core::apply_model_edit(
      db, {ModelEditOp::revert(ModelElementKind::opening, d.guid)});
  e = db.opening_by_guid(d.guid);
  CHECK(e->swing_side == "unknown");
  CHECK(e->hinge == "unknown");
  CHECK(e->operation == "unknown");
  CHECK(e->frame_depth == -1.0);

  SECTION("a create carries them") {
    core::apply_model_edit(
        db, wire(db, R"([{"op": "opening.create", "record": {"guid": "o-new",
        "wall": ")" + w.guid +
                         R"(", "kind": "door", "s_start": 3.0,
        "s_end": 3.9, "z_head": 2.1, "swing_side": "b", "hinge": "start",
        "operation": "double_swing", "type_name": "Dobbeltdør"}}])"));
    const auto n = db.opening_by_guid("o-new");
    REQUIRE(n);
    CHECK(n->operation == "double_swing");
    CHECK(n->swing_side == "b");
  }
  SECTION("a bad value is refused and nothing is written") {
    const auto rev = db.model_revision();
    CHECK_THROWS_AS(
        core::apply_model_edit(
            db, wire(db, R"([{"op": "opening.update", "guid": ")" + d.guid +
                             R"(", "set": {"swing_side": "in"}}])")),
        std::invalid_argument);
    CHECK(db.model_revision() == rev);
    CHECK_THROWS_AS(wire(db, R"([{"op": "opening.update", "guid": ")" + d.guid +
                                 R"(", "set": {"frame_depth": "deep"}}])"),
                    std::invalid_argument);
  }
  SECTION("record json round-trips the fields") {
    auto o = door(db, "openings/x", w.guid, 0.5, 1.2);
    o.swing_side = "b";
    o.hinge = "start";
    o.operation = "sliding";
    o.type_name = "Skydedør";
    o.frame_depth = 0.07;
    const json j = core::model_record_json(o);
    CHECK(j.at("swing_side") == "b");
    CHECK(j.at("operation") == "sliding");
    CHECK(j.at("frame_depth") == 0.07);
    CHECK(core::model_record_json(core::opening_from_json(j)) == j);
  }
}

TEST_CASE("OpeningLeaf_UndoOfAnEditLoggedBeforeV42_IsNoConflict",
          "[core][model_edits][leaf]") {
  TempPath tmp("test_opening_leaf_legacy_log");
  std::string guid;
  {
    ProjectDB db(tmp.path);
    const auto w = wall(db, "walls/a", 5.0);
    db.save_walls({w}, {});
    const auto d = door(db, "openings/a", w.guid, 1.0, 1.9);
    guid = d.guid;
    db.save_openings({d});
    core::apply_model_edit(
        db, wire(db, R"([{"op": "opening.update", "guid": ")" + guid +
                         R"(", "set": {"s_end": 2.0}}])"));
  }
  // Strip the v42 keys from the logged records, as a v41 build wrote them.
  {
    sqlite3 *raw = nullptr;
    REQUIRE(sqlite3_open(tmp.path.string().c_str(), &raw) == SQLITE_OK);
    sqlite3_stmt *s = nullptr;
    REQUIRE(sqlite3_prepare_v2(raw, "SELECT id, changes FROM model_edits;", -1,
                               &s, nullptr) == SQLITE_OK);
    std::vector<std::pair<int64_t, std::string>> rows;
    while (sqlite3_step(s) == SQLITE_ROW)
      rows.emplace_back(
          sqlite3_column_int64(s, 0),
          reinterpret_cast<const char *>(sqlite3_column_text(s, 1)));
    sqlite3_finalize(s);
    REQUIRE(rows.size() == 1);
    json changes = json::parse(rows[0].second);
    const auto strip = [](json &rec) {
      for (const char *k :
           {"swing_side", "hinge", "operation", "type_name", "frame_depth"})
        rec.erase(k);
    };
    for (auto &c : changes)
      for (const char *side : {"before", "after"})
        if (!c.at(side).is_null())
          strip(c.at(side).at("record"));
    REQUIRE(sqlite3_prepare_v2(raw,
                               "UPDATE model_edits SET changes = ? WHERE "
                               "id = ?;",
                               -1, &s, nullptr) == SQLITE_OK);
    const std::string text = changes.dump();
    sqlite3_bind_text(s, 1, text.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int64(s, 2, rows[0].first);
    REQUIRE(sqlite3_step(s) == SQLITE_DONE);
    sqlite3_finalize(s);
    sqlite3_close(raw);
  }
  ProjectDB db(tmp.path);
  core::undo_model_edit(db);
  CHECK(db.opening_by_guid(guid)->s_end == 1.9);
  core::redo_model_edit(db);
  CHECK(db.opening_by_guid(guid)->s_end == 2.0);
}

TEST_CASE("OpeningFit_Issues", "[core][openings][fit]") {
  core::WallRecord w;
  w.guid = "w";
  w.x1 = 4.0; // length 4
  w.z_base = 0.0;
  w.height = 2.5;
  const auto mk = [](std::string g, double s0, double s1) {
    core::OpeningRecord o;
    o.guid = std::move(g);
    o.wall = "w";
    o.s_start = s0;
    o.s_end = s1;
    o.z_sill = 0.0;
    o.z_head = 2.1;
    return o;
  };
  const auto codes = [](const std::vector<core::OpeningFitIssue> &v) {
    std::vector<std::string> out;
    for (const auto &i : v)
      out.push_back(i.code);
    return out;
  };

  CHECK(core::opening_fit_issues(mk("a", 1.0, 1.9), w, {}).empty());
  // Within the tolerance at the ends.
  CHECK(core::opening_fit_issues(mk("a", -0.004, 4.004), w, {}).empty());
  CHECK(codes(core::opening_fit_issues(mk("a", 3.5, 4.4), w, {})) ==
        std::vector<std::string>{"past_end"});
  CHECK(codes(core::opening_fit_issues(mk("a", -0.3, 0.6), w, {})) ==
        std::vector<std::string>{"before_start"});
  auto tall = mk("a", 1.0, 1.9);
  tall.z_head = 2.7;
  CHECK(codes(core::opening_fit_issues(tall, w, {})) ==
        std::vector<std::string>{"above_top"});
  auto low = mk("a", 1.0, 1.9);
  low.z_sill = -0.2;
  CHECK(codes(core::opening_fit_issues(low, w, {})) ==
        std::vector<std::string>{"below_base"});

  auto elsewhere = mk("elsewhere", 0.0, 4.0);
  elsewhere.wall = "another wall";
  const std::vector<core::OpeningRecord> others = {
      mk("a", 1.0, 1.9), mk("b", 2.5, 3.2), elsewhere};
  auto c = mk("c", 1.8, 2.6);
  const auto issues = core::opening_fit_issues(c, w, others);
  REQUIRE(issues.size() == 2);
  CHECK(issues[0].code == "overlaps");
  CHECK(issues[0].other == "a");
  CHECK(issues[1].other == "b");
  // Touching (end to end) is no overlap, nor is the opening itself.
  CHECK(core::opening_fit_issues(mk("a", 1.9, 2.5), w, others).empty());

  // A sloped top: the head must clear the top at both jambs.
  w.top = {{0.0, 2.0}, {4.0, 2.5}};
  CHECK(codes(core::opening_fit_issues(mk("x", 0.2, 1.0), w, {})) ==
        std::vector<std::string>{"above_top"});
  CHECK(core::opening_fit_issues(mk("x", 3.0, 3.8), w, {}).empty());
}

TEST_CASE("OpeningFit_ModelEdit_WarnsOnlyAboutWhatTheEditBroke",
          "[core][model_edits][fit]") {
  TempPath tmp("test_opening_fit_edit");
  ProjectDB db(tmp.path);
  const auto w = wall(db, "walls/a", 5.0);
  auto v = wall(db, "walls/b", 3.0);
  v.y0 = v.y1 = 4.0;
  db.save_walls({w, v}, {});
  const auto d1 = door(db, "openings/a", w.guid, 1.0, 1.9);
  const auto d2 = door(db, "openings/b", w.guid, 3.0, 3.9);
  // A detected opening that already overlaps its neighbour on wall b.
  const auto v1 = door(db, "openings/c", v.guid, 0.5, 1.5);
  const auto v2 = door(db, "openings/d", v.guid, 1.4, 2.0);
  db.save_openings({d1, d2, v1, v2});

  SECTION("a fitting edit warns about nothing, old problems stay silent") {
    const auto r = core::apply_model_edit(
        db, wire(db, R"([{"op": "opening.update", "guid": ")" + v1.guid +
                         R"(", "set": {"hinge": "end"}}])"));
    CHECK(r.warnings.empty());
  }
  SECTION("moving a door onto its neighbour warns, and still applies") {
    const auto r = core::apply_model_edit(
        db, wire(db, R"([{"op": "opening.update", "guid": ")" + d1.guid +
                         R"(", "set": {"s_start": 2.5, "s_end": 3.4}}])"));
    CHECK(any_contains(r.warnings, "overlaps"));
    CHECK(any_contains(r.warnings, d2.guid));
    CHECK(db.opening_by_guid(d1.guid)->s_start == 2.5);
  }
  SECTION("an opening placed past the wall's end warns") {
    const auto r = core::apply_model_edit(
        db, wire(db, R"([{"op": "opening.create", "record": {"guid": "o-x",
        "wall": ")" + w.guid +
                         R"(", "kind": "window", "s_start": 4.6,
        "s_end": 5.6, "z_sill": 0.9, "z_head": 2.1}}])"));
    CHECK(any_contains(r.warnings, "past the end"));
    CHECK(any_contains(r.warnings, "o-x"));
  }
  SECTION("shortening a wall under a door warns about the door") {
    const auto r = core::apply_model_edit(
        db, wire(db, R"([{"op": "wall.update", "guid": ")" + w.guid +
                         R"(", "set": {"x1": 3.5}}])"));
    CHECK(any_contains(r.warnings, d2.guid));
    CHECK(any_contains(r.warnings, "past the end"));
  }
}
