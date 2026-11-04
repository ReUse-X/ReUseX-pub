// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Stairs (schema v45, reusex-xvnl): the record, its checks and advice,
// storage and the model edit kind 'stair'.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/model_edits.hpp>
#include <reusex/core/solid_model.hpp>
#include <reusex/core/stairs.hpp>

#include "../../support/temp_path.hpp"

#include <nlohmann/json.hpp>

#include <string>
#include <vector>

using Catch::Matchers::WithinAbs;
using namespace reusex;
using core::ModelElementKind;
using reusex::test_support::TempPath;
using json = nlohmann::json;

namespace {

/// A U stair from storey 0 (z 0) to storey 1 (z 2.8): two runs of 8
/// risers, 1.0 m wide, a landing between them.
core::StairRecord u_stair() {
  core::StairRecord s;
  s.guid = "stair-u";
  s.storey = 0;
  s.storey_to = 1;
  s.kind = "U";
  s.runs = {{{0.5, 0.0}, {0.5, 1.82}, 1.0, 8},
            {{1.6, 1.82}, {1.6, 0.0}, 1.0, 8}};
  s.landings.push_back(
      {{{0.0, 1.82}, {2.1, 1.82}, {2.1, 2.82}, {0.0, 2.82}}, {}});
  s.z_base = 0.0;
  s.z_top = 2.8;
  return s;
}

core::ModelEditResult edit(ProjectDB &db, const json &ops) {
  return core::apply_model_edit(db, core::model_edit_ops_from_json(db, ops));
}

} // namespace

TEST_CASE("Stairs_MeasuresChecksAndAdvice", "[core][stairs]") {
  auto s = u_stair();
  CHECK(core::check_stair(s).empty());
  CHECK(core::stair_riser_count(s) == 16);
  CHECK_THAT(core::stair_riser_height(s), WithinAbs(0.175, 1e-12));
  CHECK_THAT(core::stair_going(s, 0), WithinAbs(0.26, 1e-12));
  CHECK_THAT(core::stair_run_base(s, 1), WithinAbs(1.4, 1e-12));
  // 2 x 0.175 + 0.26 = 0.61: fine.
  CHECK(core::stair_advice(s).empty());

  auto steep = s;
  steep.runs[0].risers = 5;
  steep.runs[1].risers = 5; // 0.28 m risers
  const auto advice = core::stair_advice(steep);
  REQUIRE_FALSE(advice.empty());
  CHECK(advice[0].find("0.20") != std::string::npos);

  auto bad = s;
  bad.kind = "zigzag";
  CHECK_FALSE(core::check_stair(bad).empty());
  bad = s;
  bad.runs.clear();
  CHECK_FALSE(core::check_stair(bad).empty());
  bad = s;
  bad.z_top = -1.0;
  CHECK_FALSE(core::check_stair(bad).empty());
  bad = s;
  bad.runs[0].width = 0.0;
  CHECK_FALSE(core::check_stair(bad).empty());

  // Stored forms round-trip.
  CHECK(core::decode_stair_runs(core::encode_stair_runs(s.runs)).size() == 2);
  CHECK(core::decode_stair_landings(core::encode_stair_landings(s.landings))
            .front()
            .outer.size() == 4);
  const auto j = core::model_record_json(s);
  const auto back = core::stair_from_json(j);
  CHECK(back.runs.size() == 2);
  CHECK(back.runs[1].end[1] == 0.0);
  CHECK(back.landings.size() == 1);
  CHECK(back.kind == "U");
  CHECK_THROWS_AS(core::stair_from_json(json{{"runs", "x"}}),
                  std::invalid_argument);
}

TEST_CASE("Stairs_EditKind_CreateUpdateDeleteUndo", "[core][stairs]") {
  TempPath project("test_stairs_edits");
  ProjectDB db(project.path);
  core::StoreyRecord s0;
  s0.guid = "s0";
  s0.z_ceiling = 2.8;
  core::StoreyRecord s1 = s0;
  s1.guid = "s1";
  s1.idx = 1;
  s1.z_floor = 2.8;
  s1.z_ceiling = 5.6;
  db.save_storeys({s0, s1});
  CHECK(db.stairs().empty());

  edit(db, json::array({{{"op", "stair.create"},
                         {"record", core::model_record_json(u_stair())}}}));
  auto st = db.stair_by_guid("stair-u");
  REQUIRE(st);
  CHECK(st->source == "manual");
  CHECK(st->locked);
  CHECK(st->runs.size() == 2);
  CHECK(st->landings.size() == 1);
  CHECK(st->cut_slab_above);

  edit(db, json::array({{{"op", "stair.update"},
                         {"guid", "stair-u"},
                         {"set", {{"location", "left"}, {"nosing", 0.03}}}}}));
  CHECK(db.stair_by_guid("stair-u")->location == "left");
  CHECK_THROWS_AS(edit(db, json::array({{{"op", "stair.update"},
                                         {"guid", "stair-u"},
                                         {"set", {{"z_top", -1.0}}}}})),
                  std::invalid_argument);

  // A storey with a stair on it cannot be deleted alone.
  CHECK_THROWS_AS(
      edit(db, json::array({{{"op", "storey.delete"}, {"guid", "s0"}}})),
      std::invalid_argument);

  edit(db, json::array({{{"op", "stair.delete"}, {"guid", "stair-u"}}}));
  CHECK_FALSE(db.stair_by_guid("stair-u"));
  CHECK(db.dismissed_model_elements().empty()); // drawn: no tombstone
  core::undo_model_edit(db);
  CHECK(db.stair_by_guid("stair-u")->location == "left");
}
