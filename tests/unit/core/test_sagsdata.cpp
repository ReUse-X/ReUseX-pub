// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// core/sagsdata.hpp: the BEK 496 Sagsdata key scheme shared by ruxd and the
// PDF report.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <core/ProjectDB.hpp>
#include <core/sagsdata.hpp>

#include "../../support/temp_path.hpp"

#include <nlohmann/json.hpp>

#include <stdexcept>

using Catch::Matchers::ContainsSubstring;
using reusex::ProjectDB;
using reusex::test_support::TempPath;
namespace core = reusex::core;
using nlohmann::json;

TEST_CASE("Sagsdata_SettingKey_IsTheRuxdScheme", "[sagsdata]") {
  CHECK(core::sagsdata_setting_key("p1", "bbr_number") ==
        "sagsdata/p1/bbr_number");
  CHECK(core::sagsdata_fields().size() == 12);
  CHECK(core::is_sagsdata_field("screening_approved_at"));
  CHECK(core::is_sagsdata_field("bbr_building_id"));
  CHECK_FALSE(core::is_sagsdata_field("name"));
  // The register ids come last: a client that knows only the nine texts
  // reads the same wire order as before.
  CHECK(core::sagsdata_fields()[9] == "husnummer_id");
  CHECK(core::sagsdata_fields()[10] == "bfe_number");
  CHECK(core::sagsdata_fields()[11] == "bbr_building_id");
}

TEST_CASE("Sagsdata_Json_ReadsStoredValuesAndNulls", "[sagsdata]") {
  TempPath tmp("test_sagsdata");
  ProjectDB db(tmp.path);
  db.set_setting(core::sagsdata_setting_key("p1", "client"), "Region Syd");
  db.set_setting(core::sagsdata_setting_key("p1", "demolished_floor_area_m2"),
                 "1240.5");
  db.set_setting(core::sagsdata_setting_key("p1", "cadastral"), "");
  const auto j = core::sagsdata_json(db, "p1");
  CHECK(j.at("client") == "Region Syd");
  CHECK(j.at("demolished_floor_area_m2") == 1240.5);
  CHECK(j.at("cadastral").is_null()); // "" is unset
  CHECK(j.at("bbr_number").is_null());
  CHECK(j.at("bfe_number").is_null());
  CHECK(j.size() == core::sagsdata_fields().size());
  db.set_setting(core::sagsdata_setting_key("p1", "bfe_number"), "6033528");
  db.set_setting(core::sagsdata_setting_key("p1", "husnummer_id"),
                 "0a3f507a-ec01-32b8-e044-0003ba298018");
  const auto k = core::sagsdata_json(db, "p1");
  CHECK(k.at("bfe_number") == 6033528);
  CHECK(k.at("bfe_number").is_number_integer());
  CHECK(k.at("husnummer_id") == "0a3f507a-ec01-32b8-e044-0003ba298018");
  db.set_setting(core::sagsdata_setting_key("p1", "bfe_number"), "garbage");
  CHECK(core::sagsdata_json(db, "p1").at("bfe_number").is_null());
  CHECK_FALSE(core::sagsdata_value(db, "p1", "cadastral").has_value());
  CHECK(core::sagsdata_value(db, "other", "client") == std::nullopt);
}

TEST_CASE("Sagsdata_StoredValue_ValidatesEachField", "[sagsdata]") {
  CHECK(core::sagsdata_stored_value("client", json(nullptr)).empty());
  CHECK(core::sagsdata_stored_value("client", json("X")) == "X");
  CHECK(core::sagsdata_stored_value("demolished_floor_area_m2", json(12.5)) ==
        "12.5");
  CHECK_THROWS_AS(
      core::sagsdata_stored_value("demolished_floor_area_m2", json(-1)),
      std::invalid_argument);
  CHECK_THROWS_AS(
      core::sagsdata_stored_value("demolished_floor_area_m2", json("12")),
      std::invalid_argument);
  CHECK_THROWS_WITH(
      core::sagsdata_stored_value("demolition_start", json("2026-02-29")),
      ContainsSubstring("demolition_start"));
  CHECK(core::sagsdata_stored_value("demolition_start", json("2028-02-29")) ==
        "2028-02-29");
  CHECK(core::sagsdata_stored_value("screening_approved_at",
                                    json("2026-10-01T09:30:00Z")) ==
        "2026-10-01T09:30:00Z");
  CHECK_THROWS_AS(
      core::sagsdata_stored_value("screening_approved_at", json("1. oktober")),
      std::invalid_argument);
  CHECK_THROWS_AS(core::sagsdata_stored_value("client", json(3)),
                  std::invalid_argument);
  CHECK_THROWS_AS(core::sagsdata_stored_value("name", json("x")),
                  std::invalid_argument);
}

TEST_CASE("Sagsdata_StoredValue_RegisterIds", "[sagsdata]") {
  // The ids the ruxd lookups give (rux-frontend contract gap 17).
  CHECK(core::sagsdata_stored_value("bfe_number", json(6033528)) ==
        "6033528");
  CHECK(core::sagsdata_stored_value("bfe_number", json(nullptr)).empty());
  CHECK_THROWS_WITH(core::sagsdata_stored_value("bfe_number", json(0)),
                    ContainsSubstring("positive integer"));
  CHECK_THROWS_AS(core::sagsdata_stored_value("bfe_number", json(12.5)),
                  std::invalid_argument);
  CHECK_THROWS_AS(core::sagsdata_stored_value("bfe_number", json("6033528")),
                  std::invalid_argument);
  const std::string dar = "0a3f507a-ec01-32b8-e044-0003ba298018";
  CHECK(core::sagsdata_stored_value("husnummer_id", json(dar)) == dar);
  CHECK(core::sagsdata_stored_value(
            "bbr_building_id", json("78fcbe68-49a7-4913-b3b4-4b041c588bda")) ==
        "78fcbe68-49a7-4913-b3b4-4b041c588bda");
  CHECK(core::sagsdata_stored_value("husnummer_id", json("")).empty());
  CHECK_THROWS_WITH(
      core::sagsdata_stored_value("husnummer_id",
                                  json("0A3F507A-EC01-32B8-E044-0003BA298018")),
      ContainsSubstring("lowercase UUID"));
  CHECK_THROWS_AS(core::sagsdata_stored_value("bbr_building_id", json("../x")),
                  std::invalid_argument);
  CHECK_THROWS_AS(core::sagsdata_stored_value("husnummer_id", json(7)),
                  std::invalid_argument);
}

TEST_CASE("Sagsdata_DanishDate_FormatsIsoDatesOnly", "[sagsdata]") {
  CHECK(core::danish_date("2026-09-21") == "21.09.2026");
  CHECK(core::danish_date("2026-09-21T10:00:00Z") == "21.09.2026");
  CHECK(core::danish_date("i morgen") == "i morgen");
}
