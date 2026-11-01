// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// The Ressourcer workspace's Qt-free half (rux_qt/resource_list.hpp):
// template choice, key lookup, storey lookup, search / facet filters,
// grouping by type and the thumbnail crop math.

#include <catch2/catch_test_macros.hpp>

#include <rux_qt/resource_list.hpp>

#include <string>
#include <vector>

using namespace rux::qt;

namespace {

ResourceRowData row(std::string code, std::int64_t type, std::string type_name,
                    std::string designation, std::string room,
                    std::string storey, int storey_idx,
                    std::string condition = {}, std::string potential = {}) {
  ResourceRowData r;
  r.code = std::move(code);
  r.type_id = type;
  r.type_name = std::move(type_name);
  r.designation = std::move(designation);
  r.room = std::move(room);
  r.storey = std::move(storey);
  r.storey_idx = storey_idx;
  r.condition = std::move(condition);
  r.potential = std::move(potential);
  return r;
}

/// Sygehusgrunden-like: windows and radiators on two storeys.
std::vector<ResourceRowData> rows() {
  return {
      row("RX-001", 1, "Vinduer", "Koblet trævindue", "S0-06 Depot", "Etage 0",
          0, "Middel", "Middel"),
      row("RX-002", 1, "Vinduer", "Koblet trævindue", "S0-07 Teknikrum",
          "Etage 0", 0, "God", "Høj"),
      row("RX-003", 2, "Radiatorer", "Panelradiator", "S1-09 Kontor", "Etage 1",
          1, "Dårlig", "Lav"),
      row("RX-004", 2, "Radiatorer", "Ribberadiator", "S1-09 Kontor", "Etage 1",
          1, "Rimelig", "Middel"),
      row("RX-005", 1, "Vinduer", "Ovenlysvindue", "", "", -1),
  };
}

std::vector<TypeRef> types() { return {{1, "Vinduer"}, {2, "Radiatorer"}}; }

} // namespace

TEST_CASE("FoldDa_LowersAsciiAndDanishCapitals", "[rux_qt][resources]") {
  CHECK(fold_da("RX-001 Kontor") == "rx-001 kontor");
  CHECK(fold_da("ÆBLE ØST Å") == "æble øst å");
  CHECK(fold_da("Høj") == "høj");
}

TEST_CASE("GradeTone_MapsDanishGradesToPillTones", "[rux_qt][resources]") {
  CHECK(grade_tone("God") == "good");
  CHECK(grade_tone("høj") == "good");
  CHECK(grade_tone("Rimelig") == "warn");
  CHECK(grade_tone("Middel") == "warn");
  CHECK(grade_tone("Dårlig") == "crit");
  CHECK(grade_tone("LAV") == "crit");
  CHECK(grade_tone("Ukendt") == "outline");
  CHECK(grade_tone("") == "");
}

TEST_CASE("DisplayValue_FormatsStoredValuesForReading", "[rux_qt][resources]") {
  CHECK(display_value("text", "", std::nullopt) == "—");
  CHECK(display_value("text", "", std::string{}) == "—");
  CHECK(display_value("text", "", std::string{"Koblet vindue"}) ==
        "Koblet vindue");
  CHECK(display_value("boolean", "", std::string{"true"}) == "Ja");
  CHECK(display_value("boolean", "", std::string{"false"}) == "Nej");
  CHECK(display_value("number", "kg", std::string{"12.5"}) == "12,5 kg");
  CHECK(display_value("number", "", std::string{"290"}) == "290");
  CHECK(display_value("multiselect", "",
                      std::string{R"(["concrete","steel"])"}) ==
        "concrete, steel");
  CHECK(display_value("multiselect", "", std::string{R"(["a \"b\""])"}) ==
        "a \"b\"");
  CHECK(display_value("multiselect", "", std::string{"[]"}) == "—");
  CHECK(display_value("enum", "", std::string{"Høj"}) == "Høj");
}

TEST_CASE("DefaultTemplate_PrefersRememberedThenUserMadeThenScreening",
          "[rux_qt][resources]") {
  const std::vector<TemplateRef> seeds = {
      {1, "Materialepas (fuld)", "materialepas"},
      {2, "Hurtig genbrugsscreening", "screening"}};
  CHECK(default_template(seeds) == 2);
  CHECK(default_template(seeds, 1) == 1);
  CHECK(default_template(seeds, 99) == 2); // remembered one is gone

  auto with_user = seeds;
  with_user.push_back({3, "Genbrugsoversigt", ""});
  with_user.push_back({4, "Kladde", ""});
  CHECK(default_template(with_user) == 4);
  CHECK(default_template(with_user, 1) == 1);

  CHECK(default_template({{7, "A", "materialepas"}}) == 7);
  CHECK_FALSE(default_template({}).has_value());
}

TEST_CASE("FindKey_MatchesLabelOrFieldCaseInsensitivelyInNameOrder",
          "[rux_qt][resources]") {
  const std::vector<KeyRef> keys = {
      {"sys:name", "Betegnelse", "name"},
      {"lex:a", "Designation", "designation"},
      {"col:1", "Tilstand", "Tilstand"},
      {"col:2", "Genbrugspotentiale", "Genbrugspotentiale"},
      {"col:3", "Etage", "Etage"},
  };
  CHECK(find_key(keys, designation_key_names()) == 1u);
  CHECK(find_key(keys, condition_key_names()) == 2u);
  CHECK(find_key(keys, potential_key_names()) == 3u);
  CHECK(find_key(keys, storey_key_names()) == 4u);
  CHECK(find_key(keys, {"TILSTAND"}) == 2u);
  CHECK_FALSE(find_key(keys, {"Vægt"}).has_value());
  // Without a designation field the built-in name is used.
  CHECK(find_key({{"sys:name", "Betegnelse", "name"}},
                 designation_key_names()) == 0u);
}

TEST_CASE("StoreyIndex_LooksUpTheRoomsStoreyByName", "[rux_qt][resources]") {
  const StoreyIndex idx(
      {{"S0-06 Depot", 0}, {"S1-09 Kontor", 1}, {"S3-01 Loft", 3}},
      {{0, "Etage 0"}, {1, "Stueetage"}});
  REQUIRE(idx.lookup("S1-09 Kontor").has_value());
  CHECK(idx.lookup("S1-09 Kontor")->first == 1);
  CHECK(idx.lookup("S1-09 Kontor")->second == "Stueetage");
  CHECK(idx.lookup("S3-01 Loft")->second == "Etage 3"); // no storeys row
  CHECK_FALSE(idx.lookup("S9-99 Ukendt").has_value());
  CHECK_FALSE(idx.lookup("").has_value());
}

TEST_CASE("Matches_EveryQueryWordMustOccurSomewhere", "[rux_qt][resources]") {
  const auto r = rows();
  ResourceFilter f;
  CHECK(matches(r[0], f));
  f.query = "rx-003";
  CHECK(matches(r[2], f));
  CHECK_FALSE(matches(r[0], f));
  f.query = "VINDUE depot";
  CHECK(matches(r[0], f));
  CHECK_FALSE(matches(r[1], f));
  f.query = "høj";
  CHECK(matches(r[1], f));
  f.query = "  ";
  CHECK(matches(r[3], f));

  ResourceFilter s;
  s.storey = "Etage 1";
  CHECK(matches(r[2], s));
  CHECK_FALSE(matches(r[0], s));
  s.room = "S1-09 Kontor";
  CHECK(matches(r[3], s));

  ResourceFilter d;
  auto gone = r[0];
  gone.dismissed = true;
  CHECK_FALSE(matches(gone, d));
  d.show_dismissed = true;
  CHECK(matches(gone, d));
}

TEST_CASE("GroupResources_GroupsByTypeOrderAndDropsEmptyGroups",
          "[rux_qt][resources]") {
  const auto r = rows();
  ResourceFilter f;
  auto g = group_resources(r, types(), f);
  REQUIRE(g.size() == 2);
  CHECK(g[0].type_name == "Vinduer");
  CHECK(g[0].rows == std::vector<std::size_t>{0, 1, 4});
  CHECK(g[0].total == 3);
  CHECK(g[1].rows == std::vector<std::size_t>{2, 3});

  f.storey = "Etage 1";
  g = group_resources(r, types(), f);
  REQUIRE(g.size() == 1);
  CHECK(g[0].type_name == "Radiatorer");
  CHECK(g[0].total == 2);

  // A type not in the type list still shows, after the listed ones.
  auto extra = r;
  extra.push_back(row("RX-009", 9, "Døre", "Fyldingsdør", "", "", -1));
  g = group_resources(extra, types(), {});
  REQUIRE(g.size() == 3);
  CHECK(g[2].type_name == "Døre");
}

TEST_CASE("FacetOptions_ListStoreysAndRoomsWithCounts", "[rux_qt][resources]") {
  const auto r = rows();
  const auto s = storey_options(r);
  REQUIRE(s.size() == 3);
  CHECK(s[0].value == "Etage 0");
  CHECK(s[0].count == 2);
  CHECK(s[1].value == "Etage 1");
  CHECK(s[2].value.empty()); // unknown storey last
  CHECK(s[2].count == 1);

  const auto all = room_options(r, "");
  REQUIRE(all.size() == 3); // the room-less row is not a room
  CHECK(all[0].value == "S0-06 Depot");
  const auto first = room_options(r, "Etage 1");
  REQUIRE(first.size() == 1);
  CHECK(first[0].value == "S1-09 Kontor");
  CHECK(first[0].count == 2);
}

TEST_CASE("ListSummary_SaysWhatIsShown", "[rux_qt][resources]") {
  const auto g = group_resources(rows(), types(), {});
  CHECK(list_summary(g, 5, false) == "5 ressourcer i 2 typer");
  ResourceFilter f;
  f.query = "radiator";
  const auto h = group_resources(rows(), types(), f);
  CHECK(list_summary(h, 5, true) == "Viser 2 af 5 ressourcer i 1 type");
  CHECK(resource_count_text(1) == "1 ressource");
  CHECK(resource_count_text(1211) == "1.211 ressourcer");
}

TEST_CASE("RegionPixels_RoundsOutwardAndClamps", "[rux_qt][resources]") {
  CHECK(region_pixels(std::nullopt, 640, 480) == PixelRect{0, 0, 640, 480});
  CHECK(region_pixels(NormRegion{0.25, 0.5, 0.5, 0.25}, 640, 480) ==
        PixelRect{160, 240, 320, 120});
  // Outward rounding: 0.1 * 99 = 9.9 -> 9, (0.1 + 0.2) * 99 = 29.7 -> 30.
  CHECK(region_pixels(NormRegion{0.1, 0.1, 0.2, 0.2}, 99, 99) ==
        PixelRect{9, 9, 21, 21});
  // A seam-crossing panorama region is clamped at the right edge.
  CHECK(region_pixels(NormRegion{0.9, 0.0, 0.2, 1.0}, 1000, 500) ==
        PixelRect{900, 0, 100, 500});
  // Never empty.
  const auto tiny = region_pixels(NormRegion{0.5, 0.5, 1e-6, 1e-6}, 10, 10);
  CHECK(tiny.w >= 1);
  CHECK(tiny.h >= 1);
}

TEST_CASE("ThumbnailCrop_GrowsTheShortSideToTheAspectInsideTheImage",
          "[rux_qt][resources]") {
  // A wide, low window strip (RX-001-like) in a 960x720 frame, 4:3 thumb:
  // the height grows around the strip's centre.
  const NormRegion strip{0.0713, 0.0, 0.7774, 0.1411};
  auto c = thumbnail_crop(strip, 960, 720, 4.0 / 3.0);
  REQUIRE(c.size() == 1);
  CHECK(c[0].x == 68);
  CHECK(c[0].w == 747);
  CHECK(c[0].y == 0); // shifted down to stay inside
  CHECK(c[0].h == 560);

  // A tall region grows its width, centred.
  c = thumbnail_crop(NormRegion{0.5, 0.25, 0.1, 0.5}, 1000, 1000, 1.0);
  REQUIRE(c.size() == 1);
  CHECK(c[0] == PixelRect{300, 250, 500, 500});

  // Larger than the image allows: capped at the image, never cut.
  c = thumbnail_crop(NormRegion{0.0, 0.0, 1.0, 0.9}, 400, 300, 4.0);
  REQUIRE(c.size() == 1);
  CHECK(c[0] == PixelRect{0, 0, 400, 270});

  // No region: the whole image (the widget letterboxes it).
  c = thumbnail_crop(std::nullopt, 400, 300, 1.0);
  REQUIRE(c.size() == 1);
  CHECK(c[0] == PixelRect{0, 0, 400, 300});
}

TEST_CASE("ThumbnailCrop_WrapsAPanoramaAcrossTheSeam", "[rux_qt][resources]") {
  // 60 x 45 degree window centred on the seam of a 3600x1800 equirect.
  const NormRegion seam{1.0 - 30.0 / 360.0, 0.375, 60.0 / 360.0, 0.25};
  const auto c = thumbnail_crop(seam, 3600, 1800, 600.0 / 450.0, true);
  REQUIRE(c.size() == 2);
  CHECK(c[0].x == 3300);
  CHECK(c[0].x + c[0].w == 3600);
  CHECK(c[1].x == 0);
  CHECK(c[0].w + c[1].w == 600);
  CHECK(c[0].h == 450);
  CHECK(c[1].y == c[0].y);

  // Not crossing: one piece, no shift needed.
  const auto one = thumbnail_crop(NormRegion{0.25, 0.375, 1.0 / 6.0, 0.25},
                                  3600, 1800, 600.0 / 450.0, true);
  REQUIRE(one.size() == 1);
  CHECK(one[0] == PixelRect{900, 675, 600, 450});
}

TEST_CASE("RegionInCrop_PlacesTheRegionInsideTheJoinedCrop",
          "[rux_qt][resources]") {
  const NormRegion r{0.5, 0.25, 0.1, 0.5};
  const auto c = thumbnail_crop(r, 1000, 1000, 1.0);
  CHECK(region_in_crop(r, 1000, 1000, c) == PixelRect{200, 0, 100, 500});

  const NormRegion seam{1.0 - 30.0 / 360.0, 0.375, 60.0 / 360.0, 0.25};
  const auto p = thumbnail_crop(seam, 3600, 1800, 600.0 / 450.0, true);
  const auto in = region_in_crop(seam, 3600, 1800, p);
  CHECK(in.x == 0);
  CHECK(in.w == 600);
}
