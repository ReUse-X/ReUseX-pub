// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// core::assign_space_guids (reusex-z1v.6.3): a space's natural key is its
// storey and its polygon's centroid rounded to 5 cm; a re-detected space
// keeps the GUID of the stored one it lies on.

#include <catch2/catch_test_macros.hpp>

#include <reusex/core/guid.hpp>
#include <reusex/core/space_identity.hpp>
#include <reusex/core/space_polygon.hpp>

#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

using namespace reusex::core;
using Catch::Matchers::WithinAbs;

namespace {

constexpr const char *kProject = "6f1c2a8e-4b1d-4c5e-9a7f-0d3e2b1a4c5d";

SpaceRecord sp(const std::string &guid, int storey, double x0, double y0,
               double w, double h) {
  SpaceRecord s;
  s.guid = guid;
  s.storey = storey;
  s.polygon.outer = {{x0, y0}, {x0 + w, y0}, {x0 + w, y0 + h}, {x0, y0 + h}};
  s.area = w * h;
  return s;
}

} // namespace

TEST_CASE("SpaceIdentity_NaturalKey_StoreyAndCentroidAt5cm",
          "[core][spaces][identity]") {
  // Centroid (2.012, 1.537) -> 2.00, 1.55
  CHECK(space_natural_key(sp("x", 1, 0.012, 0.037, 4.0, 3.0)) ==
        "spaces/1/2.00,1.55");
  // -0.024 rounds to -0.00 -> printed as 0.00
  CHECK(space_natural_key(sp("x", 0, -1.024, -1.0, 2.0, 2.0)) ==
        "spaces/0/0.00,0.00");
}

TEST_CASE("SpaceIdentity_FreshSpaces_MintFromTheirKey",
          "[core][spaces][identity]") {
  std::vector<SpaceRecord> fresh{sp("space-0-1", 0, 0, 0, 4, 4)};
  std::vector<SpaceBoundaryRecord> bounds{{"space-0-1", "w", "a", 0, 4}};
  const auto r = assign_space_guids(fresh, bounds, {}, {}, kProject);
  CHECK(r.minted == 1);
  CHECK(r.matched == 0);
  CHECK(fresh[0].guid == deterministic_guid(kProject, "spaces/0/2.00,2.00"));
  CHECK(bounds[0].space == fresh[0].guid);

  // Deterministic: the same input gives the same GUID.
  std::vector<SpaceRecord> again{sp("space-0-7", 0, 0, 0, 4, 4)};
  std::vector<SpaceBoundaryRecord> none;
  assign_space_guids(again, none, {}, {}, kProject);
  CHECK(again[0].guid == fresh[0].guid);
}

TEST_CASE("SpaceIdentity_MovedOutline_KeepsTheStoredGuid",
          "[core][spaces][identity]") {
  auto stored = sp("stored-guid", 0, 0, 0, 4, 4);
  stored.locked = true;
  std::vector<SpaceRecord> fresh{
      sp("p1", 0, 0.1, -0.05, 4.1, 4.0), // same room, outline moved
      sp("p2", 1, 0, 0, 4, 4),           // same place, other storey: new
      sp("p3", 0, 6, 0, 4, 4)};          // elsewhere: new
  std::vector<SpaceBoundaryRecord> bounds{{"p1", "w", "a", 0, 4},
                                          {"p3", "w", "b", 6, 10}};
  const auto r = assign_space_guids(fresh, bounds, {stored}, {}, kProject);
  CHECK(r.matched == 1);
  CHECK(r.locked_matched == 1);
  CHECK(r.minted == 2);
  // The stored row is locked: it is kept as stored (reusex-s4mm), and the
  // detection that matched it goes, with its boundaries.
  REQUIRE(fresh.size() == 2);
  CHECK(fresh[0].guid != "stored-guid");
  CHECK(fresh[1].guid != "stored-guid");
  REQUIRE(r.kept.size() == 1);
  CHECK(r.kept[0].guid == "stored-guid");
  CHECK(r.kept[0].polygon.outer == stored.polygon.outer);
  CHECK(r.unmatched_locked == 0);
  REQUIRE(bounds.size() == 1);
  CHECK(bounds[0].space == fresh[1].guid);

  // Unlocked, it hands its GUID to the new outline.
  stored.locked = false;
  std::vector<SpaceRecord> again{sp("p1", 0, 0.1, -0.05, 4.1, 4.0)};
  std::vector<SpaceBoundaryRecord> b2{{"p1", "w", "a", 0, 4}};
  const auto r2 = assign_space_guids(again, b2, {stored}, {}, kProject);
  CHECK(r2.matched == 1);
  CHECK(r2.kept.empty());
  REQUIRE(again.size() == 1);
  CHECK(again[0].guid == "stored-guid");
  CHECK(b2[0].space == "stored-guid");
}

TEST_CASE("SpaceIdentity_SplitRoom_OnlyOneHalfInherits",
          "[core][spaces][identity]") {
  // A stored 8 x 4 room is now two 4 x 4 rooms. Neither half is the same
  // room by the first round (half the larger footprint, below 0.6; half the
  // stored area, below min_area_ratio), but each lies wholly inside it, so
  // the second round gives the identity to one of them, never both.
  // Named but not locked (source manual): the name follows the detection.
  auto stored = sp("stored-guid", 0, 0, 0, 8, 4);
  stored.name = "Sal";
  stored.source = "manual";
  std::vector<SpaceRecord> fresh{sp("p1", 0, 0, 0, 4, 4),
                                 sp("p2", 0, 4, 0, 4, 4)};
  std::vector<SpaceBoundaryRecord> none;
  SpaceIdentityOptions opt;
  opt.match_m = 3.0;
  opt.min_area_ratio = 0.6; // 16 / 32 = 0.5 < 0.6
  opt.min_overlap = 0.6;
  const auto r = assign_space_guids(fresh, none, {stored}, {}, kProject, opt);
  CHECK(r.matched == 1);
  CHECK(r.minted == 1);
  CHECK(r.kept.empty());
  REQUIRE(fresh.size() == 2);
  CHECK(fresh[0].guid == "stored-guid"); // tie: the first new space
  CHECK(fresh[0].name == "Sal");
  CHECK(fresh[1].guid != "stored-guid");
  CHECK(fresh[1].name.empty());
  CHECK_FALSE(fresh[1].locked);
}

TEST_CASE("SpaceIdentity_SplitLockedRoom_StaysWholeNoPieceInsideIt",
          "[core][spaces][identity]") {
  // reusex-s4mm: the same split under a LOCKED room. The room is kept as
  // stored; neither half becomes a space of its own on top of it.
  auto stored = sp("stored-guid", 0, 0, 0, 8, 4);
  stored.name = "Sal";
  stored.source = "manual";
  stored.locked = true;
  std::vector<SpaceRecord> fresh{sp("p1", 0, 0, 0, 4, 4),
                                 sp("p2", 0, 4, 0, 4, 4),
                                 sp("p3", 0, 10, 0, 4, 4)}; // elsewhere
  std::vector<SpaceBoundaryRecord> bounds{
      {"p1", "w", "a", 0, 4}, {"p2", "w", "a", 4, 8}, {"p3", "w", "b", 10, 14}};
  SpaceIdentityOptions opt;
  opt.match_m = 3.0;
  opt.min_area_ratio = 0.6;
  opt.min_overlap = 0.6;
  const auto r = assign_space_guids(fresh, bounds, {stored}, {}, kProject, opt);
  CHECK(r.matched == 1);
  CHECK(r.locked_matched == 1);
  CHECK(r.inside_locked == 1);
  CHECK(r.minted == 1);
  REQUIRE(fresh.size() == 1);
  CHECK(fresh[0].polygon.outer == sp("", 0, 10, 0, 4, 4).polygon.outer);
  REQUIRE(bounds.size() == 1);
  CHECK(bounds[0].space == fresh[0].guid);
  REQUIRE(r.kept.size() == 1);
  CHECK(r.kept[0].guid == "stored-guid");
  CHECK(r.kept[0].polygon.outer == stored.polygon.outer);
}

TEST_CASE("SpaceIdentity_RejectsDuplicatePlaceholdersAndABadNamespace",
          "[core][spaces][identity]") {
  std::vector<SpaceRecord> twice{sp("k", 0, 0, 0, 1, 1),
                                 sp("k", 0, 5, 0, 1, 1)};
  std::vector<SpaceBoundaryRecord> none;
  CHECK_THROWS_AS(assign_space_guids(twice, none, {}, {}, kProject),
                  std::invalid_argument);
  std::vector<SpaceRecord> one{sp("k", 0, 0, 0, 1, 1)};
  CHECK_THROWS_AS(assign_space_guids(one, none, {}, {}, "not-a-uuid"),
                  std::invalid_argument);
}

TEST_CASE("SpaceIdentity_SameKeyTwice_GetsASuffix",
          "[core][spaces][identity]") {
  // Two spaces whose centroids round to the same 5 cm cell (a ring room and
  // the room it encloses) still get distinct GUIDs.
  std::vector<SpaceRecord> fresh{sp("p1", 0, 0, 0, 4, 4),
                                 sp("p2", 0, 1.99, 1.99, 0.03, 0.03)};
  std::vector<SpaceBoundaryRecord> none;
  assign_space_guids(fresh, none, {}, {}, kProject);
  CHECK(fresh[0].guid == deterministic_guid(kProject, "spaces/0/2.00,2.00"));
  CHECK(fresh[1].guid == deterministic_guid(kProject, "spaces/0/2.00,2.00#2"));
}

namespace {

/// A storey whose world -> level transform translates by (tx, ty, tz).
StoreyRecord storey(const std::string &guid, int idx, double tx, double ty,
                    double tz = 0.0) {
  StoreyRecord s;
  s.guid = guid;
  s.idx = idx;
  s.z_floor = 0.0;
  s.z_ceiling = 2.5;
  s.level_transform = {1, 0, 0, tx, 0, 1, 0, ty, 0, 0, 1, tz, 0, 0, 0, 1};
  return s;
}

} // namespace

TEST_CASE("SpaceIdentity_OverlapArea_IsExact", "[core][spaces][identity]") {
  const auto a = sp("a", 0, 0, 0, 4, 4).polygon;
  const auto b = sp("b", 0, 3, 1, 4, 4).polygon; // overlap 1 x 3
  CHECK_THAT(space_overlap_area(a, b), WithinAbs(3.0, 0.02));
  CHECK_THAT(space_overlap_area(a, a), WithinAbs(16.0, 0.02));
  CHECK(space_overlap_area(a, sp("c", 0, 5, 5, 1, 1).polygon) == 0.0);
  // A hole in a is not overlap; ring orientation does not matter.
  auto holed = a;
  holed.holes.push_back({{1, 1}, {1, 2}, {2, 2}, {2, 1}});
  CHECK_THAT(space_overlap_area(holed, a), WithinAbs(15.0, 0.02));
  auto reversed = b;
  std::reverse(reversed.outer.begin(), reversed.outer.end());
  CHECK_THAT(space_overlap_area(a, reversed), WithinAbs(3.0, 0.02));
}

TEST_CASE("SpaceIdentity_LockedRoomRedetected_IsKeptAsStored",
          "[core][spaces][identity]") {
  // reusex-s4mm: a long corridor, named and locked by hand, with a sloped
  // ceiling of its own; re-detected 1.5 m longer at one end with another
  // ceiling. Same room by overlap, so the detection is dropped and the
  // stored row is kept: outline, heights and ceiling planes as stored. Only
  // the link into today's rooms cloud (room_label) is the detection's.
  auto stored = sp("corridor-guid", 0, 0, 0, 10, 2);
  stored.name = "S1-12 Gang";
  stored.kind = "corridor";
  stored.source = "manual";
  stored.locked = true;
  stored.z_floor = 0.1;
  stored.z_ceiling = 2.4;
  stored.room_label = 7;
  stored.ceiling_planes = {CeilingPlane{{1.0, 1.0, 2.2}, {0.1, 0.0}}};
  stored.metadata = R"({"note":"by hand","pure_share":0.4,"unmatched":true})";
  auto fresh_c = sp("p1", 0, 0, 0, 11.5, 2);
  fresh_c.name = "Space 3";
  fresh_c.kind = "room";
  fresh_c.z_ceiling = 2.6;
  fresh_c.room_label = 3;
  fresh_c.metadata = R"({"pure_share":0.9})";
  std::vector<SpaceRecord> fresh{fresh_c, sp("p2", 0, 0, 3, 4, 4)};
  std::vector<SpaceBoundaryRecord> bounds{{"p1", "w", "a", 0, 10},
                                          {"p2", "w", "a", 0, 4}};
  const auto r = assign_space_guids(fresh, bounds, {stored},
                                    {storey("st0", 0, -2.2, -1.7)}, kProject);
  CHECK(r.matched == 1);
  CHECK(r.locked_matched == 1);
  CHECK(r.named_matched == 1);
  CHECK(r.minted == 1);
  CHECK(r.unmatched_locked == 0);
  REQUIRE(fresh.size() == 1); // only the other room is new
  CHECK(fresh[0].name.empty());
  CHECK_FALSE(fresh[0].locked);
  REQUIRE(bounds.size() == 1);
  CHECK(bounds[0].space == fresh[0].guid);
  REQUIRE(r.kept.size() == 1);
  const auto &k = r.kept[0];
  CHECK(k.guid == "corridor-guid");
  CHECK(k.name == "S1-12 Gang");
  CHECK(k.kind == "corridor");
  CHECK(k.source == "manual");
  CHECK(k.locked);
  CHECK(k.polygon.outer == stored.polygon.outer);
  CHECK(k.z_floor == stored.z_floor);
  CHECK(k.z_ceiling == stored.z_ceiling);
  REQUIRE(k.ceiling_planes.size() == 1);
  CHECK(k.ceiling_planes[0].at == stored.ceiling_planes[0].at);
  CHECK(k.ceiling_planes[0].slope == stored.ceiling_planes[0].slope);
  CHECK(k.room_label == 3); // today's room
  const auto md = nlohmann::json::parse(k.metadata);
  CHECK(md.at("note") == "by hand");
  CHECK(md.at("pure_share") == 0.4); // the stored row's, not the detection's
  CHECK_FALSE(md.contains("unmatched")); // it matched
}

TEST_CASE("SpaceIdentity_NamedUnlockedRoomRedetected_InheritsNameAndKind",
          "[core][spaces][identity]") {
  // The same corridor named but unlocked (`rux set spaces.<id>.locked
  // false` after naming): the name, kind and the person's metadata keys
  // follow the new outline.
  auto stored = sp("corridor-guid", 0, 0, 0, 10, 2);
  stored.name = "S1-12 Gang";
  stored.kind = "corridor";
  stored.source = "manual";
  stored.locked = false;
  stored.metadata = R"({"note":"by hand","pure_share":0.4})";
  auto fresh_c = sp("p1", 0, 0, 0, 11.5, 2);
  fresh_c.name = "Space 3";
  fresh_c.kind = "room";
  fresh_c.metadata = R"({"pure_share":0.9})";
  std::vector<SpaceRecord> fresh{fresh_c, sp("p2", 0, 0, 3, 4, 4)};
  std::vector<SpaceBoundaryRecord> none;
  const auto r = assign_space_guids(fresh, none, {stored},
                                    {storey("st0", 0, -2.2, -1.7)}, kProject);
  CHECK(r.matched == 1);
  CHECK(r.locked_matched == 0);
  CHECK(r.named_matched == 1);
  CHECK(r.kept.empty());
  REQUIRE(fresh.size() == 2);
  CHECK(fresh[0].guid == "corridor-guid");
  CHECK(fresh[0].name == "S1-12 Gang");
  CHECK(fresh[0].kind == "corridor");
  CHECK(fresh[0].source == "manual");
  CHECK_FALSE(fresh[0].locked);
  CHECK(fresh[0].polygon.outer == fresh_c.polygon.outer); // the new outline
  const auto md = nlohmann::json::parse(fresh[0].metadata);
  CHECK(md.at("note") == "by hand"); // the person's key stays
  CHECK(md.at("pure_share") == 0.9); // the detection's own is the new one
  CHECK_FALSE(md.contains("unmatched"));
  CHECK(fresh[1].name.empty()); // the other one is new
  CHECK_FALSE(fresh[1].locked);
}

TEST_CASE("SpaceIdentity_UnmatchedLocked_IsKeptAndFlagged",
          "[core][spaces][identity]") {
  auto named = sp("named-guid", 0, 20, 20, 3, 3);
  named.name = "S0-01 Depot";
  named.locked = true;
  named.source = "manual";
  auto plain = sp("plain-guid", 0, 30, 30, 3, 3); // detected, not locked
  std::vector<SpaceRecord> fresh{sp("p1", 0, 0, 0, 4, 4)};
  std::vector<SpaceBoundaryRecord> none;
  const auto r = assign_space_guids(fresh, none, {named, plain}, {}, kProject);
  CHECK(r.matched == 0);
  REQUIRE(r.kept.size() == 1);
  CHECK(r.kept[0].guid == "named-guid");
  CHECK(r.kept[0].name == "S0-01 Depot");
  CHECK(r.kept[0].locked);
  CHECK(nlohmann::json::parse(r.kept[0].metadata).at("unmatched") == true);

  CHECK(r.unmatched_locked == 1);

  // Matched again on a later run: kept as stored, and the flag goes.
  auto flagged = r.kept[0];
  std::vector<SpaceRecord> again{sp("p1", 0, 20.1, 20, 3, 3)};
  const auto r2 = assign_space_guids(again, none, {flagged}, {}, kProject);
  CHECK(r2.matched == 1);
  CHECK(r2.unmatched_locked == 0);
  CHECK(again.empty());
  REQUIRE(r2.kept.size() == 1);
  CHECK(r2.kept[0].guid == "named-guid");
  CHECK(r2.kept[0].polygon.outer == named.polygon.outer);
  CHECK(r2.kept[0].metadata.find("unmatched") == std::string::npos);
}

TEST_CASE("SpaceIdentity_TwoNewOverOneStored_NoDuplicateIdentity",
          "[core][spaces][identity]") {
  // Two stored named rooms side by side; detection now finds them shifted
  // by 0.4 m plus a third sliver: each stored name goes to exactly one new
  // space, and no stored row survives next to its successor.
  auto a = sp("a-guid", 0, 0, 0, 4, 4);
  a.name = "A";
  a.source = "manual";
  auto b = sp("b-guid", 0, 4, 0, 4, 4);
  b.name = "B";
  b.source = "manual";
  std::vector<SpaceRecord> fresh{sp("p1", 0, 0.4, 0, 4, 4),
                                 sp("p2", 0, 4.4, 0, 4, 4),
                                 sp("p3", 0, 8.4, 0, 0.5, 4)};
  std::vector<SpaceBoundaryRecord> none;
  const auto r = assign_space_guids(fresh, none, {a, b}, {}, kProject);
  CHECK(r.matched == 2);
  CHECK(r.kept.empty());
  CHECK(fresh[0].name == "A");
  CHECK(fresh[1].name == "B");
  CHECK(fresh[2].name.empty());
  std::set<std::string> ids;
  for (const auto &f : fresh)
    ids.insert(f.guid);
  CHECK(ids.size() == 3);
}

TEST_CASE("SpaceIdentity_OtherStorey_NeverMatches",
          "[core][spaces][identity]") {
  auto stored = sp("s-guid", 1, 0, 0, 4, 4);
  stored.locked = true;
  stored.name = "Upstairs";
  std::vector<SpaceRecord> fresh{sp("p1", 0, 0, 0, 4, 4)};
  std::vector<SpaceBoundaryRecord> none;
  const auto r = assign_space_guids(fresh, none, {stored}, {}, kProject);
  CHECK(r.matched == 0);
  CHECK(r.kept.size() == 1);
  CHECK(fresh[0].name.empty());
}

TEST_CASE("SpaceIdentity_Reframe_KeepsWorldCoordinatesAndFollowsTheGuid",
          "[core][spaces][identity]") {
  // Old storeys: one at idx 0, world -> level translates by (-1, -1).
  // New storeys: a basement found below, so the same storey (same GUID) is
  // idx 1, and its frame origin moved: world -> level translates by
  // (-3.2, -2.7, -3.0).
  const auto old0 = storey("ground", 0, -1, -1);
  const auto new0 = storey("basement", 0, -3.2, -2.7, 0.0);
  auto new1 = storey("ground", 1, -3.2, -2.7, -3.0);
  new1.z_floor = 0.0;
  auto s = sp("room", 0, 1, 1, 4, 3);
  s.name = "S1-01 Kontor";
  s.locked = true;
  s.z_floor = 0.0;
  s.z_ceiling = 2.5;
  const auto world_before = space_polygon_world(s, old0);

  std::vector<SpaceRecord> spaces{s};
  const auto r = reframe_spaces(spaces, {old0}, {new0, new1});
  CHECK(r.moved == 1);
  CHECK(r.lost.empty());
  CHECK(spaces[0].storey == 1);
  CHECK(spaces[0].name == "S1-01 Kontor");
  CHECK_THAT(spaces[0].z_floor, WithinAbs(-3.0, 1e-9));
  CHECK_THAT(spaces[0].z_ceiling, WithinAbs(-0.5, 1e-9));
  CHECK_THAT(spaces[0].polygon.outer[0][0], WithinAbs(1 + 1 - 3.2, 1e-9));
  CHECK_THAT(spaces[0].polygon.outer[0][1], WithinAbs(1 + 1 - 2.7, 1e-9));
  const auto world_after = space_polygon_world(spaces[0], new1);
  for (size_t i = 0; i < world_before.outer.size(); ++i) {
    CHECK_THAT(world_after.outer[i][0],
               WithinAbs(world_before.outer[i][0], 1e-9));
    CHECK_THAT(world_after.outer[i][1],
               WithinAbs(world_before.outer[i][1], 1e-9));
  }

  // Same frame, same idx: untouched.
  std::vector<SpaceRecord> same{s};
  const auto r2 = reframe_spaces(same, {old0}, {old0});
  CHECK(r2.moved == 0);
  CHECK(r2.unchanged == 1);
  CHECK(same[0].polygon.outer == s.polygon.outer);

  // Its storey vanished and no new storey's band holds it: left as it is.
  std::vector<SpaceRecord> lost{s};
  auto high = storey("high", 0, 0, 0, -50.0);
  const auto r3 = reframe_spaces(lost, {old0}, {high});
  CHECK(r3.moved == 0);
  REQUIRE(r3.lost.size() == 1);
  CHECK(r3.lost[0] == "room");
  CHECK(lost[0].polygon.outer == s.polygon.outer);
}

TEST_CASE("SpaceIdentity_Reframe_StoreyGoneButBandHoldsIt",
          "[core][spaces][identity]") {
  const auto old0 = storey("old-ground", 0, 0, 0);
  const auto replacement = storey("new-ground", 0, -0.5, 0.25);
  auto s = sp("room", 0, 1, 1, 2, 2);
  s.z_floor = 0.0;
  s.z_ceiling = 2.5;
  std::vector<SpaceRecord> spaces{s};
  const auto r = reframe_spaces(spaces, {old0}, {replacement});
  CHECK(r.moved == 1);
  CHECK(r.lost.empty());
  CHECK(spaces[0].storey == 0);
  CHECK_THAT(spaces[0].polygon.outer[0][0], WithinAbs(0.5, 1e-9));
  CHECK_THAT(spaces[0].polygon.outer[0][1], WithinAbs(1.25, 1e-9));
}
