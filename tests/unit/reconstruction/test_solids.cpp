// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Typed solids (W6.1, reusex-z1v.8.1): a wall with a door and a window cut
// out of it, a slab, a space volume -- each closed, with the volume known in
// closed form.

#include <reusex/core/sloped_ceiling.hpp>
#include <reusex/core/solid_model.hpp>
#include <reusex/io/solid_model_json.hpp>
#include <reusex/reconstruction/solids.hpp>

#include "../../support/source_dir.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Side_of_triangle_mesh.h>
#include <CGAL/Surface_mesh.h>

#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

using Catch::Matchers::WithinAbs;
using namespace reusex;
using geometry::build_solids;
using geometry::ElementSolid;
using geometry::SolidsOptions;
using geometry::SolidType;

namespace {

using K = CGAL::Exact_predicates_inexact_constructions_kernel;
using Mesh = CGAL::Surface_mesh<K::Point_3>;
namespace PMP = CGAL::Polygon_mesh_processing;

Mesh to_cgal(const pcl::PolygonMesh &m) {
  pcl::PointCloud<pcl::PointXYZ> pts;
  pcl::fromPCLPointCloud2(m.cloud, pts);
  Mesh out;
  std::vector<Mesh::Vertex_index> v;
  for (const auto &p : pts)
    v.push_back(out.add_vertex(K::Point_3(p.x, p.y, p.z)));
  for (const auto &f : m.polygons) {
    REQUIRE(f.vertices.size() == 3);
    REQUIRE(out.add_face(v[f.vertices[0]], v[f.vertices[1]],
                         v[f.vertices[2]]) != Mesh::null_face());
  }
  return out;
}

/// Inside / outside the closed mesh of @p s.
CGAL::Bounded_side side(const ElementSolid &s, double x, double y, double z) {
  const Mesh m = to_cgal(s.mesh);
  CGAL::Side_of_triangle_mesh<Mesh, K> inside(m);
  return inside(K::Point_3(x, y, z));
}

core::StoreyRecord storey(int idx, double z_floor, double z_ceiling) {
  core::StoreyRecord s;
  s.idx = idx;
  s.guid = "st" + std::to_string(idx);
  s.z_floor = z_floor;
  s.z_ceiling = z_ceiling;
  return s;
}

core::WallRecord wall(const std::string &guid, double x0, double y0, double x1,
                      double y1, double thickness, double height = 2.5) {
  core::WallRecord w;
  w.guid = guid;
  w.x0 = x0;
  w.y0 = y0;
  w.x1 = x1;
  w.y1 = y1;
  w.thickness = thickness;
  w.height = height;
  return w;
}

core::OpeningRecord opening(const std::string &guid, const std::string &host,
                            const std::string &kind, double s0, double s1,
                            double z0, double z1) {
  core::OpeningRecord o;
  o.guid = guid;
  o.wall = host;
  o.kind = kind;
  o.s_start = s0;
  o.s_end = s1;
  o.z_sill = z0;
  o.z_head = z1;
  return o;
}

core::SpaceRecord space(const std::string &guid,
                        std::vector<core::PlanPoint> outer, double z0,
                        double z1) {
  core::SpaceRecord s;
  s.guid = guid;
  s.polygon.outer = std::move(outer);
  s.z_floor = z0;
  s.z_ceiling = z1;
  s.kind = "room";
  return s;
}

} // namespace

TEST_CASE("Solids_Wall_IsAClosedBox", "[reconstruction][solids]") {
  core::SolidModel m;
  m.storeys = {storey(0, 0.0, 2.5)};
  m.walls = {wall("w1", 0, 0, 4, 0, 0.2)};
  const auto r = build_solids(m);
  REQUIRE(r.walls.size() == 1);
  const auto &w = r.walls[0];
  CHECK(w.type == SolidType::wall);
  CHECK(w.source_guid == "w1");
  CHECK(w.predefined == "STANDARD");
  CHECK(w.closed);
  CHECK_THAT(w.volume, WithinAbs(4.0 * 0.2 * 2.5, 1e-6));
  CHECK(side(w, 2.0, 0.0, 1.0) == CGAL::ON_BOUNDED_SIDE);
  CHECK(side(w, 2.0, 0.2, 1.0) == CGAL::ON_UNBOUNDED_SIDE);
  // The IFC-ready profile: a length x thickness rectangle, extruded by the
  // height, placed at the axis start on the floor.
  CHECK(w.profile.outer.size() == 4);
  CHECK_THAT(w.profile.depth, WithinAbs(2.5, 1e-9));
  CHECK(w.voids.empty());
}

TEST_CASE("Solids_WallWithDoorAndWindow_LeavesHoles",
          "[reconstruction][solids]") {
  core::SolidModel m;
  m.storeys = {storey(0, 0.0, 2.5)};
  m.walls = {wall("w1", 0, 0, 4, 0, 0.2)};
  m.openings = {opening("d1", "w1", "door", 0.5, 1.4, 0.0, 2.1),
                opening("o1", "w1", "window", 2.5, 3.5, 0.9, 2.0)};
  const auto r = build_solids(m);
  REQUIRE(r.walls.size() == 1);
  const auto &w = r.walls[0];
  CHECK(r.openings_cut == 2);
  CHECK(r.openings_failed == 0);
  CHECK(w.closed);
  CHECK(w.voids_failed == 0);
  const double box = 4.0 * 0.2 * 2.5;
  const double door = 0.9 * 0.2 * 2.1;
  const double window = 1.0 * 0.2 * 1.1;
  CHECK_THAT(w.volume, WithinAbs(box - door - window, 1e-6));
  // Through the door and the window: no wall. Below the window's sill and
  // above the door's head: wall.
  CHECK(side(w, 1.0, 0.0, 1.0) == CGAL::ON_UNBOUNDED_SIDE);
  CHECK(side(w, 1.0, 0.0, 0.01) == CGAL::ON_UNBOUNDED_SIDE); // door to floor
  CHECK(side(w, 3.0, 0.0, 1.5) == CGAL::ON_UNBOUNDED_SIDE);
  CHECK(side(w, 3.0, 0.0, 0.5) == CGAL::ON_BOUNDED_SIDE);
  CHECK(side(w, 1.0, 0.0, 2.3) == CGAL::ON_BOUNDED_SIDE);
  // The voids are kept for IFC, door first.
  REQUIRE(w.voids.size() == 2);
  CHECK(w.voids[0].guid == "d1");
  CHECK(w.voids[0].kind == "door");
  CHECK(w.voids[1].guid == "o1");
  CHECK(w.voids[0].profile.depth > 0.2);
}

TEST_CASE("Solids_Opening_OrphanOrOutside_IsSkipped",
          "[reconstruction][solids]") {
  core::SolidModel m;
  m.storeys = {storey(0, 0.0, 2.5)};
  m.walls = {wall("w1", 0, 0, 4, 0, 0.2)};
  m.openings = {opening("x", "nope", "door", 0.5, 1.4, 0.0, 2.1),
                opening("y", "w1", "window", 5.0, 6.0, 0.9, 2.0)};
  const auto r = build_solids(m);
  REQUIRE(r.walls.size() == 1);
  CHECK(r.openings_skipped == 2);
  CHECK(r.walls[0].voids.empty());
  CHECK_THAT(r.walls[0].volume, WithinAbs(2.0, 1e-6));
}

TEST_CASE("Solids_UnknownThickness_GrowsAwayFromTheSpace",
          "[reconstruction][solids]") {
  // The wall's axis is its seen face at y = 0; the room is at y > 0, so the
  // body is at y < 0.
  core::SolidModel m;
  m.storeys = {storey(0, 0.0, 2.5)};
  m.walls = {wall("w1", 0, 0, 4, 0, -1.0)};
  m.spaces = {space("s1", {{0, 0}, {4, 0}, {4, 3}, {0, 3}}, 0.0, 2.5)};
  SolidsOptions o;
  o.unknown_wall_thickness_m = 0.1;
  const auto r = build_solids(m, o);
  REQUIRE(r.walls.size() == 1);
  CHECK(r.walls[0].closed);
  CHECK(side(r.walls[0], 2.0, -0.05, 1.0) == CGAL::ON_BOUNDED_SIDE);
  CHECK(side(r.walls[0], 2.0, 0.05, 1.0) == CGAL::ON_UNBOUNDED_SIDE);
}

TEST_CASE("Solids_CornerJoin_ExtendsTheWallEnds", "[reconstruction][solids]") {
  core::SolidModel m;
  m.storeys = {storey(0, 0.0, 2.5)};
  m.walls = {wall("a", 0, 0, 4, 0, 0.2), wall("b", 4, 0, 4, 3, 0.2)};
  m.wall_joins = {{"a", "b", "corner", "end", "start"}};
  const auto r = build_solids(m);
  REQUIRE(r.walls.size() == 2);
  // Both reach the outer corner (4.1, -0.1).
  CHECK(side(r.walls[0], 4.05, -0.05, 1.0) == CGAL::ON_BOUNDED_SIDE);
  CHECK(side(r.walls[1], 4.05, -0.05, 1.0) == CGAL::ON_BOUNDED_SIDE);
  SolidsOptions o;
  o.extend_at_joins = false;
  const auto r2 = build_solids(m, o);
  CHECK(side(r2.walls[0], 4.05, -0.05, 1.0) == CGAL::ON_UNBOUNDED_SIDE);
}

TEST_CASE("Solids_Space_IsItsPolygonTimesItsHeight",
          "[reconstruction][solids]") {
  // An L-shaped room with a column hole.
  core::SolidModel m;
  m.storeys = {storey(0, 0.0, 2.5)};
  auto s =
      space("s1", {{0, 0}, {6, 0}, {6, 3}, {3, 3}, {3, 5}, {0, 5}}, 0.0, 2.5);
  s.polygon.holes = {{{1, 1}, {1.5, 1}, {1.5, 1.5}, {1, 1.5}}};
  m.spaces = {s};
  const auto r = build_solids(m);
  REQUIRE(r.spaces.size() == 1);
  const auto &e = r.spaces[0];
  CHECK(e.type == SolidType::space);
  CHECK(e.source_guid == "s1");
  CHECK(e.predefined == "room");
  CHECK(e.closed);
  const double area = 6 * 3 + 3 * 2 - 0.25;
  CHECK_THAT(e.volume, WithinAbs(area * 2.5, 1e-6));
  CHECK(side(e, 4.0, 4.0, 1.0) == CGAL::ON_UNBOUNDED_SIDE);   // the L's notch
  CHECK(side(e, 1.25, 1.25, 1.0) == CGAL::ON_UNBOUNDED_SIDE); // the column
  CHECK(side(e, 2.0, 2.0, 1.0) == CGAL::ON_BOUNDED_SIDE);
  CHECK(e.profile.holes.size() == 1);
  CHECK_THAT(e.profile.depth, WithinAbs(2.5, 1e-9));
}

TEST_CASE("Solids_Slab_FloorPolygonTimesTheGapBelow",
          "[reconstruction][solids]") {
  // Two storeys: the upper floor's slab fills the gap from the lower
  // ceiling (2.6) to its floor (3.0); the ground floor and the roof get the
  // default thickness.
  core::SolidModel m;
  m.storeys = {storey(0, 0.0, 2.6), storey(1, 3.0, 5.6)};
  auto s0 = space("s0", {{0, 0}, {4, 0}, {4, 3}, {0, 3}}, 0.0, 2.6);
  auto s1 = space("s1", {{0, 0}, {4, 0}, {4, 3}, {0, 3}}, 3.0, 5.6);
  s1.storey = 1;
  m.spaces = {s0, s1};
  SolidsOptions o;
  o.slab_thickness_m = 0.25;
  const auto r = build_solids(m, o);
  REQUIRE(r.slabs.size() == 3);
  const auto &ground = r.slabs[0];
  const auto &upper = r.slabs[1];
  const auto &roof = r.slabs[2];
  CHECK(ground.source_guid == "st0");
  CHECK(ground.predefined == "FLOOR");
  CHECK(upper.source_guid == "st1");
  CHECK(roof.predefined == "ROOF");
  for (const auto *s : {&ground, &upper, &roof})
    CHECK(s->closed);
  CHECK_THAT(ground.volume, WithinAbs(12.0 * 0.25, 1e-6));
  CHECK_THAT(upper.volume, WithinAbs(12.0 * 0.4, 1e-6));
  CHECK_THAT(roof.volume, WithinAbs(12.0 * 0.25, 1e-6));
  CHECK(side(ground, 2, 1.5, -0.1) == CGAL::ON_BOUNDED_SIDE);
  CHECK(side(upper, 2, 1.5, 2.8) == CGAL::ON_BOUNDED_SIDE);
  CHECK(side(roof, 2, 1.5, 5.7) == CGAL::ON_BOUNDED_SIDE);
}

TEST_CASE("Solids_Slab_UnionOfSpacesAndWalls", "[reconstruction][solids]") {
  // Two rooms either side of a 0.2 m partition: the slab is one piece under
  // both rooms and the wall, 4.2 x 3.
  core::SolidModel m;
  m.storeys = {storey(0, 0.0, 2.5)};
  m.spaces = {space("a", {{0, 0}, {2, 0}, {2, 3}, {0, 3}}, 0.0, 2.5),
              space("b", {{2.2, 0}, {4.2, 0}, {4.2, 3}, {2.2, 3}}, 0.0, 2.5)};
  m.walls = {wall("p", 2.1, 0, 2.1, 3, 0.2)};
  SolidsOptions o;
  o.roof = false;
  const auto r = build_solids(m, o);
  REQUIRE(r.slabs.size() == 1);
  CHECK_THAT(r.slabs[0].volume, WithinAbs(4.2 * 3 * 0.2, 1e-6));
}

TEST_CASE("Solids_Slab_StoreyHullWithoutSpaces", "[reconstruction][solids]") {
  core::SolidModel m;
  m.storeys = {storey(0, 0.0, 2.5)};
  m.walls = {wall("a", 0, 0, 4, 0, 0.2), wall("b", 4, 0, 4, 3, 0.2),
             wall("c", 4, 3, 0, 3, 0.2), wall("d", 0, 3, 0, 0, 0.2)};
  SolidsOptions o;
  o.roof = false;
  o.extend_at_joins = false;
  const auto r = build_solids(m, o);
  REQUIRE(r.slabs.size() == 1);
  CHECK(r.slabs[0].closed);
  // The hull of the four footprints: 4.2 x 3.2 less the four 0.1 x 0.1
  // corner triangles the unextended walls leave.
  CHECK_THAT(r.slabs[0].volume, WithinAbs((4.2 * 3.2 - 0.02) * 0.2, 1e-6));
}

TEST_CASE("Solids_LevelFrame_IsMappedToWorld", "[reconstruction][solids]") {
  // world -> level: z - 10, so level z 0 is world z 10.
  core::SolidModel m;
  auto st = storey(0, 0.0, 2.5);
  st.level_transform[11] = -10.0;
  m.storeys = {st};
  m.walls = {wall("w1", 0, 0, 4, 0, 0.2)};
  const auto r = build_solids(m);
  REQUIRE(r.walls.size() == 1);
  CHECK(side(r.walls[0], 2.0, 0.0, 11.0) == CGAL::ON_BOUNDED_SIDE);
  CHECK(side(r.walls[0], 2.0, 0.0, 1.0) == CGAL::ON_UNBOUNDED_SIDE);
  // The placement carries the frame too.
  CHECK_THAT(r.walls[0].profile.placement[11], WithinAbs(10.0, 1e-9));
}

TEST_CASE("Solids_DegenerateElements_AreSkipped", "[reconstruction][solids]") {
  core::SolidModel m;
  m.storeys = {storey(0, 0.0, 2.5)};
  m.walls = {wall("z", 1, 1, 1, 1, 0.2), wall("h", 0, 0, 1, 0, 0.2, 0.0)};
  m.spaces = {space("flat", {{0, 0}, {1, 0}, {1, 1}}, 1.0, 1.0)};
  SolidsOptions o;
  o.roof = false;
  const auto r = build_solids(m, o);
  CHECK(r.walls.empty());
  CHECK(r.walls_skipped == 2);
  CHECK(r.spaces.empty());
  CHECK(r.spaces_skipped == 1);
}

// ---------------------------------------------------------------------------
// Split levels (reusex-546)
// ---------------------------------------------------------------------------

namespace {

/// The z range of an element's mesh.
std::pair<double, double> z_range(const ElementSolid &e) {
  pcl::PointCloud<pcl::PointXYZ> pts;
  pcl::fromPCLPointCloud2(e.mesh.cloud, pts);
  double lo = 1e9, hi = -1e9;
  for (const auto &p : pts) {
    lo = std::min(lo, static_cast<double>(p.z));
    hi = std::max(hi, static_cast<double>(p.z));
  }
  return {lo, hi};
}

const ElementSolid &wall_solid(const geometry::SolidsResult &r,
                               const std::string &guid) {
  for (const auto &w : r.walls)
    if (w.source_guid == guid)
      return w;
  FAIL("no wall solid " << guid);
  return r.walls.front();
}

bool inside_any(const std::vector<const ElementSolid *> &v, double x, double y,
                double z) {
  for (const auto *e : v)
    if (side(*e, x, y, z) == CGAL::ON_BOUNDED_SIDE)
      return true;
  return false;
}

/// One storey (floor 0, ceiling 3.4, the walls table's storey-wide heights)
/// whose east half is raised 0.85 m: the lower room (0-4.9 x 0-4) has its
/// ceiling at 2.7, the upper (5.1-10 x 0-4) at 3.4, a 0.2 m wall between
/// them at x = 5, and a south wall along each half.
core::SolidModel two_levels() {
  core::SolidModel m;
  m.storeys = {storey(0, 0.0, 3.4)};
  auto lo = space("lo", {{0, 0}, {4.9, 0}, {4.9, 4}, {0, 4}}, 0.0, 2.7);
  auto hi = space("hi", {{5.1, 0}, {10, 0}, {10, 4}, {5.1, 4}}, 0.85, 3.4);
  lo.area = hi.area = 4.9 * 4;
  m.spaces = {lo, hi};
  m.walls = {wall("split", 5, 0, 5, 4, 0.2, 3.4),
             wall("south-lo", 0, -0.1, 4.9, -0.1, 0.2, 3.4),
             wall("south-hi", 5.1, -0.1, 10, -0.1, 0.2, 3.4)};
  return m;
}

} // namespace

TEST_CASE("Solids_SplitLevel_WallsSpanTheirSpaces",
          "[reconstruction][solids][split_level]") {
  SolidsOptions o;
  o.extend_at_joins = false;
  const auto r = build_solids(two_levels(), o);
  REQUIRE(r.walls.size() == 3);
  // Between the levels: from the lower floor to the higher ceiling.
  const auto split = z_range(wall_solid(r, "split"));
  CHECK_THAT(split.first, WithinAbs(0.0, 1e-6));
  CHECK_THAT(split.second, WithinAbs(3.4, 1e-6));
  // Beside one level only: that level's floor and ceiling.
  const auto low = z_range(wall_solid(r, "south-lo"));
  CHECK_THAT(low.first, WithinAbs(0.0, 1e-6));
  CHECK_THAT(low.second, WithinAbs(2.7, 1e-6));
  const auto high = z_range(wall_solid(r, "south-hi"));
  CHECK_THAT(high.first, WithinAbs(0.85, 1e-6));
  CHECK_THAT(high.second, WithinAbs(3.4, 1e-6));
  for (const auto &w : r.walls)
    CHECK(w.closed);
  CHECK(r.not_closed == 0);

  // A locked wall keeps its own heights.
  auto m = two_levels();
  m.walls[2].locked = true;
  const auto locked = z_range(wall_solid(build_solids(m, o), "south-hi"));
  CHECK_THAT(locked.first, WithinAbs(0.0, 1e-6));
  CHECK_THAT(locked.second, WithinAbs(3.4, 1e-6));
}

TEST_CASE("Solids_SplitLevel_SlabsAndRoofsStep",
          "[reconstruction][solids][split_level]") {
  SolidsOptions o;
  o.extend_at_joins = false;
  o.slab_thickness_m = 0.2;
  const auto r = build_solids(two_levels(), o);
  std::vector<const ElementSolid *> floors, roofs;
  for (const auto &s : r.slabs) {
    CHECK(s.closed);
    (s.predefined == "FLOOR" ? floors : roofs).push_back(&s);
  }
  REQUIRE(floors.size() == 2);
  REQUIRE(roofs.size() == 2);
  // Lowest level first: the lower slab 0.2 m thick under the lower room and
  // the split wall, the raised one filled from -0.2 up to 0.85.
  CHECK_THAT(z_range(*floors[0]).second, WithinAbs(0.0, 1e-6));
  CHECK_THAT(z_range(*floors[1]).first, WithinAbs(-0.2, 1e-6));
  CHECK_THAT(z_range(*floors[1]).second, WithinAbs(0.85, 1e-6));
  CHECK(inside_any(floors, 7, 2, 0.5));  // the fill under the raised floor
  CHECK(inside_any(floors, 5, 2, -0.1)); // under the split wall
  CHECK_FALSE(inside_any(floors, 2, 2, 0.1));
  CHECK_FALSE(inside_any(floors, 7, 2, 0.9));
  // Roofs on each ceiling; the higher one over the split wall.
  CHECK(inside_any(roofs, 2, 2, 2.8));
  CHECK(inside_any(roofs, 7, 2, 3.5));
  CHECK(inside_any(roofs, 5, 2, 3.5));
  CHECK_FALSE(inside_any(roofs, 7, 2, 2.8));
  CHECK_FALSE(inside_any(roofs, 5, 2, 2.8));
  // Space volumes use their own heights.
  REQUIRE(r.spaces.size() == 2);
  CHECK_THAT(r.spaces[0].volume, WithinAbs(4.9 * 4 * 2.7, 1e-6));
  CHECK_THAT(r.spaces[1].volume, WithinAbs(4.9 * 4 * 2.55, 1e-6));
  // Watertight floor: every plan point of the footprint has a floor slab
  // just below its level's floor.
  for (double x = 0.2; x < 10; x += 0.4)
    for (double y = 0.2; y < 4; y += 0.6)
      CHECK(inside_any(floors, x, y, (x < 5.1 ? 0.0 : 0.85) - 0.1));
}

TEST_CASE("Solids_FlatStorey_SpaceHeightsNearTheStoreyKeepIt",
          "[reconstruction][solids][split_level]") {
  // Measured floors / ceilings a centimetre off the storey's are the
  // storey's level: one slab, one roof, walls at the storey's heights.
  core::SolidModel m;
  m.storeys = {storey(0, 0.0, 2.6)};
  auto a = space("a", {{0, 0}, {2, 0}, {2, 3}, {0, 3}}, 0.012, 2.59);
  auto b = space("b", {{2.2, 0}, {4.2, 0}, {4.2, 3}, {2.2, 3}}, -0.008, 2.63);
  a.area = b.area = 6.0;
  m.spaces = {a, b};
  m.walls = {wall("p", 2.1, 0, 2.1, 3, 0.2, 2.6)};
  const auto r = build_solids(m, SolidsOptions{});
  REQUIRE(r.slabs.size() == 2);
  CHECK_THAT(z_range(r.slabs[0]).second, WithinAbs(0.0, 1e-6));
  CHECK_THAT(z_range(r.slabs[1]).first, WithinAbs(2.6, 1e-6));
  const auto p = z_range(r.walls[0]);
  CHECK_THAT(p.first, WithinAbs(0.0, 1e-6));
  CHECK_THAT(p.second, WithinAbs(2.6, 1e-6));
}

// ---------------------------------------------------------------------------
// Sloped ceilings and wall tops (reusex-546), on the synthetic attic plan
// (tests/fixtures/solid_model/attic.plan.json): a 35 deg pitched attic room
// with 1.15 m knee walls and gable walls, and a room with a flat centre
// (2.2 m) and sloped sides.

namespace {

core::SolidModel attic_plan() {
  return io::solid_model_from_file(test_support::source_dir() / "tests" /
                                   "fixtures" / "solid_model" /
                                   "attic.plan.json");
}

const ElementSolid &space_solid(const geometry::SolidsResult &r,
                                const std::string &guid) {
  for (const auto &s : r.spaces)
    if (s.source_guid == guid)
      return s;
  FAIL("no space solid " << guid);
  return r.spaces.front();
}

/// The ceiling height of @p s integrated over its (rectangular) polygon,
/// minus the floor: the volume a clipped prism must have.
double rect_volume(const core::SpaceRecord &s) {
  double x0 = 1e9, y0 = 1e9, x1 = -1e9, y1 = -1e9;
  for (const auto &p : s.polygon.outer) {
    x0 = std::min(x0, p[0]);
    y0 = std::min(y0, p[1]);
    x1 = std::max(x1, p[0]);
    y1 = std::max(y1, p[1]);
  }
  const int n = 2000;
  double v = 0.0;
  for (int i = 0; i < n; ++i) {
    const double y = y0 + (y1 - y0) * (i + 0.5) / n;
    v += (core::ceiling_at(s, 0.5 * (x0 + x1), y) - s.z_floor) * (y1 - y0) / n;
  }
  return v * (x1 - x0);
}

} // namespace

TEST_CASE("Solids_SlopedCeiling_SpaceIsCutByItsPlanes",
          "[reconstruction][solids][sloped]") {
  const auto m = attic_plan();
  const auto r = build_solids(m);
  CHECK(r.not_closed == 0);
  const ElementSolid attic = space_solid(r, "s-attic");
  REQUIRE(attic.closed);
  CHECK_THAT(attic.volume, WithinAbs(rect_volume(m.spaces[0]), 1e-3));
  CHECK(attic.clip_planes.size() == 2);
  // Under the knee-wall slope and under the ridge.
  CHECK(side(attic, 3, 0.5, 1.45) == CGAL::ON_BOUNDED_SIDE);
  CHECK(side(attic, 3, 0.5, 1.55) == CGAL::ON_UNBOUNDED_SIDE);
  CHECK(side(attic, 3, 2.0, 2.50) == CGAL::ON_BOUNDED_SIDE);
  CHECK(side(attic, 3, 2.0, 2.58) == CGAL::ON_UNBOUNDED_SIDE);
  CHECK(side(attic, 3, 3.5, 1.45) == CGAL::ON_BOUNDED_SIDE);
  CHECK(side(attic, 3, 3.5, 1.55) == CGAL::ON_UNBOUNDED_SIDE);

  const ElementSolid flat = space_solid(r, "s-flat-centre");
  REQUIRE(flat.closed);
  CHECK_THAT(flat.volume, WithinAbs(rect_volume(m.spaces[1]), 1e-3));
  CHECK(side(flat, 8, 2.0, 2.15) == CGAL::ON_BOUNDED_SIDE); // flat centre
  CHECK(side(flat, 8, 2.0, 2.25) == CGAL::ON_UNBOUNDED_SIDE);
  CHECK(side(flat, 8, 0.5, 1.30) == CGAL::ON_BOUNDED_SIDE); // sloped side
  CHECK(side(flat, 8, 0.5, 1.40) == CGAL::ON_UNBOUNDED_SIDE);
}

TEST_CASE("Solids_SlopedCeiling_GableWallFollowsItsTop",
          "[reconstruction][solids][sloped]") {
  const auto r = build_solids(attic_plan());
  const auto &g = wall_solid(r, "w-gable-w"); // axis (0,4) -> (0,0)
  REQUIRE(g.closed);
  CHECK(side(g, 0, 2.0, 2.50) == CGAL::ON_BOUNDED_SIDE); // the peak
  CHECK(side(g, 0, 2.0, 2.60) == CGAL::ON_UNBOUNDED_SIDE);
  CHECK(side(g, 0, 0.5, 1.45) == CGAL::ON_BOUNDED_SIDE);
  CHECK(side(g, 0, 0.5, 1.55) == CGAL::ON_UNBOUNDED_SIDE);
  CHECK_THAT(z_range(g).second, WithinAbs(2.5504, 1e-6));
  REQUIRE(g.top.size() >= 3);
  // Knee walls stay boxes at their own height.
  const auto &k = wall_solid(r, "w-knee-s1");
  CHECK(k.closed);
  CHECK(k.top.empty());
  CHECK_THAT(z_range(k).second, WithinAbs(1.15, 1e-6));
}

TEST_CASE("Solids_SlopedCeiling_DetectedWallTakesTheCeilingsItMeets",
          "[reconstruction][solids][sloped]") {
  // The wall between the attic and the flat-centre room as the walls stage
  // writes it: detected, flat, at the storey height.
  auto m = attic_plan();
  for (auto &w : m.walls)
    if (w.guid == "w-gable-mid") {
      w.top.clear();
      w.source = "detected";
      w.locked = false;
    }
  for (auto &s : m.spaces)
    s.area = 22.0;
  SolidsOptions o;
  o.extend_at_joins = false;
  const auto r = build_solids(m, o);
  const auto &w = wall_solid(r, "w-gable-mid");
  REQUIRE(w.closed);
  // The attic's ceiling is the higher of the two everywhere.
  CHECK(side(w, 6, 2.0, 2.50) == CGAL::ON_BOUNDED_SIDE);
  CHECK(side(w, 6, 2.0, 2.58) == CGAL::ON_UNBOUNDED_SIDE);
  CHECK(side(w, 6, 0.5, 1.45) == CGAL::ON_BOUNDED_SIDE);
  CHECK(side(w, 6, 0.5, 1.55) == CGAL::ON_UNBOUNDED_SIDE);
  CHECK(side(w, 6, 3.5, 1.45) == CGAL::ON_BOUNDED_SIDE);
  CHECK(side(w, 6, 3.5, 1.55) == CGAL::ON_UNBOUNDED_SIDE);
  // The peak is a bend of the profile, not a sample.
  CHECK_THAT(z_range(w).second, WithinAbs(2.5504, 2e-3));
}

TEST_CASE("Solids_SlopedCeiling_RoofFollowsTheSlope",
          "[reconstruction][solids][sloped]") {
  SolidsOptions o;
  o.slab_thickness_m = 0.2;
  const auto r = build_solids(attic_plan(), o);
  std::vector<const ElementSolid *> roofs;
  for (const auto &s : r.slabs) {
    CHECK(s.closed);
    if (s.predefined == "ROOF")
      roofs.push_back(&s);
  }
  REQUIRE(roofs.size() >= 2);
  // Over the attic: just above its ceiling, not below it, not 0.2 m above.
  for (const double y : {0.5, 1.0, 2.0, 3.0, 3.5}) {
    const double h = std::min(1.15 + 0.7002 * std::min(y, 4.0 - y), 2.6);
    CHECK(inside_any(roofs, 3, y, h + 0.1));
    CHECK_FALSE(inside_any(roofs, 3, y, h - 0.05));
    CHECK_FALSE(inside_any(roofs, 3, y, h + 0.25));
  }
  // Over the flat-centre room: flat at 2.2 in the middle.
  CHECK(inside_any(roofs, 8, 2.0, 2.3));
  CHECK_FALSE(inside_any(roofs, 8, 2.0, 2.15));
  CHECK(inside_any(roofs, 8, 0.5, 1.35 + 0.1));
}

TEST_CASE("Solids_SlopedCeiling_RoofOnALowerStorey",
          "[reconstruction][solids][sloped]") {
  // A storey above (elsewhere in plan): the attic is no longer the top
  // storey, but its sloped ceiling still gets its roof; the flat-roof level
  // does not.
  auto m = attic_plan();
  m.storeys.push_back(storey(1, 3.0, 5.5));
  m.walls.push_back(wall("upper", 20, 0, 24, 0, 0.2, 2.5));
  m.walls.back().storey = 1;
  SolidsOptions o;
  o.slab_thickness_m = 0.2;
  const auto r = build_solids(m, o);
  std::vector<const ElementSolid *> roofs;
  for (const auto &s : r.slabs)
    if (s.predefined == "ROOF" && s.storey == 0)
      roofs.push_back(&s);
  REQUIRE_FALSE(roofs.empty());
  for (const auto *s : roofs)
    CHECK(s->closed);
  CHECK(inside_any(roofs, 3, 1.0, 1.15 + 0.7002 + 0.1));
  CHECK(inside_any(roofs, 8, 2.0, 2.3));
}

// Eaves (reusex-ow8p): an attic, x 0..8, y 0..6, with 45 deg slopes off a
// flat centre (2.5 m) down to 0.55 m at both facades, and a dormer in the
// south facade over x 3..5 whose flat ceiling (1.6 m) runs in to where the
// slope reaches it (y = 1.05). The spaces stage measured the knee walls and
// the dormer from the cloud (core::RoofEdges in the space's metadata).

namespace {

core::SolidModel eaves_attic(bool bay_in_room) {
  core::SolidModel m;
  m.storeys = {storey(0, 0.0, 2.6)};
  const double y0 = bay_in_room ? 0.0 : 1.0;
  auto room = space("room", {{0, y0}, {8, y0}, {8, 6}, {0, 6}}, 0.0, 2.5);
  room.area = 8.0 * (6.0 - y0);
  core::CeilingPlane south, north;
  south.at = {0, 0, 0.55};
  south.slope = {0, 1};
  north.at = {0, 6, 0.55};
  north.slope = {0, -1};
  room.ceiling_planes = {south, north};
  core::RoofEdges edges;
  edges.knee_walls = {"w-s", "w-n"};
  core::Dormer d;
  d.wall = "w-s";
  d.s0 = 3.0;
  d.s1 = 5.0;
  d.z = 1.6;
  d.outline = {{3, 0}, {5, 0}, {5, 1.05}, {3, 1.05}};
  edges.dormers = {d};
  room.metadata = core::with_roof_edges("", edges);
  m.spaces = {room};
  const auto detected = [](core::WallRecord w) {
    w.source = "detected";
    w.is_external = true;
    return w;
  };
  m.walls = {detected(wall("w-s", 0, 0, 8, 0, 0.3, 2.6)),
             detected(wall("w-n", 8, 6, 0, 6, 0.3, 2.6))};
  if (!bay_in_room) // the knee partition the bay is behind
    m.walls.push_back(detected(wall("w-p", 0, 1, 8, 1, 0.1, 2.6)));
  return m;
}

std::vector<const ElementSolid *> roofs_of(const geometry::SolidsResult &r) {
  std::vector<const ElementSolid *> v;
  for (const auto &s : r.slabs)
    if (s.predefined == "ROOF")
      v.push_back(&s);
  return v;
}

} // namespace

TEST_CASE("Solids_Eaves_KneeWallStopsAtTheSlopeAndRisesAtTheDormer",
          "[reconstruction][solids][sloped][eaves]") {
  // The bay y 0..1 belongs to no space (Sygehusgrunden's roof storey): the
  // facade is not beside the room, yet stops where its slope meets it.
  SolidsOptions o;
  o.slab_thickness_m = 0.2;
  o.extend_at_joins = false;
  const auto r = build_solids(eaves_attic(false), o);
  CHECK(r.not_closed == 0);
  const auto &ws = wall_solid(r, "w-s");
  REQUIRE(ws.closed);
  CHECK(side(ws, 1.0, 0.0, 0.50) == CGAL::ON_BOUNDED_SIDE); // knee wall
  CHECK(side(ws, 1.0, 0.0, 0.60) == CGAL::ON_UNBOUNDED_SIDE);
  CHECK(side(ws, 7.0, 0.0, 0.50) == CGAL::ON_BOUNDED_SIDE);
  CHECK(side(ws, 7.0, 0.0, 0.60) == CGAL::ON_UNBOUNDED_SIDE);
  CHECK(side(ws, 4.0, 0.0, 1.55) == CGAL::ON_BOUNDED_SIDE); // dormer
  CHECK(side(ws, 4.0, 0.0, 1.65) == CGAL::ON_UNBOUNDED_SIDE);
  CHECK_FALSE(ws.top.empty());
  // The partition and the north facade take the slope they stand under.
  const auto &wp = wall_solid(r, "w-p");
  CHECK(side(wp, 1.0, 1.0, 1.50) == CGAL::ON_BOUNDED_SIDE);
  CHECK(side(wp, 1.0, 1.0, 1.60) == CGAL::ON_UNBOUNDED_SIDE);
  const auto &wn = wall_solid(r, "w-n");
  CHECK(side(wn, 4.0, 6.0, 0.50) == CGAL::ON_BOUNDED_SIDE);
  CHECK(side(wn, 4.0, 6.0, 0.60) == CGAL::ON_UNBOUNDED_SIDE);

  const auto roofs = roofs_of(r);
  for (const auto *s : roofs)
    CHECK(s->closed);
  // The dormer's own roof, flat at its ceiling, over the bay and its wall.
  CHECK(inside_any(roofs, 4.0, 0.5, 1.7));
  CHECK(inside_any(roofs, 4.0, 0.0, 1.7));
  // The main slope is cut around it.
  CHECK_FALSE(inside_any(roofs, 4.0, 0.5, 1.15));
  CHECK_FALSE(inside_any(roofs, 4.0, 0.0, 0.65));
  // Elsewhere the roof follows the slope, over the room and the knee wall.
  CHECK(inside_any(roofs, 4.0, 1.5, 2.15));
  CHECK(inside_any(roofs, 1.0, 0.0, 0.65));
  CHECK_FALSE(inside_any(roofs, 1.0, 0.0, 0.50));
  CHECK(inside_any(roofs, 4.0, 3.0, 2.6));
}

TEST_CASE("Solids_Eaves_DormerRaisesTheRoomItIsIn",
          "[reconstruction][solids][sloped][eaves]") {
  SolidsOptions o;
  o.slab_thickness_m = 0.2;
  o.extend_at_joins = false;
  const auto m = eaves_attic(true);
  const auto r = build_solids(m, o);
  CHECK(r.not_closed == 0);
  const ElementSolid &room = space_solid(r, "room");
  REQUIRE(room.closed);
  CHECK(side(room, 4.0, 0.5, 1.50) == CGAL::ON_BOUNDED_SIDE); // in the dormer
  CHECK(side(room, 4.0, 0.5, 1.70) == CGAL::ON_UNBOUNDED_SIDE);
  CHECK(side(room, 1.0, 0.5, 1.00) == CGAL::ON_BOUNDED_SIDE); // under slope
  CHECK(side(room, 1.0, 0.5, 1.10) == CGAL::ON_UNBOUNDED_SIDE);
  // Volume: the slope's, plus the dormer's wedge above it.
  const auto edges = core::roof_edges(m.spaces[0]);
  double v = 0.0;
  for (double x = 0.0025; x < 8.0; x += 0.005)
    for (double y = 0.0025; y < 6.0; y += 0.05)
      v += 0.005 * 0.05 * core::ceiling_at(m.spaces[0], edges.dormers, x, y);
  CHECK_THAT(room.volume, WithinAbs(v, 0.01 * v));
  // The facade beside the room: knee, then up to the dormer's ceiling.
  const auto &ws = wall_solid(r, "w-s");
  CHECK(side(ws, 1.0, 0.0, 0.50) == CGAL::ON_BOUNDED_SIDE);
  CHECK(side(ws, 1.0, 0.0, 0.60) == CGAL::ON_UNBOUNDED_SIDE);
  CHECK(side(ws, 4.0, 0.0, 1.55) == CGAL::ON_BOUNDED_SIDE);
  CHECK(side(ws, 4.0, 0.0, 1.65) == CGAL::ON_UNBOUNDED_SIDE);
  const auto roofs = roofs_of(r);
  CHECK(inside_any(roofs, 4.0, 0.5, 1.7));
  CHECK_FALSE(inside_any(roofs, 4.0, 0.5, 1.15));
  CHECK(inside_any(roofs, 1.0, 0.5, 1.15));
}
