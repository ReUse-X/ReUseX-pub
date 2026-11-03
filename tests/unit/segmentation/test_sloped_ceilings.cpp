// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// fit_sloped_ceilings() (reusex-546) on synthetic rooms: a 35 deg pitched
// attic with knee walls, a room with a flat centre and sloped sides, a
// mono-pitch room (house1's south block, ~9.5 deg) and a flat room.

#include <reusex/core/sloped_ceiling.hpp>
#include <reusex/core/solid_model.hpp>
#include <reusex/segmentation/spaces.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

using Catch::Matchers::WithinAbs;
using namespace reusex;

namespace {

const double kTan35 = std::tan(35.0 * 3.14159265358979323846 / 180.0);

struct Scene {
  Cloud cloud;
  CloudN normals;
  CloudL labels;

  void add(double x, double y, double z, Eigen::Vector3d n,
           std::uint32_t label) {
    PointT p;
    p.x = static_cast<float>(x);
    p.y = static_cast<float>(y);
    p.z = static_cast<float>(z);
    cloud.push_back(p);
    n.normalize();
    NormalT nn;
    nn.normal_x = static_cast<float>(n.x());
    nn.normal_y = static_cast<float>(n.y());
    nn.normal_z = static_cast<float>(n.z());
    normals.push_back(nn);
    LabelT l;
    l.label = label;
    labels.push_back(l);
  }

  /// A ceiling surface z = f(x, y) over [x0, x1] x [y0, y1], 5 cm spacing,
  /// each point labelled by @p label_of (0 = no plane).
  void ceiling(double x0, double x1, double y0, double y1,
               const std::function<double(double, double)> &f,
               const std::function<std::uint32_t(double, double)> &label_of,
               const std::function<Eigen::Vector3d(double, double)> &n_of) {
    for (double x = x0 + 0.025; x < x1; x += 0.05)
      for (double y = y0 + 0.025; y < y1; y += 0.05)
        add(x, y, f(x, y), n_of(x, y), label_of(x, y));
  }

  /// A floor at z = 0 and a little clutter (a table top at 0.75 m).
  void floor(double x0, double x1, double y0, double y1) {
    for (double x = x0 + 0.05; x < x1; x += 0.1)
      for (double y = y0 + 0.05; y < y1; y += 0.1)
        add(x, y, 0.0, {0, 0, 1}, 0);
    for (double x = x0 + 0.5; x < x0 + 1.5; x += 0.05)
      for (double y = y0 + 0.5; y < y0 + 1.0; y += 0.05)
        add(x, y, 0.75, {0, 0, 1}, 0);
  }
};

core::StoreyRecord storey() {
  core::StoreyRecord s;
  s.idx = 0;
  s.guid = "st0";
  s.z_floor = 0.0;
  s.z_ceiling = 2.6;
  return s;
}

core::SpaceRecord room(const std::string &guid, double x0, double x1, double y0,
                       double y1, double z_ceiling) {
  core::SpaceRecord s;
  s.guid = guid;
  s.polygon.outer = {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}};
  s.z_floor = 0.0;
  s.z_ceiling = z_ceiling; // what measure_space_heights() would say
  s.area = (x1 - x0) * (y1 - y0);
  return s;
}

/// Down-facing normal of z = a + sy * y.
Eigen::Vector3d down(double sy) { return {0.0, sy, -1.0}; }

} // namespace

TEST_CASE("SlopedCeilings_AtticAndFlatCentreAndMonoPitch",
          "[segmentation][spaces][sloped]") {
  Scene sc;
  // Attic 0..6 x 0..4: ridge along x at y = 2, knee walls 1.15 m.
  sc.ceiling(
      0.1, 5.9, 0.1, 3.9,
      [](double, double y) { return 1.15 + kTan35 * std::min(y, 4.0 - y); },
      [](double, double y) { return y < 2.0 ? 1u : 2u; },
      [](double, double y) { return y < 2.0 ? down(kTan35) : down(-kTan35); });
  sc.floor(0.1, 5.9, 0.1, 3.9);
  // Flat centre 6..10 x 0..4: 2.2 m between sides rising from 1.0 m.
  sc.ceiling(
      6.1, 9.9, 0.1, 3.9,
      [](double, double y) {
        return std::min(2.2, 1.0 + kTan35 * std::min(y, 4.0 - y));
      },
      [](double, double y) {
        const double h = 1.0 + kTan35 * std::min(y, 4.0 - y);
        return h >= 2.2 ? 5u : (y < 2.0 ? 3u : 4u);
      },
      [](double, double y) {
        const double h = 1.0 + kTan35 * std::min(y, 4.0 - y);
        return h >= 2.2 ? Eigen::Vector3d(0, 0, -1)
                        : (y < 2.0 ? down(kTan35) : down(-kTan35));
      });
  sc.floor(6.1, 9.9, 0.1, 3.9);
  // Mono pitch 0..5 x 5..9: 2.38 m at y = 5 rising 0.17 m/m.
  sc.ceiling(
      0.1, 4.9, 5.1, 8.9,
      [](double, double y) { return 2.38 + 0.17 * (y - 5); },
      [](double, double) { return 6u; },
      [](double, double) { return down(0.17); });
  sc.floor(0.1, 4.9, 5.1, 8.9);
  // Flat room 6..10 x 5..9 at 2.5 m, its ceiling unlabelled.
  sc.ceiling(
      6.1, 9.9, 5.1, 8.9, [](double, double) { return 2.5; },
      [](double, double) { return 0u; },
      [](double, double) { return Eigen::Vector3d(0, 0, -1); });
  sc.floor(6.1, 9.9, 5.1, 8.9);

  std::vector<core::SpaceRecord> spaces = {
      room("attic", 0.1, 5.9, 0.1, 3.9, 2.0),
      room("flat-centre", 6.1, 9.9, 0.1, 3.9, 2.2),
      room("mono", 0.1, 4.9, 5.1, 8.9, 2.7),
      room("flat", 6.1, 9.9, 5.1, 8.9, 2.5)};
  // Every ceiling plane, the flat one (5) included: it is too flat to be a
  // candidate.
  const std::vector<int> labels = {1, 2, 3, 4, 5, 6};
  const std::vector<int> label_storeys(labels.size(), 0);

  // The fit itself, so the roof is not regularised (reusex-p3gm): that
  // would put the attic and the flat centre, 0.15 m apart, on one plane.
  geometry::SpacesOptions fit_only;
  fit_only.roof_regularise = false;
  const auto stats =
      geometry::fit_sloped_ceilings(spaces, {storey()}, sc.cloud, sc.normals,
                                    sc.labels, labels, label_storeys, fit_only);
  CHECK(stats.candidates == 5);
  CHECK(stats.spaces_sloped == 3);

  // The model follows the surface it was fitted to.
  const auto err = [](const core::SpaceRecord &s,
                      const std::function<double(double, double)> &truth) {
    double worst = 0.0;
    for (double x = s.polygon.outer[0][0] + 0.2; x < s.polygon.outer[1][0];
         x += 0.3)
      for (double y = s.polygon.outer[0][1] + 0.2; y < s.polygon.outer[2][1];
           y += 0.3)
        worst =
            std::max(worst, std::abs(core::ceiling_at(s, x, y) - truth(x, y)));
    return worst;
  };

  const auto &attic = spaces[0];
  CHECK(attic.ceiling_planes.size() == 2);
  CHECK(err(attic, [](double, double y) {
          return 1.15 + kTan35 * std::min(y, 4.0 - y);
        }) < 0.03);
  CHECK_THAT(attic.z_ceiling, WithinAbs(1.15 + 1.9 * kTan35, 0.05));

  const auto &fc = spaces[1];
  CHECK(fc.ceiling_planes.size() == 2);
  CHECK_THAT(fc.z_ceiling, WithinAbs(2.2, 0.02));
  CHECK(err(fc, [](double, double y) {
          return std::min(2.2, 1.0 + kTan35 * std::min(y, 4.0 - y));
        }) < 0.03);

  const auto &mono = spaces[2];
  CHECK(mono.ceiling_planes.size() == 1);
  CHECK(err(mono, [](double, double y) { return 2.38 + 0.17 * (y - 5); }) <
        0.03);

  const auto &flat = spaces[3];
  CHECK(flat.ceiling_planes.empty());
  CHECK(flat.z_ceiling == 2.5); // untouched

  // Volumes follow the sloped ceiling; metadata records the fit.
  CHECK(attic.volume < attic.area * attic.z_ceiling);
  CHECK(attic.metadata.find("ceiling_fit") != std::string::npos);
  for (const auto &s : spaces)
    CHECK(core::check_ceiling_planes(s).empty());
}

TEST_CASE("SlopedCeilings_OffOrNoCandidates_LeaveSpacesAlone",
          "[segmentation][spaces][sloped]") {
  Scene sc;
  sc.ceiling(
      0.1, 5.9, 0.1, 3.9,
      [](double, double y) { return 1.15 + kTan35 * std::min(y, 4.0 - y); },
      [](double, double y) { return y < 2.0 ? 1u : 2u; },
      [](double, double y) { return y < 2.0 ? down(kTan35) : down(-kTan35); });
  const std::vector<int> labels = {1, 2}, label_storeys = {0, 0};
  const auto attic = room("attic", 0.1, 5.9, 0.1, 3.9, 2.0);

  geometry::SpacesOptions off;
  off.fit_ceiling_planes = false;
  std::vector<core::SpaceRecord> a = {attic};
  geometry::fit_sloped_ceilings(a, {storey()}, sc.cloud, sc.normals, sc.labels,
                                labels, label_storeys, off);
  CHECK(a[0].ceiling_planes.empty());

  // The lock is the stage's business: a detected space that carries a
  // stored row's lock (reusex-djf) is fitted like any other.
  std::vector<core::SpaceRecord> b = {attic};
  b[0].locked = true;
  geometry::fit_sloped_ceilings(b, {storey()}, sc.cloud, sc.normals, sc.labels,
                                labels, label_storeys);
  CHECK(b[0].ceiling_planes.size() == 2);

  std::vector<core::SpaceRecord> c = {attic};
  geometry::fit_sloped_ceilings(c, {storey()}, sc.cloud, sc.normals, sc.labels,
                                {}, {});
  CHECK(c[0].ceiling_planes.empty());

  // Planes of another storey are not candidates here.
  std::vector<core::SpaceRecord> d = {attic};
  geometry::fit_sloped_ceilings(d, {storey()}, sc.cloud, sc.normals, sc.labels,
                                labels, {1, 1});
  CHECK(d[0].ceiling_planes.empty());

  CHECK_THROWS_AS(geometry::fit_sloped_ceilings(d, {storey()}, sc.cloud,
                                                sc.normals, sc.labels, labels,
                                                {0}),
                  std::invalid_argument);
}

// Eaves (reusex-ow8p): an attic with 45 deg slopes off a flat centre
// (2.5 m) down to 0.55 m knee walls on both facades (y = 0, y = 6), as in
// Sygehusgrunden's roof storey.

namespace {

/// z of the attic's ceiling: 0.55 at the facades, 45 deg, flat at 2.5.
double attic45(double y) { return std::min(2.5, 0.55 + std::min(y, 6.0 - y)); }

/// 45 deg down-facing normal of the south (label 1) / north (label 2) slope.
Eigen::Vector3d slope_n(double y) { return y < 3.0 ? down(1.0) : down(-1.0); }

core::WallRecord facade(const std::string &guid, double x0, double y0,
                        double x1, double y1) {
  core::WallRecord w;
  w.guid = guid;
  w.x0 = x0;
  w.y0 = y0;
  w.x1 = x1;
  w.y1 = y1;
  w.z_base = 0.0;
  w.height = 2.6;
  w.thickness = 0.3;
  w.is_external = true;
  w.source = "detected";
  return w;
}

/// The face of a wall along x at @p y from z = 0 to top(x), facing +-y.
void wall_along_x(Scene &sc, double x0, double x1, double y,
                  const std::function<double(double)> &top, double ny) {
  for (double x = x0 + 0.025; x < x1; x += 0.05)
    for (double z = 0.025; z < top(x); z += 0.05)
      sc.add(x, y, z, {0, ny, 0}, 0);
}

/// The attic's sloped and flat ceiling over x in [x0, x1], y in [y0, y1],
/// except where @p skip says.
void attic_ceiling(Scene &sc, double x0, double x1, double y0, double y1,
                   const std::function<bool(double, double)> &skip) {
  for (double x = x0 + 0.025; x < x1; x += 0.05)
    for (double y = y0 + 0.025; y < y1; y += 0.05) {
      if (skip(x, y))
        continue;
      const double z = attic45(y);
      const bool flat = z >= 2.5;
      sc.add(x, y, z, flat ? Eigen::Vector3d(0, 0, -1) : slope_n(y),
             flat ? 3u : (y < 3.0 ? 1u : 2u));
    }
}

core::SpaceRecord poly_room(const std::string &guid, double x0, double x1,
                            double y0, double y1, double z_ceiling) {
  return room(guid, x0, x1, y0, y1, z_ceiling);
}

} // namespace

TEST_CASE("SlopedCeilings_Eaves_SlopeRunsOnToTheKneeWall",
          "[segmentation][spaces][sloped][eaves]") {
  Scene sc;
  // The room y 1..6: its south slope is seen from 1.0 (the knee partition
  // at y = 1, 1.55 m) up; the strip y 0..1 behind the partition is unseen
  // but for its floor and the facade's 0.55 m knee wall.
  attic_ceiling(sc, 0.0, 8.0, 1.0, 6.0, [](double, double) { return false; });
  sc.floor(0.1, 7.9, 1.1, 5.9);
  sc.floor(0.1, 7.9, 0.1, 0.9);
  wall_along_x(sc, 0.0, 8.0, 0.0, [](double) { return 0.55; }, 1.0);
  wall_along_x(sc, 0.0, 8.0, 6.0, [](double) { return 0.55; }, -1.0);
  wall_along_x(sc, 0.0, 8.0, 1.0, [](double) { return 1.55; }, 1.0);
  // An office beside it, x 8.2..12, its flat ceiling seen at 2.5 m all over.
  sc.ceiling(
      8.2, 12.0, 0.0, 6.0, [](double, double) { return 2.5; },
      [](double, double) { return 0u; },
      [](double, double) { return Eigen::Vector3d(0, 0, -1); });
  sc.floor(8.3, 11.9, 0.1, 5.9);

  std::vector<core::SpaceRecord> spaces = {
      poly_room("room", 0.0, 8.0, 1.0, 6.0, 2.5),
      poly_room("strip", 0.0, 8.0, 0.0, 1.0, 2.5),
      poly_room("office", 8.2, 12.0, 0.0, 6.0, 2.5)};
  const std::vector<core::WallRecord> walls = {
      facade("w-s", 0, 0, 8, 0), facade("w-n", 8, 6, 0, 6),
      facade("w-office-s", 8.2, 0, 12, 0)};
  const std::vector<int> labels = {1, 2, 3};
  const std::vector<int> label_storeys(labels.size(), 0);
  const auto stats = geometry::fit_sloped_ceilings(
      spaces, {storey()}, sc.cloud, sc.normals, sc.labels, labels,
      label_storeys, {}, &walls);

  const auto &room_s = spaces[0], &strip = spaces[1], &office = spaces[2];
  REQUIRE(room_s.ceiling_planes.size() == 2);
  CHECK_THAT(room_s.z_ceiling, WithinAbs(2.5, 0.03));
  // The strip takes the room's south slope, down to the knee wall.
  REQUIRE(strip.ceiling_planes.size() == 1);
  CHECK(stats.extended >= 1);
  CHECK_THAT(core::ceiling_at(strip, 4.0, 0.0), WithinAbs(0.55, 0.05));
  CHECK_THAT(core::ceiling_at(strip, 4.0, 0.5), WithinAbs(1.05, 0.05));
  CHECK(strip.volume < strip.area * 1.2);
  // The office's ceiling is seen flat where the slope would cut it.
  CHECK(office.ceiling_planes.empty());
  for (const auto &s : spaces)
    CHECK(core::check_ceiling_planes(s).empty());

  // The facades under the slopes are knee walls; the office's is not.
  const auto knees = [](const core::SpaceRecord &s) {
    return core::roof_edges(s).knee_walls;
  };
  const auto has = [](const std::vector<std::string> &v, const char *g) {
    return std::find(v.begin(), v.end(), g) != v.end();
  };
  CHECK(has(knees(strip), "w-s"));
  CHECK(has(knees(room_s), "w-n"));
  CHECK_FALSE(has(knees(room_s), "w-office-s"));
  CHECK_FALSE(has(knees(strip), "w-office-s"));
  CHECK(core::roof_edges(strip).dormers.empty());
  CHECK(core::roof_edges(room_s).dormers.empty());
  CHECK(knees(office).empty());
}

TEST_CASE("SlopedCeilings_Eaves_DormerRisesThroughTheSlope",
          "[segmentation][spaces][sloped][eaves]") {
  // The same attic as one room y 0..6, with a dormer in the south facade
  // over x 3..5: the facade rises to its flat ceiling at 1.6 m, which runs
  // in to where the slope reaches 1.6 m (y = 1.05), between cheek walls.
  constexpr double zd = 1.6, depth = zd - 0.55;
  const auto in_dormer = [&](double x, double y) {
    return x > 3.0 && x < 5.0 && y < depth;
  };
  Scene sc;
  attic_ceiling(sc, 0.0, 8.0, 0.0, 6.0, in_dormer);
  sc.ceiling(
      3.0, 5.0, 0.0, depth, [&](double, double) { return zd; },
      [](double, double) { return 4u; },
      [](double, double) { return Eigen::Vector3d(0, 0, -1); });
  for (const double xc : {3.0, 5.0})
    for (double y = 0.025; y < depth; y += 0.05)
      for (double z = 0.55 + y; z < zd; z += 0.05)
        sc.add(xc, y, z, {xc < 4.0 ? 1.0 : -1.0, 0, 0}, 0);
  sc.floor(0.1, 7.9, 0.1, 5.9);
  wall_along_x(
      sc, 0.0, 8.0, 0.0,
      [](double x) { return x > 3.0 && x < 5.0 ? zd : 0.55; }, 1.0);
  wall_along_x(sc, 0.0, 8.0, 6.0, [](double) { return 0.55; }, -1.0);

  std::vector<core::SpaceRecord> spaces = {
      poly_room("room", 0.0, 8.0, 0.0, 6.0, 2.5)};
  const std::vector<core::WallRecord> walls = {facade("w-s", 0, 0, 8, 0),
                                               facade("w-n", 8, 6, 0, 6)};
  const std::vector<int> labels = {1, 2, 3, 4};
  const std::vector<int> label_storeys(labels.size(), 0);
  const auto stats = geometry::fit_sloped_ceilings(
      spaces, {storey()}, sc.cloud, sc.normals, sc.labels, labels,
      label_storeys, {}, &walls);
  CHECK(stats.dormers == 1);

  const auto &r = spaces[0];
  REQUIRE(r.ceiling_planes.size() == 2);
  const auto edges = core::roof_edges(r);
  REQUIRE(edges.dormers.size() == 1);
  const auto &d = edges.dormers[0];
  CHECK(d.wall == "w-s");
  CHECK_THAT(d.s0, WithinAbs(3.0, 0.3));
  CHECK_THAT(d.s1, WithinAbs(5.0, 0.3));
  CHECK_THAT(d.z, WithinAbs(zd, 0.05));
  REQUIRE(d.outline.size() == 4);
  double inner = 0.0;
  for (const auto &p : d.outline)
    inner = std::max(inner, p[1]);
  CHECK_THAT(inner, WithinAbs(depth, 0.1));
  // Both facades are knee walls; the dormer's raises the ceiling over it.
  CHECK(edges.knee_walls.size() == 2);
  CHECK_THAT(core::ceiling_at(r, edges.dormers, 4.0, 0.5), WithinAbs(zd, 0.05));
  CHECK_THAT(core::ceiling_at(r, edges.dormers, 1.0, 0.5),
             WithinAbs(1.05, 0.05));
  // The room's volume counts the dormer.
  double v = 0.0;
  for (double x = 0.05; x < 8.0; x += 0.1)
    for (double y = 0.05; y < 6.0; y += 0.1)
      v += 0.01 * core::ceiling_at(r, edges.dormers, x, y);
  CHECK_THAT(r.volume, WithinAbs(v, 0.02 * v));
}

TEST_CASE("SlopedCeilings_RoofEdges_RoundTripInMetadata",
          "[segmentation][spaces][sloped][eaves]") {
  core::RoofEdges e;
  e.knee_walls = {"w1"};
  core::Dormer d;
  d.wall = "w1";
  d.s0 = 1.0;
  d.s1 = 2.5;
  d.z = 2.1;
  d.outline = {{1, 0}, {2.5, 0}, {2.5, 1}, {1, 1}};
  e.dormers = {d};
  e.eaves = {{{3.15, -6.0}, {3.15, 0.5}}};
  core::SpaceRecord s;
  s.metadata = core::with_roof_edges(R"({"z_floor_cells": 12})", e);
  const auto back = core::roof_edges(s);
  REQUIRE(back.dormers.size() == 1);
  REQUIRE(back.eaves.size() == 1);
  CHECK(back.eaves[0].a[1] == -6.0);
  CHECK(back.eaves[0].b[0] == 3.15);
  CHECK(back.knee_walls == e.knee_walls);
  CHECK(back.dormers[0].wall == "w1");
  CHECK(back.dormers[0].s1 == 2.5);
  CHECK(back.dormers[0].outline.size() == 4);
  CHECK(s.metadata.find("z_floor_cells") != std::string::npos);
  // No edges: a stale "roof" goes, anything else stays as it is.
  CHECK(core::with_roof_edges(s.metadata, {}) == R"({"z_floor_cells":12})");
  CHECK(core::with_roof_edges("", {}).empty());
  CHECK(core::with_roof_edges("not json", {}) == "not json");
  s.metadata = "not json";
  CHECK(core::roof_edges(s).empty());
}

// ── Roof regularisation (reusex-p3gm) ───────────────────────────────────────

namespace {

constexpr double kDegR = 3.14159265358979323846 / 180.0;

/// A plane rising toward azimuth @p az_deg at @p pitch_deg that reaches
/// @p top at the eaves line u . x = @p d.
core::CeilingPlane slope_plane(double az_deg, double pitch_deg, double d,
                               double top) {
  const double ux = std::cos(az_deg * kDegR), uy = std::sin(az_deg * kDegR);
  const double t = std::tan(pitch_deg * kDegR);
  core::CeilingPlane p;
  p.at = {ux * d, uy * d, top};
  p.slope = {t * ux, t * uy};
  return p;
}

double pitch_of(const core::CeilingPlane &p) {
  return std::atan(std::hypot(p.slope[0], p.slope[1])) / kDegR;
}

double azimuth_of(const core::CeilingPlane &p) {
  return std::atan2(p.slope[1], p.slope[0]) / kDegR;
}

/// Where @p p reaches @p top along its uphill direction.
double eaves_line(const core::CeilingPlane &p, double top) {
  const double g = std::hypot(p.slope[0], p.slope[1]);
  const double ux = p.slope[0] / g, uy = p.slope[1] / g;
  return ux * p.at[0] + uy * p.at[1] + (top - p.at[2]) / g;
}

} // namespace

TEST_CASE("SlopedCeilings_RoofRegularise_FewPlanesFlatTop45",
          "[sloped_ceilings][roof_regularise]") {
  // Sygehusgrunden's attic: a flat top, 45 deg sides, fitted per room as
  // planes 40-46 deg, a few degrees off, eaves lines and tops a few
  // decimetres apart: kinks where rooms meet. Rooms along y, 4 m deep in x;
  // the west side rises toward +x (az 0), the east one toward -x (az 180).
  std::vector<core::SpaceRecord> sp = {
      room("a", 0, 6, 0, 4, 3.00), room("b", 0, 6, 4, 8, 3.10),
      room("c", 0, 6, 8, 12, 2.95), room("d", 0, 6, 12, 14, 3.02)};
  sp[0].ceiling_planes = {slope_plane(0.0, 43.5, 1.5, 3.0),
                          slope_plane(180.0, 44.0, -4.5, 3.0)};
  sp[1].ceiling_planes = {slope_plane(3.0, 46.0, 1.8, 3.1),
                          slope_plane(-178.0, 45.5, -4.3, 3.1)};
  sp[2].ceiling_planes = {slope_plane(0.0, 40.5, 1.6, 2.95),
                          slope_plane(92.0, 44.0, 10.5, 2.95)};
  // A lower roof 2 m further out on the west: its own plane.
  sp[3].ceiling_planes = {slope_plane(0.0, 45.0, -0.6, 3.02)};
  const auto fitted = sp;
  const auto st =
      geometry::regularise_roof(sp, {0, 1, 2, 3}, geometry::SpacesOptions{});
  CHECK(st.planes_in == 7);
  CHECK(st.planes_out == 4); // west, west (lower roof), east, the hip
  CHECK_THAT(st.pitch_deg, WithinAbs(45.0, 1e-9));
  CHECK(st.spaces == 4);
  // One flat top for every room: the area-weighted median.
  for (const auto &s : sp)
    CHECK_THAT(s.z_ceiling, WithinAbs(3.0, 1e-9));
  // One west plane for a, b, c; one east plane for a, b; all 45 deg and
  // square to the plan.
  const auto &west = sp[0].ceiling_planes[0];
  CHECK(sp[1].ceiling_planes[0].at == west.at);
  CHECK(sp[1].ceiling_planes[0].slope == west.slope);
  CHECK(sp[2].ceiling_planes[0].slope == west.slope);
  CHECK(sp[2].ceiling_planes[0].at == west.at);
  CHECK(sp[0].ceiling_planes[1].at == sp[1].ceiling_planes[1].at);
  CHECK(sp[0].ceiling_planes[1].slope == sp[1].ceiling_planes[1].slope);
  for (const auto &s : sp)
    for (const auto &p : s.ceiling_planes) {
      CHECK_THAT(pitch_of(p), WithinAbs(45.0, 1e-9));
      const double az = azimuth_of(p);
      CHECK_THAT(std::remainder(az, 90.0), WithinAbs(0.0, 1e-9));
    }
  // The west eaves line: between the fitted ones (1.5 .. 1.8), and the
  // lower roof stays 2 m out.
  CHECK(eaves_line(west, 3.0) > 1.4);
  CHECK(eaves_line(west, 3.0) < 1.9);
  CHECK_THAT(eaves_line(sp[3].ceiling_planes[0], 3.0), WithinAbs(-0.6, 0.15));
  // What was fitted is kept.
  const auto meta = nlohmann::json::parse(sp[1].metadata);
  CHECK(meta["ceiling_fit"]["fitted"]["z_ceiling"].get<double>() == 3.1);
  CHECK(core::decode_ceiling_planes(
            meta["ceiling_fit"]["fitted"]["planes"].get<std::string>())
            .size() == 2);
  (void)fitted;

  // Off: nothing changes.
  auto off = fitted;
  geometry::SpacesOptions o;
  o.roof_regularise = false;
  const auto st_off = geometry::regularise_roof(off, {0, 1, 2, 3}, o);
  CHECK(st_off.spaces == 0);
  CHECK(off[1].z_ceiling == 3.1);
  CHECK(off[1].ceiling_planes[0].slope == fitted[1].ceiling_planes[0].slope);
}

TEST_CASE("SlopedCeilings_RoofRegularise_NeverDownToTheFloor",
          "[sloped_ceilings][roof_regularise]") {
  // A narrow strip whose fitted slope just clears its floor at the eaves:
  // moved onto the shared plane it would not, so it keeps its own.
  std::vector<core::SpaceRecord> sp = {room("big", 0, 6, 0, 10, 3.0),
                                       room("strip", -0.2, 0.6, 10, 12, 3.0)};
  sp[0].ceiling_planes = {slope_plane(0.0, 45.0, 1.5, 3.0)};
  sp[1].ceiling_planes = {slope_plane(0.0, 45.0, 2.0, 3.0)};
  // big: z at x = -0.2 is 3 - 1.7 = 1.3; strip: 3 - 2.2 = 0.8 > 0.1. The
  // shared line (area-weighted, 1.5 + 0.5 * 1.6 / 61.6) stays near 1.5:
  // fine. Raise the strip's floor so the shared plane would meet it.
  sp[1].z_floor = 1.25;
  sp[1].ceiling_planes = {slope_plane(0.0, 45.0, 1.1, 3.0)};
  const auto st = geometry::regularise_roof(sp, {0, 1}, {});
  CHECK(st.reverted == 1);
  CHECK_THAT(eaves_line(sp[1].ceiling_planes[0], 3.0), WithinAbs(1.1, 1e-9));
}

TEST_CASE("SlopedCeilings_RoofRegularise_PeaksComeDownLoweredCeilingsStay",
          "[sloped_ceilings][roof_regularise]") {
  // One roof: two offices at 3.0 m, a stair shaft whose top peaks at 3.5 m
  // (Sygehusgrunden S3-20: the kink), and a corridor with a lowered ceiling
  // at 2.6 m, seen, under the same slope.
  std::vector<core::SpaceRecord> sp = {
      room("office", 0, 6, 0, 4, 3.0), room("office2", 0, 6, 4, 8, 3.0),
      room("stair", 0, 6, 8, 10, 3.5), room("corridor", 0, 6, 10, 12, 2.6)};
  for (auto &s : sp)
    s.ceiling_planes = {slope_plane(0.0, 45.0, 1.5, 3.0)};
  const auto st = geometry::regularise_roof(sp, {0, 1, 2, 3}, {});
  CHECK_THAT(st.top_z, WithinAbs(3.0, 1e-9));
  CHECK_THAT(sp[2].z_ceiling, WithinAbs(3.0, 1e-9)); // the peak is gone
  CHECK_THAT(sp[3].z_ceiling, WithinAbs(2.6, 1e-9)); // seen, so it stays
}

TEST_CASE("SlopedCeilings_Eaves_NoWallAtTheFacadeIsAnEave",
          "[segmentation][spaces][sloped][eaves]") {
  // reusex-wyi0: the attic of SlopeRunsOnToTheKneeWall, but the north
  // facade's knee wall was never detected (seen too little): where the
  // room's north slope comes down to its outline at y = 6 there is an eave
  // with no wall. The south facade has its wall; the room's south edge
  // (y = 1) has the strip outside it, no facade.
  Scene sc;
  attic_ceiling(sc, 0.0, 8.0, 1.0, 6.0, [](double, double) { return false; });
  sc.floor(0.1, 7.9, 1.1, 5.9);
  sc.floor(0.1, 7.9, 0.1, 0.9);
  wall_along_x(sc, 0.0, 8.0, 0.0, [](double) { return 0.55; }, 1.0);
  wall_along_x(sc, 0.0, 8.0, 1.0, [](double) { return 1.55; }, 1.0);
  std::vector<core::SpaceRecord> spaces = {
      poly_room("room", 0.0, 8.0, 1.0, 6.0, 2.5),
      poly_room("strip", 0.0, 8.0, 0.0, 1.0, 2.5)};
  const std::vector<core::WallRecord> walls = {facade("w-s", 0, 0, 8, 0)};
  const std::vector<int> labels = {1, 2, 3};
  const std::vector<int> label_storeys(labels.size(), 0);
  const auto stats = geometry::fit_sloped_ceilings(
      spaces, {storey()}, sc.cloud, sc.normals, sc.labels, labels,
      label_storeys, {}, &walls);
  const auto room_edges = core::roof_edges(spaces[0]);
  REQUIRE(room_edges.eaves.size() == 1);
  CHECK(stats.eaves == 1);
  const auto &e = room_edges.eaves[0];
  CHECK_THAT(e.a[1], WithinAbs(6.0, 1e-6));
  CHECK_THAT(e.b[1], WithinAbs(6.0, 1e-6));
  CHECK_THAT(std::abs(e.b[0] - e.a[0]), WithinAbs(8.0, 1e-6));
  // The strip's eave has its wall.
  CHECK(core::roof_edges(spaces[1]).eaves.empty());

  // An outline that recedes from the eave for more than kEaveMergeGap
  // (2 m; a wide bay of furniture): two eaves, each on the outline, none
  // across the gap. A shorter recess is bridged: one wall, simpler.
  auto notched = spaces;
  notched[0] = poly_room("room", 0.0, 8.0, 1.0, 6.0, 2.5);
  notched[0].polygon.outer = {{0, 1},     {8, 1},   {8, 6}, {5.5, 6},
                              {5.5, 5.6}, {2, 5.6}, {2, 6}, {0, 6}};
  notched[1] = poly_room("strip", 0.0, 8.0, 0.0, 1.0, 2.5);
  geometry::fit_sloped_ceilings(notched, {storey()}, sc.cloud, sc.normals,
                                sc.labels, labels, label_storeys, {}, &walls);
  const auto n_edges = core::roof_edges(notched[0]);
  REQUIRE(n_edges.eaves.size() == 2);
  for (const auto &x : n_edges.eaves) {
    CHECK_THAT(x.a[1], WithinAbs(6.0, 1e-6));
    const double lo = std::min(x.a[0], x.b[0]), hi = std::max(x.a[0], x.b[0]);
    CHECK((hi <= 2.0 + 1e-6 || lo >= 5.5 - 1e-6));
  }
  auto recess = spaces;
  recess[0] = poly_room("room", 0.0, 8.0, 1.0, 6.0, 2.5);
  recess[0].polygon.outer = {{0, 1},   {8, 1},   {8, 6}, {4, 6},
                             {4, 5.6}, {3, 5.6}, {3, 6}, {0, 6}};
  recess[1] = poly_room("strip", 0.0, 8.0, 0.0, 1.0, 2.5);
  geometry::fit_sloped_ceilings(recess, {storey()}, sc.cloud, sc.normals,
                                sc.labels, labels, label_storeys, {}, &walls);
  REQUIRE(core::roof_edges(recess[0]).eaves.size() == 1);
}
