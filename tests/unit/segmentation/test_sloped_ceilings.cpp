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

  const auto stats =
      geometry::fit_sloped_ceilings(spaces, {storey()}, sc.cloud, sc.normals,
                                    sc.labels, labels, label_storeys);
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

TEST_CASE("SlopedCeilings_OffLockedOrNoCandidates_LeaveSpacesAlone",
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

  std::vector<core::SpaceRecord> b = {attic};
  b[0].locked = true;
  geometry::fit_sloped_ceilings(b, {storey()}, sc.cloud, sc.normals, sc.labels,
                                labels, label_storeys);
  CHECK(b[0].ceiling_planes.empty());

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
