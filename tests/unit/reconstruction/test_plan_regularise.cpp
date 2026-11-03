// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Plan regularisation (reusex-z1v.12): collinear and stacked wall lines, the
// Manhattan grid, window columns and rows -- and what must never merge.

#include <reusex/reconstruction/plan_regularise.hpp>
#include <reusex/reconstruction/walls.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <nlohmann/json.hpp>

#include <cmath>
#include <limits>
#include <map>
#include <numbers>
#include <string>
#include <vector>

using Catch::Matchers::WithinAbs;
using reusex::core::OpeningRecord;
using reusex::core::StoreyRecord;
using reusex::core::WallRecord;
using reusex::geometry::OpeningRegulariseOptions;
using reusex::geometry::PlanRegulariseOptions;
using reusex::geometry::regularise_openings;
using reusex::geometry::regularise_walls;
using reusex::geometry::wall_records;
using reusex::geometry::WallEnd;
using reusex::geometry::WallGraph;
using reusex::geometry::WallJoin;
using reusex::geometry::WallJoinKind;
using reusex::geometry::WallSegment;

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

/// A wall along family direction @p d (its normal): a one-sided wall seen
/// from -d (side < 0, axis = its lo face) or +d (side > 0), or a measured
/// one (side 0) with its centre line at @p offset.
WallSegment wall(Eigen::Vector2d d, int family, double offset, double s0,
                 double s1, int side, double thickness = 0.2) {
  WallSegment w;
  w.family = family;
  w.d = d.normalized();
  w.axis_offset = offset;
  w.s0 = s0;
  w.s1 = s1;
  w.thickness = thickness;
  if (side < 0) {
    w.face_lo = offset;
    w.assumed = true;
  } else if (side > 0) {
    w.face_hi = offset;
    w.assumed = true;
  } else {
    w.face_lo = offset - 0.5 * thickness;
    w.face_hi = offset + 0.5 * thickness;
  }
  return w;
}

const Eigen::Vector2d kX = Eigen::Vector2d::UnitX();
const Eigen::Vector2d kY = Eigen::Vector2d::UnitY();

StoreyRecord storey(int idx, double z_floor = 0.0) {
  StoreyRecord s;
  s.idx = idx;
  s.z_floor = z_floor;
  s.z_ceiling = z_floor + 2.8;
  return s;
}

} // namespace

TEST_CASE("plan regularisation: a corridor's jittered walls run straight",
          "[plan_regularise]") {
  // Corridor along x, 1.6 m wide. Walls with normal +y (family 1); s = -x.
  // North side seen from the corridor (-d): faces at y = 0.8 +- jitter, one
  // stretch measured 0.12 thick. South side seen from +d.
  WallGraph g;
  g.walls = {
      wall(kY, 1, 0.80, -4.0, 0.0, -1),              // 0: x 0..4
      wall(kY, 1, 0.86 + 0.06, -9.0, -5.0, 0, 0.12), // 1: measured
      wall(kY, 1, 0.77, -14.0, -10.0, -1),           // 2
      wall(kY, 1, 0.83, -20.0, -15.0, -1),           // 3: x 15..20
      wall(kY, 1, -0.80, -6.0, 0.0, +1),             // 4: south
      wall(kY, 1, -0.75, -13.0, -7.0, +1),           // 5
      wall(kY, 1, -0.84, -20.0, -14.0, +1),          // 6
      wall(kX, 0, 20.0, -0.84, 0.83, 0, 0.2),        // 7: the end wall, x = 20
  };
  // The end wall's ends meet the north and south walls at x = 20 (s = -20).
  g.joins = {{7, 3, WallJoinKind::corner, WallEnd::end, WallEnd::start},
             {7, 6, WallJoinKind::corner, WallEnd::start, WallEnd::start}};
  std::vector<WallGraph> graphs{g};
  const auto res = regularise_walls(graphs, {storey(0)});
  const auto &w = graphs[0].walls;

  // North: one line on the corridor face, thickness unchanged.
  const double north = w[0].face_lo;
  for (const int i : {1, 2, 3})
    CHECK_THAT(w[static_cast<size_t>(i)].face_lo, WithinAbs(north, 1e-9));
  CHECK_THAT(w[1].face_hi - w[1].face_lo, WithinAbs(0.12, 1e-9));
  CHECK(north > 0.77);
  CHECK(north < 0.86);
  // South: one line, never pulled onto the north one.
  const double south = w[4].face_hi;
  for (const int i : {5, 6})
    CHECK_THAT(w[static_cast<size_t>(i)].face_hi, WithinAbs(south, 1e-9));
  CHECK(south < -0.75);
  CHECK(south > -0.84);
  CHECK(res.lines == 2);
  CHECK(res.max_shift_m <= PlanRegulariseOptions{}.plan_collinear_m + 1e-9);

  // The end wall's corners follow the moved lines.
  CHECK_THAT(w[7].s1, WithinAbs(north, 1e-9));
  CHECK_THAT(w[7].s0, WithinAbs(south, 1e-9));
  CHECK(res.ends_rejoined >= 2);

  // Every moved wall keeps its measured axis.
  REQUIRE(w[2].regularised.applied);
  CHECK_THAT(w[2].regularised.measured_a.y(), WithinAbs(0.77, 1e-9));
  CHECK_THAT(w[2].regularised.offset_delta_m, WithinAbs(north - 0.77, 1e-9));
  CHECK(std::find(w[2].regularised.rules.begin(), w[2].regularised.rules.end(),
                  "collinear") != w[2].regularised.rules.end());
  const auto [records, joins] = wall_records(graphs[0], storey(0));
  const auto meta = nlohmann::json::parse(records[2].metadata);
  CHECK_THAT(meta["regularised"]["measured"]["y0"].get<double>(),
             WithinAbs(0.77, 1e-9));
  CHECK_THAT(meta["regularised"]["offset_delta_m"].get<double>(),
             WithinAbs(north - 0.77, 1e-9));
  CHECK_THAT(records[2].y0, WithinAbs(north, 1e-9));
}

TEST_CASE("plan regularisation: two partitions side by side never merge",
          "[plan_regularise]") {
  // Two measured partitions 0.12 m apart over the same stretch, and two
  // one-sided sheets 0.12 m apart seen from the same side.
  WallGraph g;
  g.walls = {wall(kX, 0, 0.0, 0.0, 4.0, 0, 0.07),
             wall(kX, 0, 0.12, 0.5, 4.5, 0, 0.07),
             wall(kX, 0, 5.0, 0.0, 4.0, -1), wall(kX, 0, 5.12, 0.2, 3.0, -1)};
  const auto before = g.walls;
  std::vector<WallGraph> graphs{g};
  const auto res = regularise_walls(graphs, {storey(0)});
  CHECK(res.lines == 0);
  CHECK(res.walls_moved == 0);
  for (size_t i = 0; i < before.size(); ++i) {
    CHECK(graphs[0].walls[i].axis_offset == before[i].axis_offset);
    CHECK_FALSE(graphs[0].walls[i].regularised.applied);
  }
}

TEST_CASE("plan regularisation: walls seen from opposite sides continue as "
          "one wall",
          "[plan_regularise]") {
  // Continuing, 0.1 m apart, one seen from -d and one from +d: no face in
  // common, but their bodies line up within a wall's thickness -- one wall
  // seen from either room (reusex-p3gm: simpler over closer to the scan).
  WallGraph g;
  g.walls = {wall(kX, 0, 0.0, 0.0, 3.0, -1), wall(kX, 0, 0.1, 3.5, 6.0, +1)};
  std::vector<WallGraph> graphs{g};
  const auto res = regularise_walls(graphs, {storey(0)});
  CHECK(res.lines == 1);
  CHECK(res.continuous_lines == 1);
  CHECK_THAT(graphs[0].walls[0].axis_offset,
             WithinAbs(graphs[0].walls[1].axis_offset, 1e-9));

  // With the continuity snap off, nothing says they are one line.
  std::vector<WallGraph> off{g};
  PlanRegulariseOptions o;
  o.plan_continuous_m = 0.0;
  CHECK(regularise_walls(off, {storey(0)}, o).lines == 0);
  CHECK(off[0].walls[1].axis_offset == 0.1);
}

TEST_CASE("plan regularisation: a wall continuing with an offset becomes one "
          "continuous wall",
          "[plan_regularise]") {
  // Sygehusgrunden storey 1 by S1-09 / S1-10 (reusex-p3gm): one wall comes
  // as two measured runs end to end, their axes 0.28 m apart. Family 0
  // (normal x, s = y).
  WallGraph g;
  g.walls = {wall(kX, 0, 0.00, 0.0, 4.2, 0, 0.2),   // 0: 4.2 m
             wall(kX, 0, 0.28, 4.3, 12.0, 0, 0.3),  // 1: 7.7 m, offset
             wall(kX, 0, 0.90, 12.5, 15.0, 0, 0.2), // 2: 0.9 m off: a step
             wall(kY, 1, 4.2, -3.0, -0.2, 0, 0.2)}; // 3: a partition at y 4.2
  g.joins = {{3, 0, WallJoinKind::tee, WallEnd::end, WallEnd::path}};
  std::vector<WallGraph> graphs{g};
  const auto res = regularise_walls(graphs, {storey(0)});
  const auto &w = graphs[0].walls;
  CHECK(res.continuous_lines == 1);
  // One axis, nearer the longer run: the length-weighted mean.
  CHECK_THAT(w[0].axis_offset, WithinAbs(w[1].axis_offset, 1e-9));
  CHECK_THAT(w[1].axis_offset,
             WithinAbs((0.0 * 4.2 + 0.28 * 7.7) / (4.2 + 7.7), 1e-9));
  // Each keeps its thickness.
  CHECK_THAT(w[0].face_hi - w[0].face_lo, WithinAbs(0.2, 1e-9));
  CHECK_THAT(w[1].face_hi - w[1].face_lo, WithinAbs(0.3, 1e-9));
  const auto &rules = w[0].regularised.rules;
  CHECK(std::find(rules.begin(), rules.end(), "continuous") != rules.end());
  // A real step (farther than plan_continuous_m) stays where it is.
  CHECK(w[2].axis_offset == 0.90);
  // The partition's tee end follows the moved line.
  CHECK_THAT(w[3].s1, WithinAbs(-w[0].axis_offset, 1e-9));
}

TEST_CASE("plan regularisation: a double wall becomes one wall",
          "[plan_regularise]") {
  // Family 1 (normal y, s = -x). Two one-sided walls over the same stretch,
  // seen from either side (Sygehusgrunden storey 1, y 7.48 / 7.49): one
  // wall. Two more bounding a 0.25 m body: one measured wall. A one-sided
  // face on a measured wall's face: that wall.
  WallGraph g;
  g.walls = {
      wall(kY, 1, 7.49, -5.5, -2.2, -1),    // 0: seen from -y
      wall(kY, 1, 7.48, -5.35, -2.2, +1),   // 1: seen from +y, 1 cm off
      wall(kY, 1, 0.00, -6.0, 0.0, -1),     // 2: seen from -y at 0
      wall(kY, 1, 0.25, -5.0, 0.0, +1),     // 3: seen from +y at 0.25
      wall(kY, 1, 3.00, -6.0, 0.0, 0, 0.2), // 4: measured, faces 2.9/3.1
      wall(kY, 1, 3.08, -4.0, -1.0, +1),    // 5: its +y face again
      wall(kX, 0, 0.0, -1.0, 8.0, 0, 0.2),  // 6: crosses nothing
  };
  // Wall 6 (x = 0, s = y) tees into wall 3 and wall 1's end.
  g.joins = {{6, 3, WallJoinKind::tee, WallEnd::start, WallEnd::path},
             {1, 6, WallJoinKind::tee, WallEnd::start, WallEnd::path}};
  std::vector<WallGraph> graphs{g};
  const auto res = regularise_walls(graphs, {storey(0)});
  const auto &w = graphs[0].walls;
  CHECK(res.walls_merged == 3);
  REQUIRE(w.size() == 4);
  // 0+1: one wall over the union of the stretches.
  CHECK_THAT(w[0].s0, WithinAbs(-5.5, 1e-9));
  CHECK_THAT(w[0].s1, WithinAbs(-2.2, 1e-9));
  CHECK_THAT(w[0].axis_offset, WithinAbs(7.485, 1e-9));
  // 2+3: a measured wall between the two faces.
  CHECK_THAT(w[1].face_lo, WithinAbs(0.0, 1e-9));
  CHECK_THAT(w[1].face_hi, WithinAbs(0.25, 1e-9));
  CHECK_THAT(w[1].thickness, WithinAbs(0.25, 1e-9));
  CHECK_THAT(w[1].axis_offset, WithinAbs(0.125, 1e-9));
  CHECK_FALSE(w[1].assumed);
  // 4 keeps 5.
  CHECK_THAT(w[2].axis_offset, WithinAbs(3.0, 1e-9));
  const auto &rules = w[1].regularised.rules;
  CHECK(std::find(rules.begin(), rules.end(), "double") != rules.end());
  // The joins follow the kept walls.
  REQUIRE(graphs[0].joins.size() == 2);
  CHECK(graphs[0].joins[0].a == 3);
  CHECK(graphs[0].joins[0].b == 1);
  CHECK(graphs[0].joins[1].a == 0);
  CHECK(graphs[0].joins[1].b == 3);

  // Off: nothing merges.
  std::vector<WallGraph> off{g};
  PlanRegulariseOptions o;
  o.plan_double_m = 0.0;
  CHECK(regularise_walls(off, {storey(0)}, o).walls_merged == 0);
  CHECK(off[0].walls.size() == g.walls.size());
}

TEST_CASE("plan regularisation: a wall stub across a corridor is cut",
          "[plan_regularise]") {
  // A corridor 2.2 m wide between x = 0 and x = 2.2 (family 0, s = y). A
  // room wall along y = 1 (family 1, s = -x) runs from x = -4 through the
  // corridor wall at x = 0 to the far one: the stretch across the corridor
  // is a phantom that splits it in two (Sygehusgrunden storey 1). A wall
  // along y = 5 crossing x = 0 with 3 m either side is a real crossing (the
  // east corridor wall ends at y = 4).
  WallGraph g;
  g.walls = {wall(kX, 0, 0.0, -5.0, 9.0, 0, 0.15), // 0: west corridor wall
             wall(kX, 0, 2.2, -5.0, 4.0, 0, 0.15), // 1: east corridor wall
             wall(kY, 1, 1.0, -2.2, 4.0, -1),      // 2: x -4..2.2
             wall(kY, 1, 5.0, -3.0, 3.0, 0, 0.2)}; // 3: x -3..3
  g.joins = {{2, 1, WallJoinKind::tee, WallEnd::start, WallEnd::path}};
  std::vector<WallGraph> graphs{g};
  const auto res = regularise_walls(graphs, {storey(0)});
  const auto &w = graphs[0].walls;
  CHECK(res.walls_trimmed == 1);
  // Wall 2 now ends on the west corridor wall (s = -x = 0).
  CHECK_THAT(w[2].s0, WithinAbs(0.0, 1e-9));
  CHECK_THAT(w[2].s1, WithinAbs(4.0, 1e-9));
  const auto &rules = w[2].regularised.rules;
  CHECK(std::find(rules.begin(), rules.end(), "trim") != rules.end());
  // Its join moved from the far wall to the one it ends on.
  REQUIRE(graphs[0].joins.size() == 1);
  CHECK(graphs[0].joins[0].a == 2);
  CHECK(graphs[0].joins[0].b == 0);
  CHECK(graphs[0].joins[0].at_a == WallEnd::start);
  CHECK(graphs[0].joins[0].at_b == WallEnd::path);
  // The real crossing stays.
  CHECK(w[3].s0 == -3.0);
  CHECK(w[3].s1 == 3.0);

  // Off: nothing is cut.
  std::vector<WallGraph> off{g};
  PlanRegulariseOptions o;
  o.plan_stub_m = 0.0;
  CHECK(regularise_walls(off, {storey(0)}, o).walls_trimmed == 0);
  CHECK(off[0].walls[2].s0 == -2.2);
}

TEST_CASE("plan regularisation: a facade stacks over three storeys",
          "[plan_regularise]") {
  std::vector<WallGraph> graphs(3);
  const double offsets[] = {0.0, 0.06, -0.05};
  for (int k = 0; k < 3; ++k)
    graphs[static_cast<size_t>(k)].walls = {
        wall(kY, 1, offsets[k], -10.0, 0.0, +1)};
  const auto res = regularise_walls(
      graphs, {storey(0, 0.0), storey(1, 3.0), storey(2, 6.0)});
  CHECK(res.lines == 1);
  CHECK(res.stacked_lines == 1);
  const double line = graphs[0].walls[0].face_hi;
  CHECK_THAT(graphs[1].walls[0].face_hi, WithinAbs(line, 1e-9));
  CHECK_THAT(graphs[2].walls[0].face_hi, WithinAbs(line, 1e-9));
  const auto &rules = graphs[1].walls[0].regularised.rules;
  CHECK(std::find(rules.begin(), rules.end(), "stack") != rules.end());

  // Off: nothing moves.
  std::vector<WallGraph> off(3);
  for (int k = 0; k < 3; ++k)
    off[static_cast<size_t>(k)].walls = {wall(kY, 1, offsets[k], -10, 0, +1)};
  PlanRegulariseOptions o;
  o.plan_regularise = false;
  CHECK(regularise_walls(off, {storey(0), storey(1, 3), storey(2, 6)}, o)
            .walls_moved == 0);
  CHECK(off[1].walls[0].axis_offset == 0.06);
}

TEST_CASE("plan regularisation: grid snap and rounding", "[plan_regularise]") {
  // Storey 0 on the grid (long walls), storey 1 turned 1.5 deg: it turns
  // onto the grid; a 20 deg family stays.
  const double a = 1.5 * std::numbers::pi / 180.0;
  const Eigen::Vector2d dx(std::cos(a), std::sin(a));
  const Eigen::Vector2d dy(-std::sin(a), std::cos(a));
  const double b = 20.0 * std::numbers::pi / 180.0;
  std::vector<WallGraph> graphs(2);
  graphs[0].walls = {wall(kX, 0, 0.0, 0.0, 12.0, 0),
                     wall(kY, 1, 0.0, -12.0, 0.0, 0)};
  graphs[1].walls = {wall(dx, 0, 3.0, 0.0, 4.0, 0),
                     wall(dy, 1, 3.0, -4.0, 0.0, 0),
                     wall({std::cos(b), std::sin(b)}, 2, 1.0, 0.0, 2.0, 0)};
  PlanRegulariseOptions o;
  o.plan_grid_deg = 3.0;
  o.plan_round_m = 0.05;
  const auto res = regularise_walls(graphs, {storey(0), storey(1, 3.0)}, o);
  CHECK(res.walls_turned == 2);
  CHECK_THAT(graphs[1].walls[0].d.y(), WithinAbs(0.0, 1e-9));
  CHECK_THAT(graphs[1].walls[1].d.x(), WithinAbs(0.0, 1e-9));
  CHECK_THAT(graphs[1].walls[0].regularised.angle_delta_deg,
             WithinAbs(-1.5, 1e-9));
  CHECK_THAT(graphs[1].walls[2].d.y(), WithinAbs(std::sin(b), 1e-12));
  // Rounded to 5 cm: every face lands on the grid.
  for (const auto &g : graphs)
    for (const auto &w : g.walls) {
      if (w.family == 2)
        continue;
      const double f = w.face_lo / 0.05;
      CHECK_THAT(f, WithinAbs(std::round(f), 1e-6));
    }
}

TEST_CASE("plan regularisation: a facade's windows align over 3 floors",
          "[plan_regularise]") {
  std::vector<StoreyRecord> storeys{storey(0, 0.0), storey(1, 3.0),
                                    storey(2, 6.0)};
  std::vector<WallRecord> walls;
  for (int k = 0; k < 3; ++k) {
    WallRecord w;
    w.guid = "w" + std::to_string(k);
    w.storey = k;
    w.x0 = 0.0;
    w.y0 = 0.01 * k;
    w.x1 = 10.0;
    w.y1 = 0.01 * k;
    walls.push_back(w);
  }
  // Columns at x = 2, 5, 8; centres jittered by up to 8 cm, widths by 5 cm,
  // sills and heads by 4 cm.
  const double dc[3][3] = {
      {0.00, 0.05, -0.08}, {0.07, -0.04, 0.02}, {-0.05, 0.08, 0.06}};
  const double dw[3][3] = {
      {0.00, 0.05, -0.04}, {0.03, 0.00, 0.02}, {-0.05, 0.01, 0.00}};
  const double dz[3][3] = {
      {0.00, 0.04, -0.03}, {0.02, -0.04, 0.01}, {-0.02, 0.03, 0.04}};
  std::vector<OpeningRecord> ops;
  for (int k = 0; k < 3; ++k)
    for (int c = 0; c < 3; ++c) {
      OpeningRecord o;
      o.wall = "w" + std::to_string(k);
      o.kind = "window";
      const double mid = 2.0 + 3.0 * c + dc[k][c];
      const double width = 1.2 + dw[k][c];
      o.s_start = mid - 0.5 * width;
      o.s_end = mid + 0.5 * width;
      o.z_sill = 3.0 * k + 0.9 + dz[k][c];
      o.z_head = 3.0 * k + 2.1 - dz[k][c];
      o.metadata = R"({"fill":0.1})";
      ops.push_back(o);
    }
  // A door is left alone.
  OpeningRecord door;
  door.wall = "w0";
  door.kind = "door";
  door.s_start = 9.0;
  door.s_end = 9.9;
  door.z_head = 2.05;
  ops.push_back(door);
  const auto before = ops;

  const auto res = regularise_openings(ops, walls, storeys);
  CHECK(res.columns == 3);
  CHECK(res.skipped == 0);
  for (int c = 0; c < 3; ++c) {
    const auto &a = ops[static_cast<size_t>(c)];
    for (int k = 1; k < 3; ++k) {
      const auto &b = ops[static_cast<size_t>(3 * k + c)];
      CHECK_THAT(b.s_start, WithinAbs(a.s_start, 1e-9));
      CHECK_THAT(b.s_end, WithinAbs(a.s_end, 1e-9));
    }
  }
  // One sill and one head per floor.
  for (int k = 0; k < 3; ++k) {
    const auto &a = ops[static_cast<size_t>(3 * k)];
    for (int c = 1; c < 3; ++c) {
      const auto &b = ops[static_cast<size_t>(3 * k + c)];
      CHECK_THAT(b.z_sill, WithinAbs(a.z_sill, 1e-9));
      CHECK_THAT(b.z_head, WithinAbs(a.z_head, 1e-9));
    }
  }
  // Columns stay apart, widths are the column's median.
  CHECK(ops[1].s_start - ops[0].s_end > 1.0);
  CHECK_THAT(ops[0].s_end - ops[0].s_start, WithinAbs(1.2, 1e-9));
  CHECK(ops.back().s_start == 9.0);
  CHECK(ops.back().metadata.empty());
  // Measured values are kept, the evidence too.
  const auto meta = nlohmann::json::parse(ops[4].metadata);
  CHECK(meta["fill"] == 0.1);
  CHECK_THAT(meta["regularised"]["measured"]["s_start"].get<double>(),
             WithinAbs(before[4].s_start, 1e-12));
  CHECK_THAT(meta["regularised"]["measured"]["z_sill"].get<double>(),
             WithinAbs(before[4].z_sill, 1e-12));
}

TEST_CASE("plan regularisation: a window snap never leaves its wall",
          "[plan_regularise]") {
  std::vector<StoreyRecord> storeys{storey(0, 0.0), storey(1, 3.0)};
  std::vector<WallRecord> walls(2);
  walls[0].guid = "a";
  walls[0].storey = 0;
  walls[0].x1 = 10.0;
  walls[1].guid = "b";
  walls[1].storey = 1;
  walls[1].x0 = 0.15; // starts later: a column at its start
  walls[1].x1 = 10.0;
  std::vector<OpeningRecord> ops(2);
  ops[0].wall = "a";
  ops[0].kind = "window";
  ops[0].s_start = 0.0;
  ops[0].s_end = 1.0;
  ops[0].z_sill = 0.9;
  ops[0].z_head = 2.1;
  ops[1].wall = "b";
  ops[1].kind = "window";
  ops[1].s_start = 0.0; // x 0.15 .. 1.15
  ops[1].s_end = 1.0;
  ops[1].z_sill = 3.9;
  ops[1].z_head = 5.1;
  const auto res = regularise_openings(ops, walls, storeys);
  // The mean centre (x 0.575) would push the lower one to s = 0.075 ok, and
  // the upper one to s = -0.075: off its wall, skipped.
  CHECK(res.skipped == 1);
  CHECK(ops[1].s_start == 0.0);
}

TEST_CASE("plan regularisation: openings are found on the measured wall",
          "[plan_regularise]") {
  // Regularised 6 cm sideways, its start pulled 0.1 m back by a corner.
  WallRecord w;
  w.guid = "w";
  w.x0 = -0.1;
  w.y0 = 0.06;
  w.x1 = 5.0;
  w.y1 = 0.06;
  w.metadata = R"({"regularised":{"measured":{"x0":0.0,"y0":0.0,"x1":5.0,)"
               R"("y1":0.0},"rules":["collinear","join"]}})";
  const auto measured = reusex::geometry::measured_walls({w});
  REQUIRE(measured.size() == 1);
  CHECK(measured[0].x0 == 0.0);
  CHECK(measured[0].y0 == 0.0);
  CHECK(measured[0].y1 == 0.0);

  std::vector<OpeningRecord> ops(1);
  ops[0].wall = "w";
  ops[0].s_start = 1.0; // x 1 .. 2 on the measured wall
  ops[0].s_end = 2.0;
  reusex::geometry::openings_onto_walls(ops, measured, {w});
  CHECK_THAT(ops[0].s_start, WithinAbs(1.1, 1e-12));
  CHECK_THAT(ops[0].s_end, WithinAbs(2.1, 1e-12));

  // A wall that was not regularised is its own measured wall.
  WallRecord plain = w;
  plain.metadata = R"({"low_seen_share":true})";
  CHECK(reusex::geometry::measured_walls({plain})[0].y0 == 0.06);
}

TEST_CASE("plan regularisation: the stub cut is the side not seen",
          "[plan_regularise]") {
  // Sygehusgrunden storey 1, y 7.5: a room wall seen over 0.9 m west of the
  // corridor wall (x = 0), carried 2.2 m across the corridor to x = 2.2 by a
  // doorway bridge. The seen profile says which side is evidence: the long,
  // unseen side goes, though it is the longer one.
  WallGraph g;
  g.walls = {wall(kX, 0, 0.0, -5.0, 9.0, 0, 0.15), // 0: west corridor wall
             wall(kX, 0, 2.2, -5.0, 4.0, 0, 0.15), // 1: east corridor wall
             wall(kY, 1, 1.0, -2.2, 0.9, -1)};     // 2: x -0.9..2.2
  // s = -x: columns of 0.25 m from s = -2.25; seen over s in [0, 0.9].
  auto &w = g.walls[2];
  w.seen_cell_m = 0.25;
  w.seen_i0 = -9;
  w.seen_columns.assign(13, 0.0f);
  for (int i = 0; i <= 3; ++i)
    w.seen_columns[static_cast<size_t>(i + 9)] = 0.6f;
  std::vector<WallGraph> graphs{g};
  const auto res = regularise_walls(graphs, {storey(0)});
  CHECK(res.walls_trimmed == 1);
  CHECK_THAT(graphs[0].walls[2].s0, WithinAbs(0.0, 1e-9));
  CHECK_THAT(graphs[0].walls[2].s1, WithinAbs(0.9, 1e-9));

  // Seen on both sides: a real crossing, nothing is cut.
  std::vector<WallGraph> both{g};
  std::fill(both[0].walls[2].seen_columns.begin(),
            both[0].walls[2].seen_columns.end(), 0.5f);
  CHECK(regularise_walls(both, {storey(0)}).walls_trimmed == 0);
}

TEST_CASE("plan regularisation: an unseen fragment across a corridor goes",
          "[plan_regularise]") {
  // Sygehusgrunden storey 1, y -6: a one-sided wall only across the
  // corridor (x 0..2.2), wall columns at its ends only (the corridor walls'
  // own points): it splits the corridor. The same wall seen over its
  // stretch (a corridor door wall) stays, and so does a measured one.
  WallGraph g;
  g.walls = {wall(kX, 0, 0.0, -9.0, 9.0, 0, 0.15),  // 0
             wall(kX, 0, 2.2, -9.0, 9.0, 0, 0.15),  // 1
             wall(kY, 1, -6.0, -2.2, 0.0, +1),      // 2: x 0..2.2, unseen
             wall(kY, 1, 6.0, -2.2, 0.0, +1),       // 3: x 0..2.2, seen
             wall(kY, 1, 0.0, -2.2, 0.0, 0, 0.12)}; // 4: measured
  for (size_t i : {2u, 3u, 4u}) {
    auto &w = g.walls[i];
    w.seen_cell_m = 0.25;
    w.seen_i0 = -9;
    w.seen_columns.assign(9, i == 2 ? 0.05f : 0.7f);
    w.seen_columns.front() = w.seen_columns.back() = 0.8f;
  }
  g.walls[4].seen_columns.assign(9, 0.05f);
  g.joins = {{2, 0, WallJoinKind::tee, WallEnd::end, WallEnd::path},
             {2, 1, WallJoinKind::tee, WallEnd::start, WallEnd::path},
             {3, 1, WallJoinKind::tee, WallEnd::start, WallEnd::path}};
  std::vector<WallGraph> graphs{g};
  const auto res = regularise_walls(graphs, {storey(0)});
  CHECK(res.walls_dropped == 1);
  REQUIRE(graphs[0].walls.size() == 4);
  CHECK(graphs[0].walls[2].axis_offset == 6.0);
  REQUIRE(graphs[0].joins.size() == 1);
  CHECK(graphs[0].joins[0].a == 2);
  CHECK(graphs[0].joins[0].b == 1);
}
