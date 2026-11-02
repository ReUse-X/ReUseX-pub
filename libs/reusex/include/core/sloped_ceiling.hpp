// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// @file
/// Sloped ceilings and sloped wall tops (reusex-546): evaluating
/// SpaceRecord::ceiling_planes and WallRecord::top, checking them, and their
/// stored form (spaces.ceiling_planes / walls.top_profile, schema v39: JSON
/// text, "" = none).
///
/// The model is deliberately small. A space's ceiling is the lowest of its
/// flat z_ceiling and its sloped planes (a concave, piecewise-planar
/// surface): that covers a pitched attic (two planes meeting at the ridge,
/// z_ceiling at or above the ridge), a mono-pitch ceiling (one plane,
/// z_ceiling at its high end) and a flat centre with sloped sides (z_ceiling
/// the flat part). A dormer (a ceiling that rises again inside the room) is
/// not representable; it would need a union, not a minimum. A wall's top is
/// a polyline along its axis, so gable walls (a peak) and side walls under a
/// mono pitch are both one profile.
///
/// The eaves of a sloped ceiling (reusex-ow8p) ride along in the space's
/// metadata, under "roof" (RoofEdges): the facade walls that stand under its
/// slope (knee walls: their top is where the slope meets them) and its
/// dormers (a facade stretch that rises to a flat dormer ceiling above the
/// slope). Both are measured from the cloud by fit_sloped_ceilings() and
/// drawn by build_solids().

#include "reusex/core/solid_model.hpp"

#include <array>
#include <string>
#include <vector>

namespace reusex::core {

/// The ceiling of @p space at (x, y) in its level frame:
/// min(z_ceiling, every plane's z at (x, y)).
double ceiling_at(const SpaceRecord &space, double x, double y);

/// A dormer in a sloped ceiling (reusex-ow8p), level frame: a stretch
/// [s0, s1] of facade wall @p wall (s along its axis from (x0, y0)) that
/// rises to the dormer's flat ceiling @p z instead of stopping at the knee
/// line. @p outline is the dormer's plan: the wall axis at s0 and s1, then
/// inwards to where the slope reaches z (either orientation).
struct Dormer {
  std::string wall;
  double s0 = 0.0, s1 = 0.0;
  double z = 0.0;
  std::vector<PlanPoint> outline; ///< {axis s0, axis s1, inner s1, inner s0}
};

/// What a sloped ceiling does at the facade (reusex-ow8p): the walls under
/// its slope (their top follows it) and its dormers.
struct RoofEdges {
  std::vector<std::string> knee_walls;
  std::vector<Dormer> dormers;
  [[nodiscard]] bool empty() const {
    return knee_walls.empty() && dormers.empty();
  }
};

/// @p space's RoofEdges, from its metadata ("roof"); empty when there are
/// none or the metadata does not hold a valid "roof" entry.
RoofEdges roof_edges(const SpaceRecord &space);

/// @p metadata (a JSON object, or "") with "roof" set to @p edges, or
/// removed when @p edges is empty.
std::string with_roof_edges(const std::string &metadata,
                            const RoofEdges &edges);

/// The ceiling of @p space at (x, y) with its dormers: ceiling_at(), raised
/// to a dormer's z inside its outline.
double ceiling_at(const SpaceRecord &space, const std::vector<Dormer> &dormers,
                  double x, double y);

/// The top of @p wall at @p s along its axis from (x0, y0): the top profile
/// interpolated (constant past its ends), capped at z_base + height; that
/// cap alone when the wall has no profile.
double wall_top_at(const WallRecord &wall, double s);

/// @p plane carried through @p m (4x4 row-major, a level frame change):
/// through the images of three of its points. A plane that @p m turns
/// vertical keeps its old gradient.
CeilingPlane map_ceiling_plane(const CeilingPlane &plane,
                               const std::array<double, 16> &m);

/// Steepest ceiling plane accepted anywhere [deg]: a ceiling is not a wall.
inline constexpr double kMaxCeilingSlopeDeg = 80.0;

/// The reason @p space's ceiling planes are invalid, or "" when they are
/// fine: a non-finite number, a plane steeper than kMaxCeilingSlopeDeg, or a
/// ceiling below z_floor at a vertex of the outer ring (the ceiling is
/// concave, so its minimum over the footprint lies on a vertex).
std::string check_ceiling_planes(const SpaceRecord &space);

/// The reason @p wall's top profile is invalid, or "": fewer than 2 points
/// when present, s not strictly increasing, a non-finite number, or a top at
/// or below z_base.
std::string check_wall_top(const WallRecord &wall);

/// Stored form: "" for none, else a JSON array
/// [{"at": [x, y, z], "slope": [dzdx, dzdy]}, ...].
std::string encode_ceiling_planes(const std::vector<CeilingPlane> &planes);
/// @throws std::runtime_error for text that is not that form.
std::vector<CeilingPlane> decode_ceiling_planes(const std::string &text);

/// Stored form: "" for none, else a JSON array [[s, z], ...].
std::string encode_wall_top(const std::vector<PlanPoint> &top);
/// @throws std::runtime_error for text that is not that form.
std::vector<PlanPoint> decode_wall_top(const std::string &text);

} // namespace reusex::core
