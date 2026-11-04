// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Records of the typed building model (solid-model workstream, reusex-z1v).
//
// Plain PODs, owned by core (the #227 pattern): a stage writes them, ProjectDB
// stores them (one table each, added by WP1..WP6), io/solid_model_json.hpp
// reads and writes them as one JSON document, and the benchmark
// (reconstruction/solid_metrics.hpp) compares two of them. Core needs no
// geometry types for any of this.
//
// Conventions shared by every record (docs/design/solid-model-json.md):
//  - Lengths in metres. Coordinates of walls, spaces and openings are in the
//    level frame of their storey: z up, floor of storey 0 at z = 0. A storey's
//    `level_transform` maps world -> level frame.
//  - Records refer to each other by stable keys, never by table row id:
//    a storey by its `idx`, a wall / space by its `guid`. `id` is the
//    ProjectDB row id (-1 when the record has not been stored).
//  - `source` is "detected" | "manual" | "imported"; `locked` = a re-run must
//    not overwrite the row; `confidence` 0..1, -1 unknown; `metadata` opaque
//    JSON text for evidence that has no field yet ("" = none).

#include <algorithm>
#include <array>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace reusex::core {

/// A 2D point in a storey's level frame [m].
using PlanPoint = std::array<double, 2>;

/// One plane of a sloped ceiling (reusex-546), in the storey's level frame:
/// z(x, y) = at[2] + slope[0] * (x - at[0]) + slope[1] * (y - at[1]).
/// A point on the plane plus its gradient, so an authored plan can say "2.38 m
/// at the facade, rising 0.17 m per m northwards". A ceiling cannot be
/// vertical, so the gradient is finite (the JSON reader caps it at 80 deg).
struct CeilingPlane {
  std::array<double, 3> at{0.0, 0.0, 0.0}; ///< a point on the plane [m]
  std::array<double, 2> slope{0.0, 0.0};   ///< dz/dx, dz/dy [m/m]

  [[nodiscard]] double z_at(double x, double y) const {
    return at[2] + slope[0] * (x - at[0]) + slope[1] * (y - at[1]);
  }
};

/// One storey: a floor level and the frame its elements are expressed in.
struct StoreyRecord {
  int64_t id = -1;
  std::string guid;
  int idx = 0;          ///< 0 = lowest storey; the key other records use
  std::string name;     ///< display name, e.g. "Stueetage"
  double z_floor = 0.0; ///< floor height in the level frame [m]
  double z_ceiling = 0.0;
  /// World -> level frame, 4x4 row-major. Identity when not measured.
  std::array<double, 16> level_transform{1, 0, 0, 0, 0, 1, 0, 0,
                                         0, 0, 1, 0, 0, 0, 0, 1};
  /// The ground (entrance) storey, "stueetage" (schema v44): at most one
  /// storey of a project; set by hand (storey.update), never detected.
  bool is_ground = false;
  std::string source = "detected";
  bool locked = false;
  double confidence = -1.0;
  std::string metadata;
};

/// One wall, as its centre line (axis) plus a measured thickness.
struct WallRecord {
  int64_t id = -1;
  std::string guid;
  int storey = 0; ///< StoreyRecord::idx of the storey it stands on
  double x0 = 0.0, y0 = 0.0, x1 = 0.0, y1 = 0.0; ///< axis endpoints
  double z_base = 0.0;
  double height = 0.0;
  double thickness = -1.0; ///< [m]; -1 = unknown (e.g. a single face)
  double thickness_ci_lo = -1.0, thickness_ci_hi = -1.0;
  double lean_deg = 0.0;
  bool is_external = false;
  bool assumed = false;     ///< one-sided: thickness defaulted, not measured
  double seen_share = -1.0; ///< fraction of the face area with evidence; -1 n/a
  int face_a_points = 0, face_b_points = 0;
  double co_visibility = -1.0;
  int axis_family = -1;
  /// Sloped top (reusex-546; gable and knee walls under a sloped ceiling):
  /// (s, z) points, s along the axis from (x0, y0) strictly increasing, z in
  /// the level frame. The top at s is the profile interpolated linearly
  /// (held constant past its ends), capped at z_base + height. Empty: the
  /// top is flat at z_base + height.
  std::vector<PlanPoint> top;
  std::string source = "detected";
  bool locked = false;
  double confidence = -1.0;
  std::string metadata;
};

/// How two walls meet (IfcRelConnectsPathElements).
struct WallJoinRecord {
  std::string wall_a; ///< WallRecord::guid
  std::string wall_b; ///< WallRecord::guid
  std::string kind;   ///< corner | tee | collinear
  std::string at_a;   ///< start | end | path
  std::string at_b;   ///< start | end | path
};

/// A space's footprint: one outer ring and optional holes, in the level frame.
/// Rings are open (the first point is not repeated) and have >= 3 points.
struct SpacePolygon {
  std::vector<PlanPoint> outer;
  std::vector<std::vector<PlanPoint>> holes;
};

/// One space (room, corridor, open plan).
struct SpaceRecord {
  int64_t id = -1;
  std::string guid;
  int storey = 0; ///< StoreyRecord::idx
  std::string name;
  std::string kind = "unknown"; ///< room | corridor | open_plan | unknown
  SpacePolygon polygon;
  double z_floor = 0.0, z_ceiling = 0.0;
  /// Sloped ceiling (reusex-546): the ceiling at (x, y) is the lowest of
  /// z_ceiling and these planes, so a pitched (attic) ceiling, a mono-pitch
  /// one and a flat centre with sloped sides are all z_ceiling (the flat
  /// part, or the ridge) cut by the sloped planes. Empty: flat at z_ceiling.
  /// core::ceiling_at() evaluates it.
  std::vector<CeilingPlane> ceiling_planes;
  /// Derived quantities; -1 = not computed.
  double area = -1.0, perimeter = -1.0, volume = -1.0, compactness = -1.0;
  int room_label = -1; ///< label in the "rooms" cloud, -1 if none
  std::string source = "detected";
  bool locked = false;
  double confidence = -1.0;
  std::string metadata;
};

/// The stretch of a wall that bounds a space (IfcRelSpaceBoundary).
struct SpaceBoundaryRecord {
  std::string space;                 ///< SpaceRecord::guid
  std::string wall;                  ///< WallRecord::guid
  std::string side;                  ///< a | b
  double s_start = 0.0, s_end = 0.0; ///< along the wall axis from (x0,y0) [m]
};

/// A door, window or plain opening in a wall.
struct OpeningRecord {
  int64_t id = -1;
  std::string guid;
  std::string wall;             ///< host WallRecord::guid
  std::string kind = "opening"; ///< door | window | opening
  /// Extent along the host wall's axis, measured from (x0,y0) [m].
  double s_start = 0.0, s_end = 0.0;
  /// Bottom and top in the level frame [m].
  double z_sill = 0.0, z_head = 0.0;
  std::string state = "unknown"; ///< open | closed | unknown (doors)
  bool is_external = false;
  std::string source_instance_guid;
  /// The leaf (schema v42, reusex-w6jc); "unknown" / "" / -1 for what
  /// nobody said, which is what the detection stages write.
  /// swing_side: a | b | unknown -- the side of the host wall the leaf
  /// opens to: a = left of the axis walking (x0,y0) -> (x1,y1) (+t, the
  /// wall's face_a side, as SpaceBoundaryRecord::side), b = right. Revit's
  /// "flip facing" toggles it.
  std::string swing_side = "unknown";
  /// hinge: start | end | unknown -- the jamb the leaf is hung on: start =
  /// at s_start, end = at s_end. Revit's "flip hand" toggles it. Meaningless
  /// for a double / sliding leaf (kept as set).
  std::string hinge = "unknown";
  /// How it opens (core::kOpeningOperations): doors single_swing |
  /// double_swing | sliding | folding | revolving; windows fixed | casement |
  /// tilt_turn | top_hung | bottom_hung | pivot | sliding; or unknown.
  std::string operation = "unknown";
  /// A type label, e.g. "Enkeltdør 90"; "" none. Free text.
  std::string type_name;
  /// Frame depth across the wall [m]; -1 unknown.
  double frame_depth = -1.0;
  std::string source = "detected";
  bool locked = false;
  double confidence = -1.0;
  std::string metadata;
};

/// The values OpeningRecord::operation may take.
inline constexpr std::array<std::string_view, 13> kOpeningOperations = {
    "unknown",     "single_swing", "double_swing", "sliding",   "folding",
    "revolving",   "fixed",        "casement",     "tilt_turn", "top_hung",
    "bottom_hung", "pivot",        "other"};

/// Whether @p v is one of kOpeningOperations.
[[nodiscard]] inline bool is_opening_operation(std::string_view v) {
  return std::find(kOpeningOperations.begin(), kOpeningOperations.end(), v) !=
         kOpeningOperations.end();
}

/// A floor, ceiling or roof slab (reusex-d5us, schema v44): its outline in
/// plan and its top, extruded down by its thickness. The solids stage
/// writes one detected row per slab piece it derives (floor per storey and
/// level, roof over the top storey); a row edited or drawn by hand is
/// locked, survives re-runs and is built from its fields instead.
struct SlabRecord {
  int64_t id = -1;
  std::string guid;
  int storey = 0;             ///< StoreyRecord::idx
  std::string kind = "floor"; ///< floor | ceiling | roof
  SpacePolygon outline;       ///< in the storey's level frame
  double z_top = 0.0;         ///< level frame [m]
  double thickness = 0.20;    ///< > 0 [m]; the slab is [z_top - t, z_top]
  /// Sloped top (a roof over sloped ceilings): the top at (x, y) is the
  /// lowest of z_top and these planes, as core::ceiling_at() for a space.
  /// Empty: flat at z_top.
  std::vector<CeilingPlane> top_planes;
  std::string source = "detected";
  bool locked = false;
  double confidence = -1.0;
  std::string metadata;
};

/// A zone: a named group of spaces on one or more storeys (a dwelling, a
/// fire cell, a use; IfcZone). Drawn by hand in the Rediger editor
/// (reusex-pccw); no stage detects one. Its area and volume are its
/// members' (core::zone_measures()).
struct ZoneRecord {
  int64_t id = -1;
  std::string guid;
  std::string name;
  std::string use; ///< free-text category, e.g. "bolig", "brandcelle"
  /// Member SpaceRecord::guid, in the order given; a deleted space leaves
  /// every zone it was in.
  std::vector<std::string> spaces;
  std::string source = "manual";
  bool locked = false;
  double confidence = -1.0;
  std::string metadata;
};

/// The role one plane of the "planes" cloud plays (one row per plane label;
/// the plane_classes table, schema v31, keys it by cloud + plane_label).
struct PlaneClassRecord {
  int plane_label = 0;
  std::string orientation; ///< horizontal | vertical | slanted
  int semantic_class = -1; ///< majority API class over the inliers, -1 none
  double class_confidence = -1.0; ///< vote share of that class, -1 none
  std::string role = "unknown";   ///< wall|floor|ceiling|slab|furniture|...
  int storey = -1;                ///< StoreyRecord::idx, -1 none
  int point_count = 0;
  double area = -1.0, z_min = 0.0, z_max = 0.0;
  /// Who decided `role`: semantic (the class map) | rules (geometry, no
  /// usable class) | overruled (the class's role was contradicted by the
  /// geometry; rules won) | none (no points) | manual (set by hand).
  std::string role_source = "none";
  /// Stable key: uuid5(project_guid, "plane_classes/<cloud>/<plane_label>")
  /// for a detected row. "" in a record that was never stored.
  std::string guid;
  std::string source = "detected";
  bool locked = false;
  std::string metadata;
};

/// A typed relation between two elements, by guid (#138).
struct SceneRelationRecord {
  std::string from_guid, to_guid;
  std::string relation_type; ///< contains | connects_to | adjacent_to | ...
  double weight = 1.0;
  std::string metadata;
};

/// Which element kinds a model actually carries.
///
/// An element kind with no rows is ambiguous: "this model has no walls" vs
/// "this model does not say anything about walls" (a reference that only
/// authors spaces, or a project whose walls table does not exist yet). The
/// benchmark scores the first as a miss and reports the second as n/a. A
/// non-empty vector always counts as provided; set a flag here to say "provided
/// and empty".
struct SolidModelProvided {
  bool storeys = false;
  bool walls = false;
  bool spaces = false;
  bool openings = false;
  bool plane_classes = false;
};

/// The whole typed building model of one project (or one authored reference).
struct SolidModel {
  std::string name;        ///< free text, e.g. "NewOffice reference"
  std::string description; ///< free text: who authored it, how, from what
  std::vector<StoreyRecord> storeys;
  std::vector<WallRecord> walls;
  std::vector<WallJoinRecord> wall_joins;
  std::vector<SpaceRecord> spaces;
  std::vector<SpaceBoundaryRecord> space_boundaries;
  std::vector<OpeningRecord> openings;
  std::vector<PlaneClassRecord> plane_classes;
  std::vector<SceneRelationRecord> scene_relations;
  SolidModelProvided provided;

  [[nodiscard]] bool has_storeys() const {
    return provided.storeys || !storeys.empty();
  }
  [[nodiscard]] bool has_walls() const {
    return provided.walls || !walls.empty();
  }
  [[nodiscard]] bool has_spaces() const {
    return provided.spaces || !spaces.empty();
  }
  [[nodiscard]] bool has_openings() const {
    return provided.openings || !openings.empty();
  }
  [[nodiscard]] bool has_plane_classes() const {
    return provided.plane_classes || !plane_classes.empty();
  }
};

} // namespace reusex::core
