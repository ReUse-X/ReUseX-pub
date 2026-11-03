// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "reusex/io/speckle_bundle_options.hpp"
#include "reusex/io/speckle_reader.hpp"

#include <nlohmann/json.hpp>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

/// @file
/// Speckle bundle-only versions (reusex-fo1t): read-only receive of the
/// Parquet "artefact bundle" a version carries when its `referencedObject`
/// is `bundle.<project>.<model>.<version>` (Speckle.Sdk 2026.9+).
///
/// The protocol mirrors the official SDKs' receive path (specklepy
/// `specklepy/bundle/download.py` + `bundle_reader.py`, commit 4b31244a;
/// speckle-sharp-sdk `Pipelines/Receive/Artifacts/ArtifactDownloader.cs`,
/// commit 99476158):
///  1. `GET {server}/api/v2/projects/{p}/models/{m}/versions/{v}/artifacts`
///     with the token lists the bundle's files, each with a presigned
///     storage URL (`parse_artifact_listing`);
///  2. each file except `*.viewer.*` is fetched from its presigned URL with
///     a plain GET and **no** Authorization header (the SDKs use a separate,
///     token-less HTTP client for storage);
///  3. the Parquet tables are read with DuckDB (`read_bundle_directory`,
///     needs REUSEX_HAVE_DUCKDB) into `BundleTables`;
///  4. `project_bundle_to_revit_tree` rebuilds the object tree the Speckle v3
///     Revit connector publishes (`Collection` / `levelProxies` /
///     `instanceDefinitionProxies` / RevitObjects with `displayValue`), so
///     `import_revit` reads both formats the same way.
/// The SDKs' optional `version.received` mutation is never sent.

namespace reusex::io::speckle {

/// True when reusex_io was built with DuckDB (REUSEX_HAVE_DUCKDB), i.e. it
/// can read bundle-only versions. Without it they are refused with
/// `BundleVersionUnsupported`.
bool has_bundle_support();

// ---------------------------------------------------------------- SGEO ---

/// SGEO v1 header (16 bytes, little endian): magic "SGEO", version 1,
/// primitive type, flags, units code, CRC32 of the body. Port of specklepy
/// `specklepy/bundle/sgeo.py` (itself a port of the .NET
/// `Speckle.Objects/Utils/SgeoEncoder.cs`).
struct SgeoHeader {
  std::uint8_t version = 0;
  std::uint8_t primitive = 0; ///< 0 = mesh, 1 = line, 2 = polyline, ...
  std::uint16_t flags = 0;
  std::uint16_t units_code = 0; ///< 1 mm, 2 cm, 3 m, 4 km, 5 in, 6 ft, ...
  std::uint32_t crc = 0;
};

inline constexpr std::uint8_t kSgeoMesh = 0;

/// Canonical CRC-32 (IEEE 802.3, reflected polynomial 0xEDB88320), what
/// SGEO stores for its body.
std::uint32_t sgeo_crc32(std::string_view data);

/// Speckle unit string of an SGEO units code ("" for 0 / unknown).
std::string sgeo_units(std::uint16_t code);

/// @throws std::runtime_error for a blob shorter than 16 bytes, a bad
///         magic or an unsupported version.
SgeoHeader decode_sgeo_header(std::string_view blob);

/// A decoded SGEO mesh in Speckle's flat layout: `vertices` x,y,z...,
/// `faces` [n, i0..in-1, ...] (n = 0 / 1 legacy triangle / quad markers).
struct SgeoMesh {
  std::vector<double> vertices;
  std::vector<std::int32_t> faces;
  std::string units; ///< "" when the blob gives none
};

/// Decode a MESH blob (vertex and face arrays; normals, UVs and colours are
/// not read).
/// @throws std::runtime_error for a non-mesh primitive, a CRC mismatch
///         (when @p verify_crc), or arrays that run past the blob.
SgeoMesh decode_sgeo_mesh(std::string_view blob, bool verify_crc = true);

// -------------------------------------------------------------- tables ---

/// One EAV value: SDK `coalesce` order is boolean, double, string.
struct BundleValue {
  std::optional<std::string> s;
  std::optional<double> d;
  std::optional<bool> b;
  std::string unit;                     ///< "" = none
  std::string internal_definition_name; ///< Revit parameter id, "" = none
};

struct BundleEavRow {
  std::int32_t key = 0;  ///< object_index (eav) or type_index (type_eav)
  std::int32_t path = 0; ///< path_index
  BundleValue value;
};

/// A row of `envelope.nodes` (synthetic nodes: definitions, instances,
/// materials, levels, containers).
struct BundleNode {
  std::int32_t id = 0;
  std::int32_t kind =
      0; ///< 1 definition, 2 instance, 3 material, 5 level, 7 container
  std::string name;
  std::optional<std::int32_t> def_ref;
  std::string transform; ///< 16 comma-separated row-major values, or ""
  std::string units;
  std::string subtype;
  std::optional<double> elevation;
};

/// A row of `envelope.relations` (typed edges; ids live in the namespaces
/// of `envelope.rel_types`: object, geometry or node).
struct BundleRelation {
  std::int32_t rel = 0;
  std::int32_t src = 0;
  std::int32_t dst = 0;
  std::int32_t ord = 0;
};

/// Relation ids of bundle spec 1.2.0 that the projection reads
/// (`specklepy/bundle/spec/bundle_spec.py`).
enum class BundleRel : std::int32_t {
  display = 1,          ///< object -> geometry
  subelement = 3,       ///< object -> object
  defines = 4,          ///< definition node -> geometry
  on_level = 7,         ///< object -> level node
  display_instance = 8, ///< object -> instance node
  defines_instance = 9, ///< definition node -> instance node (nested)
  hosted_on = 22,       ///< object -> object
};

enum class BundleNodeKind : std::int32_t {
  definition = 1,
  instance = 2,
  material = 3,
  color = 4,
  level = 5,
  container = 7,
};

struct BundleGeometry {
  std::int32_t index = 0;
  std::string content; ///< SGEO blob
  std::string type;    ///< "mesh", "line", ...
};

/// The bundle's tables, as `read_bundle_directory` reads them (or a test
/// builds them).
struct BundleTables {
  std::string schema_version;                          ///< envelope.meta
  std::unordered_map<std::int32_t, std::string> paths; ///< path_index -> path
  std::map<std::int32_t, std::string>
      objects; ///< object_index -> applicationId
  std::vector<BundleEavRow> eav;
  std::vector<BundleEavRow> type_eav;
  std::unordered_map<std::int32_t, std::int32_t>
      object_type; ///< object -> type
  std::vector<BundleNode> nodes;
  std::vector<BundleRelation> relations;
  std::vector<BundleGeometry> geometries;
};

/// Read a downloaded bundle directory with DuckDB (in-memory database,
/// extension autoload / autoinstall off, external access off except for
/// @p dir, configuration locked). Required tables: `*.eav.objects`,
/// `*.eav.paths`, `*.eav.eav`, `*.envelope.nodes`, `*.envelope.relations`;
/// optional: `*.eav.type_eav`, `*.eav.object_type`, `*.envelope.meta`,
/// `*.geometries*.parquet`.
/// A table is the file `<base><suffix>` whose base has no '.' (geometry
/// shards: `<base>.geometries*.parquet`). Bounds: `BundleReadOptions` (rows
/// per table, bytes of strings / blobs overall, DuckDB memory and threads,
/// cancel token per chunk).
/// @throws BundleVersionUnsupported when built without DuckDB;
///         std::runtime_error for a missing required table, a bound
///         exceeded, a cancel or a DuckDB error.
BundleTables read_bundle_directory(const std::filesystem::path &dir,
                                   const BundleReadOptions &options = {});

// ---------------------------------------------------------- projection ---

/// What `project_bundle_to_revit_tree` did.
struct BundleProjectionStats {
  std::size_t objects = 0;            ///< RevitObjects produced
  std::size_t levels = 0;             ///< levelProxies
  std::size_t definitions = 0;        ///< instanceDefinitionProxies
  std::size_t instances = 0;          ///< InstanceProxies (placed + nested)
  std::size_t meshes = 0;             ///< SGEO meshes decoded
  std::size_t geometries_skipped = 0; ///< non-mesh primitives (lines, ...)
  std::size_t geometries_bad = 0;     ///< undecodable / CRC mismatch
  std::size_t geometries_missing = 0; ///< referenced but not in the tables
  std::size_t hosted = 0; ///< parentApplicationId filled in from HOSTED_ON
  std::size_t nested = 0; ///< objects nested under a SUBELEMENT parent
  std::size_t shared_meshes = 0;        ///< DISPLAY meshes of several objects,
                                        ///< emitted once behind a definition
  std::size_t instances_unresolved = 0; ///< placements without a definition
  std::size_t paths_dropped = 0;        ///< property rows over the path bounds
  std::size_t levels_dropped = 0;       ///< levels no object is ON_LEVEL of
  std::size_t levels_renamed = 0;       ///< duplicate level names disambiguated
  std::size_t levels_with_properties = 0; ///< matched to a Levels object
  std::size_t duplicate_app_ids = 0;      ///< applicationIds suffixed
  std::size_t mesh_values = 0;            ///< vertex coords + face ints emitted
  std::size_t expanded_vertices = 0; ///< vertices once placements are baked
  std::size_t property_rows = 0;     ///< property rows applied
  std::size_t nodes = 0;             ///< JSON nodes emitted (approximate)
  std::map<std::string, std::size_t> speckle_types; ///< objects per type
};

/// Rebuild the resolved object tree of a Speckle v3 Revit model from bundle
/// tables:
///  - root `Speckle.Core.Models.Collections.Collection` with `units` (the
///    objects' most common `units`), `elements` (top-level objects; a
///    SUBELEMENT child is nested in its parent's `elements`),
///    `levelProxies` (one per LEVEL node: name, elevation, units, and the
///    applicationIds ON_LEVEL it), `instanceDefinitionProxies` (one per
///    DEFINITION node, `def-<id>`, objects `geo-<index>` / `inst-<id>`) and
///    `definitionGeometry` (those meshes and nested instance proxies);
///  - every object: its EAV paths as nested JSON (type properties first,
///    the instance's over them), a path under `properties.Parameters.` as a
///    Revit parameter `{name, value, units, internalDefinitionName}`;
///    `applicationId`, `level` (the ON_LEVEL node's name),
///    `properties.parentApplicationId` from HOSTED_ON when the EAV has none,
///    and `displayValue`: its DISPLAY meshes and DISPLAY_INSTANCE proxies,
///    in `ord` order.
/// Shared geometry: a DISPLAY mesh of several objects is emitted once, in
/// `definitionGeometry` behind a definition `shared-<geometry>`, and each
/// object places it with an identity InstanceProxy. Levels: a LEVEL node no
/// object is ON_LEVEL of is dropped; duplicate names get " (2)", " (3)" in
/// (elevation, id) order; a `Levels` object of the same name (if the bundle
/// has one) lends its applicationId and properties (building storey flag,
/// elementId, IFC GUID). Duplicate applicationIds get "#2", "#3" in object
/// order. A property path over the bounds is dropped. A placement whose
/// instance has no definition is skipped.
/// Pure: no I/O. Undecodable geometry is skipped, counted and logged at
/// warn.
/// @throws std::runtime_error past a `BundleProjectionOptions` budget
///         (mesh values, expanded vertices, property rows, nodes) or on a
///         cancel.
nlohmann::json
project_bundle_to_revit_tree(const BundleTables &tables,
                             BundleProjectionStats *stats = nullptr,
                             const BundleProjectionOptions &options = {});

// --------------------------------------------------------------- fetch ---

/// What `fetch_bundle_tree` downloaded and built.
struct BundleFetchReport {
  std::size_t files = 0;
  std::size_t bytes = 0;
  BundleProjectionStats projection;
};

/// Download a bundle-only version (`SpeckleReader::download_bundle` into a
/// private temporary directory, removed afterwards), read it
/// (`read_bundle_directory`) and project it (`project_bundle_to_revit_tree`).
/// @p reference must be a bundle reference of @p project.
/// @throws BundleVersionUnsupported when built without DuckDB;
///         std::invalid_argument for a malformed reference or one of
///         another project; std::runtime_error for HTTP / read errors.
nlohmann::json
fetch_bundle_tree(const SpeckleReader &reader, const std::string &project,
                  const std::string &reference,
                  BundleFetchReport *report = nullptr,
                  const BundleReadOptions &read_options = {},
                  const BundleProjectionOptions &projection_options = {});

namespace detail {
/// Run one SQL statement in the sandbox `read_bundle_directory` uses for
/// @p dir; returns "" on success, else DuckDB's error. Exposed for the
/// sandbox regression tests only.
std::string bundle_sandbox_query_for_tests(const std::filesystem::path &dir,
                                           const std::string &sql);
} // namespace detail

} // namespace reusex::io::speckle
