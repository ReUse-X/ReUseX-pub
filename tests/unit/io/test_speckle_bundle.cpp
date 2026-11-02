// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Offline tests for Speckle bundle-only versions (reusex-fo1t): the SGEO
// decoder (hand-built blobs), the artifact listing / URL policy, the
// projection of bundle tables onto the Revit connector's object tree, and
// (with DuckDB) the Parquet read path over fixtures this test writes itself.
// All data is synthetic: the bundle is derived from the hand-written
// RevitFixture model; no customer data.

#include <reusex/core/ProjectDB.hpp>
#include <reusex/io/speckle_bundle.hpp>
#include <reusex/io/speckle_revit.hpp>

#include "../../support/revit_fixture.hpp"
#include "../../support/temp_path.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <nlohmann/json.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#ifdef REUSEX_HAVE_DUCKDB
#include <duckdb.h>
#endif

using namespace reusex::io::speckle;
using Catch::Approx;
using nlohmann::json;

namespace {

// ---------------------------------------------------- SGEO test encoder ---
// The layout of specklepy sgeo.py _encode_mesh / _assemble.

void put16(std::string &b, std::uint16_t v) {
  b += static_cast<char>(v & 0xff);
  b += static_cast<char>(v >> 8);
}
void put32(std::string &b, std::uint32_t v) {
  for (int i = 0; i < 4; ++i)
    b += static_cast<char>((v >> (8 * i)) & 0xff);
}
void put_f64(std::string &b, double d) {
  std::uint64_t bits;
  std::memcpy(&bits, &d, sizeof d);
  for (int i = 0; i < 8; ++i)
    b += static_cast<char>((bits >> (8 * i)) & 0xff);
}

std::uint16_t units_code(const std::string &u) {
  const std::map<std::string, std::uint16_t> codes = {
      {"mm", 1}, {"cm", 2}, {"m", 3}, {"km", 4}, {"in", 5}, {"ft", 6}};
  auto it = codes.find(u);
  return it == codes.end() ? 0 : it->second;
}

std::string sgeo_blob(std::uint8_t primitive, const std::string &units,
                      const std::string &body) {
  std::string b = "SGEO";
  b += static_cast<char>(1);
  b += static_cast<char>(primitive);
  put16(b, 0);
  put16(b, units_code(units));
  put16(b, 0);
  put32(b, sgeo_crc32(body));
  return b + body;
}

std::string sgeo_mesh(const std::vector<double> &v,
                      const std::vector<std::int32_t> &f,
                      const std::string &units) {
  std::string body;
  put32(body, static_cast<std::uint32_t>(v.size() / 3));
  put32(body, static_cast<std::uint32_t>(f.size()));
  for (double x : v)
    put_f64(body, x);
  for (auto i : f)
    put32(body, static_cast<std::uint32_t>(i));
  return sgeo_blob(0, units, body);
}

// ------------------------------------------- classic tree -> bundle ---
// Flattens a resolved Revit-connector tree (RevitFixture) into the bundle's
// tables, the way the connector's bundle writer stores the same model.

struct BundleBuilder {
  BundleTables t;
  std::map<std::string, std::int32_t> path_ids;
  std::map<std::string, std::int32_t> object_of_app;
  std::map<std::string, std::int32_t> level_node;
  std::map<std::string, std::int32_t> def_node;
  std::map<std::string, const json *> by_app;
  std::int32_t next_node = 0;
  std::int32_t next_geometry = 0;

  std::int32_t path(const std::string &p) {
    auto [it, fresh] =
        path_ids.emplace(p, static_cast<std::int32_t>(path_ids.size()));
    if (fresh)
      t.paths[it->second] = p;
    return it->second;
  }

  void row(std::int32_t obj, const std::string &p, const json &v,
           const std::string &unit = "", const std::string &idn = "") {
    BundleEavRow r;
    r.key = obj;
    r.path = path(p);
    r.value.unit = unit;
    r.value.internal_definition_name = idn;
    if (v.is_boolean())
      r.value.b = v.get<bool>();
    else if (v.is_number()) {
      r.value.d = v.get<double>();
      r.value.s = v.dump(); // the writer stores both
    } else if (v.is_string())
      r.value.s = v.get<std::string>();
    else
      return;
    t.eav.push_back(std::move(r));
  }

  void flatten(std::int32_t obj, const std::string &prefix, const json &v) {
    if (v.is_object() && v.contains("internalDefinitionName") &&
        v.contains("value")) {
      // A parameter: one row, keyed by its display name.
      const std::string at = prefix.substr(0, prefix.rfind('.')) + "." +
                             v.value("name", std::string("p"));
      row(obj, at, v["value"], v.value("units", std::string{}),
          v["internalDefinitionName"].get<std::string>());
      return;
    }
    if (v.is_object()) {
      for (const auto &[k, x] : v.items())
        flatten(obj, prefix + "." + k, x);
      return;
    }
    row(obj, prefix, v);
  }

  std::int32_t geometry(const json &mesh) {
    std::vector<double> v = mesh["vertices"].get<std::vector<double>>();
    std::vector<std::int32_t> f =
        mesh["faces"].get<std::vector<std::int32_t>>();
    const std::int32_t g = next_geometry++;
    t.geometries.push_back(
        {g, sgeo_mesh(v, f, mesh.value("units", std::string{})), "mesh"});
    return g;
  }

  std::int32_t instance(const json &proxy) {
    BundleNode n;
    n.id = next_node++;
    n.kind = static_cast<int>(BundleNodeKind::instance);
    n.def_ref = def_node.at(proxy["definitionId"].get<std::string>());
    std::string csv;
    for (const auto &x : proxy["transform"])
      csv += (csv.empty() ? "" : ",") + json(x.get<double>()).dump();
    n.transform = csv;
    n.units = proxy.value("units", std::string{});
    t.nodes.push_back(n);
    return n.id;
  }

  void element(const json &e) {
    const auto k = static_cast<std::int32_t>(t.objects.size());
    const std::string app = e["applicationId"].get<std::string>();
    t.objects[k] = app;
    object_of_app[app] = k;
    for (const auto &[key, v] : e.items()) {
      if (key == "displayValue" || key == "elements" ||
          key == "applicationId" || key == "level" || key == "id")
        continue;
      flatten(k, key, v);
    }
    if (auto l = level_node.find(e.value("level", std::string{}));
        l != level_node.end())
      t.relations.push_back(
          {static_cast<int>(BundleRel::on_level), k, l->second, 0});
    std::int32_t ord = 0;
    for (const auto &d : e["displayValue"]) {
      const std::string type = d.value("speckle_type", std::string{});
      if (type.find("InstanceProxy") != std::string::npos)
        t.relations.push_back({static_cast<int>(BundleRel::display_instance), k,
                               instance(d), ord++});
      else
        t.relations.push_back(
            {static_cast<int>(BundleRel::display), k, geometry(d), ord++});
    }
  }

  void walk(const json &node) {
    if (node.is_array()) {
      for (const auto &x : node)
        walk(x);
      return;
    }
    if (!node.is_object())
      return;
    if (node.contains("category") && node.contains("displayValue"))
      element(node);
    if (node.contains("elements"))
      walk(node["elements"]);
  }

  void index(const json &node) {
    if (node.is_object()) {
      if (auto a = node.find("applicationId");
          a != node.end() && a->is_string())
        by_app.emplace(a->get<std::string>(), &node);
      for (const auto &[k, v] : node.items())
        index(v);
    } else if (node.is_array()) {
      for (const auto &v : node)
        index(v);
    }
  }

  BundleTables build(const json &root) {
    index(root);
    for (const auto &lp : root["levelProxies"]) {
      BundleNode n;
      n.id = next_node++;
      n.kind = static_cast<int>(BundleNodeKind::level);
      n.name = lp["value"]["name"].get<std::string>();
      n.elevation = lp["value"]["elevation"].get<double>();
      n.units = lp["value"]["units"].get<std::string>();
      level_node[n.name] = n.id;
      t.nodes.push_back(n);
    }
    for (const auto &d : root["instanceDefinitionProxies"]) {
      BundleNode n;
      n.id = next_node++;
      n.kind = static_cast<int>(BundleNodeKind::definition);
      n.name = d["applicationId"].get<std::string>();
      def_node[n.name] = n.id;
      t.nodes.push_back(n);
      for (const auto &o : d["objects"])
        t.relations.push_back({static_cast<int>(BundleRel::defines), n.id,
                               geometry(*by_app.at(o.get<std::string>())), 0});
    }
    walk(root["elements"]);
    t.schema_version = "1.2.0";
    return t;
  }
};

BundleTables fixture_bundle(const reusex::test_support::RevitFixture &f) {
  return BundleBuilder().build(f.model());
}

/// The parts of two imports that must agree.
void check_same(const RevitImport &a, const RevitImport &b) {
  REQUIRE(a.model.storeys.size() == b.model.storeys.size());
  for (std::size_t i = 0; i < a.model.storeys.size(); ++i) {
    CHECK(a.model.storeys[i].name == b.model.storeys[i].name);
    CHECK(a.model.storeys[i].z_floor ==
          Approx(b.model.storeys[i].z_floor).margin(1e-9));
    CHECK(a.model.storeys[i].z_ceiling ==
          Approx(b.model.storeys[i].z_ceiling).margin(1e-9));
  }
  CHECK(a.stats.elevation_offset_m ==
        Approx(b.stats.elevation_offset_m).margin(1e-9));
  REQUIRE(a.model.walls.size() == b.model.walls.size());
  for (std::size_t i = 0; i < a.model.walls.size(); ++i) {
    CHECK(a.model.walls[i].x0 == Approx(b.model.walls[i].x0).margin(1e-9));
    CHECK(a.model.walls[i].x1 == Approx(b.model.walls[i].x1).margin(1e-9));
    CHECK(a.model.walls[i].thickness ==
          Approx(b.model.walls[i].thickness).margin(1e-9));
    CHECK(a.model.walls[i].height ==
          Approx(b.model.walls[i].height).margin(1e-9));
    CHECK(a.model.walls[i].is_external == b.model.walls[i].is_external);
  }
  REQUIRE(a.model.openings.size() == b.model.openings.size());
  for (std::size_t i = 0; i < a.model.openings.size(); ++i) {
    CHECK(a.model.openings[i].kind == b.model.openings[i].kind);
    CHECK(a.model.openings[i].s_start ==
          Approx(b.model.openings[i].s_start).margin(1e-9));
    CHECK(a.model.openings[i].z_head ==
          Approx(b.model.openings[i].z_head).margin(1e-9));
  }
  CHECK(a.stats.openings_by_parent == b.stats.openings_by_parent);
  REQUIRE(a.model.spaces.size() == b.model.spaces.size());
  for (std::size_t i = 0; i < a.model.spaces.size(); ++i) {
    CHECK(a.model.spaces[i].name == b.model.spaces[i].name);
    CHECK(a.model.spaces[i].area == Approx(b.model.spaces[i].area));
  }
  REQUIRE(a.meshes.size() == b.meshes.size());
  for (std::size_t i = 0; i < a.meshes.size(); ++i) {
    CHECK(a.meshes[i].name == b.meshes[i].name);
    CHECK(a.meshes[i].triangles.size() == b.meshes[i].triangles.size());
    CHECK(a.meshes[i].elements == b.meshes[i].elements);
  }
  CHECK(a.stats.instance_proxies == b.stats.instance_proxies);
  CHECK(a.stats.instance_unresolved == 0);
  CHECK(a.stats.categories == b.stats.categories);
}

} // namespace

// ================================================================ SGEO ===

TEST_CASE("SgeoCrc32_CheckValue_MatchesIeee", "[io][speckle][bundle]") {
  CHECK(sgeo_crc32("123456789") == 0xCBF43926u);
  CHECK(sgeo_crc32("") == 0u);
}

TEST_CASE("DecodeSgeoMesh_HandBuiltBlob_RoundTrips", "[io][speckle][bundle]") {
  const std::vector<double> v = {0, 0, 0, 1.5, 0, 0, 0, 2.25, -3};
  const std::vector<std::int32_t> f = {3, 0, 1, 2};
  const std::string blob = sgeo_mesh(v, f, "mm");
  const auto h = decode_sgeo_header(blob);
  CHECK(h.version == 1);
  CHECK(h.primitive == kSgeoMesh);
  CHECK(h.units_code == 1);
  const auto m = decode_sgeo_mesh(blob);
  CHECK(m.vertices == v);
  CHECK(m.faces == f);
  CHECK(m.units == "mm");
  CHECK(sgeo_units(3) == "m");
  CHECK(sgeo_units(6) == "ft");
  CHECK(sgeo_units(0).empty());
  CHECK(sgeo_units(99).empty());
}

TEST_CASE("DecodeSgeoMesh_BadBlobs_Throw", "[io][speckle][bundle]") {
  const std::string good =
      sgeo_mesh({0, 0, 0, 1, 0, 0, 0, 1, 0}, {3, 0, 1, 2}, "m");
  CHECK_THROWS_AS(decode_sgeo_header("SGEO"), std::runtime_error);
  std::string magic = good;
  magic[0] = 'X';
  CHECK_THROWS_AS(decode_sgeo_header(magic), std::runtime_error);
  std::string version = good;
  version[4] = 2;
  CHECK_THROWS_AS(decode_sgeo_header(version), std::runtime_error);
  std::string flipped = good;
  flipped[30] ^= 0x01; // a vertex byte: CRC mismatch
  CHECK_THROWS_AS(decode_sgeo_mesh(flipped), std::runtime_error);
  CHECK_NOTHROW(decode_sgeo_mesh(flipped, /*verify_crc=*/false));
  // Truncated: a vertex count larger than the body (CRC unchecked).
  std::string body;
  put32(body, 1000000);
  put32(body, 0);
  CHECK_THROWS_AS(decode_sgeo_mesh(sgeo_blob(0, "m", body)),
                  std::runtime_error);
  std::string huge;
  put32(huge, 0xFFFFFFFFu);
  put32(huge, 0xFFFFFFFFu);
  CHECK_THROWS_AS(decode_sgeo_mesh(sgeo_blob(0, "m", huge)),
                  std::runtime_error);
  // A line is not a mesh.
  CHECK_THROWS_AS(decode_sgeo_mesh(sgeo_blob(1, "m", std::string(64, '\0'))),
                  std::runtime_error);
}

// ======================================================= listing / URLs ===

TEST_CASE("IsAllowedArtifactUrl_Hosts_OnlyStorageOrServer",
          "[io][speckle][bundle]") {
  const std::string srv = "https://app.speckle.systems";
  CHECK(is_allowed_artifact_url(
      "https://ams3.digitaloceanspaces.com/bucket/x.parquet?X-Amz-Sig=1", srv));
  CHECK(is_allowed_artifact_url(
      "https://bucket.s3.eu-west-1.amazonaws.com/x.parquet", srv));
  CHECK(is_allowed_artifact_url("https://app.speckle.systems/api/x", srv));
  CHECK(is_allowed_artifact_url("https://files.app.speckle.systems/x", srv));
  CHECK_FALSE(
      is_allowed_artifact_url("http://ams3.digitaloceanspaces.com/x", srv));
  CHECK_FALSE(is_allowed_artifact_url("https://evil.com/x", srv));
  CHECK_FALSE(is_allowed_artifact_url(
      "https://digitaloceanspaces.com.evil.com/x", srv));
  CHECK_FALSE(
      is_allowed_artifact_url("https://evildigitaloceanspaces.com/x", srv));
  CHECK_FALSE(is_allowed_artifact_url(
      "https://ams3.digitaloceanspaces.com@evil.com/x", srv));
  CHECK_FALSE(is_allowed_artifact_url("https://127.0.0.1/x", srv));
  CHECK_FALSE(is_allowed_artifact_url("file:///etc/passwd", srv));
  CHECK_FALSE(is_allowed_artifact_url("https://a.amazonaws.com/x y", srv));
  CHECK_FALSE(is_allowed_artifact_url("", srv));
  // A localhost dev server may hand out http URLs to itself only.
  CHECK(is_allowed_artifact_url("http://localhost:3000/minio/x",
                                "http://localhost:3000"));
  CHECK_FALSE(
      is_allowed_artifact_url("http://10.0.0.1/x", "http://localhost:3000"));
  CHECK(url_without_query("https://h/p/x.parquet?sig=abc#f") ==
        "https://h/p/x.parquet");
}

TEST_CASE("ParseArtifactListing_Files_FiltersAndRefuses",
          "[io][speckle][bundle]") {
  const std::string srv = "https://app.speckle.systems";
  const std::string u = "https://ams3.digitaloceanspaces.com/b/";
  const json listing = {
      {"files",
       {{{"name", "v1.eav.eav.parquet"}, {"url", u + "1?s=1"}},
        {{"name", "v1.viewer.dat"}, {"url", u + "2"}},
        {{"name", "v1.viewer.idx"}, {"url", u + "3"}},
        {{"name", "v1.geometries.parquet"}, {"url", u + "4"}}}},
      {"geometryStream", {{"path", "/api/v2/x"}}}};
  const auto files = parse_artifact_listing(listing, srv);
  REQUIRE(files.size() == 2);
  CHECK(files[0].name == "v1.eav.eav.parquet");
  CHECK(files[1].name == "v1.geometries.parquet");

  auto with = [&](const std::string &name, const std::string &url) {
    return json{{"files", {{{"name", name}, {"url", url}}}}};
  };
  CHECK_THROWS_AS(parse_artifact_listing(with("../x.parquet", u), srv),
                  std::runtime_error);
  CHECK_THROWS_AS(parse_artifact_listing(with("a/b.parquet", u), srv),
                  std::runtime_error);
  CHECK_THROWS_AS(parse_artifact_listing(with("x'.parquet", u), srv),
                  std::runtime_error);
  CHECK_THROWS_AS(
      parse_artifact_listing(with("x.parquet", "https://evil.com/x"), srv),
      std::runtime_error);
  json dup = {{"files",
               {{{"name", "a.parquet"}, {"url", u}},
                {{"name", "a.parquet"}, {"url", u}}}}};
  CHECK_THROWS_AS(parse_artifact_listing(dup, srv), std::runtime_error);
  CHECK_THROWS_AS(parse_artifact_listing(json::array(), srv),
                  std::runtime_error);
  CHECK(is_bundle_file_name("e5557cd2dc.eav.eav.parquet"));
  CHECK_FALSE(is_bundle_file_name(".hidden.parquet"));
  CHECK_FALSE(is_bundle_file_name(std::string(201, 'a')));
}

// ========================================================== projection ===

TEST_CASE("ProjectBundle_FixtureModel_ImportsLikeTheClassicTree",
          "[io][speckle][bundle][revit]") {
  for (const auto &[units, per_mm] :
       {std::pair<std::string, double>{"mm", 1.0}, {"m", 1000.0}}) {
    DYNAMIC_SECTION(units) {
      reusex::test_support::RevitFixture f;
      f.units = units;
      f.per_mm = per_mm;
      BundleProjectionStats st;
      const json root = project_bundle_to_revit_tree(fixture_bundle(f), &st);
      CHECK(st.levels == 2);
      CHECK(st.definitions == 2);
      CHECK(st.instances == 2);
      CHECK(st.geometries_bad == 0);
      CHECK(root["units"] == units);
      check_same(import_revit(root), import_revit(f.model()));
    }
  }
}

TEST_CASE("ProjectBundle_HostedOnAndSubelements_BecomeParentAndNesting",
          "[io][speckle][bundle][revit]") {
  const reusex::test_support::RevitFixture f;
  BundleBuilder b;
  BundleTables t = b.build(f.model());
  // Drop the door's parentApplicationId row; say it with HOSTED_ON instead.
  const std::int32_t door = b.object_of_app.at("door-1");
  const std::int32_t wall = b.object_of_app.at("wall-a");
  const std::int32_t parent_path =
      b.path_ids.at("properties.parentApplicationId");
  std::erase_if(t.eav, [&](const BundleEavRow &r) {
    return r.key == door && r.path == parent_path;
  });
  t.relations.push_back(
      {static_cast<int>(BundleRel::hosted_on), door, wall, 0});
  // And nest the window under the wall (SUBELEMENT).
  const std::int32_t win = b.object_of_app.at("win-1");
  t.relations.push_back(
      {static_cast<int>(BundleRel::subelement), wall, win, 0});
  BundleProjectionStats st;
  const json root = project_bundle_to_revit_tree(t, &st);
  CHECK(st.hosted == 1);
  CHECK(st.nested == 1);
  bool nested = false;
  for (const auto &e : root["elements"])
    if (e["applicationId"] == "wall-a" && e.contains("elements"))
      nested = e["elements"][0]["applicationId"] == "win-1";
  CHECK(nested);
  const auto r = import_revit(root);
  CHECK(r.stats.openings_by_parent == 2); // door by HOSTED_ON, window nested
  CHECK(r.model.openings.size() == 2);
}

TEST_CASE("ProjectBundle_ParameterValues_KeepIdentityStrings",
          "[io][speckle][bundle]") {
  BundleTables t;
  t.objects[0] = "obj-0";
  t.paths = {
      {0, "properties.Parameters.Instance Parameters.Identity Data.Mark"},
      {1, "properties.Parameters.Type Parameters.Construction.Width"},
      {2, "properties.elementId"},
      {3, "category"},
      {4, "properties.flag"}};
  auto add = [&](std::int32_t p, std::optional<std::string> s,
                 std::optional<double> d, std::string unit = "",
                 std::string idn = "") {
    BundleEavRow r;
    r.key = 0;
    r.path = p;
    r.value.s = std::move(s);
    r.value.d = d;
    r.value.unit = std::move(unit);
    r.value.internal_definition_name = std::move(idn);
    t.eav.push_back(r);
  };
  add(0, "011", 11.0, "", "ALL_MODEL_MARK");
  add(1, "543", 543.0, "millimeters", "WALL_ATTR_WIDTH_PARAM");
  add(2, "3777948", 3777948.0);
  add(3, "Walls", std::nullopt);
  BundleEavRow flag;
  flag.key = 0;
  flag.path = 4;
  flag.value.b = true;
  t.eav.push_back(flag);
  const json root = project_bundle_to_revit_tree(t);
  const json &o = root["elements"][0];
  const json &mark = o["properties"]["Parameters"]["Instance Parameters"]
                      ["Identity Data"]["Mark"];
  CHECK(mark["value"] == "011");
  CHECK(mark["internalDefinitionName"] == "ALL_MODEL_MARK");
  CHECK(mark["name"] == "Mark");
  const json &width =
      o["properties"]["Parameters"]["Type Parameters"]["Construction"]["Width"];
  CHECK(width["value"] == 543);
  CHECK(width["units"] == "millimeters");
  CHECK(o["properties"]["elementId"].dump() == "3777948");
  CHECK(o["properties"]["flag"] == true);
  CHECK(o["category"] == "Walls");
  CHECK(o["speckle_type"] == "Objects.Data.DataObject");
}

TEST_CASE("ProjectBundle_BadGeometry_IsSkippedAndCounted",
          "[io][speckle][bundle]") {
  BundleTables t;
  t.objects = {{0, "a"}, {1, "b"}, {2, "c"}};
  t.paths = {{0, "category"}};
  std::string bad = sgeo_mesh({0, 0, 0}, {}, "m");
  bad.back() ^= 0x7f; // CRC mismatch
  t.geometries = {{0, bad, "mesh"},
                  {1, sgeo_blob(1, "m", std::string(56, '\0')), "line"}};
  t.relations = {{static_cast<int>(BundleRel::display), 0, 0, 0},
                 {static_cast<int>(BundleRel::display), 1, 1, 0},
                 {static_cast<int>(BundleRel::display), 2, 7, 0}};
  BundleProjectionStats st;
  const json root = project_bundle_to_revit_tree(t, &st);
  CHECK(st.geometries_bad == 1);
  CHECK(st.geometries_skipped == 1);
  CHECK(st.geometries_missing == 1);
  CHECK(st.meshes == 0);
  for (const auto &e : root["elements"])
    CHECK(e["displayValue"].empty());
}

// ============================================================ budgets ===

namespace {

/// A box mesh (8 vertices, 12 triangles) as an SGEO blob.
std::string box_blob() {
  std::vector<double> v;
  for (int i = 0; i < 8; ++i)
    v.insert(v.end(), {double(i & 1), double((i >> 1) & 1), double(i >> 2)});
  std::vector<std::int32_t> f;
  for (int t = 0; t < 12; ++t)
    f.insert(f.end(), {3, t % 8, (t + 1) % 8, (t + 2) % 8});
  return sgeo_mesh(v, f, "m");
}

/// @p n objects (category Walls) that all DISPLAY one geometry.
BundleTables fan_out(std::size_t n) {
  BundleTables t;
  t.paths = {{0, "category"}};
  t.geometries = {{0, box_blob(), "mesh"}};
  for (std::size_t i = 0; i < n; ++i) {
    const auto k = static_cast<std::int32_t>(i);
    t.objects[k] = "o" + std::to_string(i);
    BundleEavRow r;
    r.key = k;
    r.path = 0;
    r.value.s = "Walls";
    t.eav.push_back(r);
    t.relations.push_back({static_cast<int>(BundleRel::display), k, 0, 0});
  }
  return t;
}

} // namespace

TEST_CASE("ProjectBundle_SharedDisplayMesh_IsEmittedOnce",
          "[io][speckle][bundle]") {
  BundleProjectionStats st;
  const json root = project_bundle_to_revit_tree(fan_out(1000), &st);
  CHECK(st.shared_meshes == 1);
  CHECK(st.meshes == 1);
  CHECK(st.mesh_values == 8 * 3 + 12 * 4); // one copy
  CHECK(st.expanded_vertices == 1000 * 8);
  CHECK(root["definitionGeometry"].size() == 1);
  for (const auto &e : root["elements"]) {
    REQUIRE(e["displayValue"].size() == 1);
    CHECK(e["displayValue"][0]["definitionId"] == "shared-0");
  }
  // Every use still bakes to the same geometry.
  json with_level = root;
  with_level["levelProxies"] = json::array(
      {{{"applicationId", "l"},
        {"value", {{"name", "L"}, {"elevation", 0}, {"units", "m"}}}}});
  const auto r = import_revit(with_level);
  REQUIRE(r.meshes.size() == 1);
  CHECK(r.meshes[0].elements == 1000);
  CHECK(r.stats.instance_unresolved == 0);
}

TEST_CASE("ProjectBundle_FanOutBomb_ThrowsOnTheBudget",
          "[io][speckle][bundle]") {
  BundleProjectionOptions small;
  small.max_expanded_vertices = 8 * 100;
  CHECK_THROWS_WITH(project_bundle_to_revit_tree(fan_out(101), nullptr, small),
                    Catch::Matchers::ContainsSubstring("expanded vertices"));

  // A small definition placed many times, nested three deep (x10 each).
  BundleTables t;
  t.geometries = {{0, box_blob(), "mesh"}};
  std::int32_t id = 0;
  std::int32_t inner = -1;
  for (int level = 0; level < 3; ++level) {
    const std::int32_t def = id++;
    t.nodes.push_back({def, static_cast<int>(BundleNodeKind::definition), "d",
                       std::nullopt, "", "", "", std::nullopt});
    if (inner < 0)
      t.relations.push_back({static_cast<int>(BundleRel::defines), def, 0, 0});
    else
      for (int i = 0; i < 10; ++i) {
        const std::int32_t inst = id++;
        t.nodes.push_back({inst, static_cast<int>(BundleNodeKind::instance), "",
                           inner, "", "m", "", std::nullopt});
        t.relations.push_back(
            {static_cast<int>(BundleRel::defines_instance), def, inst, i});
      }
    inner = def;
  }
  t.objects[0] = "o";
  for (int i = 0; i < 10; ++i) {
    const std::int32_t inst = id++;
    t.nodes.push_back({inst, static_cast<int>(BundleNodeKind::instance), "",
                       inner, "", "m", "", std::nullopt});
    t.relations.push_back(
        {static_cast<int>(BundleRel::display_instance), 0, inst, i});
  }
  BundleProjectionStats st;
  CHECK_NOTHROW(project_bundle_to_revit_tree(t, &st));
  CHECK(st.expanded_vertices == 8 * 1000);
  BundleProjectionOptions tight;
  tight.max_expanded_vertices = 8 * 999;
  CHECK_THROWS_WITH(project_bundle_to_revit_tree(t, nullptr, tight),
                    Catch::Matchers::ContainsSubstring("expanded vertices"));
}

TEST_CASE("ProjectBundle_TypeRowsTimesObjects_ThrowOnTheRowBudget",
          "[io][speckle][bundle]") {
  BundleTables t;
  for (int p = 0; p < 100; ++p) {
    t.paths[p] = "properties.p" + std::to_string(p);
    BundleEavRow r;
    r.key = 7; // type 7
    r.path = p;
    r.value.s = "x";
    t.type_eav.push_back(r);
  }
  for (int k = 0; k < 100; ++k) {
    t.objects[k] = "o" + std::to_string(k);
    t.object_type[k] = 7;
  }
  BundleProjectionOptions opt;
  opt.max_property_rows = 9'999;
  CHECK_THROWS_WITH(project_bundle_to_revit_tree(t, nullptr, opt),
                    Catch::Matchers::ContainsSubstring("property rows"));
  opt.max_property_rows = 10'000;
  BundleProjectionStats st;
  CHECK_NOTHROW(project_bundle_to_revit_tree(t, &st, opt));
  CHECK(st.property_rows == 10'000);
}

TEST_CASE("ProjectBundle_DeepPathBomb_DropsThePath", "[io][speckle][bundle]") {
  BundleTables t;
  t.objects[0] = "o";
  std::string deep = "properties";
  for (int i = 0; i < 5000; ++i)
    deep += ".a";
  std::string long_name = "properties." + std::string(2000, 'x');
  t.paths = {{0, deep}, {1, long_name}, {2, "category"}};
  for (int p = 0; p < 3; ++p) {
    BundleEavRow r;
    r.key = 0;
    r.path = p;
    r.value.s = "v";
    t.eav.push_back(r);
  }
  BundleProjectionStats st;
  const json root = project_bundle_to_revit_tree(t, &st);
  CHECK(st.paths_dropped == 2);
  const json &o = root["elements"][0];
  CHECK(o["category"] == "v");
  CHECK_FALSE(o.contains("properties"));
}

TEST_CASE("ImportRevit_DeepClassicTree_IsRefusedClearly",
          "[io][speckle][revit]") {
  const reusex::test_support::RevitFixture f;
  json root = f.model();
  // 300 nested collections around the walls collection.
  json inner = root["elements"];
  for (int i = 0; i < 300; ++i)
    inner = json::array(
        {{{"speckle_type", "Speckle.Core.Models.Collections.Collection"},
          {"elements", std::move(inner)}}});
  root["elements"] = std::move(inner);
  CHECK_THROWS_WITH(import_revit(root),
                    Catch::Matchers::ContainsSubstring("nests deeper"));
  // A deep parameter tree too.
  json params = {{"internalDefinitionName", "X"}, {"value", 1}};
  for (int i = 0; i < 300; ++i)
    params = {{"g", std::move(params)}};
  json r2 = f.model();
  r2["levelProxies"][0]["value"]["properties"]["Parameters"] = params;
  CHECK_THROWS_WITH(import_revit(r2),
                    Catch::Matchers::ContainsSubstring("nests deeper"));
}

TEST_CASE("ProjectBundle_Cancel_StopsTheProjection", "[io][speckle][bundle]") {
  std::atomic_bool cancel{true};
  BundleProjectionOptions opt;
  opt.cancel = &cancel;
  CHECK_THROWS_WITH(project_bundle_to_revit_tree(fan_out(10), nullptr, opt),
                    Catch::Matchers::ContainsSubstring("cancelled"));
}

TEST_CASE("ProjectBundle_Levels_DropRenameAndBorrowProperties",
          "[io][speckle][bundle][revit]") {
  const reusex::test_support::RevitFixture f;
  BundleBuilder b;
  b.build(f.model());
  BundleTables &t = b.t; // b.row() below writes into b.t
  auto level = [&](const std::string &name, double elevation) {
    BundleNode n;
    n.id = b.next_node++;
    n.kind = static_cast<int>(BundleNodeKind::level);
    n.name = name;
    n.elevation = elevation;
    n.units = "mm";
    t.nodes.push_back(n);
    return n.id;
  };
  level("Empty", 20000);                // no member: dropped
  const auto dup = level("L0", 16000);  // a second "L0", higher
  const auto ref = level("Ref", 14000); // a reference level
  std::erase_if(t.relations, [&](const BundleRelation &r) {
    return r.rel == static_cast<int>(BundleRel::on_level) &&
           r.src == b.object_of_app.at("chair-1");
  });
  t.relations.push_back({static_cast<int>(BundleRel::on_level),
                         b.object_of_app.at("chair-1"), dup, 0});
  // The window moves to "Ref", which a Levels object flags as no storey.
  std::erase_if(t.relations, [&](const BundleRelation &r) {
    return r.rel == static_cast<int>(BundleRel::on_level) &&
           r.src == b.object_of_app.at("win-1");
  });
  t.relations.push_back({static_cast<int>(BundleRel::on_level),
                         b.object_of_app.at("win-1"), ref, 0});
  const auto lvl_obj = static_cast<std::int32_t>(t.objects.size());
  t.objects[lvl_obj] = "level-ref-app";
  b.row(lvl_obj, "category", "Levels");
  b.row(lvl_obj, "name", "Ref");
  b.row(lvl_obj,
        "properties.Parameters.Instance Parameters.Identity Data.Building "
        "Story",
        "No", "", "LEVEL_IS_BUILDING_STORY");
  // And a duplicated applicationId.
  t.objects[b.object_of_app.at("wall-b")] = "wall-a";

  BundleProjectionStats st;
  const json root = project_bundle_to_revit_tree(t, &st);
  CHECK(st.levels_dropped == 1);
  CHECK(st.levels_renamed == 1);
  CHECK(st.levels_with_properties == 1);
  CHECK(st.duplicate_app_ids == 1);
  std::vector<std::string> names;
  for (const auto &lp : root["levelProxies"])
    names.push_back(lp["value"]["name"]);
  CHECK(names == std::vector<std::string>{"L0", "L1", "Ref", "L0 (2)"});
  CHECK(root["levelProxies"][2]["applicationId"] == "level-ref-app");
  const auto r = import_revit(root);
  CHECK(r.stats.reference_levels == 1);
  CHECK(r.model.storeys.size() == 3); // L0, L1, L0 (2)
}

TEST_CASE("ProjectBundle_InstanceWithoutDefinition_IsSkipped",
          "[io][speckle][bundle]") {
  BundleTables t;
  t.objects[0] = "o";
  t.nodes.push_back({5, static_cast<int>(BundleNodeKind::instance), "",
                     std::nullopt, "", "m", "", std::nullopt});
  t.nodes.push_back({6, static_cast<int>(BundleNodeKind::instance), "", 99, "",
                     "m", "", std::nullopt});
  t.relations = {{static_cast<int>(BundleRel::display_instance), 0, 5, 0},
                 {static_cast<int>(BundleRel::display_instance), 0, 6, 1}};
  BundleProjectionStats st;
  const json root = project_bundle_to_revit_tree(t, &st);
  CHECK(st.instances_unresolved == 2);
  CHECK(root["elements"][0]["displayValue"].empty());
}

// =================================================== local HTTP server ===

namespace {

/// One-thread HTTP/1.1 server on 127.0.0.1 with fixed routes (GET path ->
/// status, extra headers, body); records each request's path and whether it
/// carried an Authorization header. Connection: close on every answer.
class RouteServer {
    public:
  struct Answer {
    int status = 200;
    std::string headers; ///< extra "Name: value\r\n" lines
    std::string body;
  };

  RouteServer() {
    fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    REQUIRE(fd_ >= 0);
    int one = 1;
    ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    REQUIRE(::bind(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof addr) == 0);
    REQUIRE(::listen(fd_, 16) == 0);
    socklen_t len = sizeof addr;
    ::getsockname(fd_, reinterpret_cast<sockaddr *>(&addr), &len);
    port_ = ntohs(addr.sin_port);
  }
  ~RouteServer() {
    stop_ = true;
    if (thread_.joinable())
      thread_.join();
    ::close(fd_);
  }
  void route(const std::string &path, Answer a) {
    routes_[path] = std::move(a);
  }
  void start() {
    thread_ = std::thread([this] { loop(); });
  }
  std::string url() const {
    return "http://127.0.0.1:" + std::to_string(port_);
  }
  struct Seen {
    std::string path;
    bool authorization = false;
  };
  std::vector<Seen> seen() {
    std::lock_guard lock(mutex_);
    return seen_;
  }

    private:
  void loop() {
    while (!stop_) {
      pollfd p{fd_, POLLIN, 0};
      if (::poll(&p, 1, 50) <= 0)
        continue;
      const int c = ::accept(fd_, nullptr, nullptr);
      if (c < 0)
        continue;
      serve(c);
      ::close(c);
    }
  }
  void serve(int c) {
    std::string in;
    char buf[4096];
    while (in.find("\r\n\r\n") == std::string::npos) {
      const auto n = ::recv(c, buf, sizeof buf, 0);
      if (n <= 0)
        return;
      in.append(buf, static_cast<std::size_t>(n));
    }
    const auto sp1 = in.find(' ');
    const auto sp2 = in.find(' ', sp1 + 1);
    std::string path = in.substr(sp1 + 1, sp2 - sp1 - 1);
    path = path.substr(0, path.find('?'));
    std::string lower = in;
    for (auto &ch : lower)
      ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    {
      std::lock_guard lock(mutex_);
      seen_.push_back(
          {path, lower.find("\r\nauthorization:") != std::string::npos});
    }
    Answer a{404, "", "not found"};
    if (auto it = routes_.find(path); it != routes_.end())
      a = it->second;
    std::string out = "HTTP/1.1 " + std::to_string(a.status) + " X\r\n" +
                      a.headers +
                      "Content-Length: " + std::to_string(a.body.size()) +
                      "\r\nConnection: close\r\n\r\n" + a.body;
    std::size_t off = 0;
    while (off < out.size()) {
      const auto n =
          ::send(c, out.data() + off, out.size() - off, MSG_NOSIGNAL);
      if (n <= 0)
        return;
      off += static_cast<std::size_t>(n);
    }
  }

  int fd_ = -1;
  int port_ = 0;
  std::atomic_bool stop_{false};
  std::thread thread_;
  std::mutex mutex_;
  std::map<std::string, Answer> routes_;
  std::vector<Seen> seen_;
};

const BundleReference kRef{"proj01", "model01", "ver01"};
const std::string kListing =
    "/api/v2/projects/proj01/models/model01/versions/ver01/artifacts";

} // namespace

TEST_CASE("DownloadBundle_Redirect_IsNotFollowed", "[io][speckle][bundle]") {
  RouteServer srv;
  srv.route(kListing, {200, "Content-Type: application/json\r\n",
                       json{{"files",
                             {{{"name", "ver01.eav.eav.parquet"},
                               {"url", srv.url() + "/store/a?sig=s3cr3t"}}}}}
                           .dump()});
  srv.route("/store/a", {302, "Location: " + srv.url() + "/evil\r\n", ""});
  srv.route("/evil", {200, "", "PAR1"});
  srv.start();
  SpeckleReader reader(srv.url(), "tok");
  reusex::test_support::TempDir dir("test_speckle_bundle_redirect");
  try {
    (void)reader.download_bundle(kRef, dir.path);
    FAIL("expected a refused redirect");
  } catch (const std::runtime_error &e) {
    const std::string msg = e.what();
    CHECK(msg.find("redirect refused") != std::string::npos);
    CHECK(msg.find("s3cr3t") == std::string::npos); // no signature in logs
  }
  for (const auto &s : srv.seen())
    CHECK(s.path != "/evil");
}

TEST_CASE("DownloadBundle_OffPolicyUrl_IsRefusedBeforeAnyDownload",
          "[io][speckle][bundle]") {
  RouteServer srv;
  srv.route(kListing, {200, "",
                       json{{"files",
                             {{{"name", "ver01.eav.eav.parquet"},
                               {"url", "http://10.1.2.3/x.parquet"}}}}}
                           .dump()});
  srv.start();
  SpeckleReader reader(srv.url(), "tok");
  reusex::test_support::TempDir dir("test_speckle_bundle_policy");
  CHECK_THROWS_WITH(reader.download_bundle(kRef, dir.path),
                    Catch::Matchers::ContainsSubstring("refused by policy"));
  CHECK(srv.seen().size() == 1); // the listing only
}

// ========================================================= DuckDB path ===

#ifdef REUSEX_HAVE_DUCKDB
namespace {

std::string sql_str(const std::string &s) {
  std::string out = "'";
  for (char c : s) {
    out += c;
    if (c == '\'')
      out += '\'';
  }
  return out + "'";
}
std::string sql_opt(const std::optional<std::string> &s) {
  return s ? sql_str(*s) : "NULL";
}
std::string sql_hex(const std::string &bytes) {
  static const char *hex = "0123456789abcdef";
  std::string h;
  for (unsigned char c : bytes) {
    h += hex[c >> 4];
    h += hex[c & 15];
  }
  return "from_hex('" + h + "')";
}

/// Writes @p t as the bundle's Parquet files (ZSTD) into @p dir with a
/// throwaway DuckDB of its own.
void write_bundle(const BundleTables &t, const std::filesystem::path &dir,
                  const std::string &base) {
  duckdb_database db;
  duckdb_connection con;
  REQUIRE(duckdb_open(nullptr, &db) == DuckDBSuccess);
  REQUIRE(duckdb_connect(db, &con) == DuckDBSuccess);
  auto exec = [&](const std::string &sql) {
    duckdb_result r;
    const bool ok = duckdb_query(con, sql.c_str(), &r) == DuckDBSuccess;
    if (!ok)
      FAIL(std::string(duckdb_result_error(&r)) + " in " + sql.substr(0, 200));
    duckdb_destroy_result(&r);
  };
  auto insert = [&](const std::string &table,
                    const std::vector<std::string> &rows) {
    for (std::size_t i = 0; i < rows.size(); i += 500) {
      std::string sql = "INSERT INTO " + table + " VALUES ";
      for (std::size_t j = i; j < std::min(rows.size(), i + 500); ++j)
        sql += (j > i ? "," : "") + rows[j];
      exec(sql);
    }
  };
  auto copy = [&](const std::string &table, const std::string &suffix) {
    exec("COPY " + table + " TO " + sql_str((dir / (base + suffix)).string()) +
         " (FORMAT parquet, COMPRESSION zstd)");
  };
  exec("CREATE TABLE objects(object_index INTEGER, application_id VARCHAR)");
  std::vector<std::string> rows;
  for (const auto &[k, a] : t.objects)
    rows.push_back("(" + std::to_string(k) + "," + sql_str(a) + ")");
  insert("objects", rows);
  copy("objects", ".eav.objects.parquet");

  exec("CREATE TABLE paths(path_index INTEGER, path VARCHAR)");
  rows.clear();
  for (const auto &[k, p] : t.paths)
    rows.push_back("(" + std::to_string(k) + "," + sql_str(p) + ")");
  insert("paths", rows);
  copy("paths", ".eav.paths.parquet");

  exec("CREATE TABLE eav(object_index INTEGER, path_index INTEGER, "
       "value_string VARCHAR, value_double DOUBLE, value_boolean BOOLEAN, "
       "unit VARCHAR, internal_definition_name VARCHAR)");
  rows.clear();
  for (const auto &r : t.eav)
    rows.push_back(
        "(" + std::to_string(r.key) + "," + std::to_string(r.path) + "," +
        sql_opt(r.value.s) + "," +
        (r.value.d ? json(*r.value.d).dump() : "NULL") + "," +
        (r.value.b ? (*r.value.b ? "true" : "false") : "NULL") + "," +
        (r.value.unit.empty() ? "NULL" : sql_str(r.value.unit)) + "," +
        (r.value.internal_definition_name.empty()
             ? "NULL"
             : sql_str(r.value.internal_definition_name)) +
        ")");
  insert("eav", rows);
  copy("eav", ".eav.eav.parquet");

  exec("CREATE TABLE nodes(id INTEGER, kind INTEGER, name VARCHAR, def_ref "
       "INTEGER, transform VARCHAR, units VARCHAR, subtype VARCHAR, "
       "elevation DOUBLE)");
  rows.clear();
  for (const auto &n : t.nodes)
    rows.push_back("(" + std::to_string(n.id) + "," + std::to_string(n.kind) +
                   "," + sql_str(n.name) + "," +
                   (n.def_ref ? std::to_string(*n.def_ref) : "NULL") + "," +
                   sql_str(n.transform) + "," + sql_str(n.units) + ",NULL," +
                   (n.elevation ? json(*n.elevation).dump() : "NULL") + ")");
  insert("nodes", rows);
  copy("nodes", ".envelope.nodes.parquet");

  exec("CREATE TABLE relations(rel INTEGER, src INTEGER, dst INTEGER, ord "
       "INTEGER)");
  rows.clear();
  for (const auto &r : t.relations)
    rows.push_back("(" + std::to_string(r.rel) + "," + std::to_string(r.src) +
                   "," + std::to_string(r.dst) + "," + std::to_string(r.ord) +
                   ")");
  insert("relations", rows);
  copy("relations", ".envelope.relations.parquet");

  exec("CREATE TABLE meta(schema_version VARCHAR, produced_by VARCHAR)");
  exec("INSERT INTO meta VALUES ('1.2.0', 'reusex-test')");
  copy("meta", ".envelope.meta.parquet");

  exec("CREATE TABLE geometries(geometryIndex INTEGER, content BLOB, id "
       "VARCHAR, type VARCHAR)");
  rows.clear();
  for (const auto &g : t.geometries)
    rows.push_back("(" + std::to_string(g.index) + "," + sql_hex(g.content) +
                   ",'g'," + sql_str(g.type) + ")");
  insert("geometries", rows);
  copy("geometries", ".geometries.parquet");

  duckdb_disconnect(&con);
  duckdb_close(&db);
}

} // namespace

TEST_CASE("ReadBundleDirectory_SyntheticParquet_RoundTrips",
          "[io][speckle][bundle][duckdb]") {
  REQUIRE(has_bundle_support());
  reusex::test_support::TempDir dir("test_speckle_bundle");
  const reusex::test_support::RevitFixture f;
  const BundleTables written = fixture_bundle(f);
  write_bundle(written, dir.path, "v0");
  // A viewer file next to the tables is ignored.
  std::ofstream(dir.path / "v0.viewer.dat") << "not parquet";

  const BundleTables read = read_bundle_directory(dir.path);
  CHECK(read.schema_version == "1.2.0");
  CHECK(read.objects == written.objects);
  CHECK(read.paths.size() == written.paths.size());
  CHECK(read.eav.size() == written.eav.size());
  CHECK(read.nodes.size() == written.nodes.size());
  CHECK(read.relations.size() == written.relations.size());
  REQUIRE(read.geometries.size() == written.geometries.size());
  for (std::size_t i = 0; i < read.geometries.size(); ++i)
    CHECK(read.geometries[i].content == written.geometries[i].content);

  check_same(import_revit(project_bundle_to_revit_tree(read)),
             import_revit(f.model()));
}

TEST_CASE("ReadBundleDirectory_Bounds_RefuseMissingAndOversizedTables",
          "[io][speckle][bundle][duckdb]") {
  reusex::test_support::TempDir dir("test_speckle_bundle_bounds");
  const reusex::test_support::RevitFixture f;
  write_bundle(fixture_bundle(f), dir.path, "v0");
  BundleReadOptions small;
  small.max_rows = 3;
  CHECK_THROWS_AS(read_bundle_directory(dir.path, small), std::runtime_error);
  std::filesystem::remove(dir.path / "v0.envelope.relations.parquet");
  CHECK_THROWS_AS(read_bundle_directory(dir.path), std::runtime_error);
  CHECK_THROWS_AS(read_bundle_directory(dir.path / "missing"),
                  std::runtime_error);
}

TEST_CASE("BundleSandbox_QueriesOutsideTheDirectory_AreRefused",
          "[io][speckle][bundle][duckdb]") {
  reusex::test_support::TempDir dir("test_speckle_bundle_sandbox");
  const reusex::test_support::RevitFixture f;
  write_bundle(fixture_bundle(f), dir.path, "v0");
  const std::string inside =
      (std::filesystem::canonical(dir.path) / "v0.eav.paths.parquet").string();
  using detail::bundle_sandbox_query_for_tests;
  CHECK(bundle_sandbox_query_for_tests(
            dir.path, "SELECT count(*) FROM read_parquet('" + inside + "')")
            .empty());
  for (const std::string sql :
       {"SELECT * FROM read_csv('/etc/passwd')",
        "SELECT * FROM read_text('/etc/hostname')",
        "SELECT * FROM read_parquet('https://example.com/x.parquet')",
        "COPY (SELECT 1) TO '/tmp/reusex-sandbox-escape.parquet'",
        "ATTACH '/tmp/reusex-sandbox-escape.duckdb'", "INSTALL httpfs",
        "LOAD httpfs", "SET enable_external_access = true",
        "SET allowed_directories = ['/']"}) {
    INFO(sql);
    CHECK_FALSE(bundle_sandbox_query_for_tests(dir.path, sql).empty());
  }
  CHECK_FALSE(std::filesystem::exists("/tmp/reusex-sandbox-escape.parquet"));
}

TEST_CASE("ReadBundleDirectory_CancelAndByteBudget_Abort",
          "[io][speckle][bundle][duckdb]") {
  reusex::test_support::TempDir dir("test_speckle_bundle_cancel");
  const reusex::test_support::RevitFixture f;
  write_bundle(fixture_bundle(f), dir.path, "v0");
  std::atomic_bool cancel{true};
  BundleReadOptions opt;
  opt.cancel = &cancel;
  CHECK_THROWS_WITH(read_bundle_directory(dir.path, opt),
                    Catch::Matchers::ContainsSubstring("cancelled"));
  BundleReadOptions bytes;
  bytes.max_bytes = 64;
  CHECK_THROWS_WITH(read_bundle_directory(dir.path, bytes),
                    Catch::Matchers::ContainsSubstring("strings and blobs"));
}

TEST_CASE("ReadBundleDirectory_TableNames_MatchExactly",
          "[io][speckle][bundle][duckdb]") {
  reusex::test_support::TempDir dir("test_speckle_bundle_names");
  const reusex::test_support::RevitFixture f;
  const BundleTables written = fixture_bundle(f);
  write_bundle(written, dir.path, "v0");
  // Decoys that sort first and would break the read if picked.
  for (const char *decoy : {"a.b.eav.objects.parquet", "a.b.eav.eav.parquet",
                            "a.x.geometries.parquet"})
    std::ofstream(dir.path / decoy) << "not parquet";
  const BundleTables read = read_bundle_directory(dir.path);
  CHECK(read.objects == written.objects);
  CHECK(read.geometries.size() == written.geometries.size());
}

TEST_CASE("FetchBundleTree_LocalServer_DownloadsWithoutTokenAndImports",
          "[io][speckle][bundle][duckdb]") {
  const reusex::test_support::RevitFixture f;
  reusex::test_support::TempDir src("test_speckle_bundle_srv");
  write_bundle(fixture_bundle(f), src.path, "ver01");
  RouteServer srv;
  json files = json::array();
  for (const auto &e : std::filesystem::directory_iterator(src.path)) {
    const std::string name = e.path().filename().string();
    std::ifstream in(e.path(), std::ios::binary);
    std::string bytes((std::istreambuf_iterator<char>(in)), {});
    srv.route("/store/" + name, {200, "", bytes});
    files.push_back(
        {{"name", name}, {"url", srv.url() + "/store/" + name + "?X-Sig=abc"}});
  }
  files.push_back(
      {{"name", "ver01.viewer.dat"}, {"url", srv.url() + "/store/viewer"}});
  srv.route(kListing, {200, "", json{{"files", files}}.dump()});
  srv.start();

  SpeckleReader reader(srv.url(), "tok");
  BundleFetchReport report;
  const json root = fetch_bundle_tree(reader, "proj01",
                                      "bundle.proj01.model01.ver01", &report);
  CHECK(report.files == files.size() - 1);
  check_same(import_revit(root), import_revit(f.model()));
  std::size_t downloads = 0;
  for (const auto &s : srv.seen()) {
    CHECK(s.path != "/store/viewer");
    if (s.path == kListing) {
      CHECK(s.authorization); // the listing carries the token
    } else {
      CHECK_FALSE(s.authorization); // presigned GETs never do
      ++downloads;
    }
  }
  CHECK(downloads == report.files);
  CHECK_THROWS_AS(
      fetch_bundle_tree(reader, "other", "bundle.proj01.model01.ver01"),
      std::invalid_argument);
}
#else
TEST_CASE("ReadBundleDirectory_WithoutDuckDb_IsRefused",
          "[io][speckle][bundle]") {
  CHECK_FALSE(has_bundle_support());
  CHECK_THROWS_AS(read_bundle_directory("/nonexistent"),
                  BundleVersionUnsupported);
}
#endif

// ============================================================ live run ===

// Live, read-only import of a bundle-only version. Hidden by default; runs
// only with `"[.network]"` and SPECKLE_TOKEN, SPECKLE_TEST_BUNDLE_PROJECT,
// SPECKLE_TEST_BUNDLE_MODEL and SPECKLE_TEST_BUNDLE_VERSION set. GraphQL
// queries and GETs only (the presigned downloads without the token).
// SPECKLE_TEST_BUNDLE_RUX names the project file to import into (kept);
// otherwise a temporary one.
TEST_CASE("ImportSpeckleRevit_LiveBundleVersion_ImportsTheModel",
          "[.network][io][speckle][bundle]") {
  const char *token = std::getenv("SPECKLE_TOKEN");
  const char *project = std::getenv("SPECKLE_TEST_BUNDLE_PROJECT");
  const char *model = std::getenv("SPECKLE_TEST_BUNDLE_MODEL");
  const char *version = std::getenv("SPECKLE_TEST_BUNDLE_VERSION");
  if (!token || !project || !model || !version)
    SKIP("set SPECKLE_TOKEN, SPECKLE_TEST_BUNDLE_PROJECT, "
         "SPECKLE_TEST_BUNDLE_MODEL and SPECKLE_TEST_BUNDLE_VERSION");
  if (!has_bundle_support())
    SKIP("built without DuckDB");
  const char *server_env = std::getenv("SPECKLE_SERVER");
  const char *rux_env = std::getenv("SPECKLE_TEST_BUNDLE_RUX");
  reusex::test_support::TempPath tmp("test_speckle_bundle_live");
  const std::filesystem::path rux = rux_env ? rux_env : tmp.path;

  SpeckleRevitImportOptions opt;
  opt.server_url = server_env ? server_env : "https://app.speckle.systems";
  opt.project = project;
  opt.model = model;
  opt.version = version;
  opt.token = token;
  reusex::ProjectDB db(rux);
  const auto r = import_speckle_revit(db, opt);
  CHECK(r.version.is_bundle());
  CHECK(r.stored.storeys > 0);
  std::cout << "version " << r.version.id << " ("
            << r.version.source_application << "): " << r.stats.elements
            << " elements, " << r.stats.levels << " levels, storeys "
            << r.stored.storeys << ", walls " << r.stored.walls << ", openings "
            << r.stored.openings << ", spaces " << r.stored.spaces
            << ", meshes " << r.meshes_saved << "\n  elevation offset "
            << r.stats.elevation_offset_m << " m ("
            << r.stats.elevation_offset_source << "), world offset ("
            << r.stats.world_offset_m[0] << ", " << r.stats.world_offset_m[1]
            << "), walls thin " << r.stats.walls_thin << " / not straight "
            << r.stats.walls_not_straight << " / no geometry "
            << r.stats.walls_no_geometry << ", openings by parent "
            << r.stats.openings_by_parent << " / geometry "
            << r.stats.openings_by_geometry << " / unhosted "
            << r.stats.openings_unhosted << ", instance proxies "
            << r.stats.instance_proxies << " (" << r.stats.instance_unresolved
            << " unresolved), no geometry " << r.stats.elements_no_geometry
            << "\n";
  for (const auto &[c, n] : r.stats.categories)
    std::cout << "  " << n << "  " << c << "\n";
  // The same tree once more, mapped without storing, for the mesh extents
  // (instance placements land where the walls are).
  SpeckleReader reader(opt.server_url, token);
  BundleFetchReport report;
  const json root =
      fetch_bundle_tree(reader, project, r.version.referenced_object, &report);
  std::cout << report.files << " files, " << report.bytes << " bytes; "
            << report.projection.objects << " objects, "
            << report.projection.meshes << " meshes, "
            << report.projection.instances << " instances, "
            << report.projection.definitions << " definitions, "
            << report.projection.geometries_skipped << " non-mesh, "
            << report.projection.geometries_bad << " bad; "
            << report.projection.shared_meshes << " shared meshes, "
            << report.projection.mesh_values << " mesh values, "
            << report.projection.expanded_vertices << " expanded vertices, "
            << report.projection.property_rows << " property rows, "
            << report.projection.nodes << " nodes, levels dropped "
            << report.projection.levels_dropped << " / renamed "
            << report.projection.levels_renamed << ", hosted "
            << report.projection.hosted << ", nested "
            << report.projection.nested << ", duplicate ids "
            << report.projection.duplicate_app_ids << "\n";
  const auto mapped = import_revit(root);
  for (const auto &m : mapped.meshes) {
    std::array<double, 3> lo{1e30, 1e30, 1e30}, hi{-1e30, -1e30, -1e30};
    for (const auto &v : m.vertices)
      for (int i = 0; i < 3; ++i) {
        lo[i] = std::min(lo[i], v[i]);
        hi[i] = std::max(hi[i], v[i]);
      }
    std::cout << "  mesh " << m.name << ": " << m.elements << " elements, "
              << m.triangles.size() << " triangles, x [" << lo[0] << ", "
              << hi[0] << "] y [" << lo[1] << ", " << hi[1] << "] z [" << lo[2]
              << ", " << hi[2] << "]\n";
  }
}
