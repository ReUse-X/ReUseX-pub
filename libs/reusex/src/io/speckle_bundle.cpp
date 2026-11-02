// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Speckle bundle-only versions: SGEO decoding, the projection of the bundle
// tables onto the Speckle v3 Revit connector's object tree, and the fetch
// that ties download, DuckDB read and projection together. DuckDB itself is
// confined to speckle_bundle_duckdb.cpp.
//
// Sources (read for the format, not copied verbatim):
//  - specklepy 4b31244a: specklepy/bundle/sgeo.py (SGEO v1 codec),
//    bundle_reader.py (tables), property_table.py (EAV -> nested, coalesce),
//    base_projection.py (bundle -> Base tree), model.py (parse_transform);
//  - speckle-sharp-sdk 99476158: Speckle.Objects/Utils/SgeoEncoder.cs,
//    Speckle.Sdk/Pipelines/Receive/Artifacts/.

#include "io/speckle_bundle.hpp"

#include "core/logging.hpp"

#include <fmt/format.h>

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <random>
#include <set>
#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace reusex::io::speckle {

using nlohmann::json;

// ================================================================ SGEO ===

namespace {

constexpr std::size_t kSgeoHeaderSize = 16;
constexpr std::uint8_t kSgeoVersion1 = 1;
constexpr std::uint16_t kFlagHasNormals = 1u << 4;
constexpr std::uint16_t kFlagHasUvs = 1u << 5;

std::uint16_t le16(const unsigned char *p) {
  return static_cast<std::uint16_t>(p[0] | (p[1] << 8));
}
std::uint32_t le32(const unsigned char *p) {
  return static_cast<std::uint32_t>(p[0]) |
         (static_cast<std::uint32_t>(p[1]) << 8) |
         (static_cast<std::uint32_t>(p[2]) << 16) |
         (static_cast<std::uint32_t>(p[3]) << 24);
}
double le_f64(const unsigned char *p) {
  std::uint64_t bits = 0;
  for (int i = 7; i >= 0; --i)
    bits = (bits << 8) | p[i];
  double d;
  std::memcpy(&d, &bits, sizeof d);
  return d;
}

const std::array<std::uint32_t, 256> &crc_table() {
  static const auto table = [] {
    std::array<std::uint32_t, 256> t{};
    for (std::uint32_t i = 0; i < 256; ++i) {
      std::uint32_t c = i;
      for (int k = 0; k < 8; ++k)
        c = (c & 1) ? 0xEDB88320u ^ (c >> 1) : c >> 1;
      t[i] = c;
    }
    return t;
  }();
  return table;
}

/// Bounds-checked little-endian reader over an SGEO body.
class BodyReader {
    public:
  explicit BodyReader(std::string_view body) : body_(body) {}
  std::uint32_t u32() {
    need(4);
    const auto v = le32(at());
    pos_ += 4;
    return v;
  }
  void f64s(std::size_t n, std::vector<double> &out) {
    if (n > (body_.size() - pos_) / 8)
      fail(n * 8);
    out.resize(n);
    for (std::size_t i = 0; i < n; ++i)
      out[i] = le_f64(at() + 8 * i);
    pos_ += 8 * n;
  }
  void i32s(std::size_t n, std::vector<std::int32_t> &out) {
    if (n > (body_.size() - pos_) / 4)
      fail(n * 4);
    out.resize(n);
    for (std::size_t i = 0; i < n; ++i)
      out[i] = static_cast<std::int32_t>(le32(at() + 4 * i));
    pos_ += 4 * n;
  }

    private:
  const unsigned char *at() const {
    return reinterpret_cast<const unsigned char *>(body_.data()) + pos_;
  }
  void need(std::size_t n) const {
    if (n > body_.size() - pos_)
      fail(n);
  }
  [[noreturn]] void fail(std::size_t n) const {
    throw std::runtime_error(fmt::format(
        "SGEO: body truncated ({} bytes needed at offset {}, {} left)", n, pos_,
        body_.size() - pos_));
  }
  std::string_view body_;
  std::size_t pos_ = 0;
};

} // anonymous namespace

std::uint32_t sgeo_crc32(std::string_view data) {
  const auto &t = crc_table();
  std::uint32_t c = 0xFFFFFFFFu;
  for (const char ch : data)
    c = t[(c ^ static_cast<unsigned char>(ch)) & 0xFFu] ^ (c >> 8);
  return c ^ 0xFFFFFFFFu;
}

std::string sgeo_units(std::uint16_t code) {
  // Speckle.Sdk.Common.Units.GetEncodingFromUnit (sgeo.py _UNIT_ENCODING).
  switch (code) {
  case 1:
    return "mm";
  case 2:
    return "cm";
  case 3:
    return "m";
  case 4:
    return "km";
  case 5:
    return "in";
  case 6:
    return "ft";
  case 7:
    return "yd";
  case 8:
    return "mi";
  default:
    return {};
  }
}

SgeoHeader decode_sgeo_header(std::string_view blob) {
  if (blob.size() < kSgeoHeaderSize)
    throw std::runtime_error(
        fmt::format("SGEO: blob is {} bytes, shorter than the 16-byte header",
                    blob.size()));
  if (blob.substr(0, 4) != "SGEO")
    throw std::runtime_error("SGEO: bad magic");
  const auto *p = reinterpret_cast<const unsigned char *>(blob.data());
  SgeoHeader h;
  h.version = p[4];
  if (h.version != kSgeoVersion1)
    throw std::runtime_error(
        fmt::format("SGEO: unsupported version {}", h.version));
  h.primitive = p[5];
  h.flags = le16(p + 6);
  h.units_code = le16(p + 8);
  h.crc = le32(p + 12);
  return h;
}

SgeoMesh decode_sgeo_mesh(std::string_view blob, bool verify_crc) {
  const SgeoHeader h = decode_sgeo_header(blob);
  if (h.primitive != kSgeoMesh)
    throw std::runtime_error(
        fmt::format("SGEO: primitive {} is not a mesh", h.primitive));
  const std::string_view body = blob.substr(kSgeoHeaderSize);
  if (verify_crc) {
    const std::uint32_t actual = sgeo_crc32(body);
    if (actual != h.crc)
      throw std::runtime_error(
          fmt::format("SGEO: CRC mismatch (header {:#010x}, body {:#010x})",
                      h.crc, actual));
  }
  // Body: u32 vertex count, u32 face-int count, f64 vertices[3n], i32
  // faces[m], then (flags) pad8 + normals, pad8 + UVs, colours — not read.
  BodyReader r(body);
  SgeoMesh m;
  const std::uint32_t nv = r.u32();
  const std::uint32_t nf = r.u32();
  r.f64s(static_cast<std::size_t>(nv) * 3, m.vertices);
  r.i32s(nf, m.faces);
  (void)kFlagHasNormals;
  (void)kFlagHasUvs;
  m.units = sgeo_units(h.units_code);
  return m;
}

// ========================================================== projection ===

namespace {

constexpr std::string_view kParametersPrefix = "properties.Parameters.";
constexpr std::size_t kMaxNesting = 64; ///< SUBELEMENT depth (SDK: 64)

/// "-12", "0.5", "1e3": a plain JSON number literal (no leading zero, so
/// "011" stays a string).
bool is_number_literal(std::string_view s) {
  std::size_t i = 0;
  if (i < s.size() && s[i] == '-')
    ++i;
  if (i >= s.size() || s[i] < '0' || s[i] > '9')
    return false;
  if (s[i] == '0' && i + 1 < s.size() && s[i + 1] >= '0' && s[i + 1] <= '9')
    return false;
  while (i < s.size() && s[i] >= '0' && s[i] <= '9')
    ++i;
  if (i < s.size() && s[i] == '.') {
    ++i;
    if (i >= s.size() || s[i] < '0' || s[i] > '9')
      return false;
    while (i < s.size() && s[i] >= '0' && s[i] <= '9')
      ++i;
  }
  if (i < s.size() && (s[i] == 'e' || s[i] == 'E')) {
    ++i;
    if (i < s.size() && (s[i] == '+' || s[i] == '-'))
      ++i;
    if (i >= s.size() || s[i] < '0' || s[i] > '9')
      return false;
    while (i < s.size() && s[i] >= '0' && s[i] <= '9')
      ++i;
  }
  return i == s.size();
}

json number_json(double d) {
  if (std::isfinite(d) && d == std::floor(d) && std::fabs(d) < 9.0e15)
    return static_cast<std::int64_t>(d);
  if (!std::isfinite(d))
    return nullptr;
  return d;
}

/// The SDK's coalesce (boolean, double, string), except that a string the
/// writer also parsed as a number stays a string when it is not a plain
/// number literal and has no unit ("011", "1-02"): identity values (marks,
/// room numbers) keep their spelling.
json value_json(const BundleValue &v) {
  if (v.b)
    return *v.b;
  if (v.d) {
    if (!v.s || !v.unit.empty() || is_number_literal(*v.s))
      return number_json(*v.d);
    return *v.s;
  }
  if (v.s)
    return *v.s;
  return nullptr;
}

/// Set @p leaf at the dotted @p path (property_table.py to_nested: a
/// non-object on the way is replaced by an object; the last part is
/// overwritten).
void set_path(json &root, std::string_view path, json leaf) {
  json *cur = &root;
  while (true) {
    const auto dot = path.find('.');
    const std::string part(path.substr(0, dot));
    if (dot == std::string_view::npos) {
      (*cur)[part] = std::move(leaf);
      return;
    }
    json &child = (*cur)[part];
    if (!child.is_object())
      child = json::object();
    cur = &child;
    path.remove_prefix(dot + 1);
  }
}

void apply_row(json &obj, const std::string &path, const BundleValue &v) {
  if (path.empty())
    return;
  if (path.rfind(kParametersPrefix, 0) == 0 &&
      path.size() > kParametersPrefix.size()) {
    // A Revit parameter in the connector's shape.
    json p = {{"name", path.substr(path.rfind('.') + 1)},
              {"value", value_json(v)}};
    if (!v.unit.empty())
      p["units"] = v.unit;
    if (!v.internal_definition_name.empty())
      p["internalDefinitionName"] = v.internal_definition_name;
    set_path(obj, path, std::move(p));
    return;
  }
  set_path(obj, path, value_json(v));
}

/// "a,b,...": 16 row-major values (model.py parse_transform), or nullopt.
std::optional<std::array<double, 16>> parse_transform(std::string_view csv) {
  std::array<double, 16> m{};
  std::size_t i = 0;
  while (true) {
    const auto comma = csv.find(',');
    std::string part(csv.substr(0, comma));
    if (i >= 16)
      return std::nullopt;
    try {
      std::size_t used = 0;
      m[i] = std::stod(part, &used);
      if (used != part.size())
        return std::nullopt;
    } catch (const std::exception &) {
      return std::nullopt;
    }
    ++i;
    if (comma == std::string_view::npos)
      break;
    csv.remove_prefix(comma + 1);
  }
  if (i != 16)
    return std::nullopt;
  for (const double x : m)
    if (!std::isfinite(x))
      return std::nullopt;
  return m;
}

constexpr std::array<double, 16> kIdentity = {1, 0, 0, 0, 0, 1, 0, 0,
                                              0, 0, 1, 0, 0, 0, 0, 1};

json mesh_json(const SgeoMesh &m, const std::string &app_id) {
  json j = {{"speckle_type", "Objects.Geometry.Mesh"},
            {"vertices", m.vertices},
            {"faces", m.faces}};
  if (!m.units.empty())
    j["units"] = m.units;
  if (!app_id.empty())
    j["applicationId"] = app_id;
  return j;
}

struct Edge {
  std::int32_t dst;
  std::int32_t ord;
};

using EdgeMap = std::unordered_map<std::int32_t, std::vector<Edge>>;

void sort_edges(EdgeMap &m) {
  for (auto &[k, v] : m)
    std::stable_sort(v.begin(), v.end(), [](const Edge &a, const Edge &b) {
      return a.ord < b.ord || (a.ord == b.ord && a.dst < b.dst);
    });
}

class Projector {
    public:
  Projector(const BundleTables &t, BundleProjectionStats &st) : t_(t), st_(st) {
    for (const auto &g : t_.geometries)
      geometry_.emplace(g.index, &g);
    for (const auto &n : t_.nodes)
      nodes_.emplace(n.id, &n);
    for (const auto &r : t_.relations) {
      switch (static_cast<BundleRel>(r.rel)) {
      case BundleRel::display:
        display_[r.src].push_back({r.dst, r.ord});
        break;
      case BundleRel::subelement:
        children_[r.src].push_back({r.dst, r.ord});
        break;
      case BundleRel::defines:
        defines_[r.src].push_back({r.dst, r.ord});
        break;
      case BundleRel::on_level:
        level_of_.emplace(r.src, r.dst); // first wins
        break;
      case BundleRel::display_instance:
        placements_[r.src].push_back({r.dst, r.ord});
        break;
      case BundleRel::defines_instance:
        nested_[r.src].push_back({r.dst, r.ord});
        break;
      case BundleRel::hosted_on:
        host_of_.emplace(r.src, r.dst);
        break;
      default:
        break;
      }
    }
    for (auto *m : {&display_, &children_, &defines_, &placements_, &nested_})
      sort_edges(*m);
  }

  json run() {
    json root = {{"speckle_type", "Speckle.Core.Models.Collections.Collection"},
                 {"name", "Speckle bundle"},
                 {"applicationId", "bundle-root"},
                 {"version", "3"}};

    // Objects (unnested first).
    std::map<std::int32_t, json> objs;
    std::map<std::string, std::size_t> unit_votes;
    for (const auto &[k, app] : t_.objects)
      objs.emplace(k, json::object());
    {
      // Type properties first, the instance's over them.
      std::unordered_map<std::int32_t, std::vector<const BundleEavRow *>>
          by_type;
      for (const auto &r : t_.type_eav)
        by_type[r.key].push_back(&r);
      for (auto &[k, o] : objs)
        if (auto ty = t_.object_type.find(k); ty != t_.object_type.end())
          if (auto rows = by_type.find(ty->second); rows != by_type.end())
            for (const auto *r : rows->second)
              apply_row(o, path(r->path), r->value);
      for (const auto &r : t_.eav)
        if (auto o = objs.find(r.key); o != objs.end())
          apply_row(o->second, path(r.path), r.value);
    }
    for (auto &[k, o] : objs) {
      const std::string &app = app_id(k);
      o["applicationId"] = app;
      o["id"] = app;
      if (!o.contains("speckle_type") || !o["speckle_type"].is_string())
        o["speckle_type"] = "Objects.Data.DataObject";
      if (auto u = o.find("units"); u != o.end() && u->is_string())
        ++unit_votes[u->get<std::string>()];
      if (auto l = level_of_.find(k); l != level_of_.end())
        if (auto n = nodes_.find(l->second);
            n != nodes_.end() &&
            n->second->kind == static_cast<int>(BundleNodeKind::level))
          o["level"] = n->second->name;
      if (auto h = host_of_.find(k); h != host_of_.end()) {
        json &props = o["properties"];
        if (!props.is_object())
          props = json::object();
        if (!props.contains("parentApplicationId") &&
            t_.objects.count(h->second)) {
          props["parentApplicationId"] = app_id(h->second);
          ++st_.hosted;
        }
      }
      json dv = json::array();
      if (auto d = display_.find(k); d != display_.end())
        for (const auto &e : d->second)
          if (auto m = mesh_of(e.dst))
            dv.push_back(mesh_json(*m, {}));
      if (auto p = placements_.find(k); p != placements_.end())
        for (const auto &e : p->second)
          if (auto proxy = instance_proxy(e.dst, {}); !proxy.is_null())
            dv.push_back(std::move(proxy));
      o["displayValue"] = std::move(dv);
      ++st_.speckle_types[o["speckle_type"].get<std::string>()];
    }
    std::string units = "m";
    if (!unit_votes.empty())
      units = std::max_element(unit_votes.begin(), unit_votes.end(),
                               [](const auto &a, const auto &b) {
                                 return a.second < b.second;
                               })
                  ->first;
    else
      core::warn("Speckle bundle: no object has units; assuming metres");
    root["units"] = units;

    // Nesting: SUBELEMENT children go into their parent's elements.
    std::unordered_set<std::int32_t> is_child;
    for (const auto &[parent, kids] : children_)
      if (objs.count(parent))
        for (const auto &e : kids)
          if (e.dst != parent && objs.count(e.dst))
            is_child.insert(e.dst);
    json elements = json::array();
    std::unordered_set<std::int32_t> placed;
    std::function<json(std::int32_t, std::size_t)> take =
        [&](std::int32_t k, std::size_t depth) {
          placed.insert(k);
          json o = std::move(objs.at(k));
          if (auto c = children_.find(k);
              c != children_.end() && depth < kMaxNesting) {
            json kids = json::array();
            for (const auto &e : c->second)
              if (objs.count(e.dst) && !placed.count(e.dst)) {
                kids.push_back(take(e.dst, depth + 1));
                ++st_.nested;
              }
            if (!kids.empty())
              o["elements"] = std::move(kids);
          }
          return o;
        };
    for (const auto &[k, o] : objs)
      if (!is_child.count(k) && !placed.count(k))
        elements.push_back(take(k, 0));
    for (const auto &[k, o] : objs) // a SUBELEMENT cycle: no root of its own
      if (!placed.count(k))
        elements.push_back(take(k, 0));
    st_.objects = objs.size();
    root["elements"] = std::move(elements);

    // Levels.
    std::map<std::int32_t, json> level_objects;
    for (const auto &[obj, lvl] : level_of_)
      if (t_.objects.count(obj))
        level_objects[lvl].push_back(app_id(obj));
    json levels = json::array();
    for (const auto &n : sorted_nodes(BundleNodeKind::level)) {
      json value = {{"speckle_type", "Objects.Data.DataObject"},
                    {"name", n->name},
                    {"units", n->units.empty() ? units : n->units},
                    {"properties", json::object()}};
      if (n->elevation)
        value["elevation"] = *n->elevation;
      json objects =
          level_objects.count(n->id) ? level_objects[n->id] : json::array();
      levels.push_back({{"speckle_type", "Objects.Other.LevelProxy"},
                        {"applicationId", ""},
                        {"objects", std::move(objects)},
                        {"value", std::move(value)}});
    }
    st_.levels = levels.size();
    root["levelProxies"] = std::move(levels);

    // Instance definitions and their geometry.
    json defs = json::array();
    json def_geometry = json::array();
    std::set<std::int32_t> geometry_emitted, nested_emitted;
    for (const auto &n : sorted_nodes(BundleNodeKind::definition)) {
      json members = json::array();
      if (auto d = defines_.find(n->id); d != defines_.end())
        for (const auto &e : d->second) {
          const std::string gid = fmt::format("geo-{}", e.dst);
          if (!geometry_emitted.count(e.dst)) {
            auto m = mesh_of(e.dst);
            if (!m)
              continue;
            def_geometry.push_back(mesh_json(*m, gid));
            geometry_emitted.insert(e.dst);
          }
          members.push_back(gid);
        }
      if (auto d = nested_.find(n->id); d != nested_.end())
        for (const auto &e : d->second) {
          const std::string iid = fmt::format("inst-{}", e.dst);
          if (!nested_emitted.count(e.dst)) {
            json proxy = instance_proxy(e.dst, iid);
            if (proxy.is_null())
              continue;
            def_geometry.push_back(std::move(proxy));
            nested_emitted.insert(e.dst);
          }
          members.push_back(iid);
        }
      defs.push_back({{"speckle_type",
                       "Speckle.Core.Models.Instances.InstanceDefinitionProxy"},
                      {"applicationId", fmt::format("def-{}", n->id)},
                      {"name", n->name},
                      {"objects", std::move(members)}});
    }
    st_.definitions = defs.size();
    root["instanceDefinitionProxies"] = std::move(defs);
    root["definitionGeometry"] = std::move(def_geometry);

    if (st_.geometries_bad > 0)
      core::warn("Speckle bundle: {} geometries could not be decoded (bad "
                 "SGEO / CRC); left out",
                 st_.geometries_bad);
    if (st_.geometries_missing > 0)
      core::warn("Speckle bundle: {} geometry references point at no "
                 "geometry row; left out",
                 st_.geometries_missing);
    core::info("Speckle bundle: projected {} objects ({} nested, {} hosted), "
               "{} levels, {} definitions, {} instance proxies, {} meshes "
               "({} non-mesh geometries skipped)",
               st_.objects, st_.nested, st_.hosted, st_.levels, st_.definitions,
               st_.instances, st_.meshes, st_.geometries_skipped);
    return root;
  }

    private:
  const std::string &path(std::int32_t idx) const {
    static const std::string empty;
    auto it = t_.paths.find(idx);
    return it == t_.paths.end() ? empty : it->second;
  }
  std::string app_id(std::int32_t k) const {
    auto it = t_.objects.find(k);
    if (it == t_.objects.end() || it->second.empty())
      return fmt::format("bundle-object-{}", k);
    return it->second;
  }

  std::vector<const BundleNode *> sorted_nodes(BundleNodeKind kind) const {
    std::vector<const BundleNode *> out;
    for (const auto &n : t_.nodes)
      if (n.kind == static_cast<int>(kind))
        out.push_back(&n);
    std::sort(out.begin(), out.end(),
              [](const auto *a, const auto *b) { return a->id < b->id; });
    return out;
  }

  /// The decoded mesh of geometry @p g (cached), or nullopt for a non-mesh
  /// primitive, a missing row or a bad blob (counted once each).
  const SgeoMesh *mesh_of(std::int32_t g) {
    if (auto c = meshes_.find(g); c != meshes_.end())
      return c->second ? &*c->second : nullptr;
    auto &slot = meshes_[g];
    auto it = geometry_.find(g);
    if (it == geometry_.end()) {
      ++st_.geometries_missing;
      return nullptr;
    }
    try {
      const SgeoHeader h = decode_sgeo_header(it->second->content);
      if (h.primitive != kSgeoMesh) {
        ++st_.geometries_skipped;
        return nullptr;
      }
      slot = decode_sgeo_mesh(it->second->content);
      ++st_.meshes;
      return &*slot;
    } catch (const std::exception &e) {
      ++st_.geometries_bad;
      core::debug("Speckle bundle: geometry {}: {}", g, e.what());
      return nullptr;
    }
  }

  json instance_proxy(std::int32_t instance, const std::string &app) {
    auto n = nodes_.find(instance);
    if (n == nodes_.end() ||
        n->second->kind != static_cast<int>(BundleNodeKind::instance))
      return nullptr;
    const auto m = parse_transform(n->second->transform);
    json j = {{"speckle_type", "Speckle.Core.Models.Instances.InstanceProxy"},
              {"definitionId",
               fmt::format("def-{}", n->second->def_ref.value_or(-1))},
              {"transform", m ? *m : kIdentity},
              {"maxDepth", 0}};
    if (!n->second->units.empty())
      j["units"] = n->second->units;
    if (!app.empty())
      j["applicationId"] = app;
    ++st_.instances;
    return j;
  }

  const BundleTables &t_;
  BundleProjectionStats &st_;
  std::unordered_map<std::int32_t, const BundleGeometry *> geometry_;
  std::unordered_map<std::int32_t, const BundleNode *> nodes_;
  std::unordered_map<std::int32_t, std::optional<SgeoMesh>> meshes_;
  EdgeMap display_, children_, defines_, placements_, nested_;
  std::unordered_map<std::int32_t, std::int32_t> level_of_, host_of_;
};

} // anonymous namespace

json project_bundle_to_revit_tree(const BundleTables &tables,
                                  BundleProjectionStats *stats) {
  BundleProjectionStats local;
  BundleProjectionStats &st = stats ? *stats : local;
  st = {};
  return Projector(tables, st).run();
}

// =============================================================== fetch ===

namespace {

/// A private, empty temporary directory, removed with its contents.
class TempDir {
    public:
  TempDir() {
    const auto base = std::filesystem::temp_directory_path();
    std::random_device rd;
    for (int attempt = 0; attempt < 16; ++attempt) {
      const auto p =
          base / fmt::format("reusex-speckle-bundle-{:016x}",
                             (static_cast<std::uint64_t>(rd()) << 32) | rd());
      std::error_code ec;
      if (std::filesystem::create_directory(p, ec)) {
        std::filesystem::permissions(p, std::filesystem::perms::owner_all,
                                     std::filesystem::perm_options::replace,
                                     ec);
        path_ = p;
        return;
      }
    }
    throw std::runtime_error(
        "Speckle bundle: cannot create a temporary directory");
  }
  ~TempDir() {
    std::error_code ec;
    std::filesystem::remove_all(path_, ec);
  }
  TempDir(const TempDir &) = delete;
  TempDir &operator=(const TempDir &) = delete;
  [[nodiscard]] const std::filesystem::path &path() const { return path_; }

    private:
  std::filesystem::path path_;
};

} // anonymous namespace

json fetch_bundle_tree(const SpeckleReader &reader, const std::string &project,
                       const std::string &reference,
                       BundleFetchReport *report) {
  const auto ref = parse_bundle_reference(reference);
  if (!ref)
    throw std::invalid_argument("Speckle: malformed bundle reference (expected "
                                "bundle.<project>.<model>.<version>)");
  if (ref->project != project)
    throw std::invalid_argument(fmt::format(
        "Speckle: bundle reference belongs to project '{}', not '{}'",
        ref->project, project));
  if (!has_bundle_support())
    throw BundleVersionUnsupported(reference);
  stopwatch sw;
  const TempDir dir;
  std::vector<std::filesystem::path> files;
  const std::size_t bytes = reader.download_bundle(*ref, dir.path(), &files);
  BundleTables tables = read_bundle_directory(dir.path());
  BundleFetchReport local;
  BundleFetchReport &r = report ? *report : local;
  r.files = files.size();
  r.bytes = bytes;
  json root = project_bundle_to_revit_tree(tables, &r.projection);
  core::info("Speckle bundle: version {} received in {:.1f} s", ref->version,
             sw.elapsed());
  return root;
}

} // namespace reusex::io::speckle
