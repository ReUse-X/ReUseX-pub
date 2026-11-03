// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/pose_versions.hpp"

#include "core/logging.hpp"
#include "core/pose_graph.hpp"
#include "core/working_pose_edits.hpp"

#include <fmt/format.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <map>
#include <set>
#include <stdexcept>

namespace reusex::core {
namespace {

namespace pg = pose_graph;
using Bytes = std::vector<std::uint8_t>;

// v40's fixed-width full snapshots: still read, no longer written.
constexpr std::uint32_t kFramesMagicV1 = 0x31465852; // "RXF1"
constexpr std::uint32_t kEdgesMagicV1 = 0x31455852;  // "RXE1"
// reusex-md4a: a full keyframe or a delta against a base version, with
// varints, a string table and compact matrices.
constexpr std::uint32_t kFramesMagic = 0x32465852; // "RXF2"
constexpr std::uint32_t kEdgesMagic = 0x32455852;  // "RXE2"

// ── Packing ────────────────────────────────────────────────────────────────

bool same_bits(double a, double b) {
  return std::memcmp(&a, &b, sizeof(double)) == 0;
}
template <size_t N>
bool same_bits(const std::array<double, N> &a, const std::array<double, N> &b) {
  return std::memcmp(a.data(), b.data(), N * sizeof(double)) == 0;
}
bool same_bits(const std::optional<double> &a, const std::optional<double> &b) {
  return a.has_value() == b.has_value() && (!a || same_bits(*a, *b));
}
template <size_t N>
bool same_bits(const std::optional<std::array<double, N>> &a,
               const std::optional<std::array<double, N>> &b) {
  return a.has_value() == b.has_value() && (!a || same_bits(*a, *b));
}

/// Bottom row exactly (0, 0, 0, 1): stored as its top 12 values.
bool is_affine(const std::array<double, 16> &T) {
  return same_bits(T[12], 0.0) && same_bits(T[13], 0.0) &&
         same_bits(T[14], 0.0) && same_bits(T[15], 1.0);
}
/// Every off-diagonal value exactly +0.0: stored as the 6 diagonal values.
bool is_diagonal(const std::array<double, 36> &I) {
  for (int r = 0; r < 6; ++r)
    for (int c = 0; c < 6; ++c)
      if (r != c && !same_bits(I[static_cast<size_t>(r * 6 + c)], 0.0))
        return false;
  return true;
}
/// Bitwise symmetric: stored as the 21 upper-triangle values.
bool is_symmetric(const std::array<double, 36> &I) {
  for (int r = 0; r < 6; ++r)
    for (int c = r + 1; c < 6; ++c)
      if (!same_bits(I[static_cast<size_t>(r * 6 + c)],
                     I[static_cast<size_t>(c * 6 + r)]))
        return false;
  return true;
}

struct Writer {
  Bytes out;
  template <typename T> void put(const T &v) {
    const auto *p = reinterpret_cast<const std::uint8_t *>(&v);
    out.insert(out.end(), p, p + sizeof(T));
  }
  void put_doubles(const double *v, size_t n) {
    const auto *p = reinterpret_cast<const std::uint8_t *>(v);
    out.insert(out.end(), p, p + n * sizeof(double));
  }
  void put_varint(std::uint64_t v) {
    while (v >= 0x80) {
      out.push_back(static_cast<std::uint8_t>(v | 0x80));
      v >>= 7;
    }
    out.push_back(static_cast<std::uint8_t>(v));
  }
  void put_svarint(std::int64_t v) {
    put_varint((static_cast<std::uint64_t>(v) << 1) ^
               static_cast<std::uint64_t>(v >> 63));
  }
  void put_str(const std::string &s) {
    put_varint(s.size());
    out.insert(out.end(), s.begin(), s.end());
  }
  /// 12 values when the bottom row is (0, 0, 0, 1), else 16; @returns
  /// whether it was the short form.
  bool put_pose(const std::array<double, 16> &T) {
    const bool affine = is_affine(T);
    put_doubles(T.data(), affine ? 12 : 16);
    return affine;
  }
};

struct Reader {
  const Bytes &in;
  size_t pos = 0;
  void need(size_t n) const {
    if (n > in.size() - pos)
      throw std::runtime_error("pose version: truncated snapshot blob");
  }
  template <typename T> T get() {
    need(sizeof(T));
    T v;
    std::memcpy(&v, in.data() + pos, sizeof(T));
    pos += sizeof(T);
    return v;
  }
  void get_doubles(double *v, size_t n) {
    need(n * sizeof(double));
    std::memcpy(v, in.data() + pos, n * sizeof(double));
    pos += n * sizeof(double);
  }
  template <size_t N> std::array<double, N> get_array() {
    std::array<double, N> a{};
    get_doubles(a.data(), N);
    return a;
  }
  std::uint64_t get_varint() {
    std::uint64_t v = 0;
    for (int shift = 0; shift < 64; shift += 7) {
      const auto b = get<std::uint8_t>();
      v |= static_cast<std::uint64_t>(b & 0x7f) << shift;
      if (!(b & 0x80))
        return v;
    }
    throw std::runtime_error("pose version: bad varint in snapshot blob");
  }
  std::int64_t get_svarint() {
    const std::uint64_t u = get_varint();
    return static_cast<std::int64_t>(u >> 1) ^
           -static_cast<std::int64_t>(u & 1);
  }
  std::string get_str() {
    const auto n = get_varint();
    need(n);
    std::string s(reinterpret_cast<const char *>(in.data() + pos), n);
    pos += n;
    return s;
  }
  std::array<double, 16> get_pose(bool affine) {
    std::array<double, 16> T{};
    get_doubles(T.data(), affine ? 12 : 16);
    if (affine) {
      T[12] = T[13] = T[14] = 0.0;
      T[15] = 1.0;
    }
    return T;
  }
  bool done() const { return pos == in.size(); }
};

// --- v1 (read only) ---

std::optional<double> get_optional_v1(Reader &r) {
  const bool has = r.get<std::uint8_t>() != 0;
  const double v = r.get<double>();
  return has ? std::optional<double>(v) : std::nullopt;
}

std::string get_str_v1(Reader &r) {
  const auto n = r.get<std::uint32_t>();
  r.need(n);
  std::string s(reinterpret_cast<const char *>(r.in.data() + r.pos), n);
  r.pos += n;
  return s;
}

ProjectDB::PoseGraphEdge unpack_edge_v1(Reader &r) {
  ProjectDB::PoseGraphEdge e;
  e.id = r.get<std::int64_t>();
  e.from_node_id = r.get<std::int32_t>();
  e.to_node_id = r.get<std::int32_t>();
  e.edge_type = get_str_v1(r);
  e.residual = r.get<double>();
  e.weight = r.get<double>();
  const auto flags = r.get<std::uint8_t>();
  if (flags & 1)
    e.transform = r.get_array<16>();
  if (flags & 2)
    e.information = r.get_array<36>();
  e.source = get_str_v1(r);
  e.status = get_str_v1(r);
  e.created_at = get_str_v1(r);
  e.seed_disagreement_m = get_optional_v1(r);
  e.seed_disagreement_deg = get_optional_v1(r);
  e.deviation_m = get_optional_v1(r);
  e.deviation_deg = get_optional_v1(r);
  return e;
}

// --- v2 ---

/// Frame record flags.
enum : std::uint8_t {
  kFrameTransform = 1,
  kFrameTransformAffine = 2,
  kFrameInitial = 4,
  kFrameInitialAffine = 8,
  kFrameInitialCleared = 16,
};

/// Edge record kinds and flags.
enum : std::uint8_t { kEdgeFull = 0, kEdgeMetrics = 1 };
enum : std::uint8_t {
  kEdgeTransform = 1,
  kEdgeTransformAffine = 2,
  kEdgeInformation = 4,
  kEdgeInfoSymmetric = 8,
  kEdgeInfoDiagonal = 16,
};

/// Interned strings of one edges blob.
struct StringTable {
  std::vector<std::string> strings;
  std::map<std::string, std::uint64_t> index;
  std::uint64_t id(const std::string &s) {
    auto [it, fresh] = index.emplace(s, strings.size());
    if (fresh)
      strings.push_back(s);
    return it->second;
  }
};

const std::string &lookup(const std::vector<std::string> &table,
                          std::uint64_t i) {
  if (i >= table.size())
    throw std::runtime_error("pose version: bad string index in edges blob");
  return table[i];
}

void put_metrics(Writer &w, const ProjectDB::PoseGraphEdge &e) {
  w.put(e.residual);
  w.put(e.weight);
  const std::optional<double> *opts[] = {&e.seed_disagreement_m,
                                         &e.seed_disagreement_deg,
                                         &e.deviation_m, &e.deviation_deg};
  std::uint8_t mask = 0;
  for (int k = 0; k < 4; ++k)
    if (opts[k]->has_value())
      mask |= static_cast<std::uint8_t>(1u << k);
  w.put(mask);
  for (const auto *o : opts)
    if (o->has_value())
      w.put(**o);
}

void get_metrics(Reader &r, ProjectDB::PoseGraphEdge &e) {
  e.residual = r.get<double>();
  e.weight = r.get<double>();
  const auto mask = r.get<std::uint8_t>();
  std::optional<double> *opts[] = {&e.seed_disagreement_m,
                                   &e.seed_disagreement_deg, &e.deviation_m,
                                   &e.deviation_deg};
  for (int k = 0; k < 4; ++k)
    if (mask & (1u << k))
      *opts[k] = r.get<double>();
    else
      opts[k]->reset();
}

void put_edge_full(Writer &w, StringTable &strings,
                   const ProjectDB::PoseGraphEdge &e) {
  w.put_svarint(e.from_node_id);
  w.put_svarint(e.to_node_id);
  w.put_varint(strings.id(e.edge_type));
  std::uint8_t flags = 0;
  if (e.transform) {
    flags |= kEdgeTransform;
    if (is_affine(*e.transform))
      flags |= kEdgeTransformAffine;
  }
  if (e.information) {
    flags |= kEdgeInformation;
    if (is_diagonal(*e.information))
      flags |= kEdgeInfoDiagonal;
    else if (is_symmetric(*e.information))
      flags |= kEdgeInfoSymmetric;
  }
  w.put(flags);
  if (e.transform)
    w.put_pose(*e.transform);
  if (e.information) {
    const auto &I = *e.information;
    if (flags & kEdgeInfoDiagonal) {
      for (int k = 0; k < 6; ++k)
        w.put(I[static_cast<size_t>(k * 7)]);
    } else if (flags & kEdgeInfoSymmetric) {
      for (int r = 0; r < 6; ++r)
        for (int c = r; c < 6; ++c)
          w.put(I[static_cast<size_t>(r * 6 + c)]);
    } else {
      w.put_doubles(I.data(), 36);
    }
  }
  w.put_varint(strings.id(e.source));
  w.put_varint(strings.id(e.status));
  w.put_varint(strings.id(e.created_at));
  put_metrics(w, e);
}

ProjectDB::PoseGraphEdge get_edge_full(Reader &r,
                                       const std::vector<std::string> &strs) {
  ProjectDB::PoseGraphEdge e;
  e.from_node_id = static_cast<int>(r.get_svarint());
  e.to_node_id = static_cast<int>(r.get_svarint());
  e.edge_type = lookup(strs, r.get_varint());
  const auto flags = r.get<std::uint8_t>();
  if (flags & kEdgeTransform)
    e.transform = r.get_pose((flags & kEdgeTransformAffine) != 0);
  if (flags & kEdgeInformation) {
    std::array<double, 36> I{};
    if (flags & kEdgeInfoDiagonal) {
      for (int k = 0; k < 6; ++k)
        I[static_cast<size_t>(k * 7)] = r.get<double>();
    } else if (flags & kEdgeInfoSymmetric) {
      for (int a = 0; a < 6; ++a)
        for (int c = a; c < 6; ++c)
          I[static_cast<size_t>(a * 6 + c)] =
              I[static_cast<size_t>(c * 6 + a)] = r.get<double>();
    } else {
      r.get_doubles(I.data(), 36);
    }
    e.information = I;
  }
  e.source = lookup(strs, r.get_varint());
  e.status = lookup(strs, r.get_varint());
  e.created_at = lookup(strs, r.get_varint());
  get_metrics(r, e);
  return e;
}

/// Everything but the solve metrics (residual, weight, seed disagreement,
/// deviation) is bitwise the same: a metrics-only record carries the change.
bool only_metrics_differ(const ProjectDB::PoseGraphEdge &a,
                         const ProjectDB::PoseGraphEdge &b) {
  return a.from_node_id == b.from_node_id && a.to_node_id == b.to_node_id &&
         a.edge_type == b.edge_type && same_bits(a.transform, b.transform) &&
         same_bits(a.information, b.information) && a.source == b.source &&
         a.status == b.status && a.created_at == b.created_at;
}

bool metrics_equal(const ProjectDB::PoseGraphEdge &a,
                   const ProjectDB::PoseGraphEdge &b) {
  return same_bits(a.residual, b.residual) && same_bits(a.weight, b.weight) &&
         same_bits(a.seed_disagreement_m, b.seed_disagreement_m) &&
         same_bits(a.seed_disagreement_deg, b.seed_disagreement_deg) &&
         same_bits(a.deviation_m, b.deviation_m) &&
         same_bits(a.deviation_deg, b.deviation_deg);
}

void put_frame(Writer &w, std::int64_t node_id, std::uint8_t flags,
               const FramePoseState &f) {
  if ((flags & kFrameTransform) && is_affine(f.transform))
    flags |= kFrameTransformAffine;
  if ((flags & kFrameInitial) && is_affine(*f.initial))
    flags |= kFrameInitialAffine;
  w.put_svarint(node_id);
  w.put(flags);
  if (flags & kFrameTransform)
    w.put_pose(f.transform);
  if (flags & kFrameInitial)
    w.put_pose(*f.initial);
}

/// Pack @p state against @p base (nullptr: a full keyframe). @p base_id is
/// the version @p base was materialised from (0 for a keyframe).
ProjectDB::PoseVersionSnapshot
pack_against(const PoseState &state, const PoseState *base, int64_t base_id) {
  ProjectDB::PoseVersionSnapshot snap;

  // Frames.
  std::map<int, const FramePoseState *> before;
  if (base)
    for (const auto &f : base->frames)
      before[f.node_id] = &f;
  Writer fw;
  Writer body;
  std::uint64_t upserts = 0;
  std::set<int> seen;
  for (const auto &f : state.frames) {
    seen.insert(f.node_id);
    std::uint8_t flags = 0;
    const auto it = before.find(f.node_id);
    if (it == before.end()) {
      flags = kFrameTransform | (f.initial ? kFrameInitial : 0);
    } else {
      const FramePoseState &b = *it->second;
      if (!same_bits(b.transform, f.transform))
        flags |= kFrameTransform;
      if (!same_bits(b.initial, f.initial))
        flags |= f.initial ? kFrameInitial : kFrameInitialCleared;
      if (flags == 0)
        continue;
    }
    put_frame(body, f.node_id, flags, f);
    ++upserts;
  }
  std::vector<int> removed_frames;
  for (const auto &[id, f] : before)
    if (!seen.count(id))
      removed_frames.push_back(id);
  fw.put(kFramesMagic);
  fw.put_varint(static_cast<std::uint64_t>(base_id));
  fw.put_varint(upserts);
  fw.out.insert(fw.out.end(), body.out.begin(), body.out.end());
  fw.put_varint(removed_frames.size());
  for (const int id : removed_frames)
    fw.put_svarint(id);
  snap.frames = std::move(fw.out);

  // Edges.
  std::map<int64_t, const ProjectDB::PoseGraphEdge *> old_edges;
  if (base)
    for (const auto &e : base->edges)
      old_edges[e.id] = &e;
  StringTable strings;
  Writer eb;
  std::uint64_t records = 0;
  std::set<int64_t> kept;
  for (const auto &e : state.edges) {
    kept.insert(e.id);
    const auto it = old_edges.find(e.id);
    if (it != old_edges.end() && only_metrics_differ(*it->second, e)) {
      if (metrics_equal(*it->second, e))
        continue;
      eb.put_svarint(e.id);
      eb.put(kEdgeMetrics);
      put_metrics(eb, e);
    } else {
      eb.put_svarint(e.id);
      eb.put(kEdgeFull);
      put_edge_full(eb, strings, e);
    }
    ++records;
  }
  std::vector<int64_t> removed_edges;
  for (const auto &[id, e] : old_edges)
    if (!kept.count(id))
      removed_edges.push_back(id);
  Writer ew;
  ew.put(kEdgesMagic);
  ew.put_varint(static_cast<std::uint64_t>(base_id));
  ew.put_varint(strings.strings.size());
  for (const auto &str : strings.strings)
    ew.put_str(str);
  ew.put_varint(records);
  ew.out.insert(ew.out.end(), eb.out.begin(), eb.out.end());
  ew.put_varint(removed_edges.size());
  for (const auto id : removed_edges)
    ew.put_svarint(id);
  snap.edges = std::move(ew.out);

  snap.edits = state.edits;
  return snap;
}

/// The base version a snapshot is a delta against (0: a keyframe).
int64_t snapshot_base(const ProjectDB::PoseVersionSnapshot &snap) {
  Reader f{snap.frames};
  const auto magic = f.get<std::uint32_t>();
  if (magic == kFramesMagicV1)
    return 0;
  if (magic != kFramesMagic)
    throw std::runtime_error("pose version: unknown frames blob format");
  return static_cast<int64_t>(f.get_varint());
}

/// Apply @p snap to @p state (empty for a keyframe).
void apply_snapshot(PoseState &state,
                    const ProjectDB::PoseVersionSnapshot &snap) {
  Reader f{snap.frames};
  const auto fmagic = f.get<std::uint32_t>();
  if (fmagic == kFramesMagicV1) {
    state = PoseState{};
    const auto nf = f.get<std::uint32_t>();
    state.frames.reserve(nf);
    for (std::uint32_t i = 0; i < nf; ++i) {
      FramePoseState fr;
      fr.node_id = f.get<std::int32_t>();
      fr.transform = f.get_array<16>();
      if (f.get<std::uint8_t>() != 0)
        fr.initial = f.get_array<16>();
      state.frames.push_back(fr);
    }
    Reader e{snap.edges};
    if (e.get<std::uint32_t>() != kEdgesMagicV1)
      throw std::runtime_error("pose version: unknown edges blob format");
    const auto ne = e.get<std::uint32_t>();
    state.edges.reserve(ne);
    for (std::uint32_t i = 0; i < ne; ++i)
      state.edges.push_back(unpack_edge_v1(e));
    state.edits = snap.edits;
    return;
  }
  if (fmagic != kFramesMagic)
    throw std::runtime_error("pose version: unknown frames blob format");
  const auto base = f.get_varint();
  if (base == 0)
    state = PoseState{};

  std::map<int, FramePoseState> frames;
  for (auto &fr : state.frames)
    frames.emplace(fr.node_id, std::move(fr));
  const auto upserts = f.get_varint();
  for (std::uint64_t i = 0; i < upserts; ++i) {
    const int id = static_cast<int>(f.get_svarint());
    const auto flags = f.get<std::uint8_t>();
    auto &fr = frames[id];
    fr.node_id = id;
    if (flags & kFrameTransform)
      fr.transform = f.get_pose((flags & kFrameTransformAffine) != 0);
    if (flags & kFrameInitial)
      fr.initial = f.get_pose((flags & kFrameInitialAffine) != 0);
    if (flags & kFrameInitialCleared)
      fr.initial.reset();
  }
  const auto removed = f.get_varint();
  for (std::uint64_t i = 0; i < removed; ++i)
    frames.erase(static_cast<int>(f.get_svarint()));
  if (!f.done())
    throw std::runtime_error("pose version: trailing bytes in frames blob");
  state.frames.clear();
  state.frames.reserve(frames.size());
  for (auto &[id, fr] : frames)
    state.frames.push_back(std::move(fr));

  Reader e{snap.edges};
  if (e.get<std::uint32_t>() != kEdgesMagic)
    throw std::runtime_error("pose version: unknown edges blob format");
  if (e.get_varint() != base)
    throw std::runtime_error(
        "pose version: frames and edges blobs name different bases");
  std::vector<std::string> strs(e.get_varint());
  for (auto &str : strs)
    str = e.get_str();
  std::map<int64_t, ProjectDB::PoseGraphEdge> edges;
  for (auto &edge : state.edges)
    edges.emplace(edge.id, std::move(edge));
  const auto records = e.get_varint();
  for (std::uint64_t i = 0; i < records; ++i) {
    const int64_t id = e.get_svarint();
    const auto kind = e.get<std::uint8_t>();
    if (kind == kEdgeMetrics) {
      const auto it = edges.find(id);
      if (it == edges.end())
        throw std::runtime_error(fmt::format(
            "pose version: metrics for edge {} missing from the base", id));
      get_metrics(e, it->second);
    } else if (kind == kEdgeFull) {
      auto edge = get_edge_full(e, strs);
      edge.id = id;
      edges[id] = std::move(edge);
    } else {
      throw std::runtime_error("pose version: unknown edge record kind");
    }
  }
  const auto gone = e.get_varint();
  for (std::uint64_t i = 0; i < gone; ++i)
    edges.erase(e.get_svarint());
  if (!e.done())
    throw std::runtime_error("pose version: trailing bytes in edges blob");
  state.edges.clear();
  state.edges.reserve(edges.size());
  for (auto &[id, edge] : edges)
    state.edges.push_back(std::move(edge));

  state.edits = snap.edits;
}

/// Version @p id's state and how many delta links lie between it and its
/// keyframe (0: it is one).
std::pair<PoseState, int> materialise(const ProjectDB &db, int64_t id) {
  std::vector<ProjectDB::PoseVersionSnapshot> chain;
  int64_t at = id;
  while (true) {
    auto snap = db.pose_version_snapshot(at);
    if (!snap)
      throw std::invalid_argument(
          at == id ? fmt::format("no pose version {}", id)
                   : fmt::format("pose version {}: its base version {} is "
                                 "missing",
                                 id, at));
    const int64_t base = snapshot_base(*snap);
    chain.push_back(std::move(*snap));
    if (base == 0)
      break;
    if (base >= at) // bases are older versions: never a cycle
      throw std::runtime_error(fmt::format(
          "pose version {}: base {} is not older than {}", id, base, at));
    at = base;
  }
  PoseState state;
  for (auto it = chain.rbegin(); it != chain.rend(); ++it)
    apply_snapshot(state, *it);
  return {std::move(state), static_cast<int>(chain.size()) - 1};
}

/// Bitwise the same row.
bool same_edge(const ProjectDB::PoseGraphEdge &a,
               const ProjectDB::PoseGraphEdge &b) {
  return a.id == b.id && only_metrics_differ(a, b) && metrics_equal(a, b);
}

double translation_change(const std::array<double, 16> &a,
                          const std::array<double, 16> &b) {
  return std::sqrt((a[3] - b[3]) * (a[3] - b[3]) +
                   (a[7] - b[7]) * (a[7] - b[7]) +
                   (a[11] - b[11]) * (a[11] - b[11]));
}

double rotation_change_deg(const std::array<double, 16> &a,
                           const std::array<double, 16> &b) {
  if (a == b)
    return 0.0;
  // Through quaternions: RTAB-Map's float poses are not exactly orthonormal,
  // and acos of the trace of Ra^T Rb reads that as a ~0.01 deg turn.
  const Eigen::Quaterniond qa(
      Eigen::Matrix3d(pg::matrix_from_row_major(a).topLeftCorner<3, 3>()));
  const Eigen::Quaterniond qb(
      Eigen::Matrix3d(pg::matrix_from_row_major(b).topLeftCorner<3, 3>()));
  return qa.normalized().angularDistance(qb.normalized()) * 180.0 / M_PI;
}

/// The working edits; an empty setting (a restored "none") is none.
std::optional<std::string> stored_edits(const ProjectDB &db) {
  auto e = db.setting(kSettingWorkingPoseEdits);
  if (e && e->empty())
    e.reset();
  return e;
}

void set_current(ProjectDB &db, int64_t id) {
  db.set_setting(kSettingPoseVersionCurrent, std::to_string(id));
}

PoseState state_of(const ProjectDB &db, int64_t id) {
  return materialise(db, id).first;
}

/// What restoring @p target over @p live changes.
PoseVersionChange measure_change(const PoseState &live,
                                 const PoseState &target) {
  PoseVersionChange c;
  std::map<int, const FramePoseState *> now;
  for (const auto &f : live.frames)
    now[f.node_id] = &f;
  for (const auto &f : target.frames) {
    auto it = now.find(f.node_id);
    if (it == now.end() || it->second->transform == f.transform)
      continue;
    ++c.frames_changed;
    c.max_shift_m = std::max(
        c.max_shift_m, translation_change(it->second->transform, f.transform));
  }
  std::map<int64_t, const ProjectDB::PoseGraphEdge *> a, b;
  for (const auto &e : live.edges)
    a[e.id] = &e;
  for (const auto &e : target.edges)
    b[e.id] = &e;
  for (const auto &[id, edge] : b) {
    auto it = a.find(id);
    if (it == a.end() || !same_edge(*it->second, *edge))
      ++c.edges_changed;
  }
  for (const auto &[id, edge] : a)
    if (!b.count(id))
      ++c.edges_changed;
  return c;
}

} // namespace

// ── State ──────────────────────────────────────────────────────────────────

PoseState capture_pose_state(const ProjectDB &db) {
  PoseState s;
  for (const auto &p : db.posed_frames())
    s.frames.push_back({p.node_id, p.transform, p.initial});
  s.edges = db.list_pose_graph_edges();
  s.edits = stored_edits(db);
  return s;
}

void restore_pose_state(ProjectDB &db, const PoseState &state) {
  std::set<int> in_state;
  std::map<int, ProjectDB::PosedFrame> live_frames;
  for (auto &p : db.posed_frames())
    live_frames.emplace(p.node_id, std::move(p));
  for (const auto &f : state.frames) {
    in_state.insert(f.node_id);
    const auto it = live_frames.find(f.node_id);
    if (it == live_frames.end()) {
      if (!db.has_sensor_frame(f.node_id))
        continue; // a frame the project no longer has
      db.update_sensor_frame_pose(f.node_id, f.transform); // was unposed
      if (f.initial)
        db.set_initial_pose(f.node_id, *f.initial);
      continue;
    }
    if (it->second.transform != f.transform)
      db.update_sensor_frame_pose(f.node_id, f.transform);
    const auto &initial = it->second.initial;
    if (initial != f.initial) {
      if (f.initial)
        db.set_initial_pose(f.node_id, *f.initial);
      else
        db.clear_initial_pose(f.node_id);
    }
  }

  // Edges: the state's rows, plus the live rows among frames imported after
  // the state was taken.
  const auto live = db.list_pose_graph_edges();
  std::vector<ProjectDB::PoseGraphEdge> edges = state.edges;
  std::set<int64_t> ids;
  for (const auto &e : edges)
    ids.insert(e.id);
  for (auto e : live)
    if (!in_state.count(e.from_node_id) && !in_state.count(e.to_node_id)) {
      if (ids.count(e.id))
        e.id = 0; // let sqlite assign one
      edges.push_back(std::move(e));
    }
  bool same = edges.size() == live.size();
  for (size_t i = 0; same && i < edges.size(); ++i)
    same = same_edge(edges[i], live[i]);
  if (!same)
    db.replace_pose_graph_edges(edges);

  if (stored_edits(db) != state.edits)
    db.set_setting(kSettingWorkingPoseEdits, state.edits.value_or(""));
}

ProjectDB::PoseVersionSnapshot pack_pose_state(const PoseState &state) {
  return pack_against(state, nullptr, 0);
}

PoseState unpack_pose_state(const ProjectDB::PoseVersionSnapshot &snap) {
  if (snapshot_base(snap) != 0)
    throw std::runtime_error(
        "pose version: a delta snapshot needs its base version; read it with "
        "pose_version_state()");
  PoseState s;
  apply_snapshot(s, snap);
  return s;
}

PoseState pose_version_state(const ProjectDB &db, int64_t id) {
  return materialise(db, id).first;
}

bool pose_version_is_keyframe(const ProjectDB &db, int64_t id) {
  const auto snap = db.pose_version_snapshot(id);
  if (!snap)
    throw std::invalid_argument(fmt::format("no pose version {}", id));
  return snapshot_base(*snap) == 0;
}

// ── Versions ───────────────────────────────────────────────────────────────

std::optional<int64_t> current_pose_version(const ProjectDB &db) {
  const auto text = db.setting(kSettingPoseVersionCurrent);
  if (!text || text->empty())
    return std::nullopt;
  try {
    const int64_t id = std::stoll(*text);
    if (db.pose_version(id))
      return id;
  } catch (const std::exception &) {
  }
  reusex::warn("pose versions: the current version '{}' does not exist", *text);
  return std::nullopt;
}

int64_t record_pose_version(ProjectDB &db, const PoseVersionOptions &options) {
  const PoseState state = capture_pose_state(db);
  ProjectDB::PoseVersionRecord rec;
  rec.parent_id = current_pose_version(db).value_or(0);
  rec.kind = options.kind;
  rec.label = options.label;
  rec.author = options.author;
  rec.metrics_json = options.metrics_json;
  rec.frame_count = static_cast<int>(state.frames.size());
  rec.edge_count = static_cast<int>(state.edges.size());
  // A delta against the parent, or a keyframe every
  // kPoseVersionKeyframeInterval links so a restore walks a bounded chain.
  ProjectDB::PoseVersionSnapshot snap;
  bool delta = false;
  if (rec.parent_id > 0) {
    try {
      const auto [parent, depth] = materialise(db, rec.parent_id);
      if (depth + 1 < kPoseVersionKeyframeInterval) {
        snap = pack_against(state, &parent, rec.parent_id);
        delta = true;
      }
    } catch (const std::exception &e) {
      reusex::warn("pose versions: parent v{} unreadable ({}); recording a "
                   "full snapshot",
                   rec.parent_id, e.what());
    }
  }
  if (!delta)
    snap = pack_pose_state(state);
  const int64_t id = db.add_pose_version(rec, snap);
  set_current(db, id);
  reusex::debug("pose versions: recorded v{} ({}, parent v{})", id, rec.kind,
                rec.parent_id);
  return id;
}

int64_t ensure_pose_version(ProjectDB &db, std::string_view author) {
  const auto current = current_pose_version(db);
  if (!current)
    return record_pose_version(
        db, {.kind = db.pose_versions().empty() ? "baseline" : "external",
             .label = "poses before the first recorded step",
             .author = std::string(author)});
  const auto stored = pack_pose_state(state_of(db, *current));
  const auto live = pack_pose_state(capture_pose_state(db));
  if (stored.frames == live.frames && stored.edges == live.edges &&
      stored.edits == live.edits)
    return *current;
  return record_pose_version(db, {.kind = "external",
                                  .label = "poses changed outside the editor",
                                  .author = std::string(author)});
}

PoseVersionChange restore_pose_version(ProjectDB &db, int64_t id,
                                       std::string_view author) {
  if (!db.pose_version(id))
    throw std::invalid_argument(fmt::format("no pose version {}", id));
  ProjectDB::Transaction tx(db);
  const int64_t from = ensure_pose_version(db, author);
  const PoseState target = state_of(db, id);
  PoseVersionChange change = measure_change(capture_pose_state(db), target);
  change.from_version = from;
  change.to_version = id;
  restore_pose_state(db, target);
  set_current(db, id);
  tx.commit();
  reusex::info("pose versions: v{} -> v{}: {} frames moved (max {:.3f} m), {} "
               "edge rows changed",
               from, id, change.frames_changed, change.max_shift_m,
               change.edges_changed);
  return change;
}

PoseVersionChange undo_pose_version(ProjectDB &db, std::string_view author) {
  int64_t current = 0;
  {
    ProjectDB::Transaction tx(db);
    current = ensure_pose_version(db, author);
    tx.commit();
  }
  const auto rec = db.pose_version(current);
  if (!rec || rec->parent_id == 0)
    throw std::runtime_error(
        fmt::format("nothing to undo: pose version {} has no parent", current));
  return restore_pose_version(db, rec->parent_id, author);
}

PoseVersionChange redo_pose_version(ProjectDB &db, std::string_view author) {
  int64_t current = 0;
  {
    ProjectDB::Transaction tx(db);
    current = ensure_pose_version(db, author);
    tx.commit();
  }
  int64_t child = 0;
  for (const auto &v : db.pose_versions())
    if (v.parent_id == current)
      child = std::max(child, v.id);
  if (child == 0)
    throw std::runtime_error(fmt::format(
        "nothing to redo: pose version {} has no later version", current));
  return restore_pose_version(db, child, author);
}

std::vector<FrameMove> diff_pose_versions(const ProjectDB &db, int64_t a,
                                          int64_t b, double min_shift_m,
                                          double min_rotation_deg) {
  const PoseState sa = state_of(db, a);
  const PoseState sb = state_of(db, b);
  std::map<int, const FramePoseState *> in_a;
  for (const auto &f : sa.frames)
    in_a[f.node_id] = &f;
  std::vector<FrameMove> out;
  for (const auto &f : sb.frames) {
    auto it = in_a.find(f.node_id);
    if (it == in_a.end())
      continue;
    if (it->second->transform == f.transform)
      continue;
    FrameMove m;
    m.node_id = f.node_id;
    m.shift_m = translation_change(it->second->transform, f.transform);
    m.rotation_deg = rotation_change_deg(it->second->transform, f.transform);
    if (m.shift_m > min_shift_m || m.rotation_deg > min_rotation_deg)
      out.push_back(m);
  }
  std::sort(out.begin(), out.end(), [](const auto &x, const auto &y) {
    return x.shift_m != y.shift_m ? x.shift_m > y.shift_m
                                  : x.node_id < y.node_id;
  });
  return out;
}

// ── JSON ───────────────────────────────────────────────────────────────────

nlohmann::json to_json(const ProjectDB::PoseVersionRecord &r, bool current) {
  nlohmann::json metrics = nlohmann::json::object();
  try {
    metrics =
        nlohmann::json::parse(r.metrics_json.empty() ? "{}" : r.metrics_json);
  } catch (const nlohmann::json::exception &) {
    metrics = {{"raw", r.metrics_json}};
  }
  return {{"id", r.id},
          {"parent_id", r.parent_id > 0 ? nlohmann::json(r.parent_id)
                                        : nlohmann::json(nullptr)},
          {"kind", r.kind},
          {"label", r.label},
          {"author", r.author},
          {"created_at", r.created_at},
          {"metrics", metrics},
          {"frame_count", r.frame_count},
          {"edge_count", r.edge_count},
          {"current", current}};
}

nlohmann::json to_json(const PoseVersionChange &c) {
  return {{"from_version", c.from_version},
          {"to_version", c.to_version},
          {"frames_changed", c.frames_changed},
          {"max_shift_m", c.max_shift_m},
          {"edges_changed", c.edges_changed}};
}

nlohmann::json to_json(const FrameMove &m) {
  return {{"node_id", m.node_id},
          {"shift_m", m.shift_m},
          {"rotation_deg", m.rotation_deg}};
}

} // namespace reusex::core
