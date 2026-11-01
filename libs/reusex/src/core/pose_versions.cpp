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

constexpr std::uint32_t kFramesMagic = 0x31465852; // "RXF1"
constexpr std::uint32_t kEdgesMagic = 0x31455852;  // "RXE1"

// ── Packing ────────────────────────────────────────────────────────────────

struct Writer {
  Bytes out;
  template <typename T> void put(const T &v) {
    const auto *p = reinterpret_cast<const std::uint8_t *>(&v);
    out.insert(out.end(), p, p + sizeof(T));
  }
  template <size_t N> void put(const std::array<double, N> &a) {
    const auto *p = reinterpret_cast<const std::uint8_t *>(a.data());
    out.insert(out.end(), p, p + N * sizeof(double));
  }
  void put_str(const std::string &s) {
    put(static_cast<std::uint32_t>(s.size()));
    out.insert(out.end(), s.begin(), s.end());
  }
};

struct Reader {
  const Bytes &in;
  size_t pos = 0;
  void need(size_t n) const {
    if (pos + n > in.size())
      throw std::runtime_error("pose version: truncated snapshot blob");
  }
  template <typename T> T get() {
    need(sizeof(T));
    T v;
    std::memcpy(&v, in.data() + pos, sizeof(T));
    pos += sizeof(T);
    return v;
  }
  template <size_t N> std::array<double, N> get_array() {
    need(N * sizeof(double));
    std::array<double, N> a{};
    std::memcpy(a.data(), in.data() + pos, N * sizeof(double));
    pos += N * sizeof(double);
    return a;
  }
  std::string get_str() {
    const auto n = get<std::uint32_t>();
    need(n);
    std::string s(reinterpret_cast<const char *>(in.data() + pos), n);
    pos += n;
    return s;
  }
};

void put_optional(Writer &w, const std::optional<double> &v) {
  w.put(static_cast<std::uint8_t>(v ? 1 : 0));
  w.put(v.value_or(0.0));
}
std::optional<double> get_optional(Reader &r) {
  const bool has = r.get<std::uint8_t>() != 0;
  const double v = r.get<double>();
  return has ? std::optional<double>(v) : std::nullopt;
}

void pack_edge(Writer &w, const ProjectDB::PoseGraphEdge &e) {
  w.put(static_cast<std::int64_t>(e.id));
  w.put(static_cast<std::int32_t>(e.from_node_id));
  w.put(static_cast<std::int32_t>(e.to_node_id));
  w.put_str(e.edge_type);
  w.put(e.residual);
  w.put(e.weight);
  w.put(static_cast<std::uint8_t>((e.transform ? 1 : 0) |
                                  (e.information ? 2 : 0)));
  if (e.transform)
    w.put(*e.transform);
  if (e.information)
    w.put(*e.information);
  w.put_str(e.source);
  w.put_str(e.status);
  w.put_str(e.created_at);
  put_optional(w, e.seed_disagreement_m);
  put_optional(w, e.seed_disagreement_deg);
  put_optional(w, e.deviation_m);
  put_optional(w, e.deviation_deg);
}

ProjectDB::PoseGraphEdge unpack_edge(Reader &r) {
  ProjectDB::PoseGraphEdge e;
  e.id = r.get<std::int64_t>();
  e.from_node_id = r.get<std::int32_t>();
  e.to_node_id = r.get<std::int32_t>();
  e.edge_type = r.get_str();
  e.residual = r.get<double>();
  e.weight = r.get<double>();
  const auto flags = r.get<std::uint8_t>();
  if (flags & 1)
    e.transform = r.get_array<16>();
  if (flags & 2)
    e.information = r.get_array<36>();
  e.source = r.get_str();
  e.status = r.get_str();
  e.created_at = r.get_str();
  e.seed_disagreement_m = get_optional(r);
  e.seed_disagreement_deg = get_optional(r);
  e.deviation_m = get_optional(r);
  e.deviation_deg = get_optional(r);
  return e;
}

Bytes pack_one_edge(const ProjectDB::PoseGraphEdge &e) {
  Writer w;
  pack_edge(w, e);
  return std::move(w.out);
}

double translation_change(const std::array<double, 16> &a,
                          const std::array<double, 16> &b) {
  return std::sqrt((a[3] - b[3]) * (a[3] - b[3]) +
                   (a[7] - b[7]) * (a[7] - b[7]) +
                   (a[11] - b[11]) * (a[11] - b[11]));
}

double rotation_change_deg(const std::array<double, 16> &a,
                           const std::array<double, 16> &b) {
  const Eigen::Matrix3d Ra = pg::matrix_from_row_major(a).topLeftCorner<3, 3>();
  const Eigen::Matrix3d Rb = pg::matrix_from_row_major(b).topLeftCorner<3, 3>();
  const double c =
      std::clamp(((Ra.transpose() * Rb).trace() - 1.0) / 2.0, -1.0, 1.0);
  return std::acos(c) * 180.0 / M_PI;
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
  const auto snap = db.pose_version_snapshot(id);
  if (!snap)
    throw std::invalid_argument(fmt::format("no pose version {}", id));
  return unpack_pose_state(*snap);
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
  std::map<int64_t, Bytes> a, b;
  for (const auto &e : live.edges)
    a[e.id] = pack_one_edge(e);
  for (const auto &e : target.edges)
    b[e.id] = pack_one_edge(e);
  for (const auto &[id, bytes] : b) {
    auto it = a.find(id);
    if (it == a.end() || it->second != bytes)
      ++c.edges_changed;
  }
  for (const auto &[id, bytes] : a)
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
    same = pack_one_edge(edges[i]) == pack_one_edge(live[i]);
  if (!same)
    db.replace_pose_graph_edges(edges);

  if (stored_edits(db) != state.edits)
    db.set_setting(kSettingWorkingPoseEdits, state.edits.value_or(""));
}

ProjectDB::PoseVersionSnapshot pack_pose_state(const PoseState &state) {
  ProjectDB::PoseVersionSnapshot snap;
  Writer f;
  f.put(kFramesMagic);
  f.put(static_cast<std::uint32_t>(state.frames.size()));
  for (const auto &fr : state.frames) {
    f.put(static_cast<std::int32_t>(fr.node_id));
    f.put(fr.transform);
    f.put(static_cast<std::uint8_t>(fr.initial ? 1 : 0));
    if (fr.initial)
      f.put(*fr.initial);
  }
  snap.frames = std::move(f.out);
  Writer e;
  e.put(kEdgesMagic);
  e.put(static_cast<std::uint32_t>(state.edges.size()));
  for (const auto &edge : state.edges)
    pack_edge(e, edge);
  snap.edges = std::move(e.out);
  snap.edits = state.edits;
  return snap;
}

PoseState unpack_pose_state(const ProjectDB::PoseVersionSnapshot &snap) {
  PoseState s;
  Reader f{snap.frames};
  if (f.get<std::uint32_t>() != kFramesMagic)
    throw std::runtime_error("pose version: unknown frames blob format");
  const auto nf = f.get<std::uint32_t>();
  s.frames.reserve(nf);
  for (std::uint32_t i = 0; i < nf; ++i) {
    FramePoseState fr;
    fr.node_id = f.get<std::int32_t>();
    fr.transform = f.get_array<16>();
    if (f.get<std::uint8_t>() != 0)
      fr.initial = f.get_array<16>();
    s.frames.push_back(fr);
  }
  Reader e{snap.edges};
  if (e.get<std::uint32_t>() != kEdgesMagic)
    throw std::runtime_error("pose version: unknown edges blob format");
  const auto ne = e.get<std::uint32_t>();
  s.edges.reserve(ne);
  for (std::uint32_t i = 0; i < ne; ++i)
    s.edges.push_back(unpack_edge(e));
  s.edits = snap.edits;
  return s;
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
  const int64_t id = db.add_pose_version(rec, pack_pose_state(state));
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
  const auto stored = db.pose_version_snapshot(*current);
  const auto live = pack_pose_state(capture_pose_state(db));
  if (stored && stored->frames == live.frames && stored->edges == live.edges &&
      stored->edits == live.edits)
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
