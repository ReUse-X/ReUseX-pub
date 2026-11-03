// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/pose_repair.hpp"

#include "core/logging.hpp"
#include "core/pose_blocks.hpp"
#include "core/pose_graph.hpp"
#include "core/pose_versions.hpp"
#include "core/working_pose_edits.hpp"

#include <fmt/format.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>

namespace reusex::core {
namespace {

namespace pg = pose_graph;
using json = nlohmann::json;
using Key = std::pair<int, int>;

Key key(int a, int b) { return {std::min(a, b), std::max(a, b)}; }

/// Posed frames in node order with their scan.
std::vector<std::pair<int, int>> posed_frames(const ProjectDB &db) {
  std::vector<std::pair<int, int>> out;
  for (const auto &p : db.posed_frames())
    out.emplace_back(p.node_id, p.scan_id);
  return out;
}

/// The joints (frame before, first frame) of detached blocks within a
/// session; a session boundary is never bridged by rebuild_odometry anyway.
std::set<Key> detached_joints(const ProjectDB &db,
                              const std::vector<std::pair<int, int>> &frames) {
  std::map<int, int> scan_of;
  std::map<int, int> prev_of;
  int prev = 0;
  for (const auto &[id, scan] : frames) {
    scan_of[id] = scan;
    prev_of[id] = prev;
    prev = id;
  }
  std::set<Key> out;
  for (const auto &b : pose_blocks(db)) {
    if (!b.detached)
      continue;
    auto it = std::lower_bound(
        frames.begin(), frames.end(),
        std::make_pair(b.first_node, std::numeric_limits<int>::min()));
    if (it == frames.end())
      continue;
    const int first = it->first;
    const int before = prev_of[first];
    if (before != 0 && scan_of[before] == scan_of[first])
      out.insert(key(before, first));
  }
  return out;
}

/// Each posed frame's seed: its initial pose, else its transform.
std::map<int, Eigen::Matrix4d> seeds_of(const ProjectDB &db) {
  std::map<int, Eigen::Matrix4d> out;
  for (const auto &p : db.posed_frames())
    out.emplace(p.node_id,
                pg::matrix_from_row_major(p.initial.value_or(p.transform)));
  return out;
}

/// rebuild_odometry() without a transaction or a version.
RebuildOdometryResult rebuild_impl(ProjectDB &db,
                                   const RebuildOdometryOptions &o) {
  RebuildOdometryResult r;
  const auto frames = posed_frames(db);
  if (o.from_current)
    for (const auto &p : db.posed_frames()) {
      if (o.scan_id && p.scan_id != *o.scan_id)
        continue;
      if (!pg::is_rigid_transform(p.transform) || p.initial == p.transform)
        continue;
      db.set_initial_pose(p.node_id, p.transform);
      ++r.snapshots_written;
    }
  const auto seed = seeds_of(db);

  std::map<Key, ProjectDB::PoseGraphEdge> stored;
  for (auto &e : db.list_pose_graph_edges())
    if (e.edge_type == pg::kTypeOdometry)
      stored.emplace(key(e.from_node_id, e.to_node_id), std::move(e));
  const auto cuts = detached_joints(db, frames);

  for (size_t k = 0; k + 1 < frames.size(); ++k) {
    const auto [a, scan] = frames[k];
    const auto [b, scan_b] = frames[k + 1];
    if (scan != scan_b || (o.scan_id && scan != *o.scan_id))
      continue;
    ++r.pairs;
    const auto it = stored.find(key(a, b));
    if (cuts.count(key(a, b))) {
      ++r.cut_detached;
      if (it == stored.end()) {
        ProjectDB::PoseGraphEdge t; // a tombstone, so graph init skips it
        t.from_node_id = a;
        t.to_node_id = b;
        t.edge_type = std::string(pg::kTypeOdometry);
        t.source = std::string(pg::kSourceOdometryInit);
        t.status = std::string(pg::kStatusRejected);
        db.upsert_pose_graph_edge(t);
      } else if (it->second.status != pg::kStatusRejected) {
        db.set_pose_graph_edge_status(a, b, pg::kTypeOdometry,
                                      pg::kStatusRejected);
      }
      continue;
    }
    const bool imported =
        it != stored.end() && it->second.source == pg::kSourceRtabmap;
    if (it != stored.end()) {
      if (it->second.status == pg::kStatusConfirmed) {
        ++r.kept_confirmed;
        continue;
      }
      if (it->second.status == pg::kStatusRejected && !imported) {
        ++r.kept_rejected;
        continue;
      }
    }
    const Eigen::Matrix4d T = seed.at(a).inverse() * seed.at(b);
    bool changed = true;
    if (it != stored.end() && it->second.transform) {
      const auto err =
          pg::transform_error(pg::matrix_from_row_major(*it->second.transform),
                              Eigen::Matrix4d::Identity(), T);
      changed = err.translation_m > 1e-6 || err.rotation_deg > 1e-4 ||
                it->second.source != pg::kSourceOdometryInit;
      r.max_change_m = std::max(r.max_change_m, err.translation_m);
      r.max_change_deg = std::max(r.max_change_deg, err.rotation_deg);
    }
    if (!changed && it->second.status == pg::kStatusAuto)
      continue; // already exactly this edge
    ProjectDB::PoseGraphEdge e;
    e.from_node_id = a;
    e.to_node_id = b;
    e.edge_type = std::string(pg::kTypeOdometry);
    e.transform = pg::row_major_from_matrix(T);
    e.source = std::string(pg::kSourceOdometryInit);
    e.status = std::string(pg::kStatusAuto);
    e.seed_disagreement_m = 0.0;
    e.seed_disagreement_deg = 0.0;
    db.upsert_pose_graph_edge(e, /*overwrite_rejected=*/true);
    ++r.written;
    r.changed += changed ? 1 : 0;
    r.replaced_imported += imported ? 1 : 0;
  }
  return r;
}

json rebuild_metrics(const RebuildOdometryResult &r) {
  json j = to_json(r);
  j.erase("version_id");
  return j;
}

Eigen::Vector3d position(const std::array<double, 16> &p) {
  return {p[3], p[7], p[11]};
}

std::array<double, 16> compose(const std::array<double, 16> &D,
                               const std::array<double, 16> &P) {
  return pg::row_major_from_matrix(pg::matrix_from_row_major(D) *
                                   pg::matrix_from_row_major(P));
}

std::uint8_t dofs_from_json(const json &j) {
  if (!j.is_array())
    throw std::invalid_argument("hold.dofs: must be a list of x y z roll "
                                "pitch yaw");
  std::uint8_t out = 0;
  for (const auto &d : j) {
    const auto bit = d.is_string() ? dof_from_string(d.get<std::string>()) : 0;
    if (bit == 0)
      throw std::invalid_argument(
          fmt::format("hold.dofs: unknown DoF {}", d.dump()));
    out |= bit;
  }
  return out;
}

double number(const json &j, const char *field) {
  if (!j.is_number() || !std::isfinite(j.get<double>()))
    throw std::invalid_argument(
        fmt::format("{}: must be a finite number", field));
  return j.get<double>();
}

template <size_t N>
std::array<double, N> numbers(const json &j, const char *field) {
  if (!j.is_array() || j.size() != N)
    throw std::invalid_argument(
        fmt::format("{}: must be a list of {} numbers", field, N));
  std::array<double, N> out{};
  for (size_t i = 0; i < N; ++i)
    out[i] = number(j[i], field);
  return out;
}

} // namespace

// ── Odometry ───────────────────────────────────────────────────────────────

RebuildOdometryResult rebuild_odometry(ProjectDB &db,
                                       const RebuildOdometryOptions &o) {
  ProjectDB::Transaction tx(db);
  if (o.record_version)
    ensure_pose_version(db, o.author);
  auto r = rebuild_impl(db, o);
  if (o.record_version)
    r.version_id = record_pose_version(
        db,
        {.kind = "odometry",
         .label = o.from_current ? "odometry rebuilt from the current poses"
                                 : "odometry rebuilt from the initial poses",
         .author = o.author,
         .metrics_json = rebuild_metrics(r).dump()});
  tx.commit();
  reusex::info(
      "Odometry rebuilt: {} pairs, {} edges written ({} changed, max {:.3f} m "
      "/ {:.2f} deg; {} imported replaced), {} confirmed and {} rejected "
      "kept, {} cut at detached joints, {} initial poses re-snapshotted",
      r.pairs, r.written, r.changed, r.max_change_m, r.max_change_deg,
      r.replaced_imported, r.kept_confirmed, r.kept_rejected, r.cut_detached,
      r.snapshots_written);
  if (r.pairs == 0)
    reusex::warn("Odometry rebuild: no pair of consecutive posed frames in "
                 "one session{}; nothing written",
                 o.scan_id ? fmt::format(" (scan {})", *o.scan_id) : "");
  return r;
}

// ── Block transform ────────────────────────────────────────────────────────

double blend_weight(int k, int n) {
  if (n <= 0 || k > n)
    return 1.0;
  const double x = static_cast<double>(k) / static_cast<double>(n + 1);
  return x * x * (3.0 - 2.0 * x);
}

std::array<double, 16> partial_delta(const std::array<double, 16> &D,
                                     const std::array<double, 3> &pivot,
                                     double w) {
  const Eigen::Matrix4d M = pg::matrix_from_row_major(D);
  const Eigen::Vector3d c(pivot[0], pivot[1], pivot[2]);
  const Eigen::Quaterniond q(Eigen::Matrix3d(M.topLeftCorner<3, 3>()));
  const Eigen::Matrix3d Rw =
      Eigen::Quaterniond::Identity().slerp(w, q).toRotationMatrix();
  const Eigen::Vector3d tc = M.topLeftCorner<3, 3>() * c +
                             M.topRightCorner<3, 1>() - c; // pivot moves by
  Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
  out.topLeftCorner<3, 3>() = Rw;
  out.topRightCorner<3, 1>() = c + w * tc - Rw * c;
  return pg::row_major_from_matrix(out);
}

BlockTransformResult apply_block_transform(ProjectDB &db,
                                           const BlockTransformRequest &req) {
  if (req.blend_frames < 0)
    throw std::invalid_argument("blend_frames: must be >= 0");
  const auto frames = posed_frames(db);

  // The selection.
  std::set<int> selected;
  const auto blocks = pose_blocks(db);
  for (const int id : req.block_ids) {
    auto it = std::find_if(blocks.begin(), blocks.end(),
                           [&](const auto &b) { return b.id == id; });
    if (it == blocks.end())
      throw std::invalid_argument(fmt::format("blocks: no block {}", id));
    for (const int n : block_nodes(db, *it))
      selected.insert(n);
  }
  for (const auto &[a, b] : req.node_ranges) {
    if (a > b)
      throw std::invalid_argument(
          fmt::format("nodes: range {}..{} is reversed", a, b));
    int count = 0;
    for (const auto &[id, scan] : frames)
      if (id >= a && id <= b) {
        selected.insert(id);
        ++count;
      }
    if (count == 0)
      throw std::invalid_argument(
          fmt::format("nodes: range {}..{} holds no posed frame", a, b));
  }
  if (selected.empty())
    throw std::invalid_argument(
        "block transform: select at least one block or node range");

  std::map<int, std::array<double, 16>> pose;
  std::map<int, std::optional<std::array<double, 16>>> initial;
  for (const auto &p : db.posed_frames()) {
    pose[p.node_id] = p.transform;
    initial[p.node_id] = p.initial;
  }

  // Delta and pivot.
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (const int id : selected)
    centroid += position(pose[id]);
  centroid /= static_cast<double>(selected.size());
  Eigen::Vector3d pivot = centroid;
  if (req.delta.pivot)
    pivot = Eigen::Vector3d((*req.delta.pivot)[0], (*req.delta.pivot)[1],
                            (*req.delta.pivot)[2]);
  if (req.delta.matrix && !pg::is_rigid_transform(*req.delta.matrix))
    throw std::invalid_argument("delta.matrix: not a rigid transform");
  BlockTransformResult r;
  r.preview = req.preview;
  r.delta = pg::row_major_from_matrix(req.delta.to_matrix(pivot));
  r.pivot = {pivot.x(), pivot.y(), pivot.z()};

  // Runs of selected frames, their joints and blend weights.
  const auto cuts = detached_joints(db, frames);
  std::map<int, double> weight;
  size_t i = 0;
  while (i < frames.size()) {
    if (!selected.count(frames[i].first)) {
      ++i;
      continue;
    }
    size_t j = i;
    while (j + 1 < frames.size() && selected.count(frames[j + 1].first) &&
           frames[j + 1].second == frames[i].second)
      ++j;
    const int len = static_cast<int>(j - i + 1);
    auto joint = [&](size_t inside, std::optional<size_t> outside) {
      JointReport jr;
      jr.inside_node = frames[inside].first;
      if (!outside || frames[*outside].second != frames[inside].second) {
        jr.session_boundary = true;
        if (outside)
          jr.outside_node = frames[*outside].first;
        return jr;
      }
      jr.outside_node = frames[*outside].first;
      jr.detached = cuts.count(key(jr.inside_node, jr.outside_node)) > 0;
      if (!jr.detached)
        jr.blend_frames = std::min(req.blend_frames, len / 2);
      return jr;
    };
    JointReport start =
        joint(i, i > 0 ? std::optional<size_t>(i - 1) : std::nullopt);
    JointReport end = joint(
        j, j + 1 < frames.size() ? std::optional<size_t>(j + 1) : std::nullopt);
    if (start.blend_frames + end.blend_frames > len) {
      start.blend_frames = len / 2;
      end.blend_frames = len - start.blend_frames;
    }
    for (int k = 0; k < len; ++k)
      weight[frames[i + static_cast<size_t>(k)].first] =
          std::min(blend_weight(k + 1, start.blend_frames),
                   blend_weight(len - k, end.blend_frames));
    r.joints.push_back(start);
    r.joints.push_back(end);
    i = j + 1;
  }

  // Moved poses.
  std::map<int, std::array<double, 16>> after;
  for (const auto &[id, w] : weight) {
    MovedFrame m;
    m.node_id = id;
    m.weight = w;
    m.before = pose[id];
    m.after = compose(w >= 1.0 ? r.delta : partial_delta(r.delta, r.pivot, w),
                      m.before);
    m.shift_m = (position(m.after) - position(m.before)).norm();
    after[id] = m.after;
    r.max_shift_m = std::max(r.max_shift_m, m.shift_m);
    (w >= 1.0 ? r.frames_moved : r.frames_blended)++;
    r.frames.push_back(m);
  }
  for (auto &jr : r.joints) {
    if (jr.outside_node == 0 || !pose.count(jr.outside_node))
      continue;
    const auto out = position(pose[jr.outside_node]);
    jr.gap_before_m = (position(pose[jr.inside_node]) - out).norm();
    jr.gap_after_m = (position(after[jr.inside_node]) - out).norm();
  }

  // Loops into the selection, against the moved seed (the current poses).
  for (const auto &e : db.list_pose_graph_edges()) {
    if (e.edge_type != pg::kTypeLoopClosure ||
        e.status == pg::kStatusRejected || !e.transform)
      continue;
    const bool a_in = after.count(e.from_node_id) > 0;
    const bool b_in = after.count(e.to_node_id) > 0;
    if (a_in == b_in || !pose.count(e.from_node_id) ||
        !pose.count(e.to_node_id))
      continue;
    ++r.loops_touching;
    const auto &Pa = a_in ? after[e.from_node_id] : pose[e.from_node_id];
    const auto &Pb = b_in ? after[e.to_node_id] : pose[e.to_node_id];
    const auto err = pg::transform_error(
        pg::matrix_from_row_major(*e.transform), pg::matrix_from_row_major(Pa),
        pg::matrix_from_row_major(Pb));
    if (err.translation_m > 0.3 || err.rotation_deg > 4.0)
      ++r.loops_disagreeing;
  }

  if (req.preview)
    return r;

  // Apply.
  ProjectDB::Transaction tx(db);
  ensure_pose_version(db, req.author);
  if (req.mode == BlockTransformMode::reseed) {
    // seed := current everywhere, then the move in both.
    for (const auto &[id, p] : pose) {
      const auto target = after.count(id) ? after[id] : p;
      if (initial[id] != target)
        db.set_initial_pose(id, target);
    }
  }
  for (const auto &[id, p] : after)
    db.update_sensor_frame_pose(id, p);
  if (req.mode == BlockTransformMode::reseed)
    r.odometry =
        rebuild_impl(db, {.from_current = false, .record_version = false});
  if (req.hold) {
    std::vector<std::string> ids;
    // One hold per contiguous run of moved frames.
    int first = -1, last = -1, scan_of_first = 0;
    auto flush = [&]() {
      if (first < 0)
        return;
      const std::string id = fmt::format("hold-{}-{}", first, last);
      put_working_pose_edit(
          db, hold_edit(id, first, last, req.hold->dofs,
                        req.hold->sigma_trans_m, req.hold->sigma_rot_deg,
                        "block transform: hold the re-seeded block"));
      ids.push_back(id);
      first = -1;
    };
    for (const auto &[id, scan] : frames) {
      if (!after.count(id)) {
        flush();
        continue;
      }
      if (first >= 0 && scan != scan_of_first)
        flush();
      if (first < 0) {
        first = id;
        scan_of_first = scan;
      }
      last = id;
    }
    flush();
    std::string joined;
    for (const auto &id : ids)
      joined += (joined.empty() ? "" : ",") + id;
    r.hold_edit_id = joined;
  }
  json metrics = to_json(r, /*with_frames=*/false);
  metrics.erase("version_id");
  metrics.erase("preview");
  r.version_id = record_pose_version(
      db, {.kind = "apply",
           .label = !req.label.empty()
                        ? req.label
                        : fmt::format("{} {} frames",
                                      req.mode == BlockTransformMode::reseed
                                          ? "re-seed"
                                          : "move",
                                      after.size()),
           .author = req.author,
           .metrics_json = metrics.dump()});
  tx.commit();
  reusex::info("Block transform applied ({}): {} frames moved, {} blended, "
               "max shift {:.3f} m; {} loops into the block ({} disagree); "
               "pose version {}",
               req.mode == BlockTransformMode::reseed ? "re-seed" : "local",
               r.frames_moved, r.frames_blended, r.max_shift_m,
               r.loops_touching, r.loops_disagreeing, *r.version_id);
  return r;
}

// ── JSON ───────────────────────────────────────────────────────────────────

BlockTransformRequest block_transform_request_from_json(const json &j) {
  if (!j.is_object())
    throw std::invalid_argument("request: must be an object");
  static const std::set<std::string> known{"blocks",  "nodes",        "delta",
                                           "mode",    "blend_frames", "hold",
                                           "preview", "label"};
  for (const auto &[k, v] : j.items())
    if (!known.count(k))
      throw std::invalid_argument(fmt::format("{}: unknown field", k));
  BlockTransformRequest r;
  if (j.contains("blocks")) {
    if (!j["blocks"].is_array())
      throw std::invalid_argument("blocks: must be a list of block ids");
    for (const auto &b : j["blocks"]) {
      if (!b.is_number_integer())
        throw std::invalid_argument("blocks: must be a list of block ids");
      r.block_ids.push_back(b.get<int>());
    }
  }
  if (j.contains("nodes")) {
    if (!j["nodes"].is_array())
      throw std::invalid_argument("nodes: must be a list of [first, last] "
                                  "ranges or node ids");
    for (const auto &n : j["nodes"]) {
      if (n.is_number_integer())
        r.node_ranges.emplace_back(n.get<int>(), n.get<int>());
      else if (n.is_array() && n.size() == 2 && n[0].is_number_integer() &&
               n[1].is_number_integer())
        r.node_ranges.emplace_back(n[0].get<int>(), n[1].get<int>());
      else
        throw std::invalid_argument("nodes: must be a list of [first, last] "
                                    "ranges or node ids");
    }
  }
  if (!j.contains("delta") || !j["delta"].is_object())
    throw std::invalid_argument("delta: required object {xyz, yaw_deg, "
                                "pitch_deg, roll_deg, pivot?, matrix?}");
  const json &d = j["delta"];
  for (const auto &[k, v] : d.items()) {
    if (k == "xyz")
      r.delta.xyz = numbers<3>(v, "delta.xyz");
    else if (k == "yaw_deg")
      r.delta.yaw_deg = number(v, "delta.yaw_deg");
    else if (k == "pitch_deg")
      r.delta.pitch_deg = number(v, "delta.pitch_deg");
    else if (k == "roll_deg")
      r.delta.roll_deg = number(v, "delta.roll_deg");
    else if (k == "pivot")
      r.delta.pivot = numbers<3>(v, "delta.pivot");
    else if (k == "matrix")
      r.delta.matrix = numbers<16>(v, "delta.matrix");
    else
      throw std::invalid_argument(fmt::format("delta.{}: unknown field", k));
  }
  if (r.delta.matrix && !pg::is_rigid_transform(*r.delta.matrix))
    throw std::invalid_argument("delta.matrix: not a rigid transform");
  if (j.contains("mode")) {
    const auto m = j["mode"].is_string() ? j["mode"].get<std::string>() : "";
    if (m == "reseed")
      r.mode = BlockTransformMode::reseed;
    else if (m == "local")
      r.mode = BlockTransformMode::local;
    else
      throw std::invalid_argument("mode: must be \"reseed\" or \"local\"");
  }
  if (j.contains("blend_frames")) {
    if (!j["blend_frames"].is_number_integer() ||
        j["blend_frames"].get<int>() < 0)
      throw std::invalid_argument("blend_frames: must be an integer >= 0");
    r.blend_frames = j["blend_frames"].get<int>();
  }
  if (j.contains("hold") && !j["hold"].is_null()) {
    const json &h = j["hold"];
    if (h.is_boolean()) {
      if (h.get<bool>())
        r.hold = BlockTransformHold{};
    } else if (h.is_object()) {
      BlockTransformHold hold;
      for (const auto &[k, v] : h.items()) {
        if (k == "dofs")
          hold.dofs = dofs_from_json(v);
        else if (k == "sigma_trans_m")
          hold.sigma_trans_m = number(v, "hold.sigma_trans_m");
        else if (k == "sigma_rot_deg")
          hold.sigma_rot_deg = number(v, "hold.sigma_rot_deg");
        else
          throw std::invalid_argument(fmt::format("hold.{}: unknown field", k));
      }
      if (hold.sigma_trans_m <= 0 || hold.sigma_rot_deg <= 0)
        throw std::invalid_argument("hold: sigmas must be > 0");
      r.hold = hold;
    } else {
      throw std::invalid_argument("hold: must be true or an object");
    }
  }
  if (j.contains("preview")) {
    if (!j["preview"].is_boolean())
      throw std::invalid_argument("preview: must be a boolean");
    r.preview = j["preview"].get<bool>();
  }
  if (j.contains("label")) {
    if (!j["label"].is_string())
      throw std::invalid_argument("label: must be a string");
    r.label = j["label"].get<std::string>();
  }
  return r;
}

json to_json(const RebuildOdometryResult &r) {
  return {{"pairs", r.pairs},
          {"written", r.written},
          {"changed", r.changed},
          {"replaced_imported", r.replaced_imported},
          {"kept_confirmed", r.kept_confirmed},
          {"kept_rejected", r.kept_rejected},
          {"cut_detached", r.cut_detached},
          {"snapshots_written", r.snapshots_written},
          {"max_change_m", r.max_change_m},
          {"max_change_deg", r.max_change_deg},
          {"version_id", r.version_id ? json(*r.version_id) : json(nullptr)}};
}

json to_json(const BlockTransformResult &r, bool with_frames) {
  json joints = json::array();
  for (const auto &jr : r.joints)
    joints.push_back({{"inside_node", jr.inside_node},
                      {"outside_node", jr.outside_node},
                      {"session_boundary", jr.session_boundary},
                      {"detached", jr.detached},
                      {"blend_frames", jr.blend_frames},
                      {"gap_before_m", jr.gap_before_m},
                      {"gap_after_m", jr.gap_after_m}});
  json j{
      {"preview", r.preview},
      {"delta", r.delta},
      {"pivot", r.pivot},
      {"frames_moved", r.frames_moved},
      {"frames_blended", r.frames_blended},
      {"max_shift_m", r.max_shift_m},
      {"joints", joints},
      {"loops_touching", r.loops_touching},
      {"loops_disagreeing", r.loops_disagreeing},
      {"odometry", r.odometry ? to_json(*r.odometry) : json(nullptr)},
      {"hold_edit_id", r.hold_edit_id ? json(*r.hold_edit_id) : json(nullptr)},
      {"version_id", r.version_id ? json(*r.version_id) : json(nullptr)}};
  if (with_frames) {
    json frames = json::array();
    for (const auto &m : r.frames)
      frames.push_back({{"node_id", m.node_id},
                        {"weight", m.weight},
                        {"shift_m", m.shift_m},
                        {"after", m.after}});
    j["frames"] = std::move(frames);
  }
  return j;
}

} // namespace reusex::core
