// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/pose_blocks.hpp"

#include "core/logging.hpp"
#include "core/pose_graph.hpp"
#include "core/pose_versions.hpp"
#include "core/working_pose_edits.hpp"

#include <fmt/format.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <stdexcept>

namespace reusex::core {
namespace {

namespace pg = pose_graph;
using json = nlohmann::json;

/// Posed frames grouped by scan, each in node order, plus the scans in the
/// order they first appear (their session ordinal).
struct Sessions {
  std::map<int, std::vector<int>> frames;     // scan id -> node ids
  std::vector<int> order;                     // scan ids by ordinal
  std::vector<std::pair<int, int>> all;       // (node, scan) in node order
  std::map<int, std::array<double, 16>> pose; // node -> transform
  int ordinal(int scan) const {
    const auto it = std::find(order.begin(), order.end(), scan);
    return it == order.end() ? -1 : static_cast<int>(it - order.begin());
  }
};

Sessions sessions_of(const ProjectDB &db) {
  Sessions s;
  for (const auto &p : db.posed_frames()) {
    const int id = p.node_id, scan = p.scan_id;
    auto [it, fresh] = s.frames.try_emplace(scan);
    if (fresh)
      s.order.push_back(scan);
    it->second.push_back(id);
    s.all.emplace_back(id, scan);
    s.pose.emplace(id, p.transform);
  }
  return s;
}

std::string default_name(int ordinal, int k, int count) {
  return count <= 1 ? fmt::format("s{}", ordinal)
                    : fmt::format("s{}.{}", ordinal, k + 1);
}

/// Name the blocks with an empty name, per session in order.
void fill_names(std::vector<PoseBlock> &blocks, const Sessions &s) {
  std::map<int, int> count, seen;
  for (const auto &b : blocks)
    ++count[b.scan_id];
  for (auto &b : blocks) {
    const int k = seen[b.scan_id]++;
    if (b.name.empty())
      b.name = default_name(s.ordinal(b.scan_id), k, count[b.scan_id]);
  }
}

std::vector<PoseBlock> numbered(std::vector<PoseBlock> blocks) {
  std::stable_sort(
      blocks.begin(), blocks.end(),
      [](const auto &a, const auto &b) { return a.first_node < b.first_node; });
  int id = 0;
  for (auto &b : blocks)
    b.id = ++id;
  return blocks;
}

/// The posed frame before @p node in node order (any session), or 0.
int frame_before(const Sessions &s, int node) {
  int prev = 0;
  for (const auto &[id, scan] : s.all) {
    if (id >= node)
      break;
    prev = id;
  }
  return prev;
}

/// A block is detached when the working edits cut the odometry link at its
/// start (the detach-<a>-<b> reject_edge a detach or a session import
/// writes): one source of truth, so undoing a detach (a pose version, which
/// holds the edits) re-attaches the block too. The stored column is not
/// read.
void derive_detached(const ProjectDB &db, const Sessions &s,
                     std::vector<PoseBlock> &blocks) {
  const auto ids = working_pose_edit_ids(db);
  for (auto &b : blocks) {
    const int prev = frame_before(s, b.first_node);
    b.detached = prev != 0 && ids.count(detach_edit_id(prev, b.first_node));
  }
}

int scan_of(const Sessions &s, int node) {
  for (const auto &[id, scan] : s.all)
    if (id == node)
      return scan;
  return -1;
}

constexpr int rank(SplitReason r) { return static_cast<int>(r); }

double score_of(SplitReason r, double gap_s, double jump_m,
                const BlockSuggestOptions &o) {
  switch (r) {
  case SplitReason::session:
    return 1.0;
  case SplitReason::time_gap:
    return 0.9;
  case SplitReason::tracking_gap:
    return 0.5 +
           0.4 * std::clamp((gap_s - o.tracking_gap_s) /
                                std::max(1e-9, o.time_gap_s - o.tracking_gap_s),
                            0.0, 1.0);
  case SplitReason::storey_change:
    return 0.7;
  case SplitReason::odometry_jump:
    return std::min(1.0, 0.4 + 0.1 * jump_m);
  }
  return 0.0;
}

/// Camera-height bands: peaks of a 0.1 m histogram of camera z (smoothed
/// over 0.5 m) with at least min_frames frames, kept greedily by count when
/// they are `separation` apart.
std::vector<double> storey_levels(std::span<const TrajectorySample> traj,
                                  const BlockSuggestOptions &o) {
  if (traj.empty())
    return {};
  constexpr double kBin = 0.1;
  double lo = traj[0].position.z(), hi = lo;
  for (const auto &t : traj) {
    lo = std::min(lo, t.position.z());
    hi = std::max(hi, t.position.z());
  }
  const int n = static_cast<int>(std::floor((hi - lo) / kBin)) + 1;
  if (n > 100000) // a broken pose; no bands rather than a huge histogram
    return {};
  std::vector<double> hist(static_cast<size_t>(n), 0.0);
  for (const auto &t : traj)
    hist[static_cast<size_t>(std::min(
        n - 1, static_cast<int>((t.position.z() - lo) / kBin)))] += 1.0;
  std::vector<double> smooth(hist.size(), 0.0);
  for (int i = 0; i < n; ++i)
    for (int k = -2; k <= 2; ++k)
      if (i + k >= 0 && i + k < n)
        smooth[static_cast<size_t>(i)] += hist[static_cast<size_t>(i + k)];
  std::vector<std::pair<double, double>> peaks; // (count, z)
  for (int i = 0; i < n; ++i) {
    const double c = smooth[static_cast<size_t>(i)];
    const double l = i > 0 ? smooth[static_cast<size_t>(i - 1)] : -1.0;
    const double r = i + 1 < n ? smooth[static_cast<size_t>(i + 1)] : -1.0;
    if (c >= l && c > r && c >= o.storey_min_frames)
      peaks.emplace_back(c, lo + (i + 0.5) * kBin);
  }
  // The smoothed peak only finds a band; its level is the mean camera
  // height within half a band of it (smoothing drags the peak towards a
  // stair run next to it).
  for (auto &[c, z] : peaks) {
    double sum = 0.0;
    int n = 0;
    for (const auto &t : traj)
      if (std::abs(t.position.z() - z) <= 0.3) {
        sum += t.position.z();
        ++n;
      }
    if (n > 0)
      z = sum / n;
  }
  std::sort(peaks.begin(), peaks.end(), std::greater<>());
  std::vector<double> levels;
  for (const auto &[c, z] : peaks)
    if (std::all_of(levels.begin(), levels.end(), [&](double l) {
          return std::abs(l - z) >= o.storey_separation_m;
        }))
      levels.push_back(z);
  std::sort(levels.begin(), levels.end());
  return levels;
}

int nearest_level(const std::vector<double> &levels, double z) {
  int best = 0;
  for (int i = 1; i < static_cast<int>(levels.size()); ++i)
    if (std::abs(levels[static_cast<size_t>(i)] - z) <
        std::abs(levels[static_cast<size_t>(best)] - z))
      best = i;
  return best;
}

} // namespace

std::string_view to_string(SplitReason reason) {
  switch (reason) {
  case SplitReason::session:
    return "session";
  case SplitReason::time_gap:
    return "time_gap";
  case SplitReason::tracking_gap:
    return "tracking_gap";
  case SplitReason::storey_change:
    return "storey_change";
  case SplitReason::odometry_jump:
    return "odometry_jump";
  }
  return "unknown";
}

std::vector<TrajectorySample> trajectory_samples(const ProjectDB &db,
                                                 bool initial) {
  std::vector<TrajectorySample> out;
  for (const auto &p : db.posed_frames()) {
    const auto &pose = initial && p.initial ? *p.initial : p.transform;
    TrajectorySample t;
    t.node_id = p.node_id;
    t.scan_id = p.scan_id;
    t.timestamp = p.timestamp;
    t.position = Eigen::Vector3d(pose[3], pose[7], pose[11]);
    out.push_back(t);
  }
  return out;
}

BlockSuggestions suggest_block_splits(std::span<const TrajectorySample> traj,
                                      const BlockSuggestOptions &o) {
  BlockSuggestions out;
  std::map<int, BlockSplitSuggestion> splits;
  auto add = [&](const TrajectorySample &prev, const TrajectorySample &cur,
                 SplitReason reason, double gap, double jump,
                 std::string detail) {
    auto [it, fresh] = splits.try_emplace(cur.node_id);
    auto &s = it->second;
    if (fresh) {
      s.node_id = cur.node_id;
      s.prev_node_id = prev.node_id;
      s.reason = reason;
      s.gap_s = gap;
      s.jump_m = jump;
      s.detail = std::move(detail);
    } else if (rank(reason) < rank(s.reason)) {
      s.reason = reason;
      s.detail = std::move(detail);
    }
    s.reasons.push_back(reason);
    s.score = std::max(s.score, score_of(reason, gap, jump, o));
  };

  int session = 0;
  for (size_t i = 1; i < traj.size(); ++i) {
    const auto &a = traj[i - 1];
    const auto &b = traj[i];
    const double jump = (b.position - a.position).norm();
    const double gap = a.timestamp >= 0.0 && b.timestamp >= 0.0
                           ? b.timestamp - a.timestamp
                           : 0.0;
    if (b.scan_id != a.scan_id) {
      add(a, b, SplitReason::session, gap, jump,
          fmt::format("session s{} starts (scan {})", ++session, b.scan_id));
      continue;
    }
    if (gap > o.time_gap_s)
      add(a, b, SplitReason::time_gap, gap, jump,
          fmt::format("time gap {:.0f} s", gap));
    else if (gap > o.tracking_gap_s)
      add(a, b, SplitReason::tracking_gap, gap, jump,
          fmt::format("tracking gap {:.1f} s", gap));
    if (jump > o.odometry_jump_m) {
      // A glitch jumps away and comes back: the next frame is close to the
      // previous one again.
      if (i + 1 < traj.size() && traj[i + 1].scan_id == b.scan_id &&
          (traj[i + 1].position - b.position).norm() > o.odometry_jump_m &&
          (traj[i + 1].position - a.position).norm() <= o.odometry_jump_m) {
        out.glitches.push_back({b.node_id, jump});
        ++i; // the jump back is the same glitch
        continue;
      }
      add(a, b, SplitReason::odometry_jump, gap, jump,
          fmt::format("odometry jump {:.2f} m", jump));
    }
  }

  // Storey-band changes: per session, runs of frames in one band; a run
  // shorter than storey_min_frames joins the run before it (stairs and
  // noise), and a change between two long runs is a cut.
  out.storey_levels = storey_levels(traj, o);
  if (out.storey_levels.size() > 1) {
    struct Run {
      int band;
      size_t start;
      size_t len;
    };
    size_t i = 0;
    while (i < traj.size()) {
      size_t j = i;
      while (j < traj.size() && traj[j].scan_id == traj[i].scan_id)
        ++j;
      std::vector<Run> runs;
      for (size_t k = i; k < j; ++k) {
        const int band = nearest_level(out.storey_levels, traj[k].position.z());
        if (!runs.empty() && runs.back().band == band)
          ++runs.back().len;
        else
          runs.push_back({band, k, 1});
      }
      std::vector<Run> merged;
      for (const auto &r : runs) {
        if (!merged.empty() && (static_cast<int>(r.len) < o.storey_min_frames ||
                                merged.back().band == r.band))
          merged.back().len += r.len;
        else
          merged.push_back(r);
      }
      // A short first run belongs to the run after it.
      if (merged.size() > 1 &&
          static_cast<int>(merged[0].len) < o.storey_min_frames) {
        merged[1].start = merged[0].start;
        merged[1].len += merged[0].len;
        merged.erase(merged.begin());
      }
      for (size_t m = 1; m < merged.size(); ++m) {
        if (merged[m].band == merged[m - 1].band)
          continue;
        const auto &a = traj[merged[m].start - 1];
        const auto &b = traj[merged[m].start];
        const double gap = a.timestamp >= 0.0 && b.timestamp >= 0.0
                               ? b.timestamp - a.timestamp
                               : 0.0;
        add(a, b, SplitReason::storey_change, gap,
            (b.position - a.position).norm(),
            fmt::format(
                "camera moves from the {:.1f} m band to the {:.1f} m "
                "band",
                out.storey_levels[static_cast<size_t>(merged[m - 1].band)],
                out.storey_levels[static_cast<size_t>(merged[m].band)]));
      }
      i = j;
    }
  }

  for (auto &[node, s] : splits) {
    std::sort(s.reasons.begin(), s.reasons.end(),
              [](auto x, auto y) { return rank(x) < rank(y); });
    s.reasons.erase(std::unique(s.reasons.begin(), s.reasons.end()),
                    s.reasons.end());
    out.splits.push_back(std::move(s));
  }
  return out;
}

BlockSuggestions suggest_block_splits(const ProjectDB &db,
                                      const BlockSuggestOptions &options) {
  const auto traj = trajectory_samples(db);
  auto out =
      suggest_block_splits(std::span<const TrajectorySample>(traj), options);
  reusex::info("Block suggestions: {} frames, {} split points ({} sessions), "
               "{} single-frame glitches, {} storey bands",
               traj.size(), out.splits.size(),
               std::count_if(out.splits.begin(), out.splits.end(),
                             [](const auto &s) {
                               return s.reason == SplitReason::session;
                             }) +
                   (traj.empty() ? 0 : 1),
               out.glitches.size(), out.storey_levels.size());
  return out;
}

// ── Stored blocks ──────────────────────────────────────────────────────────

std::vector<PoseBlock> default_pose_blocks(const ProjectDB &db) {
  const Sessions s = sessions_of(db);
  std::vector<PoseBlock> out;
  for (const int scan : s.order) {
    const auto &ids = s.frames.at(scan);
    PoseBlock b;
    b.scan_id = scan;
    b.first_node = ids.front();
    b.last_node = ids.back();
    b.name = default_name(s.ordinal(scan), 0, 1);
    out.push_back(b);
  }
  out = numbered(std::move(out));
  derive_detached(db, s, out);
  return out;
}

std::vector<PoseBlock> pose_blocks(const ProjectDB &db) {
  auto stored = db.pose_blocks();
  if (stored.empty())
    return default_pose_blocks(db);
  std::set<int> covered;
  for (const auto &b : stored)
    covered.insert(b.scan_id);
  for (const auto &d : default_pose_blocks(db))
    if (!covered.count(d.scan_id))
      stored.push_back(d);
  stored = numbered(std::move(stored));
  derive_detached(db, sessions_of(db), stored);
  return stored;
}

std::vector<PoseBlock> set_pose_blocks(ProjectDB &db,
                                       std::vector<PoseBlock> blocks) {
  const Sessions s = sessions_of(db);
  std::map<int, int> owner; // node -> block index
  for (size_t bi = 0; bi < blocks.size(); ++bi) {
    const auto &b = blocks[bi];
    const std::string who = b.name.empty() ? fmt::format("#{}", bi + 1)
                                           : fmt::format("'{}'", b.name);
    if (b.first_node > b.last_node)
      throw std::invalid_argument(
          fmt::format("block {}: first_node {} > last_node {}", who,
                      b.first_node, b.last_node));
    if (!s.frames.count(b.scan_id))
      throw std::invalid_argument(fmt::format(
          "block {}: session (scan) {} has no posed frames", who, b.scan_id));
    int members = 0;
    for (const auto &[id, scan] : s.all) {
      if (id < b.first_node || id > b.last_node)
        continue;
      if (scan != b.scan_id)
        throw std::invalid_argument(fmt::format(
            "block {}: node {} belongs to session (scan) {}, not {}", who, id,
            scan, b.scan_id));
      if (auto [it, fresh] = owner.emplace(id, static_cast<int>(bi)); !fresh)
        throw std::invalid_argument(
            fmt::format("block {}: node {} is already in block '{}'", who, id,
                        blocks[static_cast<size_t>(it->second)].name));
      ++members;
    }
    if (members == 0)
      throw std::invalid_argument(
          fmt::format("block {}: nodes {}..{} hold no posed frame", who,
                      b.first_node, b.last_node));
  }
  for (const auto &[scan, ids] : s.frames) {
    std::vector<int> missing;
    for (int id : ids)
      if (!owner.count(id))
        missing.push_back(id);
    if (!missing.empty())
      throw std::invalid_argument(fmt::format(
          "session (scan) {}: {} frames are in no block (first {}, last {})",
          scan, missing.size(), missing.front(), missing.back()));
  }
  blocks = numbered(std::move(blocks));
  fill_names(blocks, s);
  db.replace_pose_blocks(blocks);
  return pose_blocks(db);
}

std::vector<PoseBlock> blocks_from_splits(const ProjectDB &db,
                                          std::vector<int> split_nodes) {
  const Sessions s = sessions_of(db);
  std::set<int> splits(split_nodes.begin(), split_nodes.end());
  std::vector<PoseBlock> out;
  for (const int scan : s.order) {
    const auto &ids = s.frames.at(scan);
    PoseBlock cur;
    cur.scan_id = scan;
    cur.first_node = ids.front();
    for (size_t k = 1; k < ids.size(); ++k) {
      if (!splits.count(ids[k]))
        continue;
      cur.last_node = ids[k - 1];
      out.push_back(cur);
      cur = PoseBlock{};
      cur.scan_id = scan;
      cur.first_node = ids[k];
    }
    cur.last_node = ids.back();
    out.push_back(cur);
  }
  out = numbered(std::move(out));
  fill_names(out, s);
  derive_detached(db, s, out);
  return out;
}

std::vector<PoseBlock> split_pose_block(ProjectDB &db, int node_id,
                                        std::string name) {
  const Sessions s = sessions_of(db);
  auto blocks = pose_blocks(db);
  const int scan = scan_of(s, node_id);
  if (scan < 0)
    throw std::invalid_argument(
        fmt::format("split: node {} is not a posed frame", node_id));
  auto it = std::find_if(blocks.begin(), blocks.end(), [&](const auto &b) {
    return b.scan_id == scan && b.first_node <= node_id &&
           node_id <= b.last_node;
  });
  if (it == blocks.end())
    throw std::invalid_argument(
        fmt::format("split: node {} is in no block", node_id));
  if (it->first_node == node_id)
    throw std::invalid_argument(fmt::format(
        "split: node {} already starts block '{}'", node_id, it->name));
  const auto &ids = s.frames.at(scan);
  const auto pos = std::lower_bound(ids.begin(), ids.end(), node_id);
  PoseBlock tail = *it;
  tail.first_node = node_id;
  tail.name = std::move(name);
  it->last_node = *(pos - 1);
  blocks.push_back(tail);
  return set_pose_blocks(db, std::move(blocks));
}

std::vector<PoseBlock> merge_pose_blocks(ProjectDB &db, int first, int second) {
  auto blocks = pose_blocks(db);
  auto find = [&](int id) {
    return std::find_if(blocks.begin(), blocks.end(),
                        [&](const auto &b) { return b.id == id; });
  };
  auto a = find(first), b = find(second);
  if (a == blocks.end() || b == blocks.end())
    throw std::invalid_argument(
        fmt::format("merge: no block {}", a == blocks.end() ? first : second));
  if (std::next(a) != b || a->scan_id != b->scan_id)
    throw std::invalid_argument(fmt::format(
        "merge: blocks '{}' and '{}' are not adjacent in one session", a->name,
        b->name));
  a->last_node = b->last_node;
  blocks.erase(b);
  return set_pose_blocks(db, std::move(blocks));
}

std::vector<PoseBlock> rename_pose_block(ProjectDB &db, int block_id,
                                         std::string name) {
  auto blocks = pose_blocks(db);
  auto it = std::find_if(blocks.begin(), blocks.end(),
                         [&](const auto &b) { return b.id == block_id; });
  if (it == blocks.end())
    throw std::invalid_argument(fmt::format("rename: no block {}", block_id));
  if (name.empty())
    throw std::invalid_argument("rename: empty name");
  it->name = std::move(name);
  return set_pose_blocks(db, std::move(blocks));
}

DetachResult detach_pose_block(ProjectDB &db, int block_id, bool attach,
                               std::string_view author) {
  const Sessions s = sessions_of(db);
  auto blocks = pose_blocks(db);
  auto it = std::find_if(blocks.begin(), blocks.end(),
                         [&](const auto &b) { return b.id == block_id; });
  if (it == blocks.end())
    throw std::invalid_argument(fmt::format("detach: no block {}", block_id));
  DetachResult r;
  r.node_id = it->first_node;
  r.prev_node_id = frame_before(s, it->first_node);
  if (r.prev_node_id == 0)
    throw std::invalid_argument(fmt::format(
        "detach: block '{}' starts at the first frame; nothing to detach "
        "from",
        it->name));
  const bool same_session = scan_of(s, r.prev_node_id) == it->scan_id;
  const PoseBlock detached = *it;
  r.edit_id = detach_edit_id(r.prev_node_id, r.node_id);

  ProjectDB::Transaction tx(db);
  ensure_pose_version(db, author);
  if (!attach) {
    put_working_pose_edit(
        db, reject_odometry_edit(
                r.prev_node_id, r.node_id,
                fmt::format("detach block '{}' from the frame before it",
                            detached.name)));
    if (auto e =
            db.pose_graph_edge(r.prev_node_id, r.node_id, pg::kTypeOdometry);
        e && e->status != pg::kStatusRejected)
      r.edge_rejected = db.set_pose_graph_edge_status(
          r.prev_node_id, r.node_id, pg::kTypeOdometry, pg::kStatusRejected);
  } else {
    remove_working_pose_edit(db, r.edit_id);
    if (same_session) {
      // Put the odometry back, measured from the initial poses.
      auto seed = [&](int id) {
        return pg::matrix_from_row_major(
            db.initial_pose(id).value_or(db.sensor_frame_pose(id)));
      };
      ProjectDB::PoseGraphEdge e;
      e.from_node_id = r.prev_node_id;
      e.to_node_id = r.node_id;
      e.edge_type = std::string(pg::kTypeOdometry);
      e.transform = pg::row_major_from_matrix(seed(r.prev_node_id).inverse() *
                                              seed(r.node_id));
      e.source = std::string(pg::kSourceOdometryInit);
      e.status = std::string(pg::kStatusAuto);
      db.upsert_pose_graph_edge(e, /*overwrite_rejected=*/true);
    }
  }
  r.blocks = pose_blocks(db);
  r.version_id = record_pose_version(
      db, {.kind = attach ? "attach" : "detach",
           .label = fmt::format("{} block '{}' at {}|{}",
                                attach ? "re-attach" : "detach", detached.name,
                                r.prev_node_id, r.node_id),
           .author = std::string(author),
           .metrics_json = json{{"block_id", block_id},
                                {"prev_node_id", r.prev_node_id},
                                {"node_id", r.node_id},
                                {"edit_id", r.edit_id}}
                               .dump()});
  tx.commit();

  for (const auto &e : db.list_pose_graph_edges()) {
    if (e.edge_type != pg::kTypeLoopClosure || e.status == pg::kStatusRejected)
      continue;
    const bool a_in = e.from_node_id >= detached.first_node &&
                      e.from_node_id <= detached.last_node;
    const bool b_in = e.to_node_id >= detached.first_node &&
                      e.to_node_id <= detached.last_node;
    if (a_in != b_in)
      ++r.holding_loops;
  }
  if (!attach && r.holding_loops == 0)
    reusex::warn("detach: block '{}' (nodes {}..{}) has no active loop edge "
                 "to the rest of the graph; a solve needs a loop, a relative "
                 "edit or a hold to place it",
                 detached.name, detached.first_node, detached.last_node);
  return r;
}

std::vector<int> block_nodes(const ProjectDB &db, const PoseBlock &block) {
  std::vector<int> out;
  for (const auto &p : db.posed_frames())
    if (p.scan_id == block.scan_id && p.node_id >= block.first_node &&
        p.node_id <= block.last_node)
      out.push_back(p.node_id);
  return out;
}

// ── JSON ───────────────────────────────────────────────────────────────────

json to_json(const PoseBlock &b) {
  return {{"id", b.id},
          {"name", b.name},
          {"first_node", b.first_node},
          {"last_node", b.last_node},
          {"scan_id", b.scan_id},
          {"detached", b.detached}};
}

json pose_blocks_json(const ProjectDB &db) {
  const Sessions s = sessions_of(db);
  json out = json::array();
  for (const auto &b : pose_blocks(db)) {
    json j = to_json(b);
    const auto &ids = s.frames.at(b.scan_id);
    const auto lo = std::lower_bound(ids.begin(), ids.end(), b.first_node);
    const auto hi = std::upper_bound(ids.begin(), ids.end(), b.last_node);
    j["frame_count"] = hi - lo;
    j["session"] = s.ordinal(b.scan_id);
    auto pos = [&](int id) {
      const auto &p = s.pose.at(id);
      return json::array({p[3], p[7], p[11]});
    };
    if (lo != hi) {
      j["start"] = pos(*lo);
      j["end"] = pos(*(hi - 1));
    }
    out.push_back(std::move(j));
  }
  return out;
}

json to_json(const BlockSplitSuggestion &s) {
  json reasons = json::array();
  for (auto r : s.reasons)
    reasons.push_back(std::string(to_string(r)));
  return {{"node_id", s.node_id},
          {"prev_node_id", s.prev_node_id},
          {"reason", std::string(to_string(s.reason))},
          {"reasons", reasons},
          {"score", s.score},
          {"gap_s", s.gap_s},
          {"jump_m", s.jump_m},
          {"detail", s.detail}};
}

json to_json(const BlockSuggestions &s) {
  json splits = json::array();
  for (const auto &x : s.splits)
    splits.push_back(to_json(x));
  json glitches = json::array();
  for (const auto &g : s.glitches)
    glitches.push_back({{"node_id", g.node_id}, {"jump_m", g.jump_m}});
  return {{"splits", splits},
          {"glitches", glitches},
          {"storey_levels", s.storey_levels}};
}

json to_json(const DetachResult &r) {
  json blocks = json::array();
  for (const auto &b : r.blocks)
    blocks.push_back(to_json(b));
  return {{"blocks", blocks},
          {"prev_node_id", r.prev_node_id},
          {"node_id", r.node_id},
          {"edit_id", r.edit_id},
          {"edge_rejected", r.edge_rejected},
          {"holding_loops", r.holding_loops},
          {"version_id", r.version_id}};
}

PoseBlock pose_block_from_json(const json &j) {
  if (!j.is_object())
    throw std::invalid_argument("block: must be an object");
  PoseBlock b;
  auto int_field = [&](const char *key, bool required, int fallback) {
    if (!j.contains(key)) {
      if (required)
        throw std::invalid_argument(fmt::format("block: {}: missing", key));
      return fallback;
    }
    if (!j[key].is_number_integer())
      throw std::invalid_argument(
          fmt::format("block: {}: must be an integer", key));
    return j[key].get<int>();
  };
  b.first_node = int_field("first_node", true, 0);
  b.last_node = int_field("last_node", true, 0);
  b.scan_id = int_field("scan_id", true, 0);
  if (j.contains("name")) {
    if (!j["name"].is_string())
      throw std::invalid_argument("block: name: must be a string");
    b.name = j["name"].get<std::string>();
  }
  if (j.contains("detached")) {
    if (!j["detached"].is_boolean())
      throw std::invalid_argument("block: detached: must be a boolean");
    b.detached = j["detached"].get<bool>();
  }
  return b;
}

} // namespace reusex::core
