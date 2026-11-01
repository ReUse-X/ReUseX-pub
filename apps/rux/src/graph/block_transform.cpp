// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "exit_status.hpp"
#include "graph/repair.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/pose_repair.hpp>

#include <fmt/format.h>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <exception>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace {

struct Opt {
  std::vector<int> blocks;
  std::vector<std::string> nodes; // "a-b" or "a"
  double dx = 0, dy = 0, dz = 0, yaw = 0, pitch = 0, roll = 0;
  std::vector<double> pivot;
  std::string mode = "reseed";
  int blend = reusex::core::BlockTransformRequest{}.blend_frames;
  bool hold = false;
  std::vector<std::string> hold_dofs;
  double hold_sigma_m = reusex::core::BlockTransformHold{}.sigma_trans_m;
  double hold_sigma_deg = reusex::core::BlockTransformHold{}.sigma_rot_deg;
  bool preview = false;
  bool json = false;
  std::string label;
  std::string request; // a JSON request file (the web editor's body)
};

std::pair<int, int> parse_range(const std::string &s) {
  const auto dash = s.find('-', 1);
  try {
    if (dash == std::string::npos) {
      const int n = std::stoi(s);
      return {n, n};
    }
    return {std::stoi(s.substr(0, dash)), std::stoi(s.substr(dash + 1))};
  } catch (const std::exception &) {
    throw std::invalid_argument(
        fmt::format("--nodes '{}': expected <first>-<last> or <node>", s));
  }
}

nlohmann::json request_json(const Opt &o) {
  if (!o.request.empty()) {
    std::ifstream in(o.request);
    if (!in)
      throw std::invalid_argument(
          fmt::format("--request: cannot read {}", o.request));
    std::stringstream ss;
    ss << in.rdbuf();
    return nlohmann::json::parse(ss.str());
  }
  nlohmann::json j;
  j["blocks"] = o.blocks;
  nlohmann::json nodes = nlohmann::json::array();
  for (const auto &n : o.nodes) {
    const auto [a, b] = parse_range(n);
    nodes.push_back({a, b});
  }
  j["nodes"] = nodes;
  j["delta"] = {{"xyz", {o.dx, o.dy, o.dz}},
                {"yaw_deg", o.yaw},
                {"pitch_deg", o.pitch},
                {"roll_deg", o.roll}};
  if (!o.pivot.empty()) {
    if (o.pivot.size() != 3)
      throw std::invalid_argument("--pivot: expected x,y,z");
    j["delta"]["pivot"] = o.pivot;
  }
  j["mode"] = o.mode;
  j["blend_frames"] = o.blend;
  if (o.hold) {
    nlohmann::json h{{"sigma_trans_m", o.hold_sigma_m},
                     {"sigma_rot_deg", o.hold_sigma_deg}};
    if (!o.hold_dofs.empty())
      h["dofs"] = o.hold_dofs;
    j["hold"] = h;
  }
  j["preview"] = o.preview;
  if (!o.label.empty())
    j["label"] = o.label;
  return j;
}

} // namespace

void setup_subcommand_graph_block_transform(
    CLI::App &parent, std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<Opt>();
  auto *sub = parent.add_subcommand(
      "block-transform",
      "Move / rotate blocks as a re-seed, with blend at the joints");
  sub->footer(R"(
DESCRIPTION:
  Applies one rigid delta to the selected blocks / node ranges. --mode reseed
  (default) writes the moved poses into transform AND initial_transform,
  seeds every other frame from its current pose, and re-derives the odometry
  from the new seed (reusex-om3, reusex-xmt), so the next 'rux optimize'
  associates planes from the moved block. --mode local moves only the
  selection's transform. The move fades out over --blend frames at a joint
  to an attached neighbour; it is rigid at a session boundary or a detached
  joint. Each apply is a pose version ('rux graph versions undo').

  Rotations are about --pivot (default: the centroid of the selection's
  cameras), R = Rz(yaw) Ry(pitch) Rx(roll), degrees.

EXAMPLES:
  rux graph block-transform --block 7 --dz -2.8 --preview --json
  rux graph block-transform --block 9 --yaw 90 --pivot 4.9,20.4,0 --hold
  rux graph block-transform --nodes 2621-2850 --dz 1.0 --blend 15
  rux graph block-transform --request move.json   # the web editor's body
)");
  sub->add_option("--block", opt->blocks, "Block id (repeatable)");
  sub->add_option("--nodes", opt->nodes,
                  "Node range <first>-<last> or one node (repeatable)");
  sub->add_option("--dx", opt->dx, "Translation x [m]");
  sub->add_option("--dy", opt->dy, "Translation y [m]");
  sub->add_option("--dz", opt->dz, "Translation z [m]");
  sub->add_option("--yaw", opt->yaw, "Rotation about z [deg]");
  sub->add_option("--pitch", opt->pitch, "Rotation about y [deg]");
  sub->add_option("--roll", opt->roll, "Rotation about x [deg]");
  sub->add_option("--pivot", opt->pivot, "Rotation centre x,y,z [m]")
      ->delimiter(',');
  sub->add_option("--mode", opt->mode, "reseed | local")
      ->check(CLI::IsMember({"reseed", "local"}))
      ->capture_default_str();
  sub->add_option("--blend", opt->blend,
                  "Frames the move fades over at an attached joint")
      ->capture_default_str();
  sub->add_flag("--hold", opt->hold,
                "Hold the moved block where it is seeded (working edits)");
  sub->add_option("--hold-dofs", opt->hold_dofs,
                  "DoFs the hold keeps (x,y,z,roll,pitch,yaw; default all)")
      ->delimiter(',');
  sub->add_option("--hold-sigma", opt->hold_sigma_m, "Hold sigma [m]")
      ->capture_default_str();
  sub->add_option("--hold-sigma-deg", opt->hold_sigma_deg, "Hold sigma [deg]")
      ->capture_default_str();
  sub->add_flag("--preview", opt->preview,
                "Report the moved poses without writing anything");
  sub->add_flag("--json", opt->json, "Print the result as JSON");
  sub->add_option("--label", opt->label, "Label of the pose version");
  sub->add_option("--request", opt->request,
                  "Read the whole request from a JSON file (the body of "
                  "POST .../posegraph/blocks/transform)")
      ->check(CLI::ExistingFile);
  sub->callback([opt, global_opt]() {
    int rc = RuxError::SUCCESS;
    try {
      auto req =
          reusex::core::block_transform_request_from_json(request_json(*opt));
      req.author = "rux";
      reusex::ProjectDB db(global_opt->project_db);
      const auto r = reusex::core::apply_block_transform(db, req);
      if (opt->json) {
        std::cout << reusex::core::to_json(r, req.preview).dump(2) << "\n";
      } else {
        spdlog::info("{}: {} frames moved, {} blended, max shift {:.3f} m; "
                     "{} loops into the selection ({} disagree > 0.3 m / 4 "
                     "deg){}",
                     req.preview ? "Preview" : "Applied", r.frames_moved,
                     r.frames_blended, r.max_shift_m, r.loops_touching,
                     r.loops_disagreeing,
                     r.version_id
                         ? fmt::format("; pose version {}", *r.version_id)
                         : std::string());
        for (const auto &jr : r.joints)
          spdlog::info("  joint {}|{}: {}{}gap {:.3f} -> {:.3f} m, blend {}",
                       jr.inside_node, jr.outside_node,
                       jr.session_boundary ? "session boundary, " : "",
                       jr.detached ? "detached, " : "", jr.gap_before_m,
                       jr.gap_after_m, jr.blend_frames);
      }
    } catch (const std::invalid_argument &e) {
      spdlog::error("graph block-transform: {}", e.what());
      rc = RuxError::INVALID_ARGUMENT;
    } catch (const std::exception &e) {
      spdlog::error("graph block-transform failed: {}", e.what());
      rc = RuxError::GENERIC;
    }
    rux::finish(rc);
  });
}
