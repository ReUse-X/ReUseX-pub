// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "exit_status.hpp"
#include "graph/repair.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/pose_blocks.hpp>

#include <fmt/format.h>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <exception>
#include <functional>
#include <iostream>
#include <stdexcept>

namespace {

using reusex::core::PoseBlock;

/// Run @p body on the project; map a bad argument to INVALID_ARGUMENT.
int guarded(const RuxOptions &g, const char *what,
            const std::function<void(reusex::ProjectDB &)> &body) {
  try {
    reusex::ProjectDB db(g.project_db);
    body(db);
    return RuxError::SUCCESS;
  } catch (const std::invalid_argument &e) {
    spdlog::error("graph blocks {}: {}", what, e.what());
    return RuxError::INVALID_ARGUMENT;
  } catch (const std::exception &e) {
    spdlog::error("graph blocks {} failed: {}", what, e.what());
    return RuxError::GENERIC;
  }
}

void print_blocks(const reusex::ProjectDB &db, bool as_json) {
  const auto j = reusex::core::pose_blocks_json(db);
  if (as_json) {
    std::cout << j.dump(2) << "\n";
    return;
  }
  std::cout << fmt::format("{:>3}  {:<14} {:>7} {:>7} {:>6} {:>5}  {}\n", "id",
                           "name", "first", "last", "frames", "scan", "joint");
  for (const auto &b : j)
    std::cout << fmt::format(
        "{:>3}  {:<14} {:>7} {:>7} {:>6} {:>5}  {}\n", b["id"].get<int>(),
        b["name"].get<std::string>(), b["first_node"].get<int>(),
        b["last_node"].get<int>(), b["frame_count"].get<long>(),
        b["scan_id"].get<int>(), b["detached"].get<bool>() ? "detached" : "");
}

} // namespace

void setup_subcommand_graph_blocks(CLI::App &parent,
                                   std::shared_ptr<RuxOptions> global_opt) {
  auto *sub = parent.add_subcommand(
      "blocks", "Split the trajectory into blocks (scan-repair editor)");
  sub->footer(R"(
DESCRIPTION:
  Blocks are runs of consecutive frames of one session that the scan-repair
  tools act on (move, rotate, level, fit, hold). They are stored in the
  project (pose_blocks); without stored blocks every session is one block,
  and every session after the first starts detached.

SUBCOMMANDS:
  suggest  Suggested cuts with a reason: session, time_gap (> 100 s),
           tracking_gap (> 5 s), storey_change, odometry_jump
  list     The stored blocks
  split    Start a new block at a node
  merge    Merge a block into the one before it
  rename   Rename a block
  detach   Cut a block from the one before it (reject_edge in the working
           pose edits); --attach undoes it

EXAMPLES:
  rux graph blocks suggest
  rux graph blocks suggest --apply          # store the suggested blocks
  rux graph blocks split 2337 --name s1-descent
  rux graph blocks detach 4
)");
  sub->require_subcommand(1);

  {
    struct Opt {
      reusex::core::BlockSuggestOptions s;
      bool json = false;
      bool apply = false;
    };
    auto opt = std::make_shared<Opt>();
    auto *c = sub->add_subcommand("suggest", "Suggested block cuts");
    c->add_option("--time-gap", opt->s.time_gap_s,
                  "Cut where frames are further apart in time [s]")
        ->capture_default_str();
    c->add_option("--tracking-gap", opt->s.tracking_gap_s,
                  "Tracking gap within a session [s]")
        ->capture_default_str();
    c->add_option("--jump", opt->s.odometry_jump_m,
                  "Odometry jump between two frames [m]")
        ->capture_default_str();
    c->add_option("--storey-separation", opt->s.storey_separation_m,
                  "Camera-height bands at least this far apart [m]")
        ->capture_default_str();
    c->add_option("--storey-min-frames", opt->s.storey_min_frames,
                  "Frames a band (and a stay in it) needs")
        ->capture_default_str();
    c->add_flag("--json", opt->json, "Print JSON");
    c->add_flag("--apply", opt->apply, "Store blocks cut at every suggestion");
    c->callback([opt, global_opt]() {
      rux::finish(guarded(*global_opt, "suggest", [&](reusex::ProjectDB &db) {
        const auto s = reusex::core::suggest_block_splits(db, opt->s);
        if (opt->json) {
          std::cout << reusex::core::to_json(s).dump(2) << "\n";
        } else {
          for (const auto &x : s.splits)
            std::cout << fmt::format("{:>7}  {:<14} score {:.2f}  {}\n",
                                     x.node_id, to_string(x.reason), x.score,
                                     x.detail);
          for (const auto &g : s.glitches)
            std::cout << fmt::format("{:>7}  glitch         {:.2f} m away "
                                     "and back\n",
                                     g.node_id, g.jump_m);
        }
        if (opt->apply) {
          std::vector<int> nodes;
          for (const auto &x : s.splits)
            nodes.push_back(x.node_id);
          reusex::core::set_pose_blocks(
              db, reusex::core::blocks_from_splits(db, nodes));
          if (!opt->json)
            print_blocks(db, false);
        }
      }));
    });
  }
  {
    auto as_json = std::make_shared<bool>(false);
    auto *c = sub->add_subcommand("list", "List the blocks");
    c->add_flag("--json", *as_json, "Print JSON");
    c->callback([as_json, global_opt]() {
      rux::finish(guarded(*global_opt, "list", [&](reusex::ProjectDB &db) {
        print_blocks(db, *as_json);
      }));
    });
  }
  {
    struct Opt {
      int node = 0;
      std::string name;
    };
    auto opt = std::make_shared<Opt>();
    auto *c = sub->add_subcommand("split", "Start a new block at a node");
    c->add_option("node", opt->node, "First node of the new block")->required();
    c->add_option("--name", opt->name, "Name of the new block");
    c->callback([opt, global_opt]() {
      rux::finish(guarded(*global_opt, "split", [&](reusex::ProjectDB &db) {
        reusex::core::split_pose_block(db, opt->node, opt->name);
        print_blocks(db, false);
      }));
    });
  }
  {
    auto ids = std::make_shared<std::vector<int>>();
    auto *c =
        sub->add_subcommand("merge", "Merge block B into block A before it");
    c->add_option("blocks", *ids, "Block ids A B")->required()->expected(2);
    c->callback([ids, global_opt]() {
      rux::finish(guarded(*global_opt, "merge", [&](reusex::ProjectDB &db) {
        reusex::core::merge_pose_blocks(db, (*ids)[0], (*ids)[1]);
        print_blocks(db, false);
      }));
    });
  }
  {
    struct Opt {
      int id = 0;
      std::string name;
    };
    auto opt = std::make_shared<Opt>();
    auto *c = sub->add_subcommand("rename", "Rename a block");
    c->add_option("id", opt->id, "Block id")->required();
    c->add_option("name", opt->name, "New name")->required();
    c->callback([opt, global_opt]() {
      rux::finish(guarded(*global_opt, "rename", [&](reusex::ProjectDB &db) {
        reusex::core::rename_pose_block(db, opt->id, opt->name);
        print_blocks(db, false);
      }));
    });
  }
  {
    struct Opt {
      int id = 0;
      bool attach = false;
      bool json = false;
    };
    auto opt = std::make_shared<Opt>();
    auto *c =
        sub->add_subcommand("detach", "Cut a block from the block before it");
    c->add_option("id", opt->id, "Block id")->required();
    c->add_flag("--attach", opt->attach, "Re-attach instead");
    c->add_flag("--json", opt->json, "Print JSON");
    c->callback([opt, global_opt]() {
      rux::finish(guarded(*global_opt, "detach", [&](reusex::ProjectDB &db) {
        const auto r =
            reusex::core::detach_pose_block(db, opt->id, opt->attach);
        if (opt->json) {
          std::cout << reusex::core::to_json(r).dump(2) << "\n";
          return;
        }
        spdlog::info("Block {} {} at {}-{} (edit '{}'); {} loop edges hold "
                     "it",
                     opt->id, opt->attach ? "re-attached" : "detached",
                     r.prev_node_id, r.node_id, r.edit_id, r.holding_loops);
      }));
    });
  }
}
