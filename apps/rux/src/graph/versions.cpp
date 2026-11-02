// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "exit_status.hpp"
#include "graph/repair.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/pose_versions.hpp>

#include <fmt/format.h>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <exception>
#include <functional>
#include <iostream>
#include <stdexcept>

namespace {

int guarded(const RuxOptions &g, const char *what,
            const std::function<void(reusex::ProjectDB &)> &body) {
  try {
    reusex::ProjectDB db(g.project_db);
    body(db);
    return RuxError::SUCCESS;
  } catch (const std::invalid_argument &e) {
    spdlog::error("graph versions {}: {}", what, e.what());
    return RuxError::INVALID_ARGUMENT;
  } catch (const std::exception &e) {
    spdlog::error("graph versions {} failed: {}", what, e.what());
    return RuxError::GENERIC;
  }
}

void report(const reusex::core::PoseVersionChange &c, bool as_json) {
  if (as_json) {
    std::cout << reusex::core::to_json(c).dump(2) << "\n";
    return;
  }
  spdlog::info("Pose version {} -> {}: {} frames moved (max {:.3f} m), {} "
               "edge rows changed",
               c.from_version, c.to_version, c.frames_changed, c.max_shift_m,
               c.edges_changed);
}

} // namespace

void setup_subcommand_graph_versions(CLI::App &parent,
                                     std::shared_ptr<RuxOptions> global_opt) {
  auto *sub = parent.add_subcommand(
      "versions", "Pose versions: list, undo, redo, restore, diff");
  sub->footer(R"(
DESCRIPTION:
  Every scan-repair step (a session import with --append, a block transform,
  an odometry rebuild) stores the poses, the pose graph and the working pose
  edits it produced as a version in the project. undo / redo / restore write
  a version back; nothing is deleted. A change made outside these steps (a
  'rux optimize' run) is recorded as an 'external' version first, so it can
  be restored too.

EXAMPLES:
  rux graph versions list
  rux graph versions undo
  rux graph versions restore 3
  rux graph versions diff 2 5
)");
  sub->require_subcommand(1);
  auto as_json = std::make_shared<bool>(false);
  sub->add_flag("--json", *as_json, "Print JSON");

  {
    auto *c = sub->add_subcommand("list", "List the versions");
    c->callback([as_json, global_opt]() {
      rux::finish(guarded(*global_opt, "list", [&](reusex::ProjectDB &db) {
        const auto current = reusex::core::current_pose_version(db);
        nlohmann::json out = nlohmann::json::array();
        for (const auto &v : db.pose_versions()) {
          if (*as_json) {
            out.push_back(reusex::core::to_json(v, current == v.id));
            continue;
          }
          std::cout << fmt::format("{} {:>4}  parent {:>4}  {:<9} {}  {}\n",
                                   current == v.id ? "*" : " ", v.id,
                                   v.parent_id, v.kind, v.created_at, v.label);
        }
        if (*as_json)
          std::cout << out.dump(2) << "\n";
      }));
    });
  }
  {
    auto *c = sub->add_subcommand("undo", "Restore the current version's "
                                          "parent");
    c->callback([as_json, global_opt]() {
      rux::finish(guarded(*global_opt, "undo", [&](reusex::ProjectDB &db) {
        report(reusex::core::undo_pose_version(db, "rux"), *as_json);
      }));
    });
  }
  {
    auto *c = sub->add_subcommand("redo", "Restore the newest child version");
    c->callback([as_json, global_opt]() {
      rux::finish(guarded(*global_opt, "redo", [&](reusex::ProjectDB &db) {
        report(reusex::core::redo_pose_version(db, "rux"), *as_json);
      }));
    });
  }
  {
    auto id = std::make_shared<int64_t>(0);
    auto *c = sub->add_subcommand("restore", "Restore a version");
    c->add_option("id", *id, "Version id")->required();
    c->callback([id, as_json, global_opt]() {
      rux::finish(guarded(*global_opt, "restore", [&](reusex::ProjectDB &db) {
        report(reusex::core::restore_pose_version(db, *id, "rux"), *as_json);
      }));
    });
  }
  {
    auto ids = std::make_shared<std::vector<int64_t>>();
    auto *c = sub->add_subcommand("diff", "Frames moved between two versions");
    c->add_option("versions", *ids, "Version ids A B")->required()->expected(2);
    c->callback([ids, as_json, global_opt]() {
      rux::finish(guarded(*global_opt, "diff", [&](reusex::ProjectDB &db) {
        const auto moves =
            reusex::core::diff_pose_versions(db, (*ids)[0], (*ids)[1]);
        if (*as_json) {
          nlohmann::json out = nlohmann::json::array();
          for (const auto &m : moves)
            out.push_back(reusex::core::to_json(m));
          std::cout << out.dump(2) << "\n";
          return;
        }
        spdlog::info("{} frames moved between versions {} and {}", moves.size(),
                     (*ids)[0], (*ids)[1]);
        for (size_t i = 0; i < moves.size() && i < 20; ++i)
          std::cout << fmt::format("{:>7}  {:.3f} m  {:.2f} deg\n",
                                   moves[i].node_id, moves[i].shift_m,
                                   moves[i].rotation_deg);
      }));
    });
  }
}
