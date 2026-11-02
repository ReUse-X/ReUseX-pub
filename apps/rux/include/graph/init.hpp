// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"

#include <reusex/core/pose_repair.hpp>
#include <reusex/slam/pose_graph.hpp>

#include <CLI/CLI.hpp>
#include <memory>

struct SubcommandGraphInitOptions {
  bool force = reusex::slam::InitPoseGraphOptions{}.force;
  /// core::rebuild_odometry instead of init (reusex-y2lg.16).
  bool rebuild_odometry = false;
  bool from_current = reusex::core::RebuildOdometryOptions{}.from_current;
  bool json = false;
};

void setup_subcommand_graph_init(CLI::App &parent,
                                 std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_graph_init(const SubcommandGraphInitOptions &opt,
                              const RuxOptions &global_opt);
