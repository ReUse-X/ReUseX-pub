// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"

#include <CLI/CLI.hpp>
#include <filesystem>
#include <memory>

namespace fs = std::filesystem;

struct SubcommandImportRTABMapOptions {
  fs::path database_path_in;
  /// io::ImportRtabmapOptions::append_session (reusex-y2lg.30).
  bool append = false;
};

// Function declarations.
void setup_subcommand_import_rtabmap(CLI::App &app,
                                     std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_import_rtabmap(SubcommandImportRTABMapOptions const &opt,
                                  const RuxOptions &global_opt);
