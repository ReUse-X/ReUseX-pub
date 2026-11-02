// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
// Scan-repair commands (reusex-y2lg): `rux graph blocks ...`,
// `rux graph block-transform` and `rux graph versions ...`, the scriptable
// face of what the web editor does through ruxd.
#include "../global-params.hpp"

#include <CLI/CLI.hpp>
#include <memory>

void setup_subcommand_graph_blocks(CLI::App &parent,
                                   std::shared_ptr<RuxOptions> global_opt);
void setup_subcommand_graph_block_transform(
    CLI::App &parent, std::shared_ptr<RuxOptions> global_opt);
void setup_subcommand_graph_versions(CLI::App &parent,
                                     std::shared_ptr<RuxOptions> global_opt);
