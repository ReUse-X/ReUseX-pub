// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "import/rtabmap.hpp"
#include "exit_status.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/io/rtabmap.hpp>
#include <spdlog/fmt/std.h>
#include <spdlog/spdlog.h>

void setup_subcommand_import_rtabmap(CLI::App &app,
                                     std::shared_ptr<RuxOptions> global_opt) {

  auto opt = std::make_shared<SubcommandImportRTABMapOptions>();
  auto *sub =
      app.add_subcommand("rtabmap", "Import sensor data from RTABMap database");

  sub->footer(R"(
DESCRIPTION:
  Extracts raw sensor data (RGB images, depth maps, camera poses, intrinsics)
  from an RTABMap SLAM database and stores it in a ReUseX project for
  further processing. This is the first step in the typical reconstruction
  pipeline. No point cloud reconstruction is performed during import.

EXAMPLES:
  rux import rtabmap scan.db           # Import to ./project.rux
  rux -p office.rux import rtabmap scan.db  # Custom project path
  rux import rtabmap ~/scans/building.db    # Absolute path

WORKFLOW:
  1. rux import rtabmap scan.db        # Import sensor frames
  2. rux create clouds                 # Reconstruct point clouds
  3. rux create annotate --net model   # Optional: ML inference
  4. rux view                          # Visualize results

NOTES:
  - Requires RTABMap database (.db) from RTABMap SLAM application
  - Extracts: RGB images, depth maps, confidence maps, camera poses
  - Camera intrinsics and local transforms preserved from RTABMap
  - Images stored in original orientation (no preprocessing)
  - Use global -p/--project flag to specify output project path
  - After import, run 'rux create clouds' to generate point clouds
  - --append imports several RTAB-Map sessions one by one into one project
    without rtabmap-reprocess; each session starts detached where its own
    map put it ('rux graph blocks suggest', 'rux graph block-transform')
)");

  sub->add_option("database", opt->database_path_in,
                  "Path to the RTABMap database file.")
      ->required()
      ->check(CLI::ExistingFile);
  sub->add_flag("--append", opt->append,
                "Append the database as one more detached session for the "
                "scan-repair editor: refuse a session already imported, "
                "replace RTAB-Map's raw odometry by the imported poses', cut "
                "the link to the previous session (working pose edits) and "
                "record a pose version");

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling run_subcommand_import_rtabmap");
    rux::finish(run_subcommand_import_rtabmap(*opt, *global_opt));
  });
}

int run_subcommand_import_rtabmap(SubcommandImportRTABMapOptions const &opt,
                                  const RuxOptions &global_opt) {

  fs::path project_path = global_opt.project_db;
  spdlog::info("Importing RTABMap database to project: {}",
               project_path.string());

  reusex::ProjectDB project_db(project_path);
  try {
    const auto r = reusex::io::import_rtabmap(
        project_db, opt.database_path_in,
        {.append_session = opt.append, .author = "rux"});
    if (opt.append)
      spdlog::info("Session {} appended: {} frames (nodes {}..{}), {} "
                   "odometry edges from the imported poses, {} loop edges; "
                   "pose version {}",
                   r.scan_id, r.frames, r.first_node, r.last_node,
                   r.odometry_rebuilt, r.loop_edges, r.version_id.value_or(0));
  } catch (const reusex::io::DuplicateSessionError &e) {
    spdlog::error("{}", e.what());
    return RuxError::INVALID_ARGUMENT;
  }

  spdlog::info("Import complete. Use 'rux create clouds' to generate "
               "point clouds.");

  return RuxError::SUCCESS;
}
