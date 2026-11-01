// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The working pose-edits document of a project (reusex-y2lg.31): one
// reusex.pose_edits.v1 JSON document (docs/design/pose-edits.md) stored in
// project_settings, so the web editor, the Qt client and a solve job see the
// same holds and cuts. Pose versions (core/pose_versions.hpp) snapshot it.
//
// Core keeps it as JSON text and checks only the document's shape (schema,
// an `edits` array of objects with unique non-empty `id` and a `kind`); the
// full reader, which names a bad field, is io::pose_edits_from_json().

#include <reusex/core/ProjectDB.hpp>

#include <nlohmann/json_fwd.hpp>

#include <cstdint>
#include <string>
#include <string_view>

namespace reusex::core {

/// project_settings key of the document.
inline constexpr std::string_view kSettingWorkingPoseEdits =
    "pose_edits.working";

/// The stored document, or an empty one ({schema, frame: world, edits: []}).
std::string working_pose_edits(const ProjectDB &db);

/// Store @p json as the working document.
/// @throws std::invalid_argument when it is not a pose-edits document shape.
void set_working_pose_edits(ProjectDB &db, std::string_view json);

/// Add @p edit (a JSON object with `id` and `kind`), replacing the edit with
/// the same id. @throws std::invalid_argument for a malformed edit.
void put_working_pose_edit(ProjectDB &db, const nlohmann::json &edit);

/// Remove the edit @p id. @returns false when there is none.
bool remove_working_pose_edit(ProjectDB &db, std::string_view id);

/// The id of the reject_edge edit a detach writes for the odometry link
/// between @p a and @p b ("detach-<min>-<max>").
std::string detach_edit_id(int a, int b);

/// A reject_edge edit for the odometry link between @p a and @p b.
nlohmann::json reject_odometry_edit(int a, int b, std::string_view note);

/// An identity `move` edit holding nodes [first, last] where they are seeded
/// (an anchor with the DoF mask, reusex-y2lg.18).
nlohmann::json hold_edit(std::string_view id, int first, int last,
                         std::uint8_t dofs, double sigma_trans_m,
                         double sigma_rot_deg, std::string_view note);

} // namespace reusex::core
