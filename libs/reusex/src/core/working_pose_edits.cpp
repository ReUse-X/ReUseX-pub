// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/working_pose_edits.hpp"

#include "core/pose_edits.hpp"

#include <fmt/format.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <set>
#include <stdexcept>

namespace reusex::core {
namespace {

using json = nlohmann::json;

constexpr const char *kSchema = "reusex.pose_edits.v1";

json empty_document() {
  return json{
      {"schema", kSchema}, {"frame", "world"}, {"edits", json::array()}};
}

void check_edit(const json &e, std::size_t index) {
  if (!e.is_object())
    throw std::invalid_argument(
        fmt::format("pose edits: edit #{}: must be an object", index));
  if (!e.contains("id") || !e["id"].is_string() ||
      e["id"].get<std::string>().empty())
    throw std::invalid_argument(fmt::format(
        "pose edits: edit #{}: id: must be a non-empty string", index));
  const std::string id = e["id"].get<std::string>();
  if (!e.contains("kind") || !e["kind"].is_string() ||
      !pose_edit_kind_from_string(e["kind"].get<std::string>()))
    throw std::invalid_argument(fmt::format(
        "pose edits: edit '{}': kind: must be one of anchor, move, relative, "
        "same_surface, on_plan_wall, reject_edge",
        id));
}

json checked_document(std::string_view text) {
  json doc;
  try {
    doc = json::parse(text);
  } catch (const json::parse_error &e) {
    throw std::invalid_argument(
        fmt::format("pose edits: not JSON: {}", e.what()));
  }
  if (!doc.is_object())
    throw std::invalid_argument("pose edits: the document must be an object");
  if (!doc.contains("schema") || doc["schema"] != kSchema)
    throw std::invalid_argument(
        fmt::format("pose edits: schema: must be '{}'", kSchema));
  if (!doc.contains("edits"))
    doc["edits"] = json::array();
  if (!doc["edits"].is_array())
    throw std::invalid_argument("pose edits: edits: must be an array");
  std::set<std::string> ids;
  std::size_t i = 0;
  for (const auto &e : doc["edits"]) {
    check_edit(e, i++);
    if (!ids.insert(e["id"].get<std::string>()).second)
      throw std::invalid_argument(fmt::format(
          "pose edits: edit '{}': id: duplicate", e["id"].get<std::string>()));
  }
  return doc;
}

json load(const ProjectDB &db) {
  if (auto text = db.setting(kSettingWorkingPoseEdits); text && !text->empty())
    return checked_document(*text);
  return empty_document();
}

} // namespace

std::string working_pose_edits(const ProjectDB &db) { return load(db).dump(2); }

void set_working_pose_edits(ProjectDB &db, std::string_view text) {
  const json doc = checked_document(text);
  db.set_setting(kSettingWorkingPoseEdits, doc.dump(2));
}

void put_working_pose_edit(ProjectDB &db, const json &edit) {
  check_edit(edit, 0);
  json doc = load(db);
  auto &edits = doc["edits"];
  const std::string id = edit["id"].get<std::string>();
  auto it = std::find_if(edits.begin(), edits.end(),
                         [&](const json &e) { return e["id"] == id; });
  if (it != edits.end())
    *it = edit;
  else
    edits.push_back(edit);
  db.set_setting(kSettingWorkingPoseEdits, doc.dump(2));
}

std::set<std::string> working_pose_edit_ids(const ProjectDB &db) {
  std::set<std::string> out;
  const json doc = load(db); // not a temporary: the loop reads into it
  for (const auto &e : doc["edits"])
    out.insert(e["id"].get<std::string>());
  return out;
}

bool remove_working_pose_edit(ProjectDB &db, std::string_view id) {
  json doc = load(db);
  auto &edits = doc["edits"];
  const auto before = edits.size();
  edits.erase(std::remove_if(edits.begin(), edits.end(),
                             [&](const json &e) { return e["id"] == id; }),
              edits.end());
  if (edits.size() == before)
    return false;
  db.set_setting(kSettingWorkingPoseEdits, doc.dump(2));
  return true;
}

std::string detach_edit_id(int a, int b) {
  return fmt::format("detach-{}-{}", std::min(a, b), std::max(a, b));
}

json reject_odometry_edit(int a, int b, std::string_view note) {
  json e{{"id", detach_edit_id(a, b)},
         {"kind", "reject_edge"},
         {"a", std::min(a, b)},
         {"b", std::max(a, b)},
         {"edge_type", "odometry"}};
  if (!note.empty())
    e["note"] = std::string(note);
  return e;
}

json hold_edit(std::string_view id, int first, int last, std::uint8_t dofs,
               double sigma_trans_m, double sigma_rot_deg,
               std::string_view note) {
  json e{{"id", std::string(id)},
         {"kind", "move"},
         {"nodes", json::array({first, last})},
         {"delta", json::object()},
         {"dofs", dof_names(dofs)},
         {"sigma", {{"trans_m", sigma_trans_m}, {"rot_deg", sigma_rot_deg}}}};
  if (!note.empty())
    e["note"] = std::string(note);
  return e;
}

} // namespace reusex::core
