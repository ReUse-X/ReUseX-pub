// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// @file
/// Solid-model row edits (reusex-yuja): the library half of the Rediger
/// editor's Geometri tab (rux-frontend frontend-u0o, ruxd model-edits API).
///
/// An edit is a batch of ops on storeys / walls / openings / spaces, applied
/// in one transaction and logged as one undo step:
///  - put: write a record. The row becomes `source = manual`, `locked = 1`
///    and carries `metadata.edited = true`; a detected or imported row keeps
///    what it was in `metadata.edited_from` (the whole record, written once,
///    at the first edit). A row nobody stored yet is created (an empty guid
///    gets a fresh one).
///  - remove: delete a row. A detected or imported row (or one edited from
///    one) is tombstoned in `model_dismissed` with the record as detected,
///    so the detection stages neither write it again nor let a re-detected
///    copy of it come back (see the identity helpers below). Its wall joins /
///    space boundaries go with it; openings on a removed wall stay and are
///    reported in ModelEditResult::warnings (re-host or remove them in the
///    same batch).
///  - revert ("Tilbage til detekteret"): a row edited from a detected /
///    imported one gets `edited_from` back exactly (a detected row is then
///    unlocked again, so the next detection run replaces it); a row drawn by
///    hand is unlocked (the next run deletes it); a tombstoned element with
///    no row is written back as it was and its tombstone dropped.
///
/// Every batch, undo and redo raises ProjectDB::model_revision() once; the
/// whole-run writers (detection stages, imports, deletes) raise it too. A
/// batch may name the revision it was made against (`base_revision`) and is
/// refused with ModelEditConflict when the model moved on.
///
/// The log holds, per row a batch touched, its state before and after (the
/// record, its joins / boundaries, its tombstone), so undo writes the
/// "before" states back and redo the "after" ones. Both first check that the
/// rows still are as the edit left them (ModelEditConflict otherwise, e.g.
/// after a detection run replaced an unlocked row). A new batch drops the
/// redo branch.
///
/// Core-only (ProjectDB + nlohmann), like core/pose_versions.hpp, so rux,
/// the Qt client and ruxd's API library share it.

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/solid_model.hpp>

#include <nlohmann/json_fwd.hpp>

#include <cstdint>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

namespace reusex::core {

enum class ModelElementKind { storey, wall, opening, space, zone, slab };

/// "storey" | "wall" | "opening" | "space" | "zone" | "slab".
std::string_view to_string(ModelElementKind kind);
/// @throws std::invalid_argument for anything else.
ModelElementKind parse_model_element_kind(std::string_view text);

/// metadata key set on every row an edit wrote (true).
inline constexpr std::string_view kModelMetaEdited = "edited";
/// metadata key holding the detected / imported record an edited row was
/// made from (core::model_record_json()).
inline constexpr std::string_view kModelMetaEditedFrom = "edited_from";

/// One op of a batch. Build it with put() / remove() / revert().
struct ModelEditOp {
  enum class Action { put, remove, revert };
  Action action = Action::put;
  ModelElementKind kind = ModelElementKind::wall;
  /// remove / revert: the row's guid (put: the record's).
  std::string guid;
  std::variant<std::monostate, StoreyRecord, WallRecord, OpeningRecord,
               SpaceRecord, ZoneRecord, SlabRecord>
      record;
  /// put wall: replace the joins touching it (each must name it); nullopt
  /// keeps the stored ones.
  std::optional<std::vector<WallJoinRecord>> joins;
  /// put space: replace its boundaries; nullopt keeps the stored ones.
  std::optional<std::vector<SpaceBoundaryRecord>> boundaries;
  /// Notes from the wire op this op was expanded from (e.g. a join a move
  /// had to drop); apply_model_edit() passes them on in
  /// ModelEditResult::warnings.
  std::vector<std::string> warnings;
  /// put: write the record exactly as given, without the edit marks
  /// (manual, locked, metadata.edited): bookkeeping a tool does to rows the
  /// user did not edit, e.g. renumbering the storeys above a split.
  bool exact = false;

  static ModelEditOp put(StoreyRecord storey);
  static ModelEditOp
  put(WallRecord wall,
      std::optional<std::vector<WallJoinRecord>> joins = std::nullopt);
  static ModelEditOp put(OpeningRecord opening);
  static ModelEditOp put(SpaceRecord space,
                         std::optional<std::vector<SpaceBoundaryRecord>>
                             boundaries = std::nullopt);
  /// A zone's members must be stored spaces once the batch's earlier ops
  /// are applied (apply_model_edit() refuses the batch otherwise).
  static ModelEditOp put(ZoneRecord zone);
  static ModelEditOp put(SlabRecord slab);
  static ModelEditOp remove(ModelElementKind kind, std::string guid);
  static ModelEditOp revert(ModelElementKind kind, std::string guid);
};

struct ModelEditOptions {
  std::string label;  ///< shown in the history, e.g. "Flyt væg"
  std::string author; ///< who made it (ruxd: the user)
  /// The model_revision() the edit was made against; nullopt = do not check.
  std::optional<int64_t> base_revision;
};

/// One row a batch (or an undo / redo) changed.
struct ModelChange {
  ModelElementKind kind = ModelElementKind::wall;
  std::string guid;
  /// created | updated | deleted | reverted | restored
  std::string action;
};

struct ModelEditResult {
  int64_t revision = 0; ///< model_revision() afterwards
  int64_t edit_id = -1; ///< the log entry applied, undone or redone
  std::vector<ModelChange> changes;
  /// Things left for the caller to fix, e.g. an opening whose host wall the
  /// batch removed, or an opening put on a wall that is not stored, and
  /// each misfit the batch introduced (an opening past its wall's ends,
  /// above its top, or overlapping another; core/opening_fit.hpp -- a
  /// misfit the model already had is not repeated).
  std::vector<std::string> warnings;
};

/// A batch made against a stale revision, or an undo / redo whose rows were
/// changed since (by another edit or a detection run). ruxd answers 409.
class ModelEditConflict : public std::runtime_error {
    public:
  ModelEditConflict(const std::string &what, int64_t current_revision)
      : std::runtime_error(what), current_revision_(current_revision) {}
  [[nodiscard]] int64_t current_revision() const noexcept {
    return current_revision_;
  }

    private:
  int64_t current_revision_;
};

/// Apply @p ops in order, as one transaction (its own, or the caller's when
/// one is open: then a throw leaves the caller to roll back) and one log
/// entry. Rows a batch leaves as they were are not listed.
/// @throws ModelEditConflict for a stale base_revision;
///         std::invalid_argument for an op a row cannot take (remove /
///         revert of an unknown guid, an op without its record, a record the
///         ProjectDB writers refuse); std::runtime_error on a read-only or
///         pre-v41 project.
ModelEditResult apply_model_edit(ProjectDB &db,
                                 const std::vector<ModelEditOp> &ops,
                                 const ModelEditOptions &options = {});

/// Undo the newest applied edit / redo the oldest undone one.
/// @throws std::runtime_error when there is nothing to undo / redo;
///         ModelEditConflict for a stale base_revision, or when a row the
///         edit touched is no longer as the edit (or its undo) left it.
ModelEditResult undo_model_edit(ProjectDB &db,
                                const ModelEditOptions &options = {});
ModelEditResult redo_model_edit(ProjectDB &db,
                                const ModelEditOptions &options = {});

/// One entry of the edit log.
struct ModelEditEntry {
  int64_t id = -1;
  int64_t revision = 0; ///< the revision the edit produced when applied
  std::string label;
  std::string author;
  std::string created_at;
  bool undone = false;
  std::vector<ModelChange> changes;
};

/// The log, oldest first; entries with id > @p after_id.
std::vector<ModelEditEntry> model_edit_history(const ProjectDB &db,
                                               int64_t after_id = 0);
bool can_undo_model_edit(const ProjectDB &db);
bool can_redo_model_edit(const ProjectDB &db);

// --- JSON (records as the log and ruxd carry them) --------------------------

/// Every field but `id`; `metadata` stays text. Doubles round-trip exactly.
nlohmann::json model_record_json(const StoreyRecord &r);
nlohmann::json model_record_json(const WallRecord &r);
nlohmann::json model_record_json(const OpeningRecord &r);
nlohmann::json model_record_json(const SpaceRecord &r);
nlohmann::json model_record_json(const ZoneRecord &r);
nlohmann::json model_record_json(const SlabRecord &r);
/// The inverse; a missing field keeps the record's default, so
/// `wall_from_json(model_record_json(w).merge_patch(partial))` applies a
/// partial update. @throws std::invalid_argument for an unknown key or a
/// value of the wrong type.
StoreyRecord storey_from_json(const nlohmann::json &j);
WallRecord wall_from_json(const nlohmann::json &j);
OpeningRecord opening_from_json(const nlohmann::json &j);
SpaceRecord space_from_json(const nlohmann::json &j);
ZoneRecord zone_from_json(const nlohmann::json &j);
SlabRecord slab_from_json(const nlohmann::json &j);

/// Ops in the ruxd wire form, read against @p db's current rows:
///   {"op": "<kind>.create", "record": {...}}             (put; guid optional)
///   {"op": "<kind>.update", "guid": g, "set": {...}}     (partial put)
///   {"op": "<kind>.delete", "guid": g}                   (remove)
///   {"op": "<kind>.revert", "guid": g}                   (revert)
///   {"op": "ceiling.set", "guid": <space>, "planes": [{at, slope}, ...]}
///   {"op": "wall.split", "guid": g, "s": metres, "new_guid"?: g2}
///   {"op": "wall.merge", "guids": [a, b]}     (b is removed)
///   {"op": "wall.join", "guids": [a, b]}      (extend / trim to a corner)
///   {"op": "wall.extend_to", "guid": g, "end": "start" | "end",
///    "target": t}            (extend / trim g's end to t: corner or tee)
///   {"op": "selection.transform", "guids": [...], "translate"?: [dx, dy],
///    "rotate_deg"?: deg, "pivot"?: [x, y], "copy"?: bool,
///    "new_guids"?: {old: new}}   (move / rotate / copy walls, openings,
///                                 spaces as one body)
///   {"op": "wall.offset", "guid": g, "distance": m, "copy"?: bool (true),
///    "new_guid"?: g2}   (move / copy g sideways; + = left of start -> end)
///   {"op": "storey.update", "guid": g, "set": {...}}
///   {"op": "storey.split", "guid": g, "z": level z, "new_guid"?: g2,
///    "name"?: text}   (g keeps [z_floor, z), a new storey g2 above it
///                     takes [z, z_ceiling), idx g.idx + 1, g's level
///                     frame; the storeys above move up one idx; g's walls
///                     / spaces / slabs whose base is at or above z move
///                     to g2)
///   {"op": "storey.merge", "guids": [a, b]}   (a spans both; b's walls /
///                     spaces / slabs move to a, b is removed, the
///                     storeys above b move down one idx; both must share
///                     one level frame)
/// with <kind> = wall | opening | space | storey | zone | slab (a slab's
/// record: storey, kind floor | ceiling | roof, outline {outer, holes},
/// z_top, thickness, top_planes; a zone's record:
/// name, use, spaces = member space guids; a space delete also drops it
/// from every zone, logged as an update of that zone, so undo restores it).
/// "joins" (wall) and "boundaries" (space) may ride along on create / update.
/// An update of a row created earlier in the same list patches that one. split
/// / merge / join / extend_to / selection.transform / wall.offset expand into
/// puts of the walls, openings and spaces they change and of the copies they
/// make (core/wall_edit_ops.hpp: extend_wall_to(), transform_selection()),
/// reading openings, spaces, joins and boundaries as stored; a transform
/// whose join to an unmoved wall had to be dropped says so in
/// ModelEditOp::warnings. rotate_deg is counter-clockwise about pivot
/// (default [0, 0]), applied before translate.
/// @throws std::invalid_argument for an unknown op, a missing guid, an
///         update of an unknown row or a bad record.
std::vector<ModelEditOp> model_edit_ops_from_json(const ProjectDB &db,
                                                  const nlohmann::json &ops);

nlohmann::json to_json(const ModelChange &change);
nlohmann::json to_json(const ModelEditResult &result);
nlohmann::json to_json(const ModelEditEntry &entry);

// --- Detection side: keep edits across a re-run -----------------------------

/// The guids a detection run must not write for one element kind.
struct ModelHolds {
  std::set<std::string, std::less<>> dismissed; ///< tombstoned
  std::set<std::string, std::less<>> edited;    ///< locked rows an edit wrote
  [[nodiscard]] bool holds(std::string_view guid) const {
    return dismissed.count(guid) > 0 || edited.count(guid) > 0;
  }
};
ModelHolds model_holds(const ProjectDB &db, ModelElementKind kind);

/// What a re-detected element is matched against (assign_*_guids'
/// `stored`): the stored rows, plus, under the same guid, each edited row's
/// `edited_from` record (where detection will find it again) and each
/// tombstone's record. A re-detected element that lands on one of the
/// latter takes its guid, and drop_held_*() then drops it: the edited row
/// stays as edited, the deleted one stays deleted. The extra records come
/// unlocked with source "detected", so no name or lock is inherited from
/// them.
std::vector<WallRecord> wall_identity_candidates(const ProjectDB &db);
std::vector<OpeningRecord> opening_identity_candidates(const ProjectDB &db);
std::vector<SpaceRecord> space_identity_candidates(const ProjectDB &db);

/// Drop the freshly detected records whose guid @p holds holds (and the
/// joins / boundaries that name a dismissed one, or a dropped space).
/// @returns how many records were dropped.
size_t drop_held_walls(std::vector<WallRecord> &walls,
                       std::vector<WallJoinRecord> &joins,
                       const ModelHolds &holds);
size_t drop_held_openings(std::vector<OpeningRecord> &openings,
                          const ModelHolds &holds);
size_t drop_held_spaces(std::vector<SpaceRecord> &spaces,
                        std::vector<SpaceBoundaryRecord> &boundaries,
                        const ModelHolds &holds);

} // namespace reusex::core
