// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/model_edits.hpp"

#include "core/guid.hpp"
#include "core/logging.hpp"

#include <fmt/format.h>
#include <fmt/ranges.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <map>
#include <set>
#include <stdexcept>
#include <tuple>
#include <type_traits>
#include <utility>

namespace reusex::core {

using json = nlohmann::json;

std::string_view to_string(ModelElementKind kind) {
  switch (kind) {
  case ModelElementKind::storey:
    return "storey";
  case ModelElementKind::wall:
    return "wall";
  case ModelElementKind::opening:
    return "opening";
  case ModelElementKind::space:
    return "space";
  }
  return "wall";
}

ModelElementKind parse_model_element_kind(std::string_view text) {
  if (text == "storey")
    return ModelElementKind::storey;
  if (text == "wall")
    return ModelElementKind::wall;
  if (text == "opening")
    return ModelElementKind::opening;
  if (text == "space")
    return ModelElementKind::space;
  throw std::invalid_argument("unknown model element kind '" +
                              std::string(text) +
                              "' (storey | wall | opening | space)");
}

// --- ModelEditOp factories ---------------------------------------------------

ModelEditOp ModelEditOp::put(StoreyRecord storey) {
  ModelEditOp op;
  op.kind = ModelElementKind::storey;
  op.guid = storey.guid;
  op.record = std::move(storey);
  return op;
}

ModelEditOp ModelEditOp::put(WallRecord wall,
                             std::optional<std::vector<WallJoinRecord>> joins) {
  ModelEditOp op;
  op.kind = ModelElementKind::wall;
  op.guid = wall.guid;
  op.record = std::move(wall);
  op.joins = std::move(joins);
  return op;
}

ModelEditOp ModelEditOp::put(OpeningRecord opening) {
  ModelEditOp op;
  op.kind = ModelElementKind::opening;
  op.guid = opening.guid;
  op.record = std::move(opening);
  return op;
}

ModelEditOp
ModelEditOp::put(SpaceRecord space,
                 std::optional<std::vector<SpaceBoundaryRecord>> boundaries) {
  ModelEditOp op;
  op.kind = ModelElementKind::space;
  op.guid = space.guid;
  op.record = std::move(space);
  op.boundaries = std::move(boundaries);
  return op;
}

ModelEditOp ModelEditOp::remove(ModelElementKind kind, std::string guid) {
  ModelEditOp op;
  op.action = Action::remove;
  op.kind = kind;
  op.guid = std::move(guid);
  return op;
}

ModelEditOp ModelEditOp::revert(ModelElementKind kind, std::string guid) {
  ModelEditOp op;
  op.action = Action::revert;
  op.kind = kind;
  op.guid = std::move(guid);
  return op;
}

// --- Record JSON -------------------------------------------------------------

namespace {

json points_json(const std::vector<PlanPoint> &pts) {
  json a = json::array();
  for (const auto &p : pts)
    a.push_back({p[0], p[1]});
  return a;
}

/// Field reader for *_from_json: every key of the object must be claimed.
class Fields {
    public:
  Fields(const json &j, const char *what) : j_(j), what_(what) {
    if (!j.is_object())
      throw std::invalid_argument(std::string(what) +
                                  ": a record must be a JSON object");
  }

  template <typename T> void get(const char *key, T &out) {
    seen_.insert(key);
    const auto it = j_.find(key);
    if (it == j_.end() || it->is_null())
      return;
    try {
      out = it->template get<T>();
    } catch (const json::exception &e) {
      throw std::invalid_argument(std::string(what_) + "." + key + ": " +
                                  e.what());
    }
  }

  void points(const char *key, std::vector<PlanPoint> &out) {
    seen_.insert(key);
    const auto it = j_.find(key);
    if (it == j_.end() || it->is_null())
      return;
    out = read_points(*it, key);
  }

  std::vector<PlanPoint> read_points(const json &a, const std::string &key) {
    if (!a.is_array())
      throw std::invalid_argument(std::string(what_) + "." + key +
                                  ": expected an array of [x, y]");
    std::vector<PlanPoint> pts;
    for (const auto &p : a) {
      if (!p.is_array() || p.size() != 2 || !p[0].is_number() ||
          !p[1].is_number())
        throw std::invalid_argument(std::string(what_) + "." + key +
                                    ": expected [x, y] number pairs");
      pts.push_back({p[0].get<double>(), p[1].get<double>()});
    }
    return pts;
  }

  const json *raw(const char *key) {
    seen_.insert(key);
    const auto it = j_.find(key);
    return it == j_.end() || it->is_null() ? nullptr : &*it;
  }

  void ignore(const char *key) { seen_.insert(key); }

  void done() const {
    for (const auto &[key, value] : j_.items())
      if (!seen_.count(key))
        throw std::invalid_argument(std::string(what_) + ": unknown field '" +
                                    key + "'");
  }

    private:
  const json &j_;
  const char *what_;
  std::set<std::string, std::less<>> seen_;
};

template <typename R> void common_json(json &j, const R &r) {
  j["source"] = r.source;
  j["locked"] = r.locked;
  j["confidence"] = r.confidence;
  j["metadata"] = r.metadata;
}

template <typename R> void common_from(Fields &f, R &r) {
  f.ignore("id");
  f.get("guid", r.guid);
  f.get("source", r.source);
  f.get("locked", r.locked);
  f.get("confidence", r.confidence);
  f.get("metadata", r.metadata);
}

} // namespace

json model_record_json(const StoreyRecord &r) {
  json j;
  j["guid"] = r.guid;
  j["idx"] = r.idx;
  j["name"] = r.name;
  j["z_floor"] = r.z_floor;
  j["z_ceiling"] = r.z_ceiling;
  j["level_transform"] = r.level_transform;
  common_json(j, r);
  return j;
}

json model_record_json(const WallRecord &r) {
  json j;
  j["guid"] = r.guid;
  j["storey"] = r.storey;
  j["x0"] = r.x0;
  j["y0"] = r.y0;
  j["x1"] = r.x1;
  j["y1"] = r.y1;
  j["z_base"] = r.z_base;
  j["height"] = r.height;
  j["thickness"] = r.thickness;
  j["thickness_ci_lo"] = r.thickness_ci_lo;
  j["thickness_ci_hi"] = r.thickness_ci_hi;
  j["lean_deg"] = r.lean_deg;
  j["is_external"] = r.is_external;
  j["assumed"] = r.assumed;
  j["seen_share"] = r.seen_share;
  j["face_a_points"] = r.face_a_points;
  j["face_b_points"] = r.face_b_points;
  j["co_visibility"] = r.co_visibility;
  j["axis_family"] = r.axis_family;
  j["top"] = points_json(r.top);
  common_json(j, r);
  return j;
}

json model_record_json(const OpeningRecord &r) {
  json j;
  j["guid"] = r.guid;
  j["wall"] = r.wall;
  j["kind"] = r.kind;
  j["s_start"] = r.s_start;
  j["s_end"] = r.s_end;
  j["z_sill"] = r.z_sill;
  j["z_head"] = r.z_head;
  j["state"] = r.state;
  j["is_external"] = r.is_external;
  j["source_instance_guid"] = r.source_instance_guid;
  common_json(j, r);
  return j;
}

json model_record_json(const SpaceRecord &r) {
  json j;
  j["guid"] = r.guid;
  j["storey"] = r.storey;
  j["name"] = r.name;
  j["kind"] = r.kind;
  json holes = json::array();
  for (const auto &h : r.polygon.holes)
    holes.push_back(points_json(h));
  j["polygon"] = {{"outer", points_json(r.polygon.outer)}, {"holes", holes}};
  j["z_floor"] = r.z_floor;
  j["z_ceiling"] = r.z_ceiling;
  json planes = json::array();
  for (const auto &p : r.ceiling_planes)
    planes.push_back({{"at", p.at}, {"slope", p.slope}});
  j["ceiling_planes"] = planes;
  j["area"] = r.area;
  j["perimeter"] = r.perimeter;
  j["volume"] = r.volume;
  j["compactness"] = r.compactness;
  j["room_label"] = r.room_label;
  common_json(j, r);
  return j;
}

StoreyRecord storey_from_json(const json &j) {
  StoreyRecord r;
  Fields f(j, "storey");
  common_from(f, r);
  f.get("idx", r.idx);
  f.get("name", r.name);
  f.get("z_floor", r.z_floor);
  f.get("z_ceiling", r.z_ceiling);
  f.get("level_transform", r.level_transform);
  f.done();
  return r;
}

WallRecord wall_from_json(const json &j) {
  WallRecord r;
  Fields f(j, "wall");
  common_from(f, r);
  f.get("storey", r.storey);
  f.get("x0", r.x0);
  f.get("y0", r.y0);
  f.get("x1", r.x1);
  f.get("y1", r.y1);
  f.get("z_base", r.z_base);
  f.get("height", r.height);
  f.get("thickness", r.thickness);
  f.get("thickness_ci_lo", r.thickness_ci_lo);
  f.get("thickness_ci_hi", r.thickness_ci_hi);
  f.get("lean_deg", r.lean_deg);
  f.get("is_external", r.is_external);
  f.get("assumed", r.assumed);
  f.get("seen_share", r.seen_share);
  f.get("face_a_points", r.face_a_points);
  f.get("face_b_points", r.face_b_points);
  f.get("co_visibility", r.co_visibility);
  f.get("axis_family", r.axis_family);
  f.points("top", r.top);
  f.done();
  return r;
}

OpeningRecord opening_from_json(const json &j) {
  OpeningRecord r;
  Fields f(j, "opening");
  common_from(f, r);
  f.get("wall", r.wall);
  f.get("kind", r.kind);
  f.get("s_start", r.s_start);
  f.get("s_end", r.s_end);
  f.get("z_sill", r.z_sill);
  f.get("z_head", r.z_head);
  f.get("state", r.state);
  f.get("is_external", r.is_external);
  f.get("source_instance_guid", r.source_instance_guid);
  f.done();
  return r;
}

SpaceRecord space_from_json(const json &j) {
  SpaceRecord r;
  Fields f(j, "space");
  common_from(f, r);
  f.get("storey", r.storey);
  f.get("name", r.name);
  f.get("kind", r.kind);
  if (const json *poly = f.raw("polygon")) {
    if (!poly->is_object())
      throw std::invalid_argument(
          "space.polygon: expected {\"outer\": [...], \"holes\": [...]}");
    for (const auto &[key, value] : poly->items())
      if (key != "outer" && key != "holes")
        throw std::invalid_argument("space.polygon: unknown field '" + key +
                                    "'");
    r.polygon = {};
    if (const auto it = poly->find("outer"); it != poly->end())
      r.polygon.outer = f.read_points(*it, "polygon.outer");
    if (const auto it = poly->find("holes"); it != poly->end()) {
      if (!it->is_array())
        throw std::invalid_argument("space.polygon.holes: expected an array");
      for (const auto &h : *it)
        r.polygon.holes.push_back(f.read_points(h, "polygon.holes"));
    }
  }
  f.get("z_floor", r.z_floor);
  f.get("z_ceiling", r.z_ceiling);
  if (const json *planes = f.raw("ceiling_planes")) {
    if (!planes->is_array())
      throw std::invalid_argument("space.ceiling_planes: expected an array");
    for (const auto &p : *planes) {
      if (!p.is_object() || !p.contains("at") || !p.contains("slope"))
        throw std::invalid_argument(
            "space.ceiling_planes: expected {\"at\": [x, y, z], \"slope\": "
            "[dx, dy]}");
      CeilingPlane c;
      try {
        c.at = p.at("at").get<std::array<double, 3>>();
        c.slope = p.at("slope").get<std::array<double, 2>>();
      } catch (const json::exception &e) {
        throw std::invalid_argument(std::string("space.ceiling_planes: ") +
                                    e.what());
      }
      r.ceiling_planes.push_back(c);
    }
  }
  f.get("area", r.area);
  f.get("perimeter", r.perimeter);
  f.get("volume", r.volume);
  f.get("compactness", r.compactness);
  f.get("room_label", r.room_label);
  f.done();
  return r;
}

// --- Row state ---------------------------------------------------------------

namespace {

json join_json(const WallJoinRecord &j) {
  return {{"wall_a", j.wall_a},
          {"wall_b", j.wall_b},
          {"kind", j.kind},
          {"at_a", j.at_a},
          {"at_b", j.at_b}};
}

WallJoinRecord join_from(const json &j) {
  WallJoinRecord r;
  r.wall_a = j.at("wall_a").get<std::string>();
  r.wall_b = j.at("wall_b").get<std::string>();
  r.kind = j.at("kind").get<std::string>();
  r.at_a = j.at("at_a").get<std::string>();
  r.at_b = j.at("at_b").get<std::string>();
  return r;
}

json boundary_json(const SpaceBoundaryRecord &b) {
  return {{"space", b.space},
          {"wall", b.wall},
          {"side", b.side},
          {"s_start", b.s_start},
          {"s_end", b.s_end}};
}

SpaceBoundaryRecord boundary_from(const json &j) {
  SpaceBoundaryRecord b;
  b.space = j.at("space").get<std::string>();
  b.wall = j.at("wall").get<std::string>();
  b.side = j.at("side").get<std::string>();
  b.s_start = j.at("s_start").get<double>();
  b.s_end = j.at("s_end").get<double>();
  return b;
}

/// One row's state: its record and relations (null: no row) and its
/// tombstone record (null: not dismissed).
struct RowState {
  json row;  ///< {"record": {...}, "joins" | "boundaries": [...]} or null
  json tomb; ///< tombstone record or null
};

json parse_text_object(std::string_view text) {
  if (text.empty())
    return json::object();
  json j = json::parse(text, nullptr, false);
  if (j.is_discarded() || !j.is_object())
    return json{{"text", std::string(text)}};
  return j;
}

RowState capture(const ProjectDB &db, ModelElementKind kind,
                 const std::string &guid) {
  RowState st;
  switch (kind) {
  case ModelElementKind::storey:
    if (auto r = db.storey_by_guid(guid))
      st.row = {{"record", model_record_json(*r)}};
    break;
  case ModelElementKind::wall:
    if (auto r = db.wall_by_guid(guid)) {
      json joins = json::array();
      for (const auto &j : db.wall_joins())
        if (j.wall_a == guid || j.wall_b == guid)
          joins.push_back(join_json(j));
      st.row = {{"record", model_record_json(*r)}, {"joins", joins}};
    }
    break;
  case ModelElementKind::opening:
    if (auto r = db.opening_by_guid(guid))
      st.row = {{"record", model_record_json(*r)}};
    break;
  case ModelElementKind::space:
    if (auto r = db.space_by_guid(guid)) {
      json bs = json::array();
      for (const auto &b : db.space_boundaries())
        if (b.space == guid)
          bs.push_back(boundary_json(b));
      st.row = {{"record", model_record_json(*r)}, {"boundaries", bs}};
    }
    break;
  }
  if (auto d = db.dismissed_model_element(guid))
    st.tomb = parse_text_object(d->record);
  return st;
}

/// Same record and same tombstone presence (relations not compared: a
/// detection run may re-join a locked wall without changing it).
bool same_state(const RowState &a, const RowState &b) {
  const bool rows =
      a.row.is_null() == b.row.is_null() &&
      (a.row.is_null() || a.row.at("record") == b.row.at("record"));
  return rows && a.tomb.is_null() == b.tomb.is_null();
}

void put_record_exact(ProjectDB &db, ModelElementKind kind, const json &rec) {
  switch (kind) {
  case ModelElementKind::storey:
    db.put_storey_row(storey_from_json(rec));
    break;
  case ModelElementKind::wall:
    db.put_wall_row(wall_from_json(rec));
    break;
  case ModelElementKind::opening:
    db.put_opening_row(opening_from_json(rec));
    break;
  case ModelElementKind::space:
    db.put_space_row(space_from_json(rec));
    break;
  }
}

bool remove_row(ProjectDB &db, ModelElementKind kind, const std::string &g) {
  switch (kind) {
  case ModelElementKind::storey:
    return db.remove_storey_row(g);
  case ModelElementKind::wall:
    return db.remove_wall_row(g);
  case ModelElementKind::opening:
    return db.remove_opening_row(g);
  case ModelElementKind::space:
    return db.remove_space_row(g);
  }
  return false;
}

/// Write @p st's record and tombstone (pass 1; relations in pass 2, once
/// every row of the step exists).
void restore_row(ProjectDB &db, ModelElementKind kind, const std::string &g,
                 const RowState &st) {
  if (st.row.is_null())
    remove_row(db, kind, g);
  else
    put_record_exact(db, kind, st.row.at("record"));
  if (st.tomb.is_null())
    db.undismiss_model_element(g);
  else
    db.dismiss_model_element(to_string(kind), g, st.tomb.dump());
}

void restore_relations(ProjectDB &db, ModelElementKind kind,
                       const std::string &g, const RowState &st) {
  if (st.row.is_null())
    return;
  if (kind == ModelElementKind::wall && st.row.contains("joins")) {
    std::vector<WallJoinRecord> joins;
    for (const auto &j : st.row.at("joins"))
      joins.push_back(join_from(j));
    db.replace_wall_joins_of(g, joins);
  } else if (kind == ModelElementKind::space && st.row.contains("boundaries")) {
    std::vector<SpaceBoundaryRecord> bs;
    for (const auto &b : st.row.at("boundaries"))
      bs.push_back(boundary_from(b));
    db.replace_space_boundaries_of(g, bs);
  }
}

std::string transition(const RowState &from, const RowState &to) {
  if (from.row.is_null() && !to.row.is_null())
    return "created";
  if (!from.row.is_null() && to.row.is_null())
    return "deleted";
  return "updated";
}

/// The record's metadata with the edit marks: `edited`, and `edited_from`
/// kept from @p stored (or taken from it, when it was detected / imported).
template <typename R>
std::string edited_metadata(const R &rec, const std::optional<R> &stored) {
  json meta = parse_text_object(rec.metadata);
  const std::string from_key(kModelMetaEditedFrom);
  if (!meta.contains(from_key) && stored) {
    const json was = parse_text_object(stored->metadata);
    if (was.contains(from_key))
      meta[from_key] = was.at(from_key);
    else if (stored->source != "manual")
      meta[from_key] = model_record_json(*stored);
  }
  meta[std::string(kModelMetaEdited)] = true;
  return meta.dump();
}

template <typename R> R mark_edited(R rec, const std::optional<R> &stored) {
  rec.metadata = edited_metadata(rec, stored);
  if (stored)
    rec.id = stored->id;
  rec.source = "manual";
  rec.locked = true;
  return rec;
}

/// The row as typed record (nullopt: none).
template <typename R>
std::optional<R> stored_as(const ProjectDB &db, const std::string &g);
template <>
std::optional<StoreyRecord> stored_as(const ProjectDB &db,
                                      const std::string &g) {
  return db.storey_by_guid(g);
}
template <>
std::optional<WallRecord> stored_as(const ProjectDB &db, const std::string &g) {
  return db.wall_by_guid(g);
}
template <>
std::optional<OpeningRecord> stored_as(const ProjectDB &db,
                                       const std::string &g) {
  return db.opening_by_guid(g);
}
template <>
std::optional<SpaceRecord> stored_as(const ProjectDB &db,
                                     const std::string &g) {
  return db.space_by_guid(g);
}

json row_record(const RowState &st) {
  return st.row.is_null() ? json() : st.row.at("record");
}

/// The record a tombstone keeps: what detection (or the import) made,
/// i.e. `edited_from` when the row was edited, else the row itself.
json tomb_record(const json &rec) {
  const json meta = parse_text_object(rec.value("metadata", std::string()));
  const std::string from_key(kModelMetaEditedFrom);
  if (meta.contains(from_key) && meta.at(from_key).is_object())
    return meta.at(from_key);
  return rec;
}

/// Whether removing this row must leave a tombstone: it came from
/// detection or an import (a row drawn by hand would otherwise suppress a
/// real wall detected where it was).
bool needs_tomb(const json &rec) {
  if (rec.value("source", std::string()) != "manual")
    return true;
  const json meta = parse_text_object(rec.value("metadata", std::string()));
  return meta.contains(std::string(kModelMetaEditedFrom));
}

class Writer {
    public:
  explicit Writer(ProjectDB &db) : db_(db) {
    if (!db_.in_transaction())
      tx_.emplace(db_);
  }
  void commit() {
    if (tx_)
      tx_->commit();
  }

    private:
  ProjectDB &db_;
  std::optional<ProjectDB::Transaction> tx_;
};

void check_base(const ProjectDB &db, const ModelEditOptions &o,
                const char *what) {
  const int64_t current = db.model_revision();
  if (o.base_revision && *o.base_revision != current)
    throw ModelEditConflict(fmt::format("{}: made against model revision {}, "
                                        "the model is at revision {}",
                                        what, *o.base_revision, current),
                            current);
}

struct Key {
  ModelElementKind kind;
  std::string guid;
  bool operator<(const Key &o) const {
    return std::tie(kind, guid) < std::tie(o.kind, o.guid);
  }
};

json change_json(const Key &k, const std::string &action, const RowState &a,
                 const RowState &b) {
  return {
      {"kind", to_string(k.kind)}, {"guid", k.guid}, {"action", action},
      {"before", a.row},           {"after", b.row}, {"tomb_before", a.tomb},
      {"tomb_after", b.tomb}};
}

} // namespace

ModelEditResult apply_model_edit(ProjectDB &db,
                                 const std::vector<ModelEditOp> &ops,
                                 const ModelEditOptions &options) {
  check_base(db, options, "model edit");
  if (ops.empty())
    throw std::invalid_argument("model edit: no ops");
  Writer w(db);

  std::vector<Key> order;
  std::map<Key, RowState> before;
  std::map<Key, std::string> actions;
  const auto touch = [&](const Key &k) {
    if (!before.count(k)) {
      before.emplace(k, capture(db, k.kind, k.guid));
      order.push_back(k);
    }
  };
  struct Relation {
    Key key;
    const ModelEditOp *op;
  };
  std::vector<Relation> relations;
  std::set<std::string> removed_walls;

  for (const auto &op : ops) {
    switch (op.action) {
    case ModelEditOp::Action::put: {
      Key k{op.kind, op.guid};
      const auto put = [&](auto rec) {
        using R = decltype(rec);
        if (rec.guid.empty())
          rec.guid = generate_guid();
        k.guid = rec.guid;
        touch(k);
        const auto stored = stored_as<R>(db, rec.guid);
        rec = mark_edited(std::move(rec), stored);
        if constexpr (std::is_same_v<R, StoreyRecord>)
          db.put_storey_row(rec);
        else if constexpr (std::is_same_v<R, WallRecord>)
          db.put_wall_row(rec);
        else if constexpr (std::is_same_v<R, OpeningRecord>)
          db.put_opening_row(rec);
        else
          db.put_space_row(rec);
        db.undismiss_model_element(rec.guid);
        actions[k] = stored ? "updated" : "created";
      };
      std::visit(
          [&](const auto &rec) {
            using R = std::decay_t<decltype(rec)>;
            if constexpr (std::is_same_v<R, std::monostate>)
              throw std::invalid_argument(
                  fmt::format("model edit: put {} {} carries no record",
                              to_string(op.kind), op.guid));
            else
              put(rec);
          },
          op.record);
      if (op.joins || op.boundaries)
        relations.push_back({k, &op});
      removed_walls.erase(k.guid);
      break;
    }
    case ModelEditOp::Action::remove: {
      const Key k{op.kind, op.guid};
      touch(k);
      const RowState now = capture(db, k.kind, k.guid);
      if (now.row.is_null())
        throw std::invalid_argument(fmt::format(
            "model edit: no {} {} to delete", to_string(op.kind), op.guid));
      const json rec = row_record(now);
      remove_row(db, k.kind, k.guid);
      if (needs_tomb(rec))
        db.dismiss_model_element(to_string(k.kind), k.guid,
                                 tomb_record(rec).dump());
      if (k.kind == ModelElementKind::wall)
        removed_walls.insert(k.guid);
      actions[k] = "deleted";
      break;
    }
    case ModelEditOp::Action::revert: {
      const Key k{op.kind, op.guid};
      touch(k);
      const RowState now = capture(db, k.kind, k.guid);
      if (!now.row.is_null()) {
        json rec = row_record(now);
        json meta = parse_text_object(rec.value("metadata", std::string()));
        const std::string from_key(kModelMetaEditedFrom);
        if (meta.contains(from_key) && meta.at(from_key).is_object()) {
          json orig = meta.at(from_key);
          orig["guid"] = k.guid;
          put_record_exact(db, k.kind, orig);
        } else {
          meta.erase(std::string(kModelMetaEdited));
          rec["metadata"] = meta.empty() ? std::string() : meta.dump();
          rec["locked"] = false;
          put_record_exact(db, k.kind, rec);
        }
        actions[k] = "reverted";
      } else if (!now.tomb.is_null()) {
        json orig = now.tomb;
        orig["guid"] = k.guid;
        put_record_exact(db, k.kind, orig);
        db.undismiss_model_element(k.guid);
        actions[k] = "restored";
      } else {
        throw std::invalid_argument(fmt::format(
            "model edit: no {} {} to revert", to_string(op.kind), op.guid));
      }
      break;
    }
    }
  }

  for (const auto &rel : relations) {
    if (rel.op->joins) {
      std::vector<WallJoinRecord> joins = *rel.op->joins;
      db.replace_wall_joins_of(rel.key.guid, joins);
    }
    if (rel.op->boundaries) {
      std::vector<SpaceBoundaryRecord> bs = *rel.op->boundaries;
      for (auto &b : bs)
        if (b.space.empty())
          b.space = rel.key.guid;
      db.replace_space_boundaries_of(rel.key.guid, bs);
    }
  }

  ModelEditResult result;
  json changes = json::array();
  for (const auto &k : order) {
    const RowState after = capture(db, k.kind, k.guid);
    const RowState &was = before.at(k);
    if (same_state(was, after) && was.row == after.row)
      continue;
    const std::string action = actions.at(k) == "updated" ||
                                       actions.at(k) == "created" ||
                                       actions.at(k) == "deleted"
                                   ? transition(was, after)
                                   : actions.at(k);
    result.changes.push_back({k.kind, k.guid, action});
    changes.push_back(change_json(k, action, was, after));
  }

  // Openings left without their host.
  std::set<std::string> walls_now;
  for (const auto &wall : db.walls())
    walls_now.insert(wall.guid);
  for (const auto &o : db.openings()) {
    if (removed_walls.count(o.wall))
      result.warnings.push_back(
          fmt::format("opening {} is hosted on wall {}, which this edit "
                      "deleted; re-host or delete it",
                      o.guid, o.wall));
    else if (!walls_now.count(o.wall) &&
             before.count(Key{ModelElementKind::opening, o.guid}))
      result.warnings.push_back(fmt::format(
          "opening {} names wall {}, which is not stored", o.guid, o.wall));
  }

  db.delete_undone_model_edits();
  result.revision = db.bump_model_revision();
  ProjectDB::ModelEditRecord rec;
  rec.revision = result.revision;
  rec.label = options.label;
  rec.author = options.author;
  rec.changes = changes.dump();
  result.edit_id = db.add_model_edit(rec);
  w.commit();
  return result;
}

namespace {

struct LoggedChange {
  Key key;
  std::string action;
  RowState before, after;
};

std::vector<LoggedChange> logged_changes(const ProjectDB::ModelEditRecord &e) {
  std::vector<LoggedChange> out;
  const json changes = json::parse(e.changes);
  for (const auto &c : changes) {
    LoggedChange lc;
    lc.key = {parse_model_element_kind(c.at("kind").get<std::string>()),
              c.at("guid").get<std::string>()};
    lc.action = c.value("action", std::string());
    lc.before = {c.at("before"), c.at("tomb_before")};
    lc.after = {c.at("after"), c.at("tomb_after")};
    out.push_back(std::move(lc));
  }
  return out;
}

/// Write the @p to side of every change of @p e, after checking the rows
/// are at its @p from side.
ModelEditResult step(ProjectDB &db, const ProjectDB::ModelEditRecord &e,
                     bool undo, const ModelEditOptions &options) {
  const char *what = undo ? "undo" : "redo";
  check_base(db, options, what);
  Writer w(db);
  auto changes = logged_changes(e);
  std::vector<std::string> moved;
  for (const auto &c : changes) {
    const RowState &expect = undo ? c.after : c.before;
    if (!same_state(capture(db, c.key.kind, c.key.guid), expect))
      moved.push_back(fmt::format("{} {}", to_string(c.key.kind), c.key.guid));
  }
  if (!moved.empty())
    throw ModelEditConflict(fmt::format("{} of edit {} ({}): changed since: {}",
                                        what, e.id, e.label,
                                        fmt::join(moved, ", ")),
                            db.model_revision());

  if (undo)
    std::reverse(changes.begin(), changes.end());
  ModelEditResult result;
  for (const auto &c : changes)
    restore_row(db, c.key.kind, c.key.guid, undo ? c.before : c.after);
  for (const auto &c : changes) {
    const RowState &from = undo ? c.after : c.before;
    const RowState &to = undo ? c.before : c.after;
    restore_relations(db, c.key.kind, c.key.guid, to);
    result.changes.push_back(
        {c.key.kind, c.key.guid,
         undo || (c.action != "reverted" && c.action != "restored")
             ? transition(from, to)
             : c.action});
  }
  db.set_model_edit_undone(e.id, undo);
  result.revision = db.bump_model_revision();
  result.edit_id = e.id;
  w.commit();
  return result;
}

} // namespace

ModelEditResult undo_model_edit(ProjectDB &db,
                                const ModelEditOptions &options) {
  const auto edits = db.model_edits();
  const auto it = std::find_if(edits.rbegin(), edits.rend(),
                               [](const auto &e) { return !e.undone; });
  if (it == edits.rend())
    throw std::runtime_error("undo: no model edit to undo");
  return step(db, *it, true, options);
}

ModelEditResult redo_model_edit(ProjectDB &db,
                                const ModelEditOptions &options) {
  const auto edits = db.model_edits();
  const auto it = std::find_if(edits.begin(), edits.end(),
                               [](const auto &e) { return e.undone; });
  if (it == edits.end())
    throw std::runtime_error("redo: no undone model edit to redo");
  return step(db, *it, false, options);
}

bool can_undo_model_edit(const ProjectDB &db) {
  const auto edits = db.model_edits();
  return std::any_of(edits.begin(), edits.end(),
                     [](const auto &e) { return !e.undone; });
}

bool can_redo_model_edit(const ProjectDB &db) {
  const auto edits = db.model_edits();
  return std::any_of(edits.begin(), edits.end(),
                     [](const auto &e) { return e.undone; });
}

std::vector<ModelEditEntry> model_edit_history(const ProjectDB &db,
                                               int64_t after_id) {
  std::vector<ModelEditEntry> out;
  for (const auto &e : db.model_edits(after_id)) {
    ModelEditEntry entry;
    entry.id = e.id;
    entry.revision = e.revision;
    entry.label = e.label;
    entry.author = e.author;
    entry.created_at = e.created_at;
    entry.undone = e.undone;
    for (const auto &c : logged_changes(e))
      entry.changes.push_back({c.key.kind, c.key.guid, c.action});
    out.push_back(std::move(entry));
  }
  return out;
}

// --- Wire ops ----------------------------------------------------------------

std::vector<ModelEditOp> model_edit_ops_from_json(const ProjectDB &db,
                                                  const json &ops) {
  if (!ops.is_array())
    throw std::invalid_argument("model edit ops: expected an array");
  std::vector<ModelEditOp> out;
  // Records put earlier in this list, by guid: a later update patches them.
  std::map<std::string, json> pending;

  const auto current = [&](ModelElementKind kind,
                           const std::string &guid) -> json {
    if (const auto it = pending.find(guid); it != pending.end())
      return it->second;
    const RowState st = capture(db, kind, guid);
    if (st.row.is_null())
      throw std::invalid_argument(fmt::format(
          "model edit ops: no {} {} to update", to_string(kind), guid));
    return st.row.at("record");
  };
  const auto make_put = [&](ModelElementKind kind, const json &rec,
                            const json &op) {
    ModelEditOp put;
    switch (kind) {
    case ModelElementKind::storey:
      put = ModelEditOp::put(storey_from_json(rec));
      break;
    case ModelElementKind::wall: {
      std::optional<std::vector<WallJoinRecord>> joins;
      if (op.contains("joins")) {
        joins.emplace();
        for (const auto &j : op.at("joins"))
          joins->push_back(join_from(j));
      }
      put = ModelEditOp::put(wall_from_json(rec), std::move(joins));
      break;
    }
    case ModelElementKind::opening:
      put = ModelEditOp::put(opening_from_json(rec));
      break;
    case ModelElementKind::space: {
      std::optional<std::vector<SpaceBoundaryRecord>> bs;
      if (op.contains("boundaries")) {
        bs.emplace();
        for (const auto &b : op.at("boundaries")) {
          json bb = b;
          if (!bb.contains("space"))
            bb["space"] = rec.value("guid", std::string());
          bs->push_back(boundary_from(bb));
        }
      }
      put = ModelEditOp::put(space_from_json(rec), std::move(bs));
      break;
    }
    }
    return put;
  };

  for (const auto &op : ops) {
    if (!op.is_object() || !op.contains("op") || !op.at("op").is_string())
      throw std::invalid_argument(
          "model edit ops: each op needs an \"op\" string");
    const std::string name = op.at("op").get<std::string>();
    const auto dot = name.find('.');
    if (dot == std::string::npos)
      throw std::invalid_argument("model edit ops: unknown op '" + name + "'");
    const std::string head = name.substr(0, dot), verb = name.substr(dot + 1);
    const auto guid_of = [&]() {
      if (!op.contains("guid") || !op.at("guid").is_string() ||
          op.at("guid").get<std::string>().empty())
        throw std::invalid_argument("model edit ops: " + name +
                                    " needs a \"guid\"");
      return op.at("guid").get<std::string>();
    };
    try {
      if (head == "ceiling" && verb == "set") {
        const std::string g = guid_of();
        json rec = current(ModelElementKind::space, g);
        rec["ceiling_planes"] = op.value("planes", json::array());
        pending[g] = rec;
        out.push_back(make_put(ModelElementKind::space, rec, op));
        continue;
      }
      const ModelElementKind kind = parse_model_element_kind(head);
      if (verb == "create") {
        json rec = op.value("record", json::object());
        if (!rec.contains("guid") || rec.at("guid").get<std::string>().empty())
          rec["guid"] = generate_guid();
        pending[rec.at("guid").get<std::string>()] = rec;
        out.push_back(make_put(kind, rec, op));
      } else if (verb == "update") {
        const std::string g = guid_of();
        json rec = current(kind, g);
        if (op.contains("set")) {
          json patch = op.at("set");
          if (!patch.is_object())
            throw std::invalid_argument("\"set\" must be an object");
          patch.erase("guid");
          rec.merge_patch(patch);
        }
        pending[g] = rec;
        out.push_back(make_put(kind, rec, op));
      } else if (verb == "delete") {
        const std::string g = guid_of();
        pending.erase(g);
        out.push_back(ModelEditOp::remove(kind, g));
      } else if (verb == "revert") {
        const std::string g = guid_of();
        pending.erase(g);
        out.push_back(ModelEditOp::revert(kind, g));
      } else {
        throw std::invalid_argument("unknown verb '" + verb + "'");
      }
    } catch (const json::exception &e) {
      throw std::invalid_argument("model edit ops: " + name + ": " + e.what());
    }
  }
  return out;
}

json to_json(const ModelChange &c) {
  return {{"kind", to_string(c.kind)}, {"guid", c.guid}, {"action", c.action}};
}

json to_json(const ModelEditResult &r) {
  json changes = json::array();
  for (const auto &c : r.changes)
    changes.push_back(to_json(c));
  return {{"revision", r.revision},
          {"edit_id", r.edit_id},
          {"changes", changes},
          {"warnings", r.warnings}};
}

json to_json(const ModelEditEntry &e) {
  json changes = json::array();
  for (const auto &c : e.changes)
    changes.push_back(to_json(c));
  return {
      {"id", e.id},         {"revision", e.revision},     {"label", e.label},
      {"author", e.author}, {"created_at", e.created_at}, {"undone", e.undone},
      {"changes", changes}};
}

// --- Detection side ----------------------------------------------------------

namespace {

template <typename R> bool is_edited(const R &r) {
  if (!r.locked)
    return false;
  const json meta = parse_text_object(r.metadata);
  const auto it = meta.find(std::string(kModelMetaEdited));
  return it != meta.end() && it->is_boolean() && it->get<bool>();
}

template <typename R, typename FromJson>
std::vector<R>
candidates(const std::vector<R> &stored,
           const std::vector<ProjectDB::DismissedModelElement> &tombs,
           FromJson from_json, const char *what) {
  std::vector<R> out = stored;
  const auto add = [&](const json &rec, const std::string &guid) {
    try {
      json r = rec;
      r["guid"] = guid;
      R c = from_json(r);
      c.guid = guid;
      c.locked = false;
      c.source = "detected";
      c.id = -1;
      out.push_back(std::move(c));
    } catch (const std::exception &e) {
      reusex::warn("{} identity: the record kept for {} is unreadable ({}); "
                   "a re-detected copy of it is not recognised",
                   what, guid, e.what());
    }
  };
  for (const auto &s : stored) {
    if (!is_edited(s))
      continue;
    const json meta = parse_text_object(s.metadata);
    const auto it = meta.find(std::string(kModelMetaEditedFrom));
    if (it != meta.end() && it->is_object())
      add(*it, s.guid);
  }
  for (const auto &t : tombs) {
    if (t.record.empty())
      continue;
    add(parse_text_object(t.record), t.guid);
  }
  return out;
}

} // namespace

ModelHolds model_holds(const ProjectDB &db, ModelElementKind kind) {
  ModelHolds h;
  for (auto &d : db.dismissed_model_elements(to_string(kind)))
    h.dismissed.insert(std::move(d.guid));
  const auto add_edited = [&](const auto &rows) {
    for (const auto &r : rows)
      if (is_edited(r))
        h.edited.insert(r.guid);
  };
  switch (kind) {
  case ModelElementKind::storey:
    add_edited(db.storeys());
    break;
  case ModelElementKind::wall:
    add_edited(db.walls());
    break;
  case ModelElementKind::opening:
    add_edited(db.openings());
    break;
  case ModelElementKind::space:
    add_edited(db.spaces());
    break;
  }
  return h;
}

std::vector<WallRecord> wall_identity_candidates(const ProjectDB &db) {
  return candidates(db.walls(), db.dismissed_model_elements("wall"),
                    wall_from_json, "walls");
}

std::vector<OpeningRecord> opening_identity_candidates(const ProjectDB &db) {
  return candidates(db.openings(), db.dismissed_model_elements("opening"),
                    opening_from_json, "openings");
}

std::vector<SpaceRecord> space_identity_candidates(const ProjectDB &db) {
  return candidates(db.spaces(), db.dismissed_model_elements("space"),
                    space_from_json, "spaces");
}

size_t drop_held_walls(std::vector<WallRecord> &walls,
                       std::vector<WallJoinRecord> &joins,
                       const ModelHolds &holds) {
  const size_t n = walls.size();
  std::erase_if(walls, [&](const auto &w) { return holds.holds(w.guid); });
  std::erase_if(joins, [&](const auto &j) {
    return holds.dismissed.count(j.wall_a) || holds.dismissed.count(j.wall_b);
  });
  return n - walls.size();
}

size_t drop_held_openings(std::vector<OpeningRecord> &openings,
                          const ModelHolds &holds) {
  const size_t n = openings.size();
  std::erase_if(openings, [&](const auto &o) { return holds.holds(o.guid); });
  return n - openings.size();
}

size_t drop_held_spaces(std::vector<SpaceRecord> &spaces,
                        std::vector<SpaceBoundaryRecord> &boundaries,
                        const ModelHolds &holds) {
  const size_t n = spaces.size();
  std::erase_if(spaces, [&](const auto &s) { return holds.holds(s.guid); });
  std::erase_if(boundaries,
                [&](const auto &b) { return holds.holds(b.space); });
  return n - spaces.size();
}

} // namespace reusex::core
