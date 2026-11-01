// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Qt-free logic of the Ressourcer workspace (the survey parts of a project,
// reusex-3vuu): which template the page shows, the row each resource gets
// (designation, room, storey, condition, reuse potential), search and the
// storey / room filters, grouping by survey type, and the crop math that
// turns a resource's view record (`sys:view`, core/resource_view.hpp) into a
// thumbnail. Standard library only, so the light test binary covers it
// (tests/unit/rux_qt/test_resource_list.cpp); the widgets in
// ResourcesWorkspace only lay these results out.

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace rux::qt {

// ---- text ------------------------------------------------------------------

/// @p s folded for a case-insensitive Danish search: ASCII lower case, and
/// the UTF-8 capitals Æ Ø Å É Ä Ö Ü to their small letters. Other bytes are
/// kept as they are.
std::string fold_da(std::string_view s);

/// The pill tone of a condition or reuse-potential value ("God", "Høj" ->
/// "good"; "Rimelig", "Middel" -> "warn"; "Dårlig", "Lav" -> "crit"; any
/// other text -> "outline"; empty -> "").
std::string grade_tone(std::string_view value);

/// A stored key value as the detail page shows it: "—" when unset or
/// blank; a boolean as "Ja" / "Nej"; a multiselect JSON array of strings
/// as "a, b"; a number with a Danish decimal comma and @p unit after it
/// ("12,5 kg"); anything else as it is. @p data_type is
/// core::ResourceKey::data_type.
std::string display_value(std::string_view data_type, std::string_view unit,
                          const std::optional<std::string> &value);

// ---- template choice ------------------------------------------------------

/// What the page needs of a template: its id, name and seed tag ("" for a
/// template the user made).
struct TemplateRef {
  std::int64_t id = 0;
  std::string name;
  std::string tag;
};

/// The template the page opens with: @p remembered when it still exists;
/// else the most recent template the user made (no seed tag, highest id) —
/// a project-specific template is made for that project; else the
/// "screening" seed; else the first. nullopt when there is none.
std::optional<std::int64_t>
default_template(const std::vector<TemplateRef> &templates,
                 std::optional<std::int64_t> remembered = std::nullopt);

// ---- keys ------------------------------------------------------------------

/// What the page needs of a catalogue key (core::ResourceKey).
struct KeyRef {
  std::string id;    ///< "sys:name", "lex:<guid>", "col:<id>"
  std::string label; ///< "Tilstand"
  std::string field; ///< storage name ("designation", the column's name)
};

/// Index of the first key in @p keys whose label or field equals one of
/// @p names (folded, so case does not matter), trying @p names in order.
std::optional<std::size_t> find_key(const std::vector<KeyRef> &keys,
                                    const std::vector<std::string> &names);

/// The names find_key looks for, per row column.
const std::vector<std::string> &designation_key_names(); ///< designation
const std::vector<std::string> &condition_key_names();   ///< Tilstand
const std::vector<std::string> &potential_key_names();   ///< potentiale
const std::vector<std::string> &storey_key_names();      ///< Etage

// ---- storeys ---------------------------------------------------------------

/// Which storey a room lies on: by the room's name in the project's spaces
/// (spaces.name -> spaces.storey -> storeys.name).
class StoreyIndex {
    public:
  StoreyIndex() = default;
  /// @p rooms: (space name, storey idx); @p storeys: (idx, display name).
  StoreyIndex(const std::vector<std::pair<std::string, int>> &rooms,
              const std::vector<std::pair<int, std::string>> &storeys);
  /// (idx, name) of @p room's storey; nullopt for an unknown room. A storey
  /// idx without a stored storey is named "Etage <idx>".
  std::optional<std::pair<int, std::string>>
  lookup(std::string_view room) const;

    private:
  std::vector<std::pair<std::string, int>> rooms_; ///< sorted by name
  std::vector<std::pair<int, std::string>> storeys_;
};

// ---- rows, filters, groups --------------------------------------------------

/// One resource as the list shows it.
struct ResourceRowData {
  std::string code; ///< "RX-001"
  std::int64_t type_id = 0;
  std::string type_name;
  std::string designation;
  std::string room;
  std::string storey; ///< display name; "" when unknown
  int storey_idx = -1;
  std::string condition;
  std::string potential;
  bool manual = false;    ///< added by hand, no instance
  bool dismissed = false; ///< marked as an error
  bool starred = false;
  bool has_view = false; ///< a sys:view image is chosen
};

/// The list's filters. Empty strings mean "all".
struct ResourceFilter {
  std::string query;  ///< searched in code, designation, type, room, …
  std::string storey; ///< exact storey display name
  std::string room;   ///< exact room name
  bool show_dismissed = false;
  bool active() const {
    return !query.empty() || !storey.empty() || !room.empty();
  }
};

/// True when @p row passes @p filter. Every word of the query must occur
/// (folded) in the row's code, designation, type, room, storey, condition
/// or potential.
bool matches(const ResourceRowData &row, const ResourceFilter &filter);

/// A survey type as the grouping orders it.
struct TypeRef {
  std::int64_t id = 0;
  std::string name;
};

/// One type's block in the list: the indices (into the rows) that pass the
/// filter, in code order, and how many rows the type has in all (dismissed
/// ones left out unless the filter shows them).
struct ResourceGroupView {
  std::int64_t type_id = 0;
  std::string type_name;
  std::vector<std::size_t> rows;
  std::size_t total = 0;
};

/// Rows grouped by type, in @p types order (a type missing from @p types
/// follows, by id). Types with no row passing the filter are left out.
std::vector<ResourceGroupView>
group_resources(const std::vector<ResourceRowData> &rows,
                const std::vector<TypeRef> &types,
                const ResourceFilter &filter);

/// One choice in a facet combo box, with how many rows it holds.
struct FacetOption {
  std::string value;
  std::size_t count = 0;
};
/// The storeys present, lowest first (unknown storey last, value "").
std::vector<FacetOption>
storey_options(const std::vector<ResourceRowData> &rows,
               bool show_dismissed = false);
/// The rooms present on @p storey ("" = every storey), by name.
std::vector<FacetOption> room_options(const std::vector<ResourceRowData> &rows,
                                      std::string_view storey,
                                      bool show_dismissed = false);

/// "Viser 12 af 211 ressourcer i 3 typer" / "211 ressourcer i 18 typer".
std::string list_summary(const std::vector<ResourceGroupView> &groups,
                         std::size_t total, bool filtered);

/// "1 ressource" / "12 ressourcer" (Danish thousands separator).
std::string resource_count_text(std::size_t n);

// ---- thumbnails ------------------------------------------------------------

/// [x, y, w, h], normalised 0..1 — core::ViewRegion's layout.
using NormRegion = std::array<double, 4>;

struct PixelRect {
  int x = 0, y = 0, w = 0, h = 0;
  bool operator==(const PixelRect &) const = default;
};

/// @p region in pixels of an @p iw x @p ih image, rounded outward and
/// clamped to the image; at least 1 x 1. nullopt region = the whole image.
/// A panorama region crossing the seam (x + w > 1) is clamped at the right
/// edge here — use thumbnail_crop for the wrapped pieces.
PixelRect region_pixels(const std::optional<NormRegion> &region, int iw,
                        int ih);

/// The crop a thumbnail of aspect @p aspect (width / height) shows: the
/// region grown on its short side to that aspect, centred on the region, and
/// kept inside the image (shifted, then capped at the image size, so a crop
/// can stay narrower than @p aspect when the image is). With @p wraps (a
/// 360° panorama) the horizontal extent wraps across the seam instead of
/// being shifted, and the crop comes back as one or two pieces whose
/// side-by-side concatenation (left to right) is the crop.
std::vector<PixelRect> thumbnail_crop(const std::optional<NormRegion> &region,
                                      int iw, int ih, double aspect,
                                      bool wraps = false);

/// Where the region sits inside a crop (thumbnail_crop's pieces joined), in
/// the crop's own pixels — to outline the part on an enlarged crop.
PixelRect region_in_crop(const std::optional<NormRegion> &region, int iw,
                         int ih, const std::vector<PixelRect> &crop);

} // namespace rux::qt
