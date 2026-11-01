// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/resource_list.hpp>

#include <algorithm>
#include <cmath>
#include <map>
#include <set>

namespace rux::qt {
namespace {

/// Words of @p s, split on ASCII blanks.
std::vector<std::string> words(std::string_view s) {
  std::vector<std::string> out;
  std::string cur;
  for (const char c : s) {
    if (c == ' ' || c == '\t' || c == '\n') {
      if (!cur.empty())
        out.push_back(std::move(cur));
      cur.clear();
    } else {
      cur.push_back(c);
    }
  }
  if (!cur.empty())
    out.push_back(std::move(cur));
  return out;
}

/// Group digits of @p n by thousands with '.' (Danish).
std::string grouped(std::size_t n) {
  std::string digits = std::to_string(n);
  std::string out;
  const int len = static_cast<int>(digits.size());
  for (int i = 0; i < len; ++i) {
    if (i > 0 && (len - i) % 3 == 0)
      out.push_back('.');
    out.push_back(digits[static_cast<std::size_t>(i)]);
  }
  return out;
}

constexpr double kRoundSlack = 1e-6;

int floor_px(double v) { return static_cast<int>(std::floor(v + kRoundSlack)); }
int ceil_px(double v) { return static_cast<int>(std::ceil(v - kRoundSlack)); }

/// The region in pixels WITHOUT clamping the right edge (a panorama region
/// may run past it); x is kept in [0, iw).
PixelRect unclamped(const std::optional<NormRegion> &region, int iw, int ih) {
  if (!region)
    return {0, 0, iw, ih};
  const auto &r = *region;
  int x0 = floor_px(r[0] * iw);
  int x1 = ceil_px((r[0] + r[2]) * iw);
  int y0 = std::clamp(floor_px(r[1] * ih), 0, std::max(ih - 1, 0));
  int y1 = std::clamp(ceil_px((r[1] + r[3]) * ih), y0 + 1, std::max(ih, 1));
  x0 = std::clamp(x0, 0, std::max(iw - 1, 0));
  x1 = std::max(x1, x0 + 1);
  return {x0, y0, x1 - x0, y1 - y0};
}

/// Grow [lo, lo + len) to @p want around its centre, inside [0, limit).
void grow_inside(int &lo, int &len, int want, int limit) {
  want = std::min(want, limit);
  if (want <= len)
    return;
  const double centre = lo + len / 2.0;
  int nlo = static_cast<int>(std::lround(centre - want / 2.0));
  nlo = std::clamp(nlo, 0, limit - want);
  lo = nlo;
  len = want;
}

} // namespace

// ---- text ------------------------------------------------------------------

std::string fold_da(std::string_view s) {
  std::string out;
  out.reserve(s.size());
  for (std::size_t i = 0; i < s.size(); ++i) {
    const auto c = static_cast<unsigned char>(s[i]);
    if (c >= 'A' && c <= 'Z') {
      out.push_back(static_cast<char>(c - 'A' + 'a'));
      continue;
    }
    // UTF-8 0xC3 0x80..0x9E are the Latin-1 capitals À..Þ (but 0x97, ×);
    // their small letters are 0x20 higher.
    if (c == 0xC3 && i + 1 < s.size()) {
      const auto d = static_cast<unsigned char>(s[i + 1]);
      out.push_back(s[i]);
      if (d >= 0x80 && d <= 0x9E && d != 0x97)
        out.push_back(static_cast<char>(d + 0x20));
      else
        out.push_back(s[i + 1]);
      ++i;
      continue;
    }
    out.push_back(s[i]);
  }
  return out;
}

std::string grade_tone(std::string_view value) {
  const std::string v = fold_da(value);
  if (v.empty())
    return {};
  static const std::set<std::string> good = {"god", "meget god", "høj", "ny",
                                             "som ny"};
  static const std::set<std::string> warn = {"rimelig", "middel", "medium",
                                             "nogen"};
  static const std::set<std::string> crit = {"dårlig", "meget dårlig", "lav",
                                             "ingen"};
  if (good.count(v))
    return "good";
  if (warn.count(v))
    return "warn";
  if (crit.count(v))
    return "crit";
  return "outline";
}

std::string display_value(std::string_view data_type, std::string_view unit,
                          const std::optional<std::string> &value) {
  static const std::string kNone = "—";
  if (!value || value->empty())
    return kNone;
  const std::string &v = *value;
  if (data_type == "boolean") {
    if (v == "true")
      return "Ja";
    if (v == "false")
      return "Nej";
    return v;
  }
  if (data_type == "multiselect" && v.front() == '[') {
    // A JSON array of strings: the items, unescaped (\" and \\ only;
    // the catalogue stores UTF-8 as is).
    std::vector<std::string> items;
    std::string cur;
    bool in = false;
    for (std::size_t i = 0; i < v.size(); ++i) {
      const char c = v[i];
      if (!in) {
        if (c == '"') {
          in = true;
          cur.clear();
        }
        continue;
      }
      if (c == '\\' && i + 1 < v.size()) {
        cur.push_back(v[++i]);
      } else if (c == '"') {
        in = false;
        items.push_back(cur);
      } else {
        cur.push_back(c);
      }
    }
    if (items.empty())
      return kNone;
    std::string out;
    for (std::size_t i = 0; i < items.size(); ++i)
      out += (i ? ", " : "") + items[i];
    return out;
  }
  if (data_type == "number") {
    std::string out = v;
    std::replace(out.begin(), out.end(), '.', ',');
    if (!unit.empty())
      out += " " + std::string(unit);
    return out;
  }
  return v;
}

// ---- template choice ------------------------------------------------------

std::optional<std::int64_t>
default_template(const std::vector<TemplateRef> &templates,
                 std::optional<std::int64_t> remembered) {
  if (templates.empty())
    return std::nullopt;
  if (remembered)
    for (const auto &t : templates)
      if (t.id == *remembered)
        return t.id;
  std::optional<std::int64_t> user;
  for (const auto &t : templates)
    if (t.tag.empty() && (!user || t.id > *user))
      user = t.id;
  if (user)
    return user;
  for (const auto &t : templates)
    if (t.tag == "screening")
      return t.id;
  return templates.front().id;
}

// ---- keys ------------------------------------------------------------------

std::optional<std::size_t> find_key(const std::vector<KeyRef> &keys,
                                    const std::vector<std::string> &names) {
  for (const auto &name : names) {
    const std::string n = fold_da(name);
    for (std::size_t i = 0; i < keys.size(); ++i)
      if (fold_da(keys[i].label) == n || fold_da(keys[i].field) == n)
        return i;
  }
  return std::nullopt;
}

const std::vector<std::string> &designation_key_names() {
  static const std::vector<std::string> n = {"designation", "betegnelse"};
  return n;
}
const std::vector<std::string> &condition_key_names() {
  static const std::vector<std::string> n = {"Tilstand", "Condition"};
  return n;
}
const std::vector<std::string> &potential_key_names() {
  static const std::vector<std::string> n = {"Genbrugspotentiale",
                                             "Genanvendelsespotentiale",
                                             "Potentiale", "Reuse potential"};
  return n;
}
const std::vector<std::string> &storey_key_names() {
  static const std::vector<std::string> n = {"Etage", "Storey"};
  return n;
}

// ---- storeys ---------------------------------------------------------------

StoreyIndex::StoreyIndex(
    const std::vector<std::pair<std::string, int>> &rooms,
    const std::vector<std::pair<int, std::string>> &storeys)
    : rooms_(rooms), storeys_(storeys) {
  std::stable_sort(
      rooms_.begin(), rooms_.end(),
      [](const auto &a, const auto &b) { return a.first < b.first; });
}

std::optional<std::pair<int, std::string>>
StoreyIndex::lookup(std::string_view room) const {
  if (room.empty())
    return std::nullopt;
  const auto it = std::lower_bound(
      rooms_.begin(), rooms_.end(), room,
      [](const auto &a, std::string_view r) { return a.first < r; });
  if (it == rooms_.end() || it->first != room)
    return std::nullopt;
  for (const auto &[idx, name] : storeys_)
    if (idx == it->second && !name.empty())
      return std::make_pair(idx, name);
  return std::make_pair(it->second, "Etage " + std::to_string(it->second));
}

// ---- rows, filters, groups --------------------------------------------------

bool matches(const ResourceRowData &row, const ResourceFilter &filter) {
  if (row.dismissed && !filter.show_dismissed)
    return false;
  if (!filter.storey.empty() && row.storey != filter.storey)
    return false;
  if (!filter.room.empty() && row.room != filter.room)
    return false;
  const auto q = words(filter.query);
  if (q.empty())
    return true;
  const std::string hay =
      fold_da(row.code + '\n' + row.designation + '\n' + row.type_name + '\n' +
              row.room + '\n' + row.storey + '\n' + row.condition + '\n' +
              row.potential);
  for (const auto &w : q)
    if (hay.find(fold_da(w)) == std::string::npos)
      return false;
  return true;
}

std::vector<ResourceGroupView>
group_resources(const std::vector<ResourceRowData> &rows,
                const std::vector<TypeRef> &types,
                const ResourceFilter &filter) {
  std::vector<ResourceGroupView> groups;
  std::map<std::int64_t, std::size_t> slot;
  for (const auto &t : types) {
    if (slot.count(t.id))
      continue;
    slot[t.id] = groups.size();
    groups.push_back({t.id, t.name, {}, 0});
  }
  // Types the list does not know, by id, after the known ones.
  std::set<std::int64_t> unknown;
  for (const auto &r : rows)
    if (!slot.count(r.type_id))
      unknown.insert(r.type_id);
  for (const auto id : unknown) {
    slot[id] = groups.size();
    std::string name;
    for (const auto &r : rows)
      if (r.type_id == id) {
        name = r.type_name;
        break;
      }
    groups.push_back({id, name, {}, 0});
  }

  std::vector<std::size_t> order(rows.size());
  for (std::size_t i = 0; i < order.size(); ++i)
    order[i] = i;
  std::stable_sort(order.begin(), order.end(),
                   [&](auto a, auto b) { return rows[a].code < rows[b].code; });
  for (const auto i : order) {
    auto &g = groups[slot[rows[i].type_id]];
    if (!rows[i].dismissed || filter.show_dismissed)
      ++g.total;
    if (matches(rows[i], filter))
      g.rows.push_back(i);
  }
  groups.erase(std::remove_if(groups.begin(), groups.end(),
                              [](const auto &g) { return g.rows.empty(); }),
               groups.end());
  return groups;
}

std::vector<FacetOption>
storey_options(const std::vector<ResourceRowData> &rows, bool show_dismissed) {
  std::map<std::pair<int, std::string>, std::size_t> counts;
  std::size_t unknown = 0;
  for (const auto &r : rows) {
    if (r.dismissed && !show_dismissed)
      continue;
    if (r.storey.empty())
      ++unknown;
    else
      ++counts[{r.storey_idx, r.storey}];
  }
  std::vector<FacetOption> out;
  for (const auto &[key, n] : counts)
    out.push_back({key.second, n});
  if (unknown > 0)
    out.push_back({"", unknown});
  return out;
}

std::vector<FacetOption> room_options(const std::vector<ResourceRowData> &rows,
                                      std::string_view storey,
                                      bool show_dismissed) {
  std::map<std::string, std::size_t> counts;
  for (const auto &r : rows) {
    if (r.dismissed && !show_dismissed)
      continue;
    if (r.room.empty() || (!storey.empty() && r.storey != storey))
      continue;
    ++counts[r.room];
  }
  std::vector<FacetOption> out;
  for (const auto &[room, n] : counts)
    out.push_back({room, n});
  return out;
}

std::string resource_count_text(std::size_t n) {
  return grouped(n) + (n == 1 ? " ressource" : " ressourcer");
}

std::string list_summary(const std::vector<ResourceGroupView> &groups,
                         std::size_t total, bool filtered) {
  std::size_t shown = 0;
  for (const auto &g : groups)
    shown += g.rows.size();
  const std::string types =
      std::to_string(groups.size()) + (groups.size() == 1 ? " type" : " typer");
  if (!filtered)
    return resource_count_text(total) + " i " + types;
  return "Viser " + grouped(shown) + " af " + resource_count_text(total) +
         " i " + types;
}

// ---- thumbnails ------------------------------------------------------------

PixelRect region_pixels(const std::optional<NormRegion> &region, int iw,
                        int ih) {
  PixelRect r = unclamped(region, iw, ih);
  r.w = std::max(1, std::min(r.w, iw - r.x));
  return r;
}

std::vector<PixelRect> thumbnail_crop(const std::optional<NormRegion> &region,
                                      int iw, int ih, double aspect,
                                      bool wraps) {
  if (iw <= 0 || ih <= 0)
    return {};
  PixelRect r =
      wraps ? unclamped(region, iw, ih) : region_pixels(region, iw, ih);
  r.w = std::min(r.w, iw);
  if (aspect > 0.0) {
    const double cur = static_cast<double>(r.w) / r.h;
    if (cur > aspect) {
      grow_inside(r.y, r.h, static_cast<int>(std::lround(r.w / aspect)), ih);
    } else if (cur < aspect) {
      const int want = static_cast<int>(std::lround(r.h * aspect));
      if (wraps) {
        const int w = std::min(want, iw);
        const double centre = r.x + r.w / 2.0;
        r.x = static_cast<int>(std::lround(centre - w / 2.0));
        r.w = w;
        r.x = ((r.x % iw) + iw) % iw;
      } else {
        grow_inside(r.x, r.w, want, iw);
      }
    }
  }
  if (r.x + r.w <= iw)
    return {r};
  const int first = iw - r.x;
  return {PixelRect{r.x, r.y, first, r.h}, PixelRect{0, r.y, r.w - first, r.h}};
}

PixelRect region_in_crop(const std::optional<NormRegion> &region, int iw,
                         int ih, const std::vector<PixelRect> &crop) {
  if (crop.empty() || iw <= 0 || ih <= 0)
    return {};
  const bool wraps = crop.size() > 1;
  PixelRect r =
      wraps ? unclamped(region, iw, ih) : region_pixels(region, iw, ih);
  int x = r.x - crop[0].x;
  if (x < 0)
    x += iw; // the region starts after the seam, the crop before it
  return {x, r.y - crop[0].y, r.w, r.h};
}

} // namespace rux::qt
