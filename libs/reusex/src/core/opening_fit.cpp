// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/opening_fit.hpp"

#include "core/sloped_ceiling.hpp"

#include <fmt/format.h>

#include <algorithm>
#include <cmath>

namespace reusex::core {

std::vector<OpeningFitIssue>
opening_fit_issues(const OpeningRecord &o, const WallRecord &host,
                   const std::vector<OpeningRecord> &others, double tol) {
  std::vector<OpeningFitIssue> out;
  const double len = std::hypot(host.x1 - host.x0, host.y1 - host.y0);
  if (o.s_start < -tol)
    out.push_back({"before_start", "",
                   fmt::format("opening {} starts {:.3f} m before the start "
                               "of wall {} (s_start {:.3f})",
                               o.guid, -o.s_start, host.guid, o.s_start)});
  if (o.s_end > len + tol)
    out.push_back(
        {"past_end", "",
         fmt::format("opening {} runs {:.3f} m past the end of "
                     "wall {} (s_end {:.3f}, wall length {:.3f} m)",
                     o.guid, o.s_end - len, host.guid, o.s_end, len)});
  if (o.z_sill < host.z_base - tol)
    out.push_back({"below_base", "",
                   fmt::format("opening {} has its sill {:.3f} m below the "
                               "base of wall {} (z_sill {:.3f}, z_base {:.3f})",
                               o.guid, host.z_base - o.z_sill, host.guid,
                               o.z_sill, host.z_base)});
  const double s0 = std::clamp(o.s_start, 0.0, std::max(len, 0.0));
  const double s1 = std::clamp(o.s_end, 0.0, std::max(len, 0.0));
  const double top = std::min(wall_top_at(host, s0), wall_top_at(host, s1));
  if (o.z_head > top + tol)
    out.push_back(
        {"above_top", "",
         fmt::format("opening {} has its head {:.3f} m above the "
                     "top of wall {} (z_head {:.3f}, top {:.3f})",
                     o.guid, o.z_head - top, host.guid, o.z_head, top)});
  for (const auto &x : others) {
    if (x.wall != host.guid || x.guid == o.guid)
      continue;
    const double overlap =
        std::min(o.s_end, x.s_end) - std::max(o.s_start, x.s_start);
    if (overlap > tol)
      out.push_back(
          {"overlaps", x.guid,
           fmt::format("opening {} overlaps opening {} by {:.3f} m on wall {} "
                       "(s {:.3f}..{:.3f} and {:.3f}..{:.3f})",
                       o.guid, x.guid, overlap, host.guid, o.s_start, o.s_end,
                       x.s_start, x.s_end)});
  }
  return out;
}

} // namespace reusex::core
