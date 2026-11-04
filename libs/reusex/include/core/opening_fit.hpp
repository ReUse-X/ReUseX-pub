// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// @file
/// Whether an opening fits its host wall (reusex-w6jc, Rediger editor slice
/// S5): inside the wall along its axis and between its base and top, and
/// clear of the wall's other openings.
///
/// A misfit is a warning, not a refusal: the editor makes a change in
/// steps (shorten a wall, then move its door), detected and imported models
/// carry small misfits of their own, and the frontend lists the warnings.
/// core::apply_model_edit() reports the misfits an edit introduced (not the
/// ones the model already had) in ModelEditResult::warnings.
///
/// Pure functions over core records; no ProjectDB.

#include "reusex/core/solid_model.hpp"

#include <string>
#include <vector>

namespace reusex::core {

/// One way an opening does not fit.
struct OpeningFitIssue {
  /// before_start | past_end | below_base | above_top | overlaps
  std::string code;
  std::string other;   ///< overlaps: the other opening's guid; else ""
  std::string message; ///< for a person, names the opening, wall and numbers
};

/// Default slack for opening_fit_issues() [m].
inline constexpr double kOpeningFitTolerance = 0.005;

/// How @p opening does not fit @p host (whose guid it is assumed to name):
/// s_start < 0, s_end > the wall's length, z_sill < z_base, z_head above
/// the wall's top at either jamb (core::wall_top_at(), so a sloped top
/// counts), and an overlap longer than @p tolerance with every opening of
/// @p others hosted on the same wall (the opening itself, by guid, and
/// openings on other walls are skipped). Empty: it fits. Overlaps come
/// last, in the order of @p others.
std::vector<OpeningFitIssue>
opening_fit_issues(const OpeningRecord &opening, const WallRecord &host,
                   const std::vector<OpeningRecord> &others,
                   double tolerance = kOpeningFitTolerance);

} // namespace reusex::core
