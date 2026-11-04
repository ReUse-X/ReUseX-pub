// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// @file
/// Stairs (reusex-xvnl): what a core::StairRecord measures, its checks and
/// its stored form. Core-only, so ruxd's API library and the Qt client
/// share it.

#include <reusex/core/solid_model.hpp>

#include <string>
#include <vector>

namespace reusex::core {

/// Sum of the runs' risers.
int stair_riser_count(const StairRecord &stair);
/// (z_top - z_base) / risers; -1 without risers.
double stair_riser_height(const StairRecord &stair);
/// The going of run @p i: tread_depth when set, else the run's walking-line
/// length / (risers - 1) (the top riser lands on the landing or floor);
/// -1 for a run of one riser.
double stair_going(const StairRecord &stair, std::size_t i);
/// The z (level frame) where run @p i starts.
double stair_run_base(const StairRecord &stair, std::size_t i);

/// The reason @p stair cannot be stored, or "": kind not straight | L | U |
/// spiral, location not left | centre | right, no run, a run with < 1
/// riser, a width <= 0 or a zero-length walking line, z_top <= z_base, a
/// non-finite number, a landing ring with < 3 points.
std::string check_stair(const StairRecord &stair);

/// Advice for a valid stair, in Danish building-regulation terms (BR18 /
/// the 2s + g rule): risers higher than 0.18 m (0.20 m for a stair inside
/// a dwelling is the hard limit), 2 x riser + going outside 0.59 .. 0.65 m,
/// a going under 0.25 m, a width under 0.80 m. Empty when none.
std::vector<std::string> stair_advice(const StairRecord &stair);

/// Stored forms: JSON arrays ("" for none).
/// runs: [{"start": [x, y], "end": [x, y], "width": w, "risers": n}, ...]
std::string encode_stair_runs(const std::vector<StairRun> &runs);
/// @throws std::runtime_error for text that is not that form.
std::vector<StairRun> decode_stair_runs(const std::string &text);
/// landings: [{"outer": [[x, y], ...], "holes": [...]}, ...]
std::string encode_stair_landings(const std::vector<SpacePolygon> &landings);
std::vector<SpacePolygon> decode_stair_landings(const std::string &text);

} // namespace reusex::core
