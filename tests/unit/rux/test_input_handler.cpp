// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// `rux set`'s input source (reusex-69c): a positional value always wins over
// a redirected stdin; "-" reads stdin explicitly.

#include <catch2/catch_test_macros.hpp>

#include <database/input_handler.hpp>

#include <optional>
#include <string>

using rux::database::InputHandler;
using rux::database::InputSource;

TEST_CASE("InputHandler_PositionalValue_WinsOverRedirectedStdin",
          "[rux][set][input]") {
  // An agent / CI shell: stdin is not a TTY but holds nothing.
  CHECK(InputHandler::detect_source(std::string("S0-01 Depot"), true) ==
        InputSource::Inline);
  CHECK(InputHandler::detect_source(std::string("S0-01 Depot"), false) ==
        InputSource::Inline);
  // An empty value is a value (clears a field), not "no input".
  CHECK(InputHandler::detect_source(std::string(), true) ==
        InputSource::Inline);
  CHECK(InputHandler::detect_source(std::string("-"), false) ==
        InputSource::Stdin);
  CHECK(InputHandler::detect_source(std::nullopt, true) == InputSource::Stdin);
  CHECK(InputHandler::detect_source(std::nullopt, false) == InputSource::None);
}
