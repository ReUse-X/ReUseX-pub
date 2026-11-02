// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// A content fingerprint of an imported file (reusex-y2lg.30): appending the
// same RTAB-Map session twice is refused, whatever the file is called (an
// upload lands under a server-chosen name). The importer stores it in the
// scan's provenance (`scans.provenance_json`, key "file_fingerprint"), and
// ruxd checks an upload against it before it queues the import (409).
//
// FNV-1a 64 over every byte plus the size: a duplicate check, not a
// security property.

#include <reusex/core/ProjectDB.hpp>

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>

namespace reusex::core {

/// provenance_json key of the fingerprint.
inline constexpr std::string_view kProvenanceFileFingerprint =
    "file_fingerprint";

/// "fnv1a64:<16 hex>:<size>". @throws std::runtime_error when @p path
/// cannot be read.
std::string file_fingerprint(const std::filesystem::path &path);

/// The scan whose provenance carries @p fingerprint, or nullopt.
std::optional<int> scan_with_fingerprint(const ProjectDB &db,
                                         std::string_view fingerprint);

} // namespace reusex::core
