// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/file_fingerprint.hpp"

#include <fmt/format.h>
#include <nlohmann/json.hpp>

#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <vector>

namespace reusex::core {

std::string file_fingerprint(const std::filesystem::path &path) {
  std::ifstream in(path, std::ios::binary);
  if (!in)
    throw std::runtime_error(
        fmt::format("file_fingerprint: cannot read {}", path.string()));
  std::uint64_t h = 0xcbf29ce484222325ULL;
  std::uint64_t size = 0;
  std::vector<char> buf(std::size_t{1} << 20);
  while (in) {
    in.read(buf.data(), static_cast<std::streamsize>(buf.size()));
    const auto n = static_cast<std::size_t>(in.gcount());
    for (std::size_t i = 0; i < n; ++i) {
      h ^= static_cast<unsigned char>(buf[i]);
      h *= 0x100000001b3ULL;
    }
    size += n;
  }
  return fmt::format("fnv1a64:{:016x}:{}", h, size);
}

std::optional<int> scan_with_fingerprint(const ProjectDB &db,
                                         std::string_view fingerprint) {
  for (const auto &s : db.scans()) {
    const auto p = nlohmann::json::parse(s.provenance_json, nullptr,
                                         /*allow_exceptions=*/false);
    if (p.is_object() && p.contains(kProvenanceFileFingerprint) &&
        p[std::string(kProvenanceFileFingerprint)] == fingerprint)
      return s.id;
  }
  return std::nullopt;
}

} // namespace reusex::core
