// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <atomic>
#include <cstddef>
#include <string>

/// @file
/// Bounds for reading a Speckle bundle-only version (`io/speckle_bundle.hpp`),
/// dependency-free so `pipeline/stages.hpp` can carry them in
/// `StageExecutorOptions` (a multi-user front end such as ruxd sets its own).
/// The defaults suit a shared server (STANDARDS §5: no unbounded work on
/// server input).

namespace reusex::io::speckle {

/// Bounds of `read_bundle_directory` (DuckDB).
struct BundleReadOptions {
  std::size_t max_rows = 2'000'000; ///< per table
  /// Bytes of all strings and blobs read, across every table.
  std::size_t max_bytes = std::size_t{1} << 30; // 1 GiB
  std::string memory_limit = "1GB";             ///< DuckDB memory_limit
  int threads = 2;                              ///< DuckDB threads
  /// Abort once set (checked per data chunk); nullptr = never. Set per run
  /// by the caller (import_speckle_revit uses its own cancel token).
  const std::atomic_bool *cancel = nullptr;
};

/// Bounds of `project_bundle_to_revit_tree`.
struct BundleProjectionOptions {
  /// Mesh numbers (vertex coordinates + face ints) emitted into the tree;
  /// each shared mesh counts once.
  std::size_t max_mesh_values = 50'000'000;
  /// Vertices the tree expands to when every instance placement is baked
  /// (what `import_revit` will build): guards a small definition placed
  /// millions of times.
  std::size_t max_expanded_vertices = 50'000'000;
  /// Property rows applied to objects (instance rows + type rows times the
  /// objects of that type).
  std::size_t max_property_rows = 20'000'000;
  /// JSON nodes emitted (objects, proxies, parameters, ...).
  std::size_t max_nodes = 50'000'000;
  /// A property path deeper / longer than this is dropped (counted, warn).
  std::size_t max_path_segments = 32;
  std::size_t max_path_bytes = 1024;
  /// Abort once set (checked per object); nullptr = never.
  const std::atomic_bool *cancel = nullptr;
};

} // namespace reusex::io::speckle
