// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Reads a downloaded Speckle bundle (Parquet tables) with DuckDB. The only
// TU that includes duckdb.h (REUSEX_HAVE_DUCKDB; see reusexLibrary.cmake).
// Table layout: bundle spec 1.2.0, as specklepy reads it
// (specklepy/bundle/bundle_reader.py, commit 4b31244a).

#include "io/speckle_bundle.hpp"

#include "core/logging.hpp"

#include <fmt/format.h>

#include <algorithm>
#include <cstdint>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#ifdef REUSEX_HAVE_DUCKDB
#include <duckdb.h>
#endif

namespace reusex::io::speckle {

bool has_bundle_support() {
#ifdef REUSEX_HAVE_DUCKDB
  return true;
#else
  return false;
#endif
}

#ifndef REUSEX_HAVE_DUCKDB

BundleTables read_bundle_directory(const std::filesystem::path &,
                                   const BundleReadOptions &) {
  throw BundleVersionUnsupported("bundle.");
}

namespace detail {
std::string bundle_sandbox_query_for_tests(const std::filesystem::path &,
                                           const std::string &) {
  return "built without DuckDB";
}
} // namespace detail

#else

namespace {

namespace fs = std::filesystem;

std::string sql_string(const std::string &s) {
  std::string out = "'";
  for (const char c : s) {
    out += c;
    if (c == '\'')
      out += '\'';
  }
  return out + "'";
}

class Db {
    public:
  Db(const fs::path &dir, const BundleReadOptions &opt)
      : cancel_(opt.cancel), max_bytes_(opt.max_bytes) {
    duckdb_config cfg = nullptr;
    if (duckdb_create_config(&cfg) == DuckDBError)
      throw std::runtime_error("DuckDB: cannot create a configuration");
    // No extension is ever fetched or loaded behind our back: parquet is
    // built into libduckdb.
    duckdb_set_config(cfg, "autoinstall_known_extensions", "false");
    duckdb_set_config(cfg, "autoload_known_extensions", "false");
    duckdb_set_config(cfg, "allow_community_extensions", "false");
    duckdb_set_config(cfg, "memory_limit", opt.memory_limit.c_str());
    const std::string threads = std::to_string(std::max(1, opt.threads));
    duckdb_set_config(cfg, "threads", threads.c_str());
    char *err = nullptr;
    const auto state = duckdb_open_ext(nullptr, &db_, cfg, &err); // in-memory
    duckdb_destroy_config(&cfg);
    if (state == DuckDBError) {
      std::string msg = err ? err : "unknown error";
      duckdb_free(err);
      throw std::runtime_error("DuckDB: cannot open an in-memory database: " +
                               msg);
    }
    if (duckdb_connect(db_, &con_) == DuckDBError) {
      duckdb_close(&db_);
      throw std::runtime_error("DuckDB: cannot connect");
    }
    // Only the bundle directory is reachable; then nothing can be changed.
    std::string d = dir.string();
    if (d.empty() || d.back() != '/')
      d += '/';
    exec("SET temp_directory = ''");
    exec("SET allowed_directories = [" + sql_string(d) + "]");
    exec("SET enable_external_access = false");
    exec("SET lock_configuration = true");
  }
  ~Db() {
    if (con_)
      duckdb_disconnect(&con_);
    if (db_)
      duckdb_close(&db_);
  }
  Db(const Db &) = delete;
  Db &operator=(const Db &) = delete;

  void exec(const std::string &sql) {
    duckdb_result r;
    const bool ok = duckdb_query(con_, sql.c_str(), &r) != DuckDBError;
    std::string msg = ok ? "" : duckdb_result_error(&r);
    duckdb_destroy_result(&r);
    if (!ok)
      throw std::runtime_error("DuckDB: " + msg);
  }

  /// Calls row(chunk-accessor) for every row of @p sql.
  template <class F> void for_each_row(const std::string &sql, F &&row);

  duckdb_connection con() const { return con_; }

  /// Count @p n bytes of strings / blobs read against the budget.
  void account(std::size_t n) {
    bytes_ += n;
    if (bytes_ > max_bytes_)
      throw std::runtime_error(fmt::format(
          "Speckle bundle: more than {} MiB of strings and blobs (limit "
          "BundleReadOptions::max_bytes)",
          max_bytes_ >> 20));
  }
  void throw_if_cancelled() const {
    if (cancel_ && cancel_->load(std::memory_order_acquire))
      throw std::runtime_error("Speckle bundle read: cancelled");
  }

    private:
  duckdb_database db_ = nullptr;
  duckdb_connection con_ = nullptr;
  const std::atomic_bool *cancel_ = nullptr;
  std::size_t max_bytes_ = 0;
  std::size_t bytes_ = 0;
};

/// One row of a data chunk, typed by the query's casts.
class Row {
    public:
  Row(duckdb_data_chunk chunk, idx_t i, Db &db)
      : chunk_(chunk), i_(i), db_(&db) {}
  [[nodiscard]] bool is_null(idx_t col) const {
    auto *v = duckdb_vector_get_validity(vec(col));
    return v && !duckdb_validity_row_is_valid(v, i_);
  }
  [[nodiscard]] std::optional<std::int32_t> i32(idx_t col) const {
    if (is_null(col))
      return std::nullopt;
    return static_cast<const std::int32_t *>(
        duckdb_vector_get_data(vec(col)))[i_];
  }
  [[nodiscard]] std::optional<double> f64(idx_t col) const {
    if (is_null(col))
      return std::nullopt;
    return static_cast<const double *>(duckdb_vector_get_data(vec(col)))[i_];
  }
  [[nodiscard]] std::optional<bool> boolean(idx_t col) const {
    if (is_null(col))
      return std::nullopt;
    return static_cast<const bool *>(duckdb_vector_get_data(vec(col)))[i_];
  }
  /// VARCHAR or BLOB.
  [[nodiscard]] std::optional<std::string> str(idx_t col) const {
    if (is_null(col))
      return std::nullopt;
    auto *s =
        static_cast<duckdb_string_t *>(duckdb_vector_get_data(vec(col))) + i_;
    const std::uint32_t n = duckdb_string_t_length(*s);
    db_->account(n);
    return std::string(duckdb_string_t_data(s), n);
  }
  [[nodiscard]] std::string str_or_empty(idx_t col) const {
    return str(col).value_or(std::string{});
  }

    private:
  [[nodiscard]] duckdb_vector vec(idx_t col) const {
    return duckdb_data_chunk_get_vector(chunk_, col);
  }
  duckdb_data_chunk chunk_;
  idx_t i_;
  Db *db_;
};

template <class F> void Db::for_each_row(const std::string &sql, F &&row) {
  duckdb_result r;
  if (duckdb_query(con_, sql.c_str(), &r) == DuckDBError) {
    std::string msg = duckdb_result_error(&r);
    duckdb_destroy_result(&r);
    throw std::runtime_error("DuckDB: " + msg);
  }
  try {
    while (duckdb_data_chunk chunk = duckdb_fetch_chunk(r)) {
      const idx_t n = duckdb_data_chunk_get_size(chunk);
      try {
        throw_if_cancelled();
        for (idx_t i = 0; i < n; ++i)
          row(Row(chunk, i, *this));
      } catch (...) {
        duckdb_destroy_data_chunk(&chunk);
        throw;
      }
      duckdb_destroy_data_chunk(&chunk);
    }
  } catch (...) {
    duckdb_destroy_result(&r);
    throw;
  }
  duckdb_destroy_result(&r);
}

/// The bundle files whose name ends with @p suffix, sorted.
/// Bundle files in @p dir (regular files, not symlinks, valid names).
std::vector<std::string> bundle_names(const fs::path &dir) {
  std::vector<std::string> out;
  for (const auto &e : fs::directory_iterator(dir)) {
    const std::string name = e.path().filename().string();
    if (!e.is_symlink() && e.is_regular_file() && is_bundle_file_name(name))
      out.push_back(name);
  }
  std::sort(out.begin(), out.end());
  return out;
}

/// The file `<base><suffix>` whose base has no '.' (exact table match:
/// ".eav.eav.parquet" never matches "<v>.x.eav.eav.parquet").
std::vector<fs::path> files_ending(const fs::path &dir,
                                   const std::string &suffix) {
  std::vector<fs::path> out;
  for (const auto &name : bundle_names(dir))
    if (name.size() > suffix.size() && name.ends_with(suffix) &&
        name.substr(0, name.size() - suffix.size()).find('.') ==
            std::string::npos)
      out.push_back(dir / name);
  return out;
}

/// Geometry shards: "<base>.geometries*.parquet" (one file or several).
std::vector<fs::path> geometry_files(const fs::path &dir) {
  std::vector<fs::path> out;
  for (const auto &name : bundle_names(dir)) {
    const auto dot = name.find('.');
    if (dot != std::string::npos && dot > 0 &&
        name.compare(dot, 11, ".geometries") == 0 && name.ends_with(".parquet"))
      out.push_back(dir / name);
  }
  return out;
}

std::string parquet_source(const std::vector<fs::path> &files) {
  std::string list;
  for (const auto &f : files)
    list += (list.empty() ? "" : ", ") + sql_string(f.string());
  return "read_parquet([" + list + "])";
}

struct Table {
  std::string name; ///< for messages
  std::vector<fs::path> files;
  std::set<std::string> columns;
};

class Reader {
    public:
  Reader(const fs::path &dir, const BundleReadOptions &opt)
      : dir_(dir), opt_(opt), db_(dir, opt) {}

  std::optional<Table> table(const std::string &suffix, bool required) {
    auto files = suffix == ".geometries" ? geometry_files(dir_)
                                         : files_ending(dir_, suffix);
    if (files.empty()) {
      if (required)
        throw std::runtime_error(fmt::format(
            "Speckle bundle: no '*{}' table among the downloaded files",
            suffix));
      return std::nullopt;
    }
    if (suffix != ".geometries")
      files.resize(1); // as the SDKs: the first match
    Table t{suffix, files, {}};
    db_.for_each_row(
        "DESCRIBE SELECT * FROM " + parquet_source(files),
        [&](const Row &r) { t.columns.insert(r.str_or_empty(0)); });
    return t;
  }

  /// `col` cast to @p type, or a typed NULL when the table lacks it.
  static std::string col(const Table &t, const std::string &name,
                         const char *type, bool required = false) {
    if (t.columns.count(name))
      return fmt::format("CAST(\"{}\" AS {})", name, type);
    if (required)
      throw std::runtime_error(fmt::format(
          "Speckle bundle: table '*{}' has no column '{}'", t.name, name));
    return fmt::format("CAST(NULL AS {})", type);
  }

  template <class F>
  void rows(const Table &t, const std::string &select, F &&f) {
    std::size_t seen = 0;
    db_.for_each_row("SELECT " + select + " FROM " + parquet_source(t.files) +
                         " LIMIT " + std::to_string(opt_.max_rows + 1),
                     [&](const Row &r) {
                       if (++seen > opt_.max_rows)
                         throw std::runtime_error(fmt::format(
                             "Speckle bundle: table '*{}' has more than {} "
                             "rows",
                             t.name, opt_.max_rows));
                       f(r);
                     });
  }

    private:
  fs::path dir_;
  BundleReadOptions opt_;
  Db db_;
};

void read_eav(Reader &rd, const Table &t, const char *key_col,
              std::vector<BundleEavRow> &out) {
  const std::string sel = Reader::col(t, key_col, "INTEGER", true) + ", " +
                          Reader::col(t, "path_index", "INTEGER", true) + ", " +
                          Reader::col(t, "value_string", "VARCHAR") + ", " +
                          Reader::col(t, "value_double", "DOUBLE") + ", " +
                          Reader::col(t, "value_boolean", "BOOLEAN") + ", " +
                          Reader::col(t, "unit", "VARCHAR") + ", " +
                          Reader::col(t, "internal_definition_name", "VARCHAR");
  rd.rows(t, sel, [&](const Row &r) {
    const auto key = r.i32(0);
    const auto path = r.i32(1);
    if (!key || !path)
      return;
    BundleEavRow row;
    row.key = *key;
    row.path = *path;
    row.value.s = r.str(2);
    row.value.d = r.f64(3);
    row.value.b = r.boolean(4);
    row.value.unit = r.str_or_empty(5);
    row.value.internal_definition_name = r.str_or_empty(6);
    if (!row.value.s && !row.value.d && !row.value.b)
      return; // as the SDK's PropertyTable: valueless rows are dropped
    out.push_back(std::move(row));
  });
}

} // anonymous namespace

BundleTables read_bundle_directory(const fs::path &dir,
                                   const BundleReadOptions &opt) {
  if (!fs::is_directory(dir))
    throw std::runtime_error(
        fmt::format("Speckle bundle: '{}' is not a directory", dir.string()));
  stopwatch sw;
  Reader rd(fs::canonical(dir), opt);
  BundleTables out;

  const auto objects = rd.table(".eav.objects.parquet", true);
  rd.rows(*objects,
          Reader::col(*objects, "object_index", "INTEGER", true) + ", " +
              Reader::col(*objects, "application_id", "VARCHAR"),
          [&](const Row &r) {
            if (auto k = r.i32(0))
              out.objects[*k] = r.str_or_empty(1);
          });

  const auto paths = rd.table(".eav.paths.parquet", true);
  rd.rows(*paths,
          Reader::col(*paths, "path_index", "INTEGER", true) + ", " +
              Reader::col(*paths, "path", "VARCHAR"),
          [&](const Row &r) {
            if (auto k = r.i32(0))
              out.paths[*k] = r.str_or_empty(1);
          });

  read_eav(rd, *rd.table(".eav.eav.parquet", true), "object_index", out.eav);
  if (auto t = rd.table(".eav.type_eav.parquet", false))
    read_eav(rd, *t, "type_index", out.type_eav);
  if (auto t = rd.table(".eav.object_type.parquet", false))
    rd.rows(*t,
            Reader::col(*t, "object_index", "INTEGER", true) + ", " +
                Reader::col(*t, "type_index", "INTEGER", true),
            [&](const Row &r) {
              const auto o = r.i32(0), ty = r.i32(1);
              if (o && ty)
                out.object_type.emplace(*o, *ty);
            });

  const auto nodes = rd.table(".envelope.nodes.parquet", true);
  rd.rows(*nodes,
          Reader::col(*nodes, "id", "INTEGER", true) + ", " +
              Reader::col(*nodes, "kind", "INTEGER", true) + ", " +
              Reader::col(*nodes, "name", "VARCHAR") + ", " +
              Reader::col(*nodes, "def_ref", "INTEGER") + ", " +
              Reader::col(*nodes, "transform", "VARCHAR") + ", " +
              Reader::col(*nodes, "units", "VARCHAR") + ", " +
              Reader::col(*nodes, "subtype", "VARCHAR") + ", " +
              Reader::col(*nodes, "elevation", "DOUBLE"),
          [&](const Row &r) {
            const auto id = r.i32(0), kind = r.i32(1);
            if (!id || !kind)
              return;
            BundleNode n;
            n.id = *id;
            n.kind = *kind;
            n.name = r.str_or_empty(2);
            n.def_ref = r.i32(3);
            n.transform = r.str_or_empty(4);
            n.units = r.str_or_empty(5);
            n.subtype = r.str_or_empty(6);
            n.elevation = r.f64(7);
            out.nodes.push_back(std::move(n));
          });

  const auto rels = rd.table(".envelope.relations.parquet", true);
  rd.rows(*rels,
          Reader::col(*rels, "rel", "INTEGER", true) + ", " +
              Reader::col(*rels, "src", "INTEGER", true) + ", " +
              Reader::col(*rels, "dst", "INTEGER", true) + ", " +
              Reader::col(*rels, "ord", "INTEGER"),
          [&](const Row &r) {
            const auto rel = r.i32(0), src = r.i32(1), dst = r.i32(2);
            if (rel && src && dst)
              out.relations.push_back({*rel, *src, *dst, r.i32(3).value_or(0)});
          });

  if (auto t = rd.table(".envelope.meta.parquet", false))
    rd.rows(*t, Reader::col(*t, "schema_version", "VARCHAR"),
            [&](const Row &r) {
              if (out.schema_version.empty())
                out.schema_version = r.str_or_empty(0);
            });

  if (auto t = rd.table(".geometries", false)) {
    rd.rows(*t,
            Reader::col(*t, "geometryIndex", "INTEGER", true) + ", " +
                Reader::col(*t, "content", "BLOB", true) + ", " +
                Reader::col(*t, "type", "VARCHAR"),
            [&](const Row &r) {
              const auto k = r.i32(0);
              auto content = r.str(1);
              if (!k || !content)
                return;
              out.geometries.push_back(
                  {*k, std::move(*content), r.str_or_empty(2)});
            });
  } else {
    core::warn("Speckle bundle: no geometry table; the model has no meshes");
  }

  core::info("Speckle bundle: read {} objects, {} eav rows ({} type), {} "
             "nodes, {} relations, {} geometries (schema {}) in {:.2f} s",
             out.objects.size(), out.eav.size(), out.type_eav.size(),
             out.nodes.size(), out.relations.size(), out.geometries.size(),
             out.schema_version.empty() ? "?" : out.schema_version,
             sw.elapsed());
  return out;
}

namespace detail {
std::string bundle_sandbox_query_for_tests(const fs::path &dir,
                                           const std::string &sql) {
  Db db(fs::canonical(dir), BundleReadOptions{});
  try {
    db.exec(sql);
  } catch (const std::exception &e) {
    return e.what();
  }
  return {};
}
} // namespace detail

#endif // REUSEX_HAVE_DUCKDB

} // namespace reusex::io::speckle
