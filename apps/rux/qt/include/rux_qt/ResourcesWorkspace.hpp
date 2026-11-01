// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Ressourcer: the project's resources (survey parts, reusex-3vuu) as the
// people at a survey read them — a list grouped by type, with each
// resource's chosen photo (its `sys:view` record, cropped to the region),
// designation, room, condition and reuse potential; search and storey / room
// filters; and a detail page with the large photo (region marked), every
// passport value of the active template, where it is, and what is linked to
// it (instance, building element, samples).
//
// Read-only, like Oversigt: values are edited in the web GUI or with `rux
// set` for now. The data is read from the session's ProjectDB on the GUI
// thread when the page is first shown (a few hundred rows); the photos are
// decoded off it (ResourceImageLoader). The filtering, grouping, key lookup
// and crop math are rux_qt_core's (resource_list.hpp).

#include <rux_qt/resource_list.hpp>
#include <rux_qt/selection.hpp>

#include <QHash>
#include <QString>
#include <QStringList>
#include <QVector>
#include <QWidget>

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

class QComboBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QScrollArea;
class QStackedWidget;
class QVBoxLayout;

namespace rux::qt {

class ProjectSession;
class ResourceImageLoader;
class ResourcePhoto;

class ResourcesWorkspace : public QWidget {
  Q_OBJECT
    public:
  /// @p persist: remember the chosen template per project in QSettings
  /// (false in a gallery shot).
  explicit ResourcesWorkspace(ProjectSession &session, bool persist = true,
                              QWidget *parent = nullptr);
  ~ResourcesWorkspace() override;

  /// Read the project's resources the first time the page is shown.
  void activate();
  const Selection &selection() const { return selection_; }

  /// The list's filters (also what the gallery drives).
  void set_query(const QString &text);
  void set_storey_filter(const QString &storey);
  void set_room_filter(const QString &room);
  /// Open the detail page of @p code; false when there is no such resource.
  bool open_resource(const QString &code);
  void show_list();
  /// Resource codes whose type or designation contains @p text (folded).
  QStringList find_codes(const QString &text) const;
  int resource_count() const;

    signals:
  void selection_changed(const rux::qt::Selection &selection);

    private:
  struct Entry;
  struct RowWidgets;
  struct KeyInfo {
    std::string id, label, data_type, unit;
  };

  void reset();
  void load();
  void build_list();
  void apply_filter();
  void rebuild_room_combo();
  void request_thumbnails();
  void image_ready(const QString &key);
  void show_detail(int index);
  void step_detail(int delta);
  void choose_template(int combo_index);
  QString value_text(const Entry &e, const KeyInfo &key) const;
  /// The inspector: the open resource in short, or nothing on the list.
  void refresh_selection();

  ProjectSession &session_;
  bool persist_ = true;
  bool loaded_ = false;
  QString error_;

  std::vector<Entry> entries_;
  std::vector<ResourceRowData> rows_;
  std::vector<TypeRef> types_;
  std::vector<KeyInfo> catalogue_;
  struct TemplateInfo {
    std::int64_t id = 0;
    QString name;
    std::vector<std::string> keys;
  };
  std::vector<TemplateInfo> templates_;
  int template_index_ = -1;
  ResourceFilter filter_;
  std::vector<int> visible_order_; ///< entry indices in list order
  int detail_ = -1;

  ResourceImageLoader *images_ = nullptr;
  QStackedWidget *pages_ = nullptr;
  QLabel *summary_ = nullptr;
  QLineEdit *search_ = nullptr;
  QComboBox *storey_combo_ = nullptr;
  QComboBox *room_combo_ = nullptr;
  QComboBox *template_combo_ = nullptr;
  QWidget *toolbar_ = nullptr;
  QScrollArea *list_scroll_ = nullptr;
  QVBoxLayout *list_body_ = nullptr;
  QWidget *empty_ = nullptr;
  QLabel *empty_title_ = nullptr;
  QLabel *empty_text_ = nullptr;
  QLabel *no_match_ = nullptr;
  std::vector<std::unique_ptr<RowWidgets>> row_widgets_;
  struct GroupWidgets;
  std::vector<std::unique_ptr<GroupWidgets>> group_widgets_;

  // detail page
  QWidget *detail_page_ = nullptr;
  QLabel *detail_pos_ = nullptr;
  QPushButton *prev_ = nullptr;
  QPushButton *next_ = nullptr;
  QWidget *detail_body_ = nullptr;
  ResourcePhoto *photo_ = nullptr;
  QLabel *photo_caption_ = nullptr;

  Selection selection_;
};

} // namespace rux::qt
