// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/ProjectSession.hpp>
#include <rux_qt/ResourceImageLoader.hpp>
#include <rux_qt/ResourcesWorkspace.hpp>
#include <rux_qt/Theme.hpp>
#include <rux_qt/widgets.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/resource_keys.hpp>
#include <reusex/core/resource_templates.hpp>
#include <reusex/core/resource_view.hpp>
#include <reusex/core/resources.hpp>
#include <reusex/core/solid_model.hpp>
#include <reusex/core/survey.hpp>

#include <QComboBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QPainterPath>
#include <QPushButton>
#include <QScrollArea>
#include <QSettings>
#include <QStackedWidget>
#include <QVBoxLayout>

#include <algorithm>
#include <set>

namespace rux::qt {

namespace core = reusex::core;

// ------------------------------------------------------------------ images --

namespace {

QPushButton *button(const QString &text, const QString &kind) {
  auto *b = new QPushButton(NavItem::escape_mnemonic(text));
  b->setProperty("kind", kind);
  b->setCursor(Qt::PointingHandCursor);
  return b;
}

QLabel *label(const QString &text, const QString &object_name,
              bool wrap = false) {
  auto *l = new QLabel(text);
  l->setObjectName(object_name);
  l->setWordWrap(wrap);
  return l;
}

void clear_layout(QLayout *layout) {
  while (QLayoutItem *item = layout->takeAt(0)) {
    if (QWidget *w = item->widget()) {
      w->hide();
      w->deleteLater();
    } else if (QLayout *l = item->layout()) {
      clear_layout(l);
    }
    delete item;
  }
}

QString qs(const std::string &s) { return QString::fromStdString(s); }

/// The rectangle @p image takes when fitted (contained) into @p box.
QRectF fitted(const QSize &image, const QRectF &box) {
  if (image.isEmpty() || box.isEmpty())
    return {};
  const double s =
      std::min(box.width() / image.width(), box.height() / image.height());
  const QSizeF sz(image.width() * s, image.height() * s);
  return QRectF(box.center().x() - sz.width() / 2,
                box.center().y() - sz.height() / 2, sz.width(), sz.height());
}

} // namespace

/// A resource's thumbnail: the cropped view image contained in a fixed
/// canvas-coloured box, or a muted "Intet billede".
class ResourceThumb : public QWidget {
    public:
  explicit ResourceThumb(QWidget *parent = nullptr) : QWidget(parent) {
    const Theme &t = theme();
    setFixedSize(t.px("--space-7") * 2 + t.px("--space-4"),
                 (t.px("--space-7") * 2 + t.px("--space-4")) * 3 / 4);
    setAttribute(Qt::WA_TransparentForMouseEvents);
  }
  void set_image(const QImage &img) {
    image_ = img;
    update();
  }
  void set_missing(const QString &text) {
    missing_ = text;
    update();
  }

    protected:
  void paintEvent(QPaintEvent *) override {
    const Theme &t = theme();
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    p.setRenderHint(QPainter::SmoothPixmapTransform);
    QPainterPath clip;
    const double r = t.px("--radius-md");
    clip.addRoundedRect(QRectF(rect()), r, r);
    p.setClipPath(clip);
    if (image_.isNull()) {
      p.fillRect(rect(), t.color("--color-surface-sunken"));
      p.setPen(t.color("--color-text-faint"));
      p.setFont(t.font(FontRole::sans, "--font-size-2xs"));
      p.drawText(rect(), Qt::AlignCenter | Qt::TextWordWrap,
                 missing_.isEmpty() ? QString("…") : missing_);
    } else {
      // Cover: the crop already has the box's aspect when the view has a
      // region; a whole image loses a little at its edges.
      const QSizeF box = size();
      const double s = std::max(box.width() / image_.width(),
                                box.height() / image_.height());
      const QSizeF sz(image_.width() * s, image_.height() * s);
      p.drawImage(QRectF(QPointF((box.width() - sz.width()) / 2,
                                 (box.height() - sz.height()) / 2),
                         sz),
                  image_);
    }
    p.setClipping(false);
    p.setPen(QPen(t.color("--color-border"), 1));
    p.setBrush(Qt::NoBrush);
    p.drawRoundedRect(QRectF(rect()).adjusted(0.5, 0.5, -0.5, -0.5), r, r);
  }

    private:
  QImage image_;
  QString missing_;
};

/// The detail page's photo: the view image contained in the pane, the
/// region outlined in the accent colour and everything around it veiled.
class ResourcePhoto : public QWidget {
    public:
  explicit ResourcePhoto(QWidget *parent = nullptr) : QWidget(parent) {
    setObjectName("resourcePhoto");
    setMinimumHeight(theme().px("--layout-panel-width"));
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  }
  void set_image(const ResourceImage *img) {
    image_ = img ? img->image : QImage();
    region_ = img ? img->region : QRectF();
    message_ = img && img->image.isNull() ? img->error : QString();
    update();
  }
  void set_message(const QString &m) {
    image_ = {};
    message_ = m;
    update();
  }

    protected:
  void paintEvent(QPaintEvent *) override {
    const Theme &t = theme();
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    p.setRenderHint(QPainter::SmoothPixmapTransform);
    const double r = t.px("--radius-md");
    QPainterPath clip;
    clip.addRoundedRect(QRectF(rect()), r, r);
    p.setClipPath(clip);
    p.fillRect(rect(), t.color("--color-canvas"));
    if (image_.isNull()) {
      p.setPen(t.color("--color-on-chrome-muted"));
      p.setFont(t.font(FontRole::sans, "--font-size-sm"));
      p.drawText(rect().adjusted(t.px("--space-4"), 0, -t.px("--space-4"), 0),
                 Qt::AlignCenter | Qt::TextWordWrap,
                 message_.isEmpty() ? QString("Henter billede…") : message_);
      return;
    }
    const QRectF pad =
        QRectF(rect()).adjusted(t.px("--space-2"), t.px("--space-2"),
                                -t.px("--space-2"), -t.px("--space-2"));
    const QRectF img = fitted(image_.size(), pad);
    p.drawImage(img, image_);
    if (region_.isNull() || region_.isEmpty())
      return;
    const QRectF reg(img.x() + region_.x() * img.width(),
                     img.y() + region_.y() * img.height(),
                     region_.width() * img.width(),
                     region_.height() * img.height());
    // Veil the image around the region: four bands, top, bottom, left,
    // right.
    QColor scrim = t.color("--color-scrim");
    scrim.setAlphaF(0.6);
    const QRectF r2 = reg.intersected(img);
    p.fillRect(QRectF(img.left(), img.top(), img.width(), r2.top() - img.top()),
               scrim);
    p.fillRect(QRectF(img.left(), r2.bottom(), img.width(),
                      img.bottom() - r2.bottom()),
               scrim);
    p.fillRect(
        QRectF(img.left(), r2.top(), r2.left() - img.left(), r2.height()),
        scrim);
    p.fillRect(
        QRectF(r2.right(), r2.top(), img.right() - r2.right(), r2.height()),
        scrim);
    QPen pen(t.color("--color-accent"), t.px("--space-1") / 2.0);
    pen.setJoinStyle(Qt::MiterJoin);
    p.setPen(pen);
    p.setBrush(Qt::NoBrush);
    p.drawRect(reg);
  }

    private:
  QImage image_;
  QRectF region_;
  QString message_;
};

// ------------------------------------------------------------------- model --

struct ResourcesWorkspace::Entry {
  reusex::ProjectDB::SurveyPartRecord part;
  std::map<std::string, std::optional<std::string>> values;
  std::optional<core::ResourceView> view;
  QString type_name;
  QString unit;
  QStringList samples; ///< "P-01 · PCB i fugemasse · svar: ren"
};

struct ResourcesWorkspace::RowWidgets {
  int entry = -1;
  QPushButton *row = nullptr;
  ResourceThumb *thumb = nullptr;
};

struct ResourcesWorkspace::GroupWidgets {
  std::int64_t type_id = 0;
  QWidget *box = nullptr;
  QLabel *count = nullptr;
};

namespace {

QString sample_text(const reusex::ProjectDB::SampleRecord &s) {
  QString status = qs(std::string(core::to_string(s.stage)));
  if (s.result != core::SampleResult::none)
    status += ": " + qs(std::string(core::to_string(s.result)));
  return QString("%1 · %2 · %3").arg(qs(s.code), qs(s.title), status);
}

} // namespace

ResourcesWorkspace::ResourcesWorkspace(ProjectSession &session, bool persist,
                                       QWidget *parent)
    : QWidget(parent), session_(session), persist_(persist) {
  setObjectName("content");
  const Theme &t = theme();
  images_ = new ResourceImageLoader(this);
  connect(images_, &ResourceImageLoader::image_ready, this,
          &ResourcesWorkspace::image_ready);

  auto *outer = new QVBoxLayout(this);
  const int m = t.px("--space-6");
  outer->setContentsMargins(m, t.px("--space-5"), m, 0);
  outer->setSpacing(t.px("--space-3"));

  auto *head = new QHBoxLayout;
  head->setSpacing(t.px("--space-3"));
  head->addWidget(new CapsLabel("Ressourcer", "viewTitle", QString()), 0,
                  Qt::AlignBottom);
  summary_ = label({}, "viewSub");
  head->addWidget(summary_, 1, Qt::AlignBottom);
  outer->addLayout(head);

  // ---- toolbar: search, storey, room, template
  toolbar_ = new QWidget;
  auto *bar = new QHBoxLayout(toolbar_);
  bar->setContentsMargins(0, 0, 0, 0);
  bar->setSpacing(t.px("--space-2"));
  search_ = new QLineEdit;
  search_->setObjectName("resourceSearch");
  search_->setPlaceholderText("Søg i kode, betegnelse, type, rum, tilstand…");
  search_->setClearButtonEnabled(true);
  bar->addWidget(search_, 3);
  storey_combo_ = new QComboBox;
  storey_combo_->setObjectName("resourceStorey");
  bar->addWidget(storey_combo_, 1);
  room_combo_ = new QComboBox;
  room_combo_->setObjectName("resourceRoom");
  bar->addWidget(room_combo_, 1);
  bar->addSpacing(t.px("--space-3"));
  bar->addWidget(new CapsLabel("Skabelon", "fieldLabel"), 0, Qt::AlignVCenter);
  template_combo_ = new QComboBox;
  template_combo_->setObjectName("resourceTemplate");
  template_combo_->setToolTip(
      "Hvilke felter detaljesiden viser, i skabelonens rækkefølge");
  bar->addWidget(template_combo_, 1);
  outer->addWidget(toolbar_);

  connect(search_, &QLineEdit::textChanged, this, [this](const QString &q) {
    filter_.query = q.toStdString();
    apply_filter();
  });
  connect(storey_combo_, &QComboBox::currentIndexChanged, this, [this](int) {
    filter_.storey = storey_combo_->currentData().toString().toStdString();
    filter_.room.clear();
    rebuild_room_combo();
    apply_filter();
  });
  connect(room_combo_, &QComboBox::currentIndexChanged, this, [this](int) {
    filter_.room = room_combo_->currentData().toString().toStdString();
    apply_filter();
  });
  connect(template_combo_, &QComboBox::currentIndexChanged, this,
          &ResourcesWorkspace::choose_template);

  pages_ = new QStackedWidget;
  pages_->setObjectName("workspaceStack");
  outer->addWidget(pages_, 1);

  // ---- page 0: the list
  list_scroll_ = new QScrollArea;
  list_scroll_->setObjectName("resourceScroll");
  list_scroll_->setWidgetResizable(true);
  list_scroll_->setFrameShape(QFrame::NoFrame);
  auto *list_host = new QWidget;
  list_host->setObjectName("resourceList");
  list_body_ = new QVBoxLayout(list_host);
  list_body_->setContentsMargins(0, 0, t.px("--space-2"), m);
  list_body_->setSpacing(t.px("--space-4"));
  list_scroll_->setWidget(list_host);
  pages_->addWidget(list_scroll_);

  // ---- page 1: the empty state (no project, no survey, a read error)
  empty_ = new QWidget;
  auto *ev = new QVBoxLayout(empty_);
  auto *card = new QFrame;
  card->setObjectName("emptyCard");
  card->setMaximumWidth(2 * t.px("--layout-panel-width") + t.px("--space-7"));
  auto *c = new QVBoxLayout(card);
  const int p = t.px("--space-6");
  c->setContentsMargins(p, p, p, p);
  c->setSpacing(t.px("--space-3"));
  empty_title_ = label({}, "emptyTitle");
  c->addWidget(empty_title_);
  empty_text_ = label({}, "emptyText", true);
  c->addWidget(empty_text_);
  auto *centre = new QHBoxLayout;
  centre->addStretch(1);
  centre->addWidget(card, 4);
  centre->addStretch(1);
  ev->addStretch(1);
  ev->addLayout(centre);
  ev->addStretch(2);
  pages_->addWidget(empty_);

  // ---- page 2: the detail
  detail_page_ = new QWidget;
  auto *dv = new QVBoxLayout(detail_page_);
  dv->setContentsMargins(0, 0, 0, t.px("--space-5"));
  dv->setSpacing(t.px("--space-3"));
  auto *nav = new QHBoxLayout;
  nav->setSpacing(t.px("--space-2"));
  auto *back = button("‹ Alle ressourcer", "secondary");
  connect(back, &QPushButton::clicked, this, &ResourcesWorkspace::show_list);
  nav->addWidget(back);
  nav->addStretch(1);
  detail_pos_ = label({}, "metaText");
  nav->addWidget(detail_pos_);
  prev_ = button("‹ Forrige", "ghost");
  next_ = button("Næste ›", "ghost");
  connect(prev_, &QPushButton::clicked, this, [this] { step_detail(-1); });
  connect(next_, &QPushButton::clicked, this, [this] { step_detail(1); });
  nav->addWidget(prev_);
  nav->addWidget(next_);
  dv->addLayout(nav);
  detail_body_ = new QWidget;
  detail_body_->setObjectName("resourceDetail");
  new QVBoxLayout(detail_body_);
  detail_body_->layout()->setContentsMargins(0, 0, 0, 0);
  dv->addWidget(detail_body_, 1);
  pages_->addWidget(detail_page_);

  connect(&session_, &ProjectSession::opened, this, [this] {
    reset();
    if (isVisible())
      activate();
  });
  connect(&session_, &ProjectSession::closed, this, &ResourcesWorkspace::reset);
  reset();
}

ResourcesWorkspace::~ResourcesWorkspace() = default;

int ResourcesWorkspace::resource_count() const {
  return static_cast<int>(entries_.size());
}

void ResourcesWorkspace::reset() {
  loaded_ = false;
  error_.clear();
  entries_.clear();
  rows_.clear();
  types_.clear();
  catalogue_.clear();
  templates_.clear();
  template_index_ = -1;
  detail_ = -1;
  visible_order_.clear();
  row_widgets_.clear();
  group_widgets_.clear();
  clear_layout(list_body_);
  images_->set_project(session_.is_open() ? session_.path() : QString());
  filter_ = {};
  for (QComboBox *cb : {storey_combo_, room_combo_, template_combo_}) {
    cb->blockSignals(true);
    cb->clear();
    cb->blockSignals(false);
  }
  search_->blockSignals(true);
  search_->clear();
  search_->blockSignals(false);
  toolbar_->setVisible(false);
  if (!session_.is_open()) {
    summary_->setText("Ingen projekt åbent");
    empty_title_->setText("Intet projekt");
    empty_text_->setText("Åbn et projekt for at se dets ressourcer: de "
                         "kortlagte bygningsdele med foto, rum og tilstand.");
    pages_->setCurrentWidget(empty_);
  } else {
    summary_->clear();
  }
  refresh_selection();
}

void ResourcesWorkspace::activate() {
  if (loaded_ || !session_.is_open())
    return;
  loaded_ = true;
  load();
  build_list();
}

void ResourcesWorkspace::load() {
  const reusex::ProjectDB *db = session_.db();
  if (!db)
    return;
  try {
    // Types, in id order.
    std::map<std::int64_t, reusex::ProjectDB::SurveyTypeRecord> types;
    for (auto &st : db->survey_types()) {
      types_.push_back({st.id, st.name});
      types.emplace(st.id, std::move(st));
    }
    // Catalogue: labels, data types and units for every key.
    std::vector<KeyRef> refs;
    for (const auto &k : core::key_catalogue(*db)) {
      catalogue_.push_back({k.id, k.label, k.data_type, k.unit});
      refs.push_back({k.id, k.label, k.field});
    }
    const auto key_id = [&](const std::vector<std::string> &names) {
      const auto i = find_key(refs, names);
      return i ? refs[*i].id : std::string();
    };
    std::string designation_key = key_id(designation_key_names());
    const std::string condition_key = key_id(condition_key_names());
    const std::string potential_key = key_id(potential_key_names());
    const std::string storey_key = key_id(storey_key_names());

    // Storeys by room name.
    std::vector<std::pair<std::string, int>> rooms;
    std::vector<std::pair<int, std::string>> storeys;
    for (const auto &s : db->storeys())
      storeys.emplace_back(s.idx, s.name);
    for (const auto &sp : db->spaces())
      rooms.emplace_back(sp.name, sp.storey);
    const StoreyIndex storey_index(rooms, storeys);

    // Samples per part code and per type.
    std::map<std::string, QStringList> part_samples;
    std::map<std::int64_t, QStringList> type_samples;
    for (const auto &s : db->samples()) {
      for (const auto &code : s.part_codes)
        part_samples[code] << sample_text(s);
      for (const auto id : s.type_ids)
        type_samples[id] << sample_text(s) + " (typen)";
    }

    std::map<std::string, core::Resource> resources;
    for (auto &r : core::list_resources(*db))
      resources.emplace(r.code, std::move(r));

    for (auto &part : db->survey_parts()) {
      Entry e;
      const auto ti = types.find(part.type_id);
      e.type_name = ti != types.end() ? qs(ti->second.name) : QString();
      e.unit = ti != types.end() ? qs(ti->second.unit) : QString();
      if (const auto ri = resources.find(part.code); ri != resources.end()) {
        for (const auto &v : ri->second.values)
          e.values[v.key] = v.value;
        e.view = ri->second.view;
      }
      if (!e.view)
        e.view = part.view;
      e.samples = part_samples[part.code];
      e.samples += type_samples[part.type_id];

      const auto value = [&](const std::string &key) -> std::string {
        if (key.empty())
          return {};
        const auto it = e.values.find(key);
        return it != e.values.end() && it->second ? *it->second : std::string();
      };
      ResourceRowData row;
      row.code = part.code;
      row.type_id = part.type_id;
      row.type_name = e.type_name.toStdString();
      row.designation = value(designation_key);
      if (row.designation.empty())
        row.designation = row.type_name;
      row.room = part.room_name;
      if (const auto s = storey_index.lookup(part.room_name)) {
        row.storey_idx = s->first;
        row.storey = s->second;
      } else if (const std::string sv = value(storey_key); !sv.empty()) {
        // A hand-entered storey column: "1" reads as "Etage 1".
        const bool digits = std::all_of(sv.begin(), sv.end(), [](char ch) {
          return ch >= '0' && ch <= '9';
        });
        row.storey = digits ? "Etage " + sv : sv;
        row.storey_idx = digits ? std::stoi(sv) : 1000;
      }
      row.condition = value(condition_key);
      row.potential = value(potential_key);
      row.manual = !part.instance_guid && !part.element_guid;
      row.dismissed = part.dismissed;
      row.starred = part.starred;
      row.has_view = e.view.has_value();
      e.part = std::move(part);
      rows_.push_back(std::move(row));
      entries_.push_back(std::move(e));
    }

    // Templates.
    std::vector<TemplateRef> refs_t;
    for (const auto &tv : core::template_views(*db)) {
      templates_.push_back(
          {tv.record.id, qs(tv.record.name), tv.resolved.keys});
      refs_t.push_back({tv.record.id, tv.record.name,
                        tv.record.seed.value_or(std::string())});
    }
    std::optional<std::int64_t> remembered;
    if (persist_) {
      const QVariant v =
          QSettings().value("ressourcer/template/" + session_.path());
      if (v.isValid())
        remembered = v.toLongLong();
    }
    if (const auto chosen = default_template(refs_t, remembered))
      for (std::size_t i = 0; i < templates_.size(); ++i)
        if (templates_[i].id == *chosen)
          template_index_ = static_cast<int>(i);
  } catch (const std::exception &e) {
    error_ = QString::fromUtf8(e.what());
    entries_.clear();
    rows_.clear();
  }
}

void ResourcesWorkspace::build_list() {
  const Theme &t = theme();
  clear_layout(list_body_);
  row_widgets_.clear();
  group_widgets_.clear();
  if (!error_.isEmpty() || entries_.empty()) {
    if (!error_.isEmpty()) {
      empty_title_->setText("Ressourcerne kunne ikke læses");
      empty_text_->setText(error_);
    } else {
      empty_title_->setText("Ingen ressourcer endnu");
      empty_text_->setText(
          "Projektet har ingen kortlagte ressourcer. Kør rux create survey "
          "for at oprette typer og ressourcer ud fra instanserne, eller "
          "tilføj dem i webklienten.");
    }
    summary_->setText(error_.isEmpty() ? "0 ressourcer" : "Fejl");
    pages_->setCurrentWidget(empty_);
    return;
  }
  toolbar_->setVisible(true);

  // Facets.
  storey_combo_->blockSignals(true);
  storey_combo_->clear();
  storey_combo_->addItem("Alle etager", QString());
  for (const auto &o : storey_options(rows_)) {
    if (o.value.empty())
      continue; // "unknown storey" is not a useful filter
    storey_combo_->addItem(
        QString("%1 (%2)").arg(qs(o.value)).arg(static_cast<int>(o.count)),
        qs(o.value));
  }
  storey_combo_->setEnabled(storey_combo_->count() > 1);
  storey_combo_->blockSignals(false);
  rebuild_room_combo();
  template_combo_->blockSignals(true);
  template_combo_->clear();
  for (const auto &tp : templates_)
    template_combo_->addItem(tp.name);
  template_combo_->setCurrentIndex(template_index_);
  template_combo_->setEnabled(!templates_.empty());
  template_combo_->blockSignals(false);

  // Groups and rows (all of them; filtering only hides).
  ResourceFilter all;
  all.show_dismissed = true;
  for (const auto &g : group_resources(rows_, types_, all)) {
    auto gw = std::make_unique<GroupWidgets>();
    gw->type_id = g.type_id;
    gw->box = new QWidget;
    gw->box->setObjectName("resourceGroup");
    auto *gv = new QVBoxLayout(gw->box);
    gv->setContentsMargins(0, 0, 0, 0);
    gv->setSpacing(t.px("--space-2"));
    auto *gh = new QHBoxLayout;
    gh->setSpacing(t.px("--space-3"));
    gh->addWidget(label(qs(g.type_name), "groupTitle"), 0, Qt::AlignBottom);
    gw->count = label({}, "groupCount");
    gh->addWidget(gw->count, 0, Qt::AlignBottom);
    gh->addStretch(1);
    gv->addLayout(gh);

    auto *panel = new QFrame;
    panel->setObjectName("resourceRows");
    auto *pv = new QVBoxLayout(panel);
    pv->setContentsMargins(0, 0, 0, 0);
    pv->setSpacing(0);
    // One column grid for the header and every row: thumbnail, name, place,
    // condition, potential, chevron.
    const int grade_w = t.px("--space-7") * 3;
    const int thumb_w = t.px("--space-7") * 2 + t.px("--space-4");
    const auto row_layout = [&](QWidget *host) {
      auto *h = new QHBoxLayout(host);
      h->setContentsMargins(t.px("--space-3"), t.px("--space-2"),
                            t.px("--space-4"), t.px("--space-2"));
      h->setSpacing(t.px("--space-4"));
      return h;
    };
    {
      auto *hdr = new QWidget;
      hdr->setObjectName("resourceHead");
      auto *h = row_layout(hdr);
      h->addSpacing(thumb_w + h->spacing());
      h->addWidget(new CapsLabel("Ressource", "fieldLabel"), 3);
      h->addWidget(new CapsLabel("Placering", "fieldLabel"), 2);
      for (const char *c : {"Tilstand", "Genbrugspotentiale"}) {
        auto *l = new CapsLabel(c, "fieldLabel");
        l->setFixedWidth(grade_w);
        h->addWidget(l);
      }
      // Keeps the columns over the rows' chevron.
      auto *chev = label("›", "rowChevron");
      QSizePolicy sp = chev->sizePolicy();
      sp.setRetainSizeWhenHidden(true);
      chev->setSizePolicy(sp);
      chev->hide();
      h->addWidget(chev);
      pv->addWidget(hdr);
    }
    for (const auto i : g.rows) {
      const ResourceRowData &r = rows_[i];
      auto rw = std::make_unique<RowWidgets>();
      rw->entry = static_cast<int>(i);
      rw->row = new QPushButton;
      rw->row->setObjectName("resourceRow");
      rw->row->setProperty("dismissed", r.dismissed);
      rw->row->setCursor(Qt::PointingHandCursor);
      auto *h = row_layout(rw->row);
      rw->thumb = new ResourceThumb;
      if (!r.has_view)
        rw->thumb->set_missing("Intet billede");
      h->addWidget(rw->thumb);

      auto *name_col = new QVBoxLayout;
      name_col->setSpacing(t.px("--space-1"));
      name_col->addStretch(1);
      auto *code_row = new QHBoxLayout;
      code_row->setSpacing(t.px("--space-2"));
      code_row->addWidget(label(qs(r.code), "rowCode"));
      if (r.starred)
        code_row->addWidget(label("★ Vigtig", "rowStar"));
      if (r.dismissed)
        code_row->addWidget(new Pill("Fejlmarkeret", "crit"));
      code_row->addStretch(1);
      name_col->addLayout(code_row);
      auto *name = new ElidedLabel(qs(r.designation), Qt::ElideRight);
      name->setObjectName("rowName");
      name_col->addWidget(name);
      name_col->addStretch(1);
      h->addLayout(name_col, 3);

      auto *place = new QVBoxLayout;
      place->setSpacing(t.px("--space-1"));
      place->addStretch(1);
      auto *room = new ElidedLabel(
          r.room.empty() ? QString("Intet rum") : qs(r.room), Qt::ElideRight);
      room->setObjectName(r.room.empty() ? "rowMuted" : "rowRoom");
      place->addWidget(room);
      place->addWidget(
          label(r.storey.empty() ? QString("Etage ukendt") : qs(r.storey),
                "rowMuted"));
      place->addStretch(1);
      h->addLayout(place, 2);

      const auto grade = [&](const std::string &v) -> QWidget * {
        QLabel *l = v.empty() ? label("—", "rowMuted")
                              : new Pill(qs(v), qs(grade_tone(v)));
        auto *w = new QWidget;
        auto *col = new QHBoxLayout(w);
        col->setContentsMargins(0, 0, 0, 0);
        col->addWidget(l, 0, Qt::AlignLeft | Qt::AlignVCenter);
        col->addStretch(1);
        w->setAttribute(Qt::WA_TransparentForMouseEvents);
        w->setFixedWidth(grade_w);
        return w;
      };
      h->addWidget(grade(r.condition));
      h->addWidget(grade(r.potential));
      h->addWidget(label("›", "rowChevron"));
      for (QLabel *l : rw->row->findChildren<QLabel *>())
        l->setAttribute(Qt::WA_TransparentForMouseEvents);
      rw->row->setMinimumHeight(h->sizeHint().height());
      const int entry = rw->entry;
      connect(rw->row, &QPushButton::clicked, this,
              [this, entry] { show_detail(entry); });
      pv->addWidget(rw->row);
      row_widgets_.push_back(std::move(rw));
    }
    gv->addWidget(panel);
    list_body_->addWidget(gw->box);
    group_widgets_.push_back(std::move(gw));
  }
  no_match_ = label("Ingen ressourcer passer på søgningen og filtrene.",
                    "emptyText", true);
  list_body_->addWidget(no_match_);
  list_body_->addStretch(1);
  pages_->setCurrentWidget(list_scroll_);
  apply_filter();
  request_thumbnails();
}

void ResourcesWorkspace::rebuild_room_combo() {
  room_combo_->blockSignals(true);
  room_combo_->clear();
  room_combo_->addItem("Alle rum", QString());
  int current = 0;
  for (const auto &o : room_options(rows_, filter_.storey)) {
    room_combo_->addItem(
        QString("%1 (%2)").arg(qs(o.value)).arg(static_cast<int>(o.count)),
        qs(o.value));
    if (o.value == filter_.room)
      current = room_combo_->count() - 1;
  }
  room_combo_->setCurrentIndex(current);
  room_combo_->setEnabled(room_combo_->count() > 1);
  room_combo_->blockSignals(false);
}

void ResourcesWorkspace::apply_filter() {
  if (entries_.empty())
    return;
  const auto groups = group_resources(rows_, types_, filter_);
  std::set<std::size_t> shown;
  std::map<std::int64_t, const ResourceGroupView *> by_type;
  visible_order_.clear();
  for (const auto &g : groups) {
    by_type[g.type_id] = &g;
    for (const auto i : g.rows) {
      shown.insert(i);
      visible_order_.push_back(static_cast<int>(i));
    }
  }
  for (auto &gw : group_widgets_) {
    const auto it = by_type.find(gw->type_id);
    gw->box->setVisible(it != by_type.end());
    if (it == by_type.end())
      continue;
    const auto &g = *it->second;
    gw->count->setText(filter_.active()
                           ? QString("%1 af %2").arg(g.rows.size()).arg(g.total)
                           : qs(resource_count_text(g.total)));
  }
  for (auto &rw : row_widgets_)
    rw->row->setVisible(shown.count(static_cast<std::size_t>(rw->entry)) > 0);
  std::size_t total = 0;
  for (const auto &r : rows_)
    total += r.dismissed ? 0 : 1;
  summary_->setText(qs(list_summary(groups, total, filter_.active())));
  if (no_match_)
    no_match_->setVisible(groups.empty());
}

void ResourcesWorkspace::request_thumbnails() {
  const Theme &t = theme();
  const int px = (t.px("--space-7") * 2 + t.px("--space-4")) *
                 static_cast<int>(std::ceil(devicePixelRatioF()));
  for (std::size_t i = 0; i < entries_.size(); ++i) {
    const Entry &e = entries_[i];
    if (!e.view)
      continue;
    ResourceImageRequest r;
    r.key = "thumb:" + qs(e.part.code);
    r.panorama = e.view->kind == core::ViewKind::panorama;
    r.id = e.view->id;
    r.region = e.view->region;
    r.thumbnail = true;
    r.aspect = 4.0 / 3.0;
    r.max_px = std::max(px, 2 * px);
    images_->request(r);
  }
}

void ResourcesWorkspace::image_ready(const QString &key) {
  const ResourceImage *img = images_->image(key);
  if (!img)
    return;
  if (key.startsWith("thumb:")) {
    const std::string code = key.mid(6).toStdString();
    for (auto &rw : row_widgets_)
      if (entries_[static_cast<std::size_t>(rw->entry)].part.code == code) {
        if (img->image.isNull())
          rw->thumb->set_missing("Billedet mangler");
        else
          rw->thumb->set_image(img->image);
      }
    if (detail_ >= 0 &&
        entries_[static_cast<std::size_t>(detail_)].part.code == code)
      refresh_selection(); // the inspector's preview
    return;
  }
  if (key.startsWith("full:") && detail_ >= 0 && photo_ &&
      key.mid(5).toStdString() ==
          entries_[static_cast<std::size_t>(detail_)].part.code)
    photo_->set_image(img);
}

QString ResourcesWorkspace::value_text(const Entry &e,
                                       const KeyInfo &key) const {
  const auto it = e.values.find(key.id);
  const std::optional<std::string> v =
      it != e.values.end() ? it->second : std::nullopt;
  if (v && !v->empty()) {
    if (key.id == "sys:view") {
      try {
        return qs(core::view_display(core::parse_view(*v)));
      } catch (const std::exception &) {
        return qs(*v);
      }
    }
    if (key.id == "sys:treatment")
      if (const auto tr = core::treatment_from_string(*v))
        return qs(std::string(core::treatment_label_da(*tr)));
    if (key.id == "sys:environment") {
      for (auto s : {core::EnvironmentStatus::ren_screening,
                     core::EnvironmentStatus::afventer,
                     core::EnvironmentStatus::forurenet,
                     core::EnvironmentStatus::ren_proevesvar})
        if (core::to_string(s) == *v)
          return qs(std::string(core::environment_label_da(s)));
    }
  }
  return qs(display_value(key.data_type, key.unit, v));
}

void ResourcesWorkspace::show_detail(int index) {
  if (index < 0 || index >= static_cast<int>(entries_.size()))
    return;
  const Theme &t = theme();
  detail_ = index;
  const Entry &e = entries_[static_cast<std::size_t>(index)];
  const ResourceRowData &r = rows_[static_cast<std::size_t>(index)];
  toolbar_->setVisible(false);
  pages_->setCurrentWidget(detail_page_);

  const auto pos =
      std::find(visible_order_.begin(), visible_order_.end(), index);
  const int at = pos == visible_order_.end()
                     ? -1
                     : static_cast<int>(pos - visible_order_.begin());
  detail_pos_->setText(
      at < 0 ? QString()
             : QString("%1 af %2").arg(at + 1).arg(visible_order_.size()));
  prev_->setEnabled(at > 0);
  next_->setEnabled(at >= 0 &&
                    at + 1 < static_cast<int>(visible_order_.size()));

  auto *body = static_cast<QVBoxLayout *>(detail_body_->layout());
  clear_layout(body);
  photo_ = nullptr;
  body->setSpacing(t.px("--space-4"));

  // ---- header: code, designation, where, grades
  auto *head = new QVBoxLayout;
  head->setSpacing(t.px("--space-1"));
  auto *code_row = new QHBoxLayout;
  code_row->setSpacing(t.px("--space-2"));
  code_row->addWidget(label(qs(r.code), "detailCode"));
  code_row->addWidget(label(e.type_name, "detailType"));
  code_row->addStretch(1);
  head->addLayout(code_row);
  auto *title = label(qs(r.designation), "detailTitle", true);
  head->addWidget(title);
  QStringList where;
  where << (r.room.empty() ? QString("Intet rum") : qs(r.room));
  if (!r.storey.empty())
    where << qs(r.storey);
  head->addWidget(label(where.join(" · "), "detailWhere"));
  auto *pills = new QHBoxLayout;
  pills->setContentsMargins(0, t.px("--space-2"), 0, 0);
  pills->setSpacing(t.px("--space-2"));
  if (!r.condition.empty())
    pills->addWidget(
        new Pill("Tilstand: " + qs(r.condition), qs(grade_tone(r.condition))));
  if (!r.potential.empty())
    pills->addWidget(new Pill("Genbrugspotentiale: " + qs(r.potential),
                              qs(grade_tone(r.potential))));
  if (r.starred)
    pills->addWidget(new Pill("★ Vigtig", "accent"));
  if (r.manual)
    pills->addWidget(new Pill("Tilføjet manuelt", "outline"));
  if (r.dismissed)
    pills->addWidget(new Pill("Fejlmarkeret", "crit"));
  pills->addStretch(1);
  head->addLayout(pills);
  body->addLayout(head);

  // ---- photo left, facts right
  auto *cols = new QHBoxLayout;
  cols->setSpacing(t.px("--space-4"));
  auto *photo_panel = new Panel("Ressourcebillede");
  photo_ = new ResourcePhoto;
  photo_panel->body()->addWidget(photo_, 1);
  photo_caption_ = label({}, "metaText", true);
  photo_panel->body()->addWidget(photo_caption_);
  if (e.view) {
    const bool pano = e.view->kind == core::ViewKind::panorama;
    photo_caption_->setText(
        qs(core::view_display(*e.view)) +
        (e.view->region ? QString(" · markeret udsnit")
                        : QString(" · hele billedet")) +
        (pano ? QString(" · udsnit af 360°-optagelsen") : QString()));
    ResourceImageRequest req;
    req.key = "full:" + qs(e.part.code);
    req.panorama = pano;
    req.id = e.view->id;
    req.region = e.view->region;
    req.thumbnail = false;
    req.aspect = 4.0 / 3.0;
    req.max_px = 1600;
    if (const ResourceImage *img = images_->image(req.key))
      photo_->set_image(img);
    else
      images_->request(req);
  } else {
    photo_->set_message("Intet billede valgt. Vælg et foto eller en "
                        "360°-optagelse med »Brug som ressourcebillede« i "
                        "webklienten.");
    photo_caption_->setText("Ingen sys:view");
  }
  cols->addWidget(photo_panel, 5);

  auto *facts_scroll = new QScrollArea;
  facts_scroll->setObjectName("resourceFacts");
  facts_scroll->setWidgetResizable(true);
  facts_scroll->setFrameShape(QFrame::NoFrame);
  facts_scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  auto *facts = new QWidget;
  facts->setObjectName("resourceFactsBody");
  auto *fv = new QVBoxLayout(facts);
  fv->setContentsMargins(0, 0, t.px("--space-2"), 0);
  fv->setSpacing(t.px("--space-3"));
  const auto grid_panel =
      [&](const QString &title_text,
          const std::vector<std::pair<QString, QString>> &rows_kv) {
        auto *panel = new Panel(title_text);
        auto *g = new QGridLayout;
        g->setHorizontalSpacing(t.px("--space-3"));
        g->setVerticalSpacing(t.px("--space-2"));
        g->setColumnStretch(1, 1);
        int row = 0;
        for (const auto &[k, v] : rows_kv) {
          auto *kl = new CapsLabel(k, "fieldLabel");
          kl->setToolTip(k);
          g->addWidget(kl, row, 0, Qt::AlignLeft | Qt::AlignTop);
          auto *vl =
              label(v, v == "—" ? "detailValueMuted" : "detailValue", true);
          vl->setTextInteractionFlags(Qt::TextSelectableByMouse);
          g->addWidget(vl, row, 1);
          ++row;
        }
        panel->body()->addLayout(g);
        fv->addWidget(panel);
        return panel;
      };

  const QString quantity = qs(display_value(
      "number", "",
      std::optional<std::string>(core::format_number(e.part.quantity))));
  grid_panel("Placering",
             {{"Rum", r.room.empty() ? QString("—") : qs(r.room)},
              {"Etage", r.storey.empty() ? QString("—") : qs(r.storey)},
              {"Type", e.type_name},
              {"Mængde", quantity + " " + e.unit}});

  if (template_index_ >= 0) {
    const auto &tp = templates_[static_cast<std::size_t>(template_index_)];
    std::vector<std::pair<QString, QString>> kv;
    for (const auto &id : tp.keys) {
      const auto it =
          std::find_if(catalogue_.begin(), catalogue_.end(),
                       [&](const KeyInfo &k) { return k.id == id; });
      if (it == catalogue_.end())
        continue;
      kv.emplace_back(qs(it->label), value_text(e, *it));
    }
    auto *p = grid_panel(QString("Egenskaber · %1").arg(tp.name), kv);
    p->setToolTip("Felterne i skabelonen, i dens rækkefølge");
  } else {
    grid_panel("Egenskaber", {{"Skabelon", "Projektet har ingen skabeloner"}});
  }

  std::vector<std::pair<QString, QString>> links;
  if (e.part.instance_guid) {
    QString inst = e.part.cloud_name && e.part.instance_id
                       ? QString("%1 #%2")
                             .arg(qs(*e.part.cloud_name))
                             .arg(*e.part.instance_id)
                       : QString("Instans");
    links.emplace_back("Instans", inst);
    links.emplace_back("Instans-GUID", qs(*e.part.instance_guid));
  } else {
    links.emplace_back("Instans", e.part.element_guid
                                      ? QString("—")
                                      : QString("Ingen — tilføjet manuelt"));
  }
  if (e.part.element_guid)
    links.emplace_back("Bygningsdel", qs(*e.part.element_guid));
  links.emplace_back("Prøver", e.samples.isEmpty()
                                   ? QString("Ingen prøver koblet")
                                   : e.samples.join("\n"));
  grid_panel("Koblinger", links);
  fv->addStretch(1);
  facts_scroll->setWidget(facts);
  facts_scroll->setFixedWidth(t.px("--layout-bench-aside-width") +
                              t.px("--space-2"));
  cols->addWidget(facts_scroll);
  body->addLayout(cols, 1);

  // The inspector: the resource in short, with its thumbnail.
  refresh_selection();
}

void ResourcesWorkspace::refresh_selection() {
  Selection s;
  if (detail_ >= 0 && detail_ < static_cast<int>(entries_.size())) {
    const Entry &e = entries_[static_cast<std::size_t>(detail_)];
    const ResourceRowData &r = rows_[static_cast<std::size_t>(detail_)];
    s.kind = "Ressource";
    s.title = qs(r.code);
    s.subtitle = qs(r.designation);
    if (const ResourceImage *img = images_->image("thumb:" + qs(r.code)))
      s.preview = img->image;
    SelectionSection sec;
    sec.title = "Ressource";
    sec.rows = {{"Type", e.type_name, SelectionRow::Style::value},
                {"Rum", r.room.empty() ? QString("—") : qs(r.room),
                 SelectionRow::Style::value},
                {"Etage", r.storey.empty() ? QString("—") : qs(r.storey),
                 SelectionRow::Style::value},
                {"Billede", e.view ? qs(core::view_display(*e.view))
                                   : QString("Intet billede valgt")}};
    s.sections << sec;
  }
  selection_ = s;
  emit selection_changed(selection_);
}

void ResourcesWorkspace::step_detail(int delta) {
  const auto pos =
      std::find(visible_order_.begin(), visible_order_.end(), detail_);
  if (pos == visible_order_.end())
    return;
  const long at = (pos - visible_order_.begin()) + delta;
  if (at < 0 || at >= static_cast<long>(visible_order_.size()))
    return;
  show_detail(visible_order_[static_cast<std::size_t>(at)]);
}

void ResourcesWorkspace::show_list() {
  detail_ = -1;
  photo_ = nullptr;
  if (entries_.empty())
    return;
  toolbar_->setVisible(true);
  pages_->setCurrentWidget(list_scroll_);
  refresh_selection();
}

void ResourcesWorkspace::choose_template(int combo_index) {
  if (combo_index < 0 || combo_index >= static_cast<int>(templates_.size()))
    return;
  template_index_ = combo_index;
  if (persist_)
    QSettings().setValue(
        "ressourcer/template/" + session_.path(),
        static_cast<qlonglong>(
            templates_[static_cast<std::size_t>(combo_index)].id));
  if (detail_ >= 0)
    show_detail(detail_);
}

void ResourcesWorkspace::set_query(const QString &text) {
  activate();
  search_->setText(text);
}

void ResourcesWorkspace::set_storey_filter(const QString &storey) {
  activate();
  const int i = storey_combo_->findData(storey);
  if (i >= 0)
    storey_combo_->setCurrentIndex(i);
}

void ResourcesWorkspace::set_room_filter(const QString &room) {
  activate();
  const int i = room_combo_->findData(room);
  if (i >= 0)
    room_combo_->setCurrentIndex(i);
}

bool ResourcesWorkspace::open_resource(const QString &code) {
  activate();
  for (std::size_t i = 0; i < entries_.size(); ++i)
    if (qs(entries_[i].part.code) == code) {
      show_detail(static_cast<int>(i));
      return true;
    }
  return false;
}

QStringList ResourcesWorkspace::find_codes(const QString &text) const {
  QStringList out;
  const std::string needle = fold_da(text.toStdString());
  for (const auto &r : rows_)
    if (fold_da(r.type_name + " " + r.designation).find(needle) !=
        std::string::npos)
      out << qs(r.code);
  return out;
}

} // namespace rux::qt
