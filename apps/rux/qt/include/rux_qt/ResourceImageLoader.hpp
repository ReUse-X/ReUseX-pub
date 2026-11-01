// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Decodes the images of the Ressourcer workspace OFF the GUI thread: each
// resource's chosen image (its `sys:view` record, a sensor frame or a 360°
// panorama) as a small thumbnail cropped to the view's region, and the large
// photo of the detail page with the region marked. One worker thread with its
// OWN read-only ProjectDB (ProjectDB is not thread-safe); requests are served
// in the order they came, except that a detail photo jumps the queue, so the
// photo the user opened never waits behind 200 thumbnails.
//
// busy() counts the requests queued or not yet delivered, process-wide; the
// gallery waits for 0 before it shoots. The thread is joined when the loader
// goes, so (unlike BackgroundWork) nothing outlives the window.

#include <rux_qt/resource_list.hpp>

#include <QHash>
#include <QImage>
#include <QObject>
#include <QRectF>
#include <QString>

#include <memory>
#include <optional>

namespace rux::qt {

struct ResourceImageRequest {
  QString key;           ///< cache key, also what image_ready names
  bool panorama = false; ///< panoramic_images.id, else sensor_frames.node_id
  int id = 0;
  std::optional<NormRegion> region;
  /// true: a thumbnail cropped to the region (thumbnail_crop) at @p aspect;
  /// false: the detail photo — a frame whole, a panorama cropped like a
  /// thumbnail (an equirect is unreadable whole), with the region marked.
  bool thumbnail = true;
  double aspect = 4.0 / 3.0;
  int max_px = 256; ///< the longer side of the result, device pixels
};

struct ResourceImage {
  QImage image;
  /// Where the region lies in @ref image, normalised 0..1; null when the
  /// whole image is the view (no region).
  QRectF region;
  QString error; ///< why there is no image (Danish), empty on success
};

class ResourceImageLoader : public QObject {
  Q_OBJECT
    public:
  explicit ResourceImageLoader(QObject *parent = nullptr);
  ~ResourceImageLoader() override;

  /// Point the worker at a project file (empty = none). Clears the cache.
  void set_project(const QString &path);

  /// The cached result for @p key, or nullptr (then request() it).
  const ResourceImage *image(const QString &key) const;
  /// Queue @p r unless it is cached or already queued.
  void request(const ResourceImageRequest &r);

  /// Requests queued or decoding, across every loader in the process.
  static int busy();

    signals:
  void image_ready(const QString &key);

    private:
  struct Worker;
  void deliver(const QString &key, const ResourceImage &img,
               unsigned generation);

  std::unique_ptr<Worker> worker_;
  QHash<QString, ResourceImage> cache_;
  QString path_;
  unsigned generation_ = 0;
};

} // namespace rux::qt
