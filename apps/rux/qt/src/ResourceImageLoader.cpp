// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/ResourceImageLoader.hpp>

#include <reusex/core/ProjectDB.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <QMetaObject>

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

namespace rux::qt {
namespace {

std::atomic<int> g_busy{0};

/// Counts one queued request until its result is delivered (or dropped).
struct BusyMark {
  BusyMark() { ++g_busy; }
  ~BusyMark() { --g_busy; }
  BusyMark(const BusyMark &) = delete;
  BusyMark &operator=(const BusyMark &) = delete;
};

QImage to_qimage(const cv::Mat &m) {
  if (m.empty())
    return {};
  cv::Mat rgb;
  if (m.type() == CV_8UC3)
    cv::cvtColor(m, rgb, cv::COLOR_BGR2RGB);
  else if (m.type() == CV_8UC4)
    cv::cvtColor(m, rgb, cv::COLOR_BGRA2RGB);
  else if (m.type() == CV_8UC1)
    cv::cvtColor(m, rgb, cv::COLOR_GRAY2RGB);
  else
    return {};
  QImage img(rgb.data, rgb.cols, rgb.rows, static_cast<int>(rgb.step),
             QImage::Format_RGB888);
  return img.copy();
}

cv::Mat scaled_to(const cv::Mat &m, int max_px) {
  const int longest = std::max(m.cols, m.rows);
  if (max_px <= 0 || longest <= max_px)
    return m;
  const double s = static_cast<double>(max_px) / longest;
  cv::Mat out;
  cv::resize(m, out, cv::Size(), s, s, cv::INTER_AREA);
  return out;
}

ResourceImage decode(const reusex::ProjectDB &db,
                     const ResourceImageRequest &r) {
  ResourceImage out;
  const cv::Mat src =
      r.panorama ? db.panoramic_image(r.id) : db.sensor_frame_image(r.id);
  if (src.empty()) {
    out.error = r.panorama
                    ? QString("360°-billede %1 findes ikke").arg(r.id)
                    : QString("Foto %1 har intet farvebillede").arg(r.id);
    return out;
  }
  const int iw = src.cols, ih = src.rows;
  const bool crop = r.thumbnail || r.panorama;
  if (!crop) {
    out.image = to_qimage(scaled_to(src, r.max_px));
    if (r.region) {
      const PixelRect px = region_pixels(r.region, iw, ih);
      out.region = QRectF(
          static_cast<double>(px.x) / iw, static_cast<double>(px.y) / ih,
          static_cast<double>(px.w) / iw, static_cast<double>(px.h) / ih);
    }
    return out;
  }
  const auto pieces = thumbnail_crop(r.region, iw, ih, r.aspect, r.panorama);
  if (pieces.empty())
    return out;
  cv::Mat joined;
  if (pieces.size() == 1) {
    const auto &p = pieces.front();
    joined = src(cv::Rect(p.x, p.y, p.w, p.h));
  } else {
    std::vector<cv::Mat> parts;
    for (const auto &p : pieces)
      parts.push_back(src(cv::Rect(p.x, p.y, p.w, p.h)));
    cv::hconcat(parts, joined);
  }
  out.image = to_qimage(scaled_to(joined, r.max_px));
  if (r.region) {
    const PixelRect in = region_in_crop(r.region, iw, ih, pieces);
    const double cw = joined.cols, ch = joined.rows;
    out.region = QRectF(in.x / cw, in.y / ch, in.w / cw, in.h / ch)
                     .intersected(QRectF(0, 0, 1, 1));
  }
  return out;
}

} // namespace

struct ResourceImageLoader::Worker {
  struct Item {
    ResourceImageRequest request;
    std::shared_ptr<BusyMark> marker;
  };

  explicit Worker(ResourceImageLoader *owner) : owner_(owner) {
    thread_ = std::thread([this] { run(); });
  }
  ~Worker() {
    {
      std::lock_guard<std::mutex> lock(m_);
      stop_ = true;
      queue_.clear();
    }
    cv_.notify_all();
    thread_.join();
  }

  void configure(const QString &path, unsigned generation) {
    std::lock_guard<std::mutex> lock(m_);
    path_ = path.toStdString();
    generation_ = generation;
    reopen_ = true;
    queue_.clear();
  }

  bool queued(const QString &key) {
    std::lock_guard<std::mutex> lock(m_);
    return std::any_of(queue_.begin(), queue_.end(),
                       [&](const Item &i) { return i.request.key == key; });
  }

  void push(const ResourceImageRequest &r) {
    {
      std::lock_guard<std::mutex> lock(m_);
      if (path_.empty())
        return;
      Item item{r, std::make_shared<BusyMark>()};
      if (r.thumbnail)
        queue_.push_back(std::move(item));
      else
        queue_.push_front(std::move(item)); // the photo the user opened
    }
    cv_.notify_one();
  }

    private:
  void run() {
    std::unique_ptr<reusex::ProjectDB> db;
    std::string open_path;
    for (;;) {
      Item item;
      std::string path;
      unsigned generation = 0;
      {
        std::unique_lock<std::mutex> lock(m_);
        cv_.wait(lock, [this] { return stop_ || !queue_.empty() || reopen_; });
        if (stop_)
          return;
        if (reopen_) {
          reopen_ = false;
          db.reset();
          open_path.clear();
        }
        if (queue_.empty())
          continue;
        item = std::move(queue_.front());
        queue_.pop_front();
        path = path_;
        generation = generation_;
      }
      ResourceImage img;
      try {
        if (!db || open_path != path) {
          db = std::make_unique<reusex::ProjectDB>(path, /*readOnly=*/true);
          open_path = path;
        }
        img = decode(*db, item.request);
      } catch (const std::exception &e) {
        img.error = QString::fromUtf8(e.what());
      }
      // Deliver on the GUI thread; the marker ends with the delivery, so a
      // waiting gallery sees the image land before it shoots.
      QMetaObject::invokeMethod(
          owner_,
          [o = owner_, key = item.request.key, img, generation,
           marker = item.marker] { o->deliver(key, img, generation); },
          Qt::QueuedConnection);
    }
  }

  ResourceImageLoader *owner_;
  std::thread thread_;
  std::mutex m_;
  std::condition_variable cv_;
  std::deque<Item> queue_;
  bool stop_ = false;
  bool reopen_ = false;
  std::string path_;
  unsigned generation_ = 0;
};

ResourceImageLoader::ResourceImageLoader(QObject *parent)
    : QObject(parent), worker_(std::make_unique<Worker>(this)) {}

ResourceImageLoader::~ResourceImageLoader() { worker_.reset(); }

int ResourceImageLoader::busy() { return g_busy.load(); }

void ResourceImageLoader::set_project(const QString &path) {
  path_ = path;
  ++generation_;
  cache_.clear();
  worker_->configure(path, generation_);
}

const ResourceImage *ResourceImageLoader::image(const QString &key) const {
  const auto it = cache_.constFind(key);
  return it == cache_.constEnd() ? nullptr : &*it;
}

void ResourceImageLoader::request(const ResourceImageRequest &r) {
  if (cache_.contains(r.key) || worker_->queued(r.key))
    return;
  worker_->push(r);
}

void ResourceImageLoader::deliver(const QString &key, const ResourceImage &img,
                                  unsigned generation) {
  if (generation != generation_)
    return;
  cache_.insert(key, img);
  emit image_ready(key);
}

} // namespace rux::qt
