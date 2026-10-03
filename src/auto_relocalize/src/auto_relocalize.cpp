/*
 * auto_relocalize: 双σ似然场粗搜+精搜，按「定位置信度(黑点命中)」取最高位姿。
 *
 * 状态机:
 *   1. 启动/手动重定位: 粗搜(宽σ辅助) → 精搜(窄σ辅助) → 发布最高置信度位姿
 *   2. ≥0.60 → locked(aligned)，进入看门狗
 *   3. 0.35~0.60 → mid_wait：只监测置信度；15s 仍不到 0.60 → ask_manual
 *      ask_manual: 网页弹窗——否=承认当前位姿并锁定；是=用户手动设姿后锁定
 *   4. locked 后: ≥0.60 健康；0.35~0.60 只更新置信度不重搜；
 *      <0.35 连续 N 次 → 再全图重定位，发布最高置信度位姿，回到 2/3
 *   5. 导航结束后自动纠偏默认关闭（post_nav_hit<=0）；开启时也仅当搜到
 *      ≥min_black_hit 才下发 /initialpose，避免把正确终点拽到错误峰
 */
#include <cmath>
#include <cstdio>
#include <vector>
#include <thread>
#include <atomic>
#include <algorithm>
#include <mutex>
#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

struct Candidate {
  double x = 0, y = 0, yaw = 0;
  double score = -1.0;     // 似然场分(粗=宽σ / 精=窄σ)，仅并列打破
  double black_hit = 0.0;  // 定位置信度：主排序 + 门槛
  double tight = 0.0;
};

enum class Phase {
  Boot,       // 等地图/激光/AMCL
  Searching,  // 正在全图搜
  MidWait,    // 首轮/重搜后置信度在 [trigger, success)
  AskManual,  // 已提示用户是否手动设姿
  Locked      // 定位就绪，看门狗监测
};

class AutoRelocalize : public rclcpp::Node {
public:
  AutoRelocalize() : Node("auto_relocalize") {
    auto_on_startup_ = declare_parameter<bool>("auto_on_startup", true);
    accept_score_ = declare_parameter<double>("accept_score", 0.40);
    min_black_hit_ = declare_parameter<double>("min_black_hit", 0.60);
    reloc_trigger_hit_ = declare_parameter<double>("reloc_trigger_hit", 0.35);
    // 端点落在障碍「附近」也算命中（纯占格对 5cm 图过严，正确位姿常只有 ~30%）
    black_hit_dist_ = declare_parameter<double>("black_hit_dist", 0.15);
    mid_wait_sec_ = declare_parameter<double>("mid_wait_sec", 15.0);
    reloc_cooldown_sec_ = declare_parameter<double>("reloc_cooldown_sec", 45.0);
    sigma_ = declare_parameter<double>("sigma", 0.25);
    coarse_step_ = declare_parameter<double>("coarse_step", 0.15);
    coarse_yaw_step_ = declare_parameter<double>("coarse_yaw_step_deg", 5.0) * M_PI / 180.0;
    max_beam_range_ = declare_parameter<double>("max_beam_range", 8.0);
    min_clearance_ = declare_parameter<double>("min_clearance", 0.12);
    watchdog_en_ = declare_parameter<bool>("watchdog_en", true);
    watchdog_sigma_ = declare_parameter<double>("watchdog_sigma", 0.08);
    watchdog_count_ = std::max(1, (int)declare_parameter<int>("watchdog_count", 3));
    post_nav_hit_ = declare_parameter<double>("post_nav_hit", 0.0);  // <=0 关闭到点自动纠偏
    post_nav_settle_sec_ = declare_parameter<double>("post_nav_settle_sec", 1.5);
    // 静止时局部激光↔地图微调（似然场小窗，类似轻量 scan-match / 回环，不是全图 NDT）
    local_refine_en_ = declare_parameter<bool>("local_refine_en", false);
    local_refine_period_sec_ = declare_parameter<double>("local_refine_period_sec", 2.5);
    local_xy_radius_ = declare_parameter<double>("local_xy_radius", 0.30);
    local_yaw_deg_ = declare_parameter<double>("local_yaw_deg", 12.0);
    local_improve_ = declare_parameter<double>("local_improve", 0.05);
    local_min_hit_ = declare_parameter<double>("local_min_hit", 0.65);
    num_threads_ = std::max(1, (int)declare_parameter<int>("num_threads", 4));

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", rclcpp::QoS(1).transient_local().reliable(),
        [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) { onMap(msg); });
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lk(scan_mutex_);
          scan_ = msg;
        });
    // 导航进行中禁止自动重定位（发 /initialpose 会打乱 AMCL → Failed to make progress）
    // TRANSIENT_LOCAL：对齐 web_ops，晚启动也能拿到当前状态
    nav_status_sub_ = create_subscription<std_msgs::msg::String>(
        "/web/nav_status", rclcpp::QoS(1).transient_local().reliable(),
        [this](std_msgs::msg::String::ConstSharedPtr msg) {
          const std::string s = msg->data.empty() ? "" : msg->data;
          const auto colon = s.find(':');
          const std::string st = colon == std::string::npos ? s : s.substr(0, colon);
          const bool now_nav = (st == "navigating" || st == "active");
          // 导航刚结束：稍后再测置信度，偏低则自动纠偏（导航中绝不发 /initialpose）
          if (navigating_ && !now_nav &&
              (st == "reached" || st == "canceled" || st == "aborted" || st == "idle")) {
            post_nav_pending_ = true;
            post_nav_ready_at_ = now() + rclcpp::Duration::from_seconds(post_nav_settle_sec_);
          }
          navigating_ = now_nav;
          if (now_nav) post_nav_pending_ = false;
        });
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/relocalize_status", 10);

    srv_ = create_service<std_srvs::srv::Trigger>(
        "relocalize",
        [this](std_srvs::srv::Trigger::Request::ConstSharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr res) {
          Candidate c;
          bool ok = runRelocalize(c, true);
          res->success = true;
          const bool published = ok && (c.black_hit >= reloc_trigger_hit_);
          const char *tag = c.black_hit >= min_black_hit_ ? "aligned" :
              (c.black_hit >= reloc_trigger_hit_ ? "mid" : "weak");
          char buf[192];
          std::snprintf(buf, sizeof(buf),
              "%s最高置信度位姿 黑点命中=%.1f%% %s",
              published ? "已下发" : "未下发",
              c.black_hit * 100.0, tag);
          res->message = buf;
          if (!ok) res->message += " [搜索未执行]";
          if (pose_pub_->get_subscription_count() == 0)
            res->message += " [警告: /initialpose 无订阅者]";
        });

    // 用户在弹窗选「否」或手动设姿完成后调用 → 锁定当前位姿
    accept_srv_ = create_service<std_srvs::srv::Trigger>(
        "accept_reloc_pose",
        [this](std_srvs::srv::Trigger::Request::ConstSharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr res) {
          phase_ = Phase::Locked;
          ask_manual_shown_ = false;
          low_score_cnt_ = 0;
          publishStatusHit("aligned", last_hit_);
          res->success = true;
          res->message = "已承认当前位姿，进入定位就绪/看门狗";
          RCLCPP_INFO(get_logger(), "用户确认位姿，锁定 (置信度=%.1f%%)", last_hit_ * 100.0);
        });

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    watchdog_timer_ = create_wall_timer(std::chrono::seconds(1),
                                        [this]() { tick(); });
    last_local_refine_ = now() - rclcpp::Duration::from_seconds(30.0);
    publishStatus(watchdog_en_ ? "hold:0.000" : "need_manual");
    RCLCPP_INFO(get_logger(),
        "重定位就绪: 手动可用; auto_on_startup=%s watchdog=%s post_nav=%s local_refine=%s",
        auto_on_startup_ ? "on" : "off",
        watchdog_en_ ? "on" : "off",
        post_nav_hit_ > 1e-6 ? "on" : "off",
        local_refine_en_ ? "on" : "off");
  }

  void publishStatus(const std::string &s) {
    std_msgs::msg::String m;
    m.data = s;
    status_pub_->publish(m);
  }
  void publishStatusHit(const std::string &tag, double hit) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%s:%.3f", tag.c_str(), hit);
    publishStatus(buf);
  }

private:
  void tick() {
    if (!field_ready_ || !haveScan()) return;
    if (phase_ == Phase::Searching) return;

    // 启动自动搜：仅 auto_on_startup=true 时执行一次
    if (phase_ == Phase::Boot) {
      if (auto_on_startup_) {
        if (pose_pub_->get_subscription_count() == 0) {
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
              "等待 AMCL 订阅 /initialpose ...");
          publishStatus("waiting_amcl");
          return;
        }
        Candidate c;
        runRelocalize(c, false);
        return;
      }
      // 手动模式：离开 Boot，只报置信度，绝不自动搜
      phase_ = Phase::Locked;
      publishStatus("need_manual");
    }

    // 始终更新定位置信度显示
    double bh = 0.0;
    const bool have = currentHit(bh);
    if (have) {
      last_hit_ = bh;
      if (bh >= min_black_hit_) publishStatusHit("aligned", bh);
      else if (bh >= reloc_trigger_hit_) publishStatusHit("hold", bh);
      else publishStatusHit("critical", bh);
    } else {
      publishStatusHit("critical", last_hit_);
    }

    // 导航中绝不发 /initialpose；静止时做局部激光贴图微调（与 watchdog 开关无关）
    if (navigating_) {
      low_score_cnt_ = 0;
      post_nav_pending_ = false;
    } else {
      maybeLocalRefine(have ? bh : last_hit_);
    }

    if (!watchdog_en_) return;  // 全图自动搜关闭；局部微调已在上方处理
    if (navigating_) return;

    // 导航结束后停稳：仅 post_nav_hit>0 时才自动搜；默认关闭，否则常把终点拽歪
    if (post_nav_pending_ && now() >= post_nav_ready_at_) {
      post_nav_pending_ = false;
      if (post_nav_hit_ > 1e-6) {
        double bh_post = have ? bh : last_hit_;
        if (!have) currentHit(bh_post);
        if (bh_post < post_nav_hit_ && phase_ != Phase::Searching) {
          RCLCPP_WARN(get_logger(),
              "导航结束置信度 %.1f%% <%.0f%%，自动纠偏",
              bh_post * 100.0, post_nav_hit_ * 100.0);
          Candidate c;
          runRelocalize(c, false);
          return;
        }
      }
    }

    if (!have) {
      if (phase_ == Phase::Locked) {
        if ((now() - last_reloc_time_).seconds() < reloc_cooldown_sec_) return;
        if (++low_score_cnt_ >= watchdog_count_) {
          low_score_cnt_ = 0;
          RCLCPP_WARN(get_logger(), "无 map→base_footprint，触发全局重定位");
          Candidate c;
          runRelocalize(c, false);
        }
      }
      return;
    }

    if (phase_ == Phase::MidWait) {
      if (bh >= min_black_hit_) {
        phase_ = Phase::Locked;
        low_score_cnt_ = 0;
        publishStatusHit("aligned", bh);
        return;
      }
      if (bh < reloc_trigger_hit_) {
        phase_ = Phase::AskManual;
        ask_manual_shown_ = false;
        publishStatusHit("ask_manual", bh);
        return;
      }
      publishStatusHit("hold", bh);
      if ((now() - mid_wait_start_).seconds() >= mid_wait_sec_) {
        phase_ = Phase::AskManual;
        ask_manual_shown_ = false;
        publishStatusHit("ask_manual", bh);
      }
      return;
    }

    if (phase_ == Phase::AskManual) {
      if (bh >= min_black_hit_) {
        phase_ = Phase::Locked;
        publishStatusHit("aligned", bh);
      } else {
        publishStatusHit("ask_manual", bh);
      }
      return;
    }

    if (phase_ == Phase::Locked) {
      if (bh >= reloc_trigger_hit_) {
        low_score_cnt_ = 0;
        return;
      }
      if ((now() - last_reloc_time_).seconds() < reloc_cooldown_sec_) return;
      if (++low_score_cnt_ >= watchdog_count_) {
        low_score_cnt_ = 0;
        RCLCPP_WARN(get_logger(),
            "置信度 %.1f%% <%.0f%% 连续%d次，触发全局重定位",
            bh * 100.0, reloc_trigger_hit_ * 100.0, watchdog_count_);
        Candidate c;
        runRelocalize(c, false);
      }
    }
  }

  bool currentHit(double &bh) {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      return false;
    }
    const auto &q = tf.transform.rotation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    const auto pts = beams(600);
    if (pts.size() < 30) return false;
    bh = blackHitRatio(pts, tf.transform.translation.x, tf.transform.translation.y,
                       std::cos(yaw), std::sin(yaw));
    return true;
  }

  void enterAfterReloc(double hit) {
    last_hit_ = hit;
    low_score_cnt_ = 0;
    ask_manual_shown_ = false;
    if (hit >= min_black_hit_) {
      phase_ = Phase::Locked;
      publishStatusHit("aligned", hit);
      RCLCPP_INFO(get_logger(), "重定位成功(≥%.0f%%)，进入看门狗", min_black_hit_ * 100.0);
    } else if (hit >= reloc_trigger_hit_) {
      phase_ = Phase::MidWait;
      mid_wait_start_ = now();
      publishStatusHit("hold", hit);
      RCLCPP_WARN(get_logger(),
          "置信度 %.1f%% 在[%.0f%%,%.0f%%)，监测 %.0fs；到期未达标则请用户确认",
          hit * 100.0, reloc_trigger_hit_ * 100.0, min_black_hit_ * 100.0, mid_wait_sec_);
    } else {
      // 首轮就 <0.35：不下发弱 /initialpose，请用户确认
      phase_ = Phase::AskManual;
      publishStatusHit("ask_manual", hit);
      RCLCPP_WARN(get_logger(),
          "置信度 %.1f%% <%.0f%%，请用户确认是否手动设姿（或再点重定位）",
          hit * 100.0, reloc_trigger_hit_ * 100.0);
    }
  }

  void onMap(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
    map_ = msg;
    const int w = msg->info.width, h = msg->info.height;
    const float res = msg->info.resolution;
    const float INF = 1e9f;
    dist_.assign((size_t)w * h, INF);
    for (int i = 0; i < w * h; ++i)
      if (msg->data[i] >= 65) dist_[i] = 0.0f;
    const float s = res, diag = res * 1.41421356f;
    for (int y = 0; y < h; ++y)
      for (int x = 0; x < w; ++x) {
        float &d = dist_[(size_t)y * w + x];
        if (x > 0) d = std::min(d, dist_[(size_t)y * w + x - 1] + s);
        if (y > 0) d = std::min(d, dist_[(size_t)(y - 1) * w + x] + s);
        if (x > 0 && y > 0) d = std::min(d, dist_[(size_t)(y - 1) * w + x - 1] + diag);
        if (x < w - 1 && y > 0) d = std::min(d, dist_[(size_t)(y - 1) * w + x + 1] + diag);
      }
    for (int y = h - 1; y >= 0; --y)
      for (int x = w - 1; x >= 0; --x) {
        float &d = dist_[(size_t)y * w + x];
        if (x < w - 1) d = std::min(d, dist_[(size_t)y * w + x + 1] + s);
        if (y < h - 1) d = std::min(d, dist_[(size_t)(y + 1) * w + x] + s);
        if (x < w - 1 && y < h - 1) d = std::min(d, dist_[(size_t)(y + 1) * w + x + 1] + diag);
        if (x > 0 && y < h - 1) d = std::min(d, dist_[(size_t)(y + 1) * w + x - 1] + diag);
      }
    score_.resize(dist_.size());
    score_tight_.resize(dist_.size());
    const float inv2s2 = 1.0f / (2.0f * sigma_ * sigma_);
    const float inv2s2t = 1.0f / (2.0f * watchdog_sigma_ * watchdog_sigma_);
    for (size_t i = 0; i < dist_.size(); ++i) {
      score_[i] = std::exp(-dist_[i] * dist_[i] * inv2s2);
      score_tight_[i] = std::exp(-dist_[i] * dist_[i] * inv2s2t);
    }
    field_ready_ = true;
    RCLCPP_INFO(get_logger(), "双σ似然场就绪 %dx%d (σ=%.2f / %.2f)", w, h, sigma_, watchdog_sigma_);
  }

  bool haveScan() {
    std::lock_guard<std::mutex> lk(scan_mutex_);
    return scan_ != nullptr;
  }

  std::vector<std::pair<float, float>> beams(int n) {
    sensor_msgs::msg::LaserScan::ConstSharedPtr scan;
    {
      std::lock_guard<std::mutex> lk(scan_mutex_);
      scan = scan_;
    }
    std::vector<std::pair<float, float>> all;
    if (!scan) return all;
    const int total = (int)scan->ranges.size();
    all.reserve(std::min(n, total));
    // 先收集全部有效回波，再均匀抽到 ≤n（勿对空 bins 做 stride，否则稀疏 /scan 会采到 <20）
    for (int i = 0; i < total; ++i) {
      const float r = scan->ranges[i];
      if (!std::isfinite(r) || r < scan->range_min || r > max_beam_range_) continue;
      const float a = scan->angle_min + i * scan->angle_increment;
      all.emplace_back(r * std::cos(a), r * std::sin(a));
    }
    if ((int)all.size() <= n || n <= 0) return all;
    std::vector<std::pair<float, float>> pts;
    pts.reserve(n);
    for (int k = 0; k < n; ++k) {
      const int idx = (int)((long long)k * (long long)all.size() / n);
      pts.push_back(all[idx]);
    }
    return pts;
  }

  inline double blackHitRatio(const std::vector<std::pair<float, float>> &pts,
                              double x, double y, double c, double s) const {
    if (!map_ || pts.empty()) return 0.0;
    const auto &info = map_->info;
    const int w = info.width, h = info.height;
    const float tol = static_cast<float>(std::max(0.0, black_hit_dist_));
    const bool use_dist = !dist_.empty() && dist_.size() == map_->data.size() && tol > 1e-6f;
    int hits = 0;
    for (const auto &p : pts) {
      const double wx = x + c * p.first - s * p.second;
      const double wy = y + s * p.first + c * p.second;
      const int gx = (int)((wx - info.origin.position.x) / info.resolution);
      const int gy = (int)((wy - info.origin.position.y) / info.resolution);
      if (gx < 0 || gx >= w || gy < 0 || gy >= h) continue;
      const size_t idx = (size_t)gy * w + gx;
      if (use_dist) {
        if (dist_[idx] <= tol) ++hits;
      } else if (map_->data[idx] >= 65) {
        ++hits;
      }
    }
    return (double)hits / (double)pts.size();
  }

  inline double scorePoseField(const std::vector<std::pair<float, float>> &pts,
                               double x, double y, double c, double s,
                               const std::vector<float> &field) const {
    if (!map_ || pts.empty() || field.empty()) return 0.0;
    const auto &info = map_->info;
    const int w = info.width, h = info.height;
    double sum = 0;
    for (const auto &p : pts) {
      const double wx = x + c * p.first - s * p.second;
      const double wy = y + s * p.first + c * p.second;
      const int gx = (int)((wx - info.origin.position.x) / info.resolution);
      const int gy = (int)((wy - info.origin.position.y) / info.resolution);
      if (gx < 0 || gx >= w || gy < 0 || gy >= h) continue;
      const size_t idx = (size_t)gy * w + gx;
      if (map_->data[idx] < 0) continue;
      sum += field[idx];
    }
    return sum / (double)pts.size();
  }

  // 主指标：置信度；并列用似然场
  static bool better(const Candidate &a, const Candidate &b) {
    if (a.black_hit != b.black_hit) return a.black_hit > b.black_hit;
    return a.score > b.score;
  }

  Candidate search(const std::vector<std::pair<double, double>> &positions,
                   const std::vector<double> &yaws,
                   const std::vector<std::pair<float, float>> &pts,
                   const std::vector<float> &field) {
    std::vector<Candidate> best(num_threads_);
    std::vector<std::thread> workers;
    std::atomic<size_t> next{0};
    for (int t = 0; t < num_threads_; ++t) {
      workers.emplace_back([&, t]() {
        size_t i;
        while ((i = next.fetch_add(1)) < positions.size()) {
          const auto &pos = positions[i];
          for (double yaw : yaws) {
            const double c = std::cos(yaw), s = std::sin(yaw);
            Candidate cand;
            cand.x = pos.first; cand.y = pos.second; cand.yaw = yaw;
            cand.black_hit = blackHitRatio(pts, cand.x, cand.y, c, s);
            cand.score = scorePoseField(pts, cand.x, cand.y, c, s, field);
            if (better(cand, best[t])) best[t] = cand;
          }
        }
      });
    }
    for (auto &w : workers) w.join();
    Candidate b;
    for (const auto &c : best)
      if (better(c, b)) b = c;
    return b;
  }

  bool runRelocalize(Candidate &result, bool from_service) {
    if (!field_ready_ || !haveScan()) {
      RCLCPP_WARN(get_logger(), "地图或激光数据未就绪, 无法重定位");
      return false;
    }
    phase_ = Phase::Searching;
    publishStatus("searching");

    const auto t0 = now();
    const auto &info = map_->info;
    const int w = info.width, h = info.height;

    const int stride = std::max(1, (int)(coarse_step_ / info.resolution));
    std::vector<std::pair<double, double>> coarse_pos;
    coarse_pos.reserve((w / stride) * (h / stride));
    for (int gy = 0; gy < h; gy += stride)
      for (int gx = 0; gx < w; gx += stride) {
        if (dist_[(size_t)gy * w + gx] < min_clearance_) continue;
        if (map_->data[(size_t)gy * w + gx] != 0) continue;
        coarse_pos.emplace_back(info.origin.position.x + (gx + 0.5) * info.resolution,
                                info.origin.position.y + (gy + 0.5) * info.resolution);
      }
    std::vector<double> coarse_yaws;
    for (double a = -M_PI; a < M_PI - 1e-9; a += coarse_yaw_step_)
      coarse_yaws.push_back(a);

    auto pts_coarse = beams(500);
    if (pts_coarse.size() < 20) {
      RCLCPP_WARN(get_logger(), "有效激光束过少(%zu), 无法重定位", pts_coarse.size());
      phase_ = Phase::AskManual;
      publishStatusHit("ask_manual", 0.0);
      return false;
    }
    RCLCPP_INFO(get_logger(),
        "粗搜(宽σ): %zu×%zu 候选", coarse_pos.size(), coarse_yaws.size());
    Candidate best = search(coarse_pos, coarse_yaws, pts_coarse, score_);

    std::vector<std::pair<double, double>> fine_pos;
    for (double dx = -0.30; dx <= 0.30 + 1e-9; dx += 0.05)
      for (double dy = -0.30; dy <= 0.30 + 1e-9; dy += 0.05)
        fine_pos.emplace_back(best.x + dx, best.y + dy);
    std::vector<double> fine_yaws;
    for (double da = -15 * M_PI / 180; da <= 15 * M_PI / 180 + 1e-9; da += 2 * M_PI / 180)
      fine_yaws.push_back(best.yaw + da);
    auto pts_fine = beams(900);
    RCLCPP_INFO(get_logger(), "精搜(窄σ): %zu×%zu 候选", fine_pos.size(), fine_yaws.size());
    Candidate fine = search(fine_pos, fine_yaws, pts_fine, score_tight_);
    if (better(best, fine)) fine = best;

    fine.black_hit = blackHitRatio(pts_fine, fine.x, fine.y,
                                   std::cos(fine.yaw), std::sin(fine.yaw));
    fine.tight = scorePoseField(pts_fine, fine.x, fine.y,
                                std::cos(fine.yaw), std::sin(fine.yaw), score_tight_);
    result = fine;

    const double dt = (now() - t0).seconds();
    RCLCPP_INFO(get_logger(),
        "最高置信度位姿: x=%.2f y=%.2f yaw=%.1f° 置信度=%.1f%% (%.1fs)%s",
        fine.x, fine.y, fine.yaw * 180 / M_PI, fine.black_hit * 100.0, dt,
        from_service ? " [服务]" : "");

    // 自动搜：必须 ≥min_black_hit 才灌 /initialpose（中等命中常是错误峰）。
    // 手动服务：≥reloc_trigger_hit 也可下发（用户显式点了重定位）。
    const double pub_min = from_service ? reloc_trigger_hit_ : min_black_hit_;
    if (fine.black_hit >= pub_min) {
      geometry_msgs::msg::PoseWithCovarianceStamped msg;
      msg.header.frame_id = "map";
      msg.pose.pose.position.x = fine.x;
      msg.pose.pose.position.y = fine.y;
      msg.pose.pose.orientation.z = std::sin(fine.yaw / 2);
      msg.pose.pose.orientation.w = std::cos(fine.yaw / 2);
      msg.pose.covariance[0] = 0.25;
      msg.pose.covariance[7] = 0.25;
      msg.pose.covariance[35] = 0.25;
      // stamp=0 → AMCL 用最新 TF，避免 now()/过期时间导致 extrapolation。
      msg.header.stamp.sec = 0;
      msg.header.stamp.nanosec = 0;
      for (int i = 0; i < 3; ++i) {
        pose_pub_->publish(msg);
        if (i < 2) rclcpp::sleep_for(std::chrono::milliseconds(30));
      }
      last_reloc_time_ = now();
      RCLCPP_INFO(get_logger(),
                  "已下发 /initialpose x=%.2f y=%.2f yaw=%.1f° (hit=%.1f%% %s)",
                  fine.x, fine.y, fine.yaw * 180 / M_PI, fine.black_hit * 100.0,
                  from_service ? "手动" : "自动");
    } else {
      RCLCPP_WARN(get_logger(),
                  "置信度 %.1f%% <%.0f%%，不下发 /initialpose，请手动设姿",
                  fine.black_hit * 100.0, pub_min * 100.0);
    }

    enterAfterReloc(fine.black_hit);
    return true;
  }

  void maybeLocalRefine(double cur_hit) {
    if (!local_refine_en_ || navigating_) return;
    if (phase_ == Phase::Searching) return;
    if (!field_ready_ || !haveScan()) return;
    if ((now() - last_local_refine_).seconds() < local_refine_period_sec_) return;
    if (cur_hit >= 0.88) return;

    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      return;
    }
    const double cx = tf.transform.translation.x;
    const double cy = tf.transform.translation.y;
    const auto &q = tf.transform.rotation;
    const double cyaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    if (have_last_pose_) {
      const double dxy = std::hypot(cx - last_pose_x_, cy - last_pose_y_);
      double dyaw = cyaw - last_pose_yaw_;
      while (dyaw > M_PI) dyaw -= 2 * M_PI;
      while (dyaw < -M_PI) dyaw += 2 * M_PI;
      if (dxy > 0.04 || std::fabs(dyaw) > 3.0 * M_PI / 180.0) {
        last_pose_x_ = cx; last_pose_y_ = cy; last_pose_yaw_ = cyaw;
        still_count_ = 0;
        return;
      }
      if (++still_count_ < 2) {
        last_pose_x_ = cx; last_pose_y_ = cy; last_pose_yaw_ = cyaw;
        return;
      }
    } else {
      have_last_pose_ = true;
      last_pose_x_ = cx; last_pose_y_ = cy; last_pose_yaw_ = cyaw;
      still_count_ = 0;
      return;
    }
    last_pose_x_ = cx; last_pose_y_ = cy; last_pose_yaw_ = cyaw;
    last_local_refine_ = now();

    auto pts = beams(700);
    if (pts.size() < 30) return;

    std::vector<std::pair<double, double>> pos;
    const double step = 0.05;
    for (double dx = -local_xy_radius_; dx <= local_xy_radius_ + 1e-9; dx += step)
      for (double dy = -local_xy_radius_; dy <= local_xy_radius_ + 1e-9; dy += step)
        pos.emplace_back(cx + dx, cy + dy);
    std::vector<double> yaws;
    const double yaw_lim = local_yaw_deg_ * M_PI / 180.0;
    for (double da = -yaw_lim; da <= yaw_lim + 1e-9; da += 2.0 * M_PI / 180.0)
      yaws.push_back(cyaw + da);

    Candidate best = search(pos, yaws, pts, score_tight_);
    best.black_hit = blackHitRatio(pts, best.x, best.y,
                                   std::cos(best.yaw), std::sin(best.yaw));
    if (best.black_hit < local_min_hit_) return;
    if (best.black_hit < cur_hit + local_improve_) return;
    if (std::hypot(best.x - cx, best.y - cy) > local_xy_radius_ + 1e-3) return;

    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.frame_id = "map";
    msg.header.stamp.sec = 0;
    msg.header.stamp.nanosec = 0;
    msg.pose.pose.position.x = best.x;
    msg.pose.pose.position.y = best.y;
    msg.pose.pose.orientation.z = std::sin(best.yaw / 2);
    msg.pose.pose.orientation.w = std::cos(best.yaw / 2);
    msg.pose.covariance[0] = 0.05;
    msg.pose.covariance[7] = 0.05;
    msg.pose.covariance[35] = 0.05;
    for (int i = 0; i < 2; ++i) {
      pose_pub_->publish(msg);
      if (i == 0) rclcpp::sleep_for(std::chrono::milliseconds(20));
    }
    last_hit_ = best.black_hit;
    last_reloc_time_ = now();
    RCLCPP_INFO(get_logger(),
        "局部贴图微调: (%.2f,%.2f,%.1f°)→(%.2f,%.2f,%.1f°) hit %.1f%%→%.1f%%",
        cx, cy, cyaw * 180 / M_PI, best.x, best.y, best.yaw * 180 / M_PI,
        cur_hit * 100.0, best.black_hit * 100.0);
    publishStatusHit("aligned", best.black_hit);
  }

  bool auto_on_startup_, watchdog_en_, local_refine_en_ = true;
  double accept_score_, min_black_hit_, reloc_trigger_hit_, mid_wait_sec_, reloc_cooldown_sec_;
  double black_hit_dist_ = 0.15;
  double sigma_, coarse_step_, coarse_yaw_step_;
  double max_beam_range_, min_clearance_, watchdog_sigma_;
  double post_nav_hit_, post_nav_settle_sec_;
  double local_refine_period_sec_ = 2.5, local_xy_radius_ = 0.30, local_yaw_deg_ = 12.0;
  double local_improve_ = 0.035, local_min_hit_ = 0.55;
  int num_threads_, watchdog_count_;
  bool field_ready_ = false;
  bool navigating_ = false;
  bool post_nav_pending_ = false;
  bool have_last_pose_ = false;
  double last_pose_x_ = 0, last_pose_y_ = 0, last_pose_yaw_ = 0;
  int still_count_ = 0;
  Phase phase_ = Phase::Boot;
  int low_score_cnt_ = 0;
  bool ask_manual_shown_ = false;
  double last_hit_ = 0.0;
  rclcpp::Time last_reloc_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time mid_wait_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time post_nav_ready_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_local_refine_{0, 0, RCL_ROS_TIME};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_;
  std::vector<float> dist_, score_, score_tight_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan_;
  std::mutex scan_mutex_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr nav_status_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr accept_srv_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutoRelocalize>());
  rclcpp::shutdown();
  return 0;
}
