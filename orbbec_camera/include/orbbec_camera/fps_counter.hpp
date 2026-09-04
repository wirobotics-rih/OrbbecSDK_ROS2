/*******************************************************************************
 * Copyright (c) 2023 Orbbec 3D Technology, Inc
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *******************************************************************************/

#pragma once

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <cstdint>
#include <chrono>
#include <functional>
#include <string>
namespace orbbec_camera {

enum class LogLevel { DEBUG, INFO };
class FpsCounter {
 public:
  explicit FpsCounter(const std::string &name, rclcpp::Logger logger, int print_interval_sec = 1)
      : name_(name),
        print_interval_(std::chrono::seconds(print_interval_sec)),
        last_print_time_(std::chrono::steady_clock::now()),
        frame_count_(0),
        log_level_(LogLevel::INFO),
        logger_(logger)

  {}

  void setLogLevel(LogLevel level) { log_level_ = level; }

  //: The rate this counter last measured, or 0 before the first interval
  //: closes. Ticked on the driver's frame path, so it is what the camera
  //: PUBLISHED -- unlike a rate any subscriber measures, which also carries
  //: whatever that subscriber dropped. Read from the diagnostic updater.
  double fps() const { return fps_.load(std::memory_order_relaxed); }

  //: The DEVICE's own frame sequence number, as of the last frame this counter
  //: saw (`ob::Frame::getIndex()`), or 0 if nothing has passed a device index.
  //:
  //: A rate cannot answer "did the camera produce it, or did the driver lose
  //: it": both are rates measured over different windows by different parties.
  //: A cumulative index can -- differenced over an interval it counts what the
  //: SENSOR produced, and a frame this driver dropped still shows up as a jump.
  //: The dashboard reads it out of /diagnostics and differences it there
  //: (`camera_watch._sensor_count`); this side only has to publish the newest
  //: value, so a missed diagnostics message costs nothing.
  uint64_t deviceIndex() const { return device_index_.load(std::memory_order_relaxed); }

  void tick(uint64_t device_index) {
    device_index_.store(device_index, std::memory_order_relaxed);
    tick();
  }

  void tick() {
    ++frame_count_;
    auto now = std::chrono::steady_clock::now();
    if (now - last_print_time_ >= print_interval_) {
      double fps =
          static_cast<double>(frame_count_) /
          std::chrono::duration_cast<std::chrono::duration<double>>(now - last_print_time_).count();
      fps_.store(fps, std::memory_order_relaxed);

      if (log_level_ == LogLevel::INFO) {
        RCLCPP_INFO_STREAM(logger_, name_ << " FPS " << fps);
      } else {
        RCLCPP_DEBUG_STREAM(logger_, name_ << " FPS " << fps);
      }

      last_print_time_ = now;
      frame_count_ = 0;
    }
  }

 private:
  std::string name_;
  std::chrono::seconds print_interval_;
  std::chrono::steady_clock::time_point last_print_time_;
  uint32_t frame_count_;
  LogLevel log_level_;
  rclcpp::Logger logger_;
  //: Written on the frame thread, read on the diagnostic timer.
  std::atomic<double> fps_{0.0};
  std::atomic<uint64_t> device_index_{0};
};

}  // namespace orbbec_camera
