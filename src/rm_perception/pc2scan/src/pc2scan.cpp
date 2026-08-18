/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

#include "pc2scan/pc2scan.hpp"

#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2_ros/create_timer_ros.h"

namespace pc2scan {
pc2scan::pc2scan(const rclcpp::NodeOptions &options)
    : rclcpp::Node("pc2scan", options) {
  target_frame_ = this->declare_parameter("target_frame", "");
  tolerance_ = this->declare_parameter("transform_tolerance", 0.01);

  input_queue_size_ = this->declare_parameter(
      "queue_size", static_cast<int>(std::thread::hardware_concurrency()));
  min_height_ =
      this->declare_parameter("min_height", std::numeric_limits<double>::min());
  max_height_ =
      this->declare_parameter("max_height", std::numeric_limits<double>::max());
  min_intensity_ = this->declare_parameter("min_intensity", 0.0);
  max_intensity_ = this->declare_parameter("max_intensity",
                                           std::numeric_limits<double>::max());
  angle_min_ = this->declare_parameter("angle_min", -M_PI);
  angle_max_ = this->declare_parameter("angle_max", M_PI);
  angle_increment_ = this->declare_parameter("angle_increment", M_PI / 180.0);
  scan_time_ = this->declare_parameter("scan_time", 1.0 / 30.0);
  range_min_ = this->declare_parameter("range_min", 0.0);
  range_max_ =
      this->declare_parameter("range_max", std::numeric_limits<double>::max());
  inf_epsilon_ = this->declare_parameter("inf_epsilon", 1.0);
  use_inf_ = this->declare_parameter("use_inf", true);

  max_gradient_threshold_ =
      this->declare_parameter("max_gradient_threshold", 1.0);

  pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS());

  using std::placeholders::_1;

  if (!target_frame_.empty()) {
    tf2_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(), this->get_node_timers_interface());
    tf2_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_);
    message_filter_ = std::make_unique<MessageFilter>(
        sub_, *tf2_, target_frame_, input_queue_size_,
        this->get_node_logging_interface(), this->get_node_clock_interface());
    message_filter_->registerCallback(
        std::bind(&pc2scan::cloudCallback, this, _1));
  } else {
    sub_.registerCallback(std::bind(&pc2scan::cloudCallback, this, _1));
  }

  subscription_listener_thread_ =
      std::thread(std::bind(&pc2scan::subscriptionListenerThreadLoop, this));
}

pc2scan::~pc2scan() {
  alive_.store(false);
  subscription_listener_thread_.join();
}

void pc2scan::subscriptionListenerThreadLoop() {
  rclcpp::Context::SharedPtr context =
      this->get_node_base_interface()->get_context();
  const std::chrono::milliseconds timeout(100);
  while (rclcpp::ok(context) && alive_.load()) {
    int subscription_count = pub_->get_subscription_count() +
                             pub_->get_intra_process_subscription_count();
    if (subscription_count > 0) {
      if (!sub_.getSubscriber()) {
        RCLCPP_INFO(
            this->get_logger(),
            "Got a subscriber to laserscan, starting pointcloud subscriber");
        rclcpp::SensorDataQoS qos;
        qos.keep_last(input_queue_size_);
        sub_.subscribe(this, "cloud_in", qos.get_rmw_qos_profile());
      }
    } else if (sub_.getSubscriber()) {
      RCLCPP_INFO(
          this->get_logger(),
          "No subscribers to laserscan, shutting down pointcloud subscriber");
      sub_.unsubscribe();
    }
    rclcpp::Event::SharedPtr event = this->get_graph_event();
    this->wait_for_graph_change(event, timeout);
  }

  sub_.unsubscribe();
}

void pc2scan::cloudCallback(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg) {
  auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
  scan_msg->header = cloud_msg->header;
  if (!target_frame_.empty()) {
    scan_msg->header.frame_id = target_frame_;
  }

  scan_msg->angle_min = angle_min_;
  scan_msg->angle_max = angle_max_;
  scan_msg->angle_increment = angle_increment_;
  scan_msg->time_increment = 0.0;
  scan_msg->scan_time = scan_time_;
  scan_msg->range_min = range_min_;
  scan_msg->range_max = range_max_;

  uint32_t ranges_size = std::ceil((scan_msg->angle_max - scan_msg->angle_min) /
                                   scan_msg->angle_increment);

  if (use_inf_) {
    scan_msg->ranges.assign(ranges_size,
                            std::numeric_limits<double>::infinity());
  } else {
    scan_msg->ranges.assign(ranges_size, scan_msg->range_max + inf_epsilon_);
  }

  if (scan_msg->header.frame_id != cloud_msg->header.frame_id) {
    try {
      auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
      tf2_->transform(*cloud_msg, *cloud, target_frame_,
                      tf2::durationFromSec(tolerance_));
      cloud_msg = cloud;
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR_STREAM(this->get_logger(),
                          "Transform failure: " << ex.what());
      return;
    }
  }

  bool has_curvature = false;
  for (const auto &field : cloud_msg->fields) {
    if (field.name == "curvature") {
      has_curvature = true;
      break;
    }
  }

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud_msg, "z");
  sensor_msgs::PointCloud2ConstIterator<float> iter_i(*cloud_msg, "intensity");
  sensor_msgs::PointCloud2ConstIterator<float> iter_grad(*cloud_msg,
                                                         "curvature");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_i) {
    float gradient = std::numeric_limits<float>::max();
    if (has_curvature) {
      gradient = *iter_grad;
      ++iter_grad;

      if (gradient <= max_gradient_threshold_) {
        continue;
      }
    }

    if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) {
      RCLCPP_DEBUG(this->get_logger(),
                   "rejected for nan in point(%f, %f, %f)\n", *iter_x, *iter_y,
                   *iter_z);
      continue;
    }

    if (*iter_z > max_height_ || *iter_z < min_height_) {
      RCLCPP_DEBUG(this->get_logger(),
                   "rejected for height %f not in range (%f, %f)\n", *iter_z,
                   min_height_, max_height_);
      continue;
    }

    if (*iter_i < min_intensity_ || *iter_i > max_intensity_) {
      continue;
    }

    double range = hypot(*iter_x, *iter_y);
    if (range < range_min_) {
      RCLCPP_DEBUG(
          this->get_logger(),
          "rejected for range %f below minimum value %f. Point: (%f, %f, %f)",
          range, range_min_, *iter_x, *iter_y, *iter_z);
      continue;
    }
    if (range > range_max_) {
      RCLCPP_DEBUG(
          this->get_logger(),
          "rejected for range %f above maximum value %f. Point: (%f, %f, %f)",
          range, range_max_, *iter_x, *iter_y, *iter_z);
      continue;
    }

    double angle = atan2(*iter_y, *iter_x);
    if (angle < scan_msg->angle_min || angle > scan_msg->angle_max) {
      RCLCPP_DEBUG(this->get_logger(),
                   "rejected for angle %f not in range (%f, %f)\n", angle,
                   scan_msg->angle_min, scan_msg->angle_max);
      continue;
    }

    int index = (angle - scan_msg->angle_min) / scan_msg->angle_increment;
    if (range < scan_msg->ranges[index]) {
      scan_msg->ranges[index] = range;
    }
  }

  pub_->publish(std::move(scan_msg));
}
} // namespace pc2scan

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(pc2scan::pc2scan)
