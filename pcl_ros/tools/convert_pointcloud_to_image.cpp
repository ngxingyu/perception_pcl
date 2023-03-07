/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 * $Id: surface_convex_hull.cpp 34612 2010-12-08 01:06:27Z rusu $
 *
 */

/**
 \author Ethan Rublee
 **/
// ROS core
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include "rclcpp_components/register_node_macro.hpp"
// Image message
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
// pcl::toROSMsg
#include <pcl/io/pcd_io.h>
// conversions from PCL custom types
#include <pcl_conversions/pcl_conversions.h>
// stl stuff
#include <string>
namespace pcl_ros
{
class PointCloudToImage: public rclcpp::Node
{
public:
  void
  cloud_cb(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
  {
    if (cloud->height <= 1) {
      RCLCPP_ERROR(this->get_logger(),
        "Input point cloud is not organized, ignoring!");
      return;
    }
    try {
      pcl::toROSMsg(*cloud, image_);  // convert the cloud
      image_.header = cloud->header;
      image_pub_->publish(image_);  // publish our cloud image
    } catch (std::runtime_error & e) {
      RCLCPP_ERROR_STREAM(
        this->get_logger(),
        "Error in converting cloud to image message: " <<
          e.what());
    }
  }

  explicit PointCloudToImage(const rclcpp::NodeOptions & options)
  : Node("point_cloud_to_image", options)
  {
    // print some info about the node
    std::string cloud_topic_ = this->declare_parameter<std::string>("input");
    std::string image_topic_ = this->declare_parameter<std::string>("output");

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(cloud_topic_, 30,
      std::bind(&PointCloudToImage::cloud_cb, this, std::placeholders::_1));
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(image_topic_, 30);

    RCLCPP_INFO_STREAM(this->get_logger(), "Listening for incoming data on topic " << cloud_topic_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing image on topic " << image_topic_);
  }

private:
  sensor_msgs::msg::Image image_; // cache the image message
  std::string cloud_topic_;  // default input
  std::string image_topic_;  // default output
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>> sub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> image_pub_;  // image message publisher
};
}

RCLCPP_COMPONENTS_REGISTER_NODE(pcl_ros::PointCloudToImage)