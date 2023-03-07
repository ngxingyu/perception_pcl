/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 * $Id: pointcloud_to_pcd.cpp 33238 2010-03-11 00:46:58Z rusu $
 *
 */

// ROS core
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include "rclcpp_components/register_node_macro.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>

// PCL includes
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Geometry>

// STL
#include <string>
#include <memory>

/**
\author Radu Bogdan Rusu

@b pointcloud_to_pcd is a simple node that retrieves a ROS point cloud message and saves it to disk into a PCD (Point
Cloud Data) file format.

**/
namespace pcl_ros
{
class PointCloudToPCD: public rclcpp::Node
{
private:
  std::string prefix_;
  bool binary_;
  bool compressed_;
  std::string fixed_frame_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>> sub_;

public:
  std::string cloud_topic_;


  ////////////////////////////////////////////////////////////////////////////////
  // Callback
  void
  cloud_cb(const sensor_msgs::msg::PointCloud2::SharedPtr ros_cloud)
  {
    pcl::PCLPointCloud2 cloud;
    pcl_conversions::toPCL(*ros_cloud, cloud);


    if ((cloud.width * cloud.height) == 0) {
      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Received %d data points in frame %s with the following fields: %s",
      (int)cloud.width * cloud.height,
      cloud.header.frame_id.c_str(),
      pcl::getFieldsList(cloud).c_str());

    Eigen::Vector4f v = Eigen::Vector4f::Zero();
    Eigen::Quaternionf q = Eigen::Quaternionf::Identity();
    if (!fixed_frame_.empty()) {
      if (!tf_buffer_->canTransform(
          fixed_frame_, cloud.header.frame_id,
          pcl_conversions::fromPCL(cloud.header.stamp), rclcpp::Duration(3, 0)))
      {
        RCLCPP_WARN(this->get_logger(), "Could not get transform!");
        return;
      }

      Eigen::Affine3d transform;
      transform =
        tf2::transformToEigen(
        tf_buffer_->lookupTransform(
          fixed_frame_, cloud.header.frame_id,
          pcl_conversions::fromPCL(cloud.header.stamp)));
      v = Eigen::Vector4f::Zero();
      v.head<3>() = transform.translation().cast<float>();
      q = transform.rotation().cast<float>();
    }

    std::stringstream ss;
    ss << prefix_ << cloud.header.stamp << ".pcd";
    RCLCPP_INFO(this->get_logger(), "Data saved to %s", ss.str().c_str());

    pcl::PCDWriter writer;
    if (binary_) {
      if (compressed_) {
        writer.writeBinaryCompressed(ss.str(), cloud, v, q);
      } else {
        writer.writeBinary(ss.str(), cloud, v, q);
      }
    } else {
      writer.writeASCII(ss.str(), cloud, v, q, 8);
    }
  }

  ////////////////////////////////////////////////////////////////////////////////
  explicit PointCloudToPCD(const rclcpp::NodeOptions & options)
  : rclcpp::Node("pointcloud_to_pcd", options), binary_(false), compressed_(false)
  {
    prefix_ = this->declare_parameter<std::string>("prefix");
    RCLCPP_INFO_STREAM(this->get_logger(), "PCD file prefix is: " << prefix_);

    fixed_frame_ = this->declare_parameter<std::string>("fixed_frame");
    binary_ = this->declare_parameter<bool>("binary");
    compressed_ = this->declare_parameter<bool>("compressed");
    if (binary_) {
      if (compressed_) {
        RCLCPP_INFO_STREAM(this->get_logger(), "Saving as binary compressed PCD");
      } else {
        RCLCPP_INFO_STREAM(this->get_logger(), "Saving as binary PCD");
      }
    } else {
      RCLCPP_INFO_STREAM(this->get_logger(), "Saving as binary PCD");
    }

    cloud_topic_ = this->declare_parameter<std::string>("input");
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2> (cloud_topic_, 1,
        std::bind(&PointCloudToPCD::cloud_cb, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(),
        "Listening for incoming data on topic %s",
        cloud_topic_.c_str());
  }
};
}
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_ros::PointCloudToPCD)