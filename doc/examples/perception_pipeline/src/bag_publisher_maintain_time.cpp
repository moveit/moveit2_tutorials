/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Ridhwan Luthra.
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
 *   * Neither the name of Ridhwan Luthra nor the names of its
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
 *********************************************************************/

/* Author: Ridhwan Luthra */
#include <thread>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/typesupport_helpers.hpp"
#include "rosbag2_cpp/storage_options.hpp"
#include "rosbag2_cpp/converter_options.hpp"
#include "rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node__ = rclcpp::Node::make_shared("bag_publisher_maintain_time", node_options);

  auto point_cloud_publisher = node__->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/camera/depth_registered/points", rclcpp::SystemDefaultsQoS());

  // Variable holding the rosbag containing point cloud data.
  rosbag2_cpp::readers::SequentialReader reader;
  rosbag2_storage::StorageOptions storage_options{};
  std::string path = ament_index_cpp::get_package_share_directory("moveit2_tutorials");
  path += "/bags/perception_tutorial";
  storage_options.uri = path;
  storage_options.storage_id = "sqlite3";

  rosbag2_cpp ::ConverterOptions converter_options{};
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";
  reader.open(storage_options, converter_options);

  std::vector<std::string> topics;
  topics.push_back("/camera/depth_registered/points");

  // Iterator for topics in bag.
  if (reader.has_next())
  {
    // serialized data
    auto serialized_message = reader.read_next();

    // deserialization and conversion to ros message
    sensor_msgs::msg::PointCloud2 msg;
    auto ros_message = std::make_shared<rosbag2_cpp::rosbag2_introspection_message_t>();
    ros_message->time_stamp = 0;
    ros_message->message = nullptr;
    ros_message->allocator = rcutils_get_default_allocator();
    ros_message->message = &msg;
    auto type_library = rosbag2_cpp::get_typesupport_library("sensor_msgs/PointCloud2", "rosidl_typesupport_cpp");
    auto type_support =
        rosbag2_cpp::get_typesupport_handle("sensor_msgs/PointCloud2", "rosidl_typesupport_cpp", type_library);

    rosbag2_cpp::SerializationFormatConverterFactory factory;
    std::unique_ptr<rosbag2_cpp::converter_interfaces::SerializationFormatDeserializer> cdr_deserializer_;
    cdr_deserializer_ = factory.load_deserializer("cdr");

    cdr_deserializer_->deserialize(serialized_message, type_support, ros_message);

    while (rclcpp::ok())
    {
      msg.header.stamp = node__->now();
      point_cloud_publisher->publish(msg);
      rclcpp::spin_some(node__);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
  reader.close();
  return 0;
}
