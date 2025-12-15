// Simple Aeva-to-ROS bridge - updated for full frame point cloud
// Shows dense point cloud like web viewer

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include "aeva/api/AevaAPI.h"

using namespace aeva;

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " SENSOR_IP SENSOR_NAME" << std::endl;
    return EXIT_FAILURE;
  }

  // Initialize ROS
  ros::init(argc, argv, "aeva_simple_bridge");
  ros::NodeHandle nh;

  std::string sensor_ip = argv[1];
  std::string sensor_name = argv[2];

  // Create publisher
  ros::Publisher pc_pub = nh.advertise<sensor_msgs::PointCloud2>(
      "/" + sensor_name + "_0/points", 1);

  // Setup Aeva API
  api::AevaAPI api;
  api::Sensor sensor;
  sensor.id_ = sensor_name;
  sensor.url_ = sensor_ip;
  sensor.protocol_ = api::Sensor::Protocol::TCP;
  sensor.platform_.type_ = api::Sensor::Platform::Type::ATLAS;

  ROS_INFO("Connecting to Aeva at %s (full frame mode)...", sensor_ip.c_str());

  // Use default mode for full compensated frames (like web viewer)
  if (!api.Connect(sensor)) {
    ROS_FATAL("Failed to connect to sensor at %s", sensor_ip.c_str());
    return EXIT_FAILURE;
  }

  ROS_INFO("Connected successfully!");

  // Register callback for full frame compensated point cloud (like web viewer)
  api.RegisterPointCloudMessageCallback(
      api::kPointCloudDataStreamId,
      [&](const api::PointCloud& message) {
        sensor_msgs::PointCloud2 ros_msg;
        ros_msg.header.stamp = ros::Time::now();
        ros_msg.header.frame_id = sensor_name + "_lidar";

        ros_msg.height = 1;
        ros_msg.width = message.points.size();
        ros_msg.is_bigendian = false;
        ros_msg.is_dense = false;

        // Define fields manually
        ros_msg.fields.resize(5);

        ros_msg.fields[0].name = "x";
        ros_msg.fields[0].offset = 0;
        ros_msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
        ros_msg.fields[0].count = 1;

        ros_msg.fields[1].name = "y";
        ros_msg.fields[1].offset = 4;
        ros_msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
        ros_msg.fields[1].count = 1;

        ros_msg.fields[2].name = "z";
        ros_msg.fields[2].offset = 8;
        ros_msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
        ros_msg.fields[2].count = 1;

        ros_msg.fields[3].name = "velocity";
        ros_msg.fields[3].offset = 12;
        ros_msg.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
        ros_msg.fields[3].count = 1;

        ros_msg.fields[4].name = "reflectivity";
        ros_msg.fields[4].offset = 16;
        ros_msg.fields[4].datatype = sensor_msgs::PointField::FLOAT32;
        ros_msg.fields[4].count = 1;

        ros_msg.point_step = 20; // 5 floats * 4 bytes
        ros_msg.row_step = ros_msg.point_step * ros_msg.width;

        // Allocate data buffer
        ros_msg.data.resize(ros_msg.row_step * ros_msg.height);

        // Fill data using iterators
        sensor_msgs::PointCloud2Iterator<float> iter_x(ros_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(ros_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(ros_msg, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_v(ros_msg, "velocity");
        sensor_msgs::PointCloud2Iterator<float> iter_r(ros_msg, "reflectivity");

        for (const auto& point : message.points) {
          *iter_x = point.x;
          *iter_y = point.y;
          *iter_z = point.z;
          *iter_v = point.v;
          *iter_r = point.reflectivity;

          ++iter_x; ++iter_y; ++iter_z; ++iter_v; ++iter_r;
        }

        pc_pub.publish(ros_msg);
      });

  api.Subscribe(sensor.id_, api::kPointCloudDataStreamId);

  ROS_INFO("Subscribed to FULL FRAME stream (compensated point cloud).");
  ROS_INFO("Publishing to /%s_0/points - full frames like web viewer!", sensor_name.c_str());

  // Poll events
  ros::Rate rate(100);  // 100 Hz polling (frames come at ~10-20Hz)
  while (ros::ok()) {
    api.PollEvents();
    ros::spinOnce();
    rate.sleep();
  }

  api.Disconnect(sensor.id_);
  return 0;
}
