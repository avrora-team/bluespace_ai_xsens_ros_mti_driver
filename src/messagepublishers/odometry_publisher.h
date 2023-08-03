//
// Created by amandil on 03.08.23.
//

#ifndef BUILD_ODOMETRY_PUBLISHER_H
#define BUILD_ODOMETRY_PUBLISHER_H

#include "packetcallback.h"
#include <nav_msgs/msg/odometry.hpp>
#include <GeographicLib/UTMUPS.hpp>

struct OdometryPublisher : public PacketCallback
{
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub;
  std::string frame_id = DEFAULT_FRAME_ID;
  XsVector UTM0;

  OdometryPublisher(rclcpp::Node &node) : UTM0(3) {
    int pub_queue_size = 5;
    node.get_parameter("publisher_queue_size", pub_queue_size);
    pub = node.create_publisher<nav_msgs::msg::Odometry>("/imu/odometry", pub_queue_size);
    node.get_parameter("frame_id", frame_id);
    UTM0[0] = std::numeric_limits<XsReal>::max();
  }

  void operator()(const XsDataPacket &packet, rclcpp::Time timestamp) {
    nav_msgs::msg::Odometry odoMsg;
    if (packet.containsPositionLLA() &&
        packet.containsOrientation() &&
        packet.containsVelocity() &&
        packet.containsCalibratedGyroscopeData()) {
      std::cout << "ZHOPA" << std::endl;
      XsVector LLA = packet.positionLLA();
      XsVector UTM(3);
      UTM[2] = LLA[2];
      int zone;
      bool northp;
      GeographicLib::UTMUPS::Forward(LLA[0], LLA[1], zone, northp, UTM[0], UTM[1]);

      if(UTM0[0] == std::numeric_limits<XsReal>::max()) {
        UTM0 = UTM;
      }

      odoMsg.header.frame_id = frame_id;
      odoMsg.header.stamp = timestamp;

      odoMsg.pose.pose.position.x = UTM[0] - UTM0[0];
      odoMsg.pose.pose.position.y = UTM[1] - UTM0[1];
      odoMsg.pose.pose.position.z = UTM[2] - UTM0[2];

      XsQuaternion q = packet.orientationQuaternion();
      odoMsg.pose.pose.orientation.w = q.w();
      odoMsg.pose.pose.orientation.x = q.x();
      odoMsg.pose.pose.orientation.y = q.y();
      odoMsg.pose.pose.orientation.z = q.z();

      XsVector v = packet.velocity();
      XsVector gyro = packet.calibratedGyroscopeData();
      odoMsg.twist.twist.linear.x = v[0];
      odoMsg.twist.twist.linear.y = v[1];
      odoMsg.twist.twist.linear.z = v[2];

      odoMsg.twist.twist.angular.x = gyro[0];
      odoMsg.twist.twist.angular.y = gyro[1];
      odoMsg.twist.twist.angular.z = gyro[2];

      pub->publish(odoMsg);
    }
  }
};


#endif //BUILD_ODOMETRY_PUBLISHER_H
