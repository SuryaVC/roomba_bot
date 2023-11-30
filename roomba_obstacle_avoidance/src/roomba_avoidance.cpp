// Copyright (c) 2023 Suryavardhan Reddy Chappidi
/**
 * @file roomba_avoidance.cpp
 * @author Suryavardhan Reddy Chappidi (chappidi@umd.edu)
 * @brief Roomba Avoidance Node for TurtleBot3
 * @details This program implements a simple obstacle avoidance algorithm for
 * TurtleBot3, similar to a Roomba vacuum cleaner. The robot uses laser scan
 * data to detect obstacles in front and alternates between moving forward and
 * rotating in place to avoid collisions.
 * @version 0.1
 * @date 2023-11-29
 * @copyright Copyright (c) 2023
 *
 */

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

/**
 * @class RoombaAvoidance
 * @brief Implements obstacle avoidance for a Turtlebot3.
 *
 * This class creates a ROS2 node that drives a TurtleBot3, utilizing laser scan
 * data to detect and avoid obstacles. The robot moves forward when the path is
 * clear and rotates in place upon detecting an obstacle directly in front.
 */
class RoombaAvoidance : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for RoombaAvoidance
   *
   * Sets up publisher and subscriber for robot control and laser scan data.
   */
  RoombaAvoidance() : Node("roomba_avoidance") {
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    laser_scan_subscriber_ =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 5,
            std::bind(&RoombaAvoidance::laser_scan_callback, this,
                      std::placeholders::_1));
  }

 private:
  /**
   * @brief Callback for laser scan data
   *
   * Processes incoming laser scan data to detect obstacles in the frontal area.
   * The robot's movement is adjusted based on the presence of obstacles,
   * alternating between moving forward and rotating in place.
   *
   * @param msg Shared pointer to the received laser scan message.
   */
  void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (msg->header.stamp.sec == 0) {
      return;
    }

    auto scan_data = msg->ranges;
    auto field_range = 60;
    auto initial_angle = 330;
    bool obstacle_detected = false;

    for (int i = initial_angle; i < initial_angle + field_range; i++) {
      if (scan_data[i % 360] < 0.75) {
        obstacle_detected = true;
        break;
      }
    }

    if (obstacle_detected) {
      move_robot(0.0, 0.3);  // Rotate in place
    } else {
      move_robot(0.5, 0.0);  // Move forward
    }
  }

  /**
   * @brief function to control the robot's movement.
   *
   * This function sets the linear and angular velocities of the robot based on
   * the parameters provided. It is used to either move the robot forward or
   * rotate it in place.
   *
   * @param linear_velocity Linear velocity for forward/backward motion.
   * @param angular_velocity Angular velocity for rotation.
   */
  void move_robot(double linear_velocity, double angular_velocity) {
    velocity_msg.linear.x = linear_velocity;
    velocity_msg.angular.z = angular_velocity;
    publisher_->publish(velocity_msg);
  }

  geometry_msgs::msg::Twist
      velocity_msg;  ///< Velocity message for robot movement
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
      publisher_;  ///< Publisher for velocity commands
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_scan_subscriber_;  ///< Subscriber for laser scan data
};

/**
 * @brief Main function
 *
 * Initializes and runs the RoombaAvoidance node. This node controls the
 * movement of the robot based on laser scan data for obstacle avoidance.
 */
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RoombaAvoidance>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}