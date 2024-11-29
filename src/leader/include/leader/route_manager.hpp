#pragma once

#include <memory>
#include <vector>
#include <thread>
#include <cmath>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;

/**
 * @class RouteManager
 * @brief Manages the route for the drone, including waypoints and current pose.
 * 
 * This class inherits from rclcpp::Node and provides functionalities to manage
 * the route of the drone. It handles the current pose, waypoints, and provides
 * methods to calculate distances and angles to waypoints.
 */
class RouteManager : public rclcpp::Node {
    public:
        /**
         * @brief Constructor for RouteManager.
         */
        RouteManager();

        /**
         * @brief Checks if the route manager should exit.
         * @return True if the route manager should exit, false otherwise.
         */
        bool shouldExit();

        /**
         * @brief Gets the distance to the current waypoint.
         * @return The distance to the current waypoint.
         */
        float getDistanceToWaypoint();

        /**
         * @brief Gets the angle to the current waypoint.
         * @return The angle to the current waypoint.
         */
        float getAngleToWaypoint();

        /**
         * @brief Sets the tolerance factor for waypoint proximity.
         * @param factor The tolerance factor.
         */
        void setToleranceFactor(float factor) { toleranceFactor = factor; }

    private:
        /**
         * @brief Callback function for the current pose subscription.
         * @param msg The current pose message.
         */
        void subCurrentPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

        /**
         * @brief Callback function for the publish timer.
         */
        void publishTimerCallback();

        /**
         * @brief Generates a new waypoint based on the given parameters.
         * @param maxDistance The maximum distance for the new waypoint.
         * @param angleLimit The angle limit for the new waypoint.
         */
        void generateWaypoint(float maxDistance, float angleLimit);

        /**
         * @brief Checks if the robot is on the current waypoint.
         * @return True if the robot is on the current waypoint, false otherwise.
         */
        bool isOnPoint();

        float toleranceFactor; ///< Tolerance factor for waypoint proximity.
        bool exit; ///< Flag indicating if the route manager should exit.
        geometry_msgs::msg::PoseStamped::SharedPtr current_pose_; ///< Current pose of the robot.
        std::vector<geometry_msgs::msg::PoseStamped> waypoints; ///< List of waypoints.
        rclcpp::TimerBase::SharedPtr timer_; ///< Timer for publishing waypoints.
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr currentPose_sub_; ///< Subscription for current pose.
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_pub_; ///< Publisher for waypoints.
};
