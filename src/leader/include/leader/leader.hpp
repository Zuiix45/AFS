#pragma once

#include <memory>
#include <thread>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_command_ack.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;

/**
 * @class Leader
 * @brief A class that represents a leader node in a ROS2 system.
 * 
 * This class inherits from rclcpp::Node and is responsible for handling
 * vehicle commands, waypoints, and publishing control modes and commands.
 */
class Leader : public rclcpp::Node {
    public:
        /**
         * @brief Constructor for the Leader class.
         */
        Leader();

        /**
         * @brief Checks if the node should exit.
         * @return True if the node should exit, false otherwise.
         */
        bool shouldExit();
    
    private:
        /**
         * @brief Callback function for vehicle command acknowledgment subscription.
         * @param msg Shared pointer to the received VehicleCommandAck message.
         */
        void subVehicleCommandAckCallback(const px4_msgs::msg::VehicleCommandAck::SharedPtr msg);

        /**
         * @brief Callback function for waypoint subscription.
         * @param msg Shared pointer to the received PoseStamped message. 
         */
        void subWaypointCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

        /**
         * @brief Callback function for the publish timer.
         */
        void publishTimerCallback();

        /**
         * @brief Publishes a vehicle command.
         * @param command The command to be sent.
         * @param param1 Optional parameter 1 (default is 0.0).
         * @param param2 Optional parameter 2 (default is 0.0).
         * @param param3 Optional parameter 3 (default is 0.0).
         * @param param4 Optional parameter 4 (default is 0.0).
         * @param param5 Optional parameter 5 (default is 0.0).
         * @param param6 Optional parameter 6 (default is 0.0).
         * @param param7 Optional parameter 7 (default is 0.0).
         */
        void pubVehicleCommand(uint16_t command, float param1=0.0, float param2=0.0, float param3=0.0,
                    float param4=0.0, float param5=0.0, float param6=0.0, float param7=0.0);

        /**
         * @brief Publishes the offboard control mode.
         */
        void pubOffboardControlMode();

        bool exit; ///< Flag indicating whether the node should exit.

        geometry_msgs::msg::PoseStamped::SharedPtr current_pose_; ///< Current pose of the vehicle.

        rclcpp::TimerBase::SharedPtr timer_; ///< Timer for periodic publishing.
        
        rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr vehicleCommandAck_sub_; ///< Subscription for vehicle command acknowledgments.

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_sub_; ///< Subscription for waypoints.

        rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboardControlMode_pub_; ///< Publisher for offboard control mode.
        rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectorySetpoint_pub_; ///< Publisher for trajectory setpoint.
        
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr currentPose_pub_; ///< Publisher for the current pose.
        rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboardControlMode_pub_; ///< Publisher for the offboard control mode.
        rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicleCommand_pub_; ///< Publisher for vehicle commands.
};
