#pragma once

#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_command_ack.hpp"
#include "px4_msgs/msg/vehicle_control_mode.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;

/**
 * @class Follower
 * @brief A class that represents a follower node in ROS2.
 * 
 * This class is responsible for subscribing to various topics, processing the received data,
 * and publishing commands to control a vehicle. It inherits from rclcpp::Node.
 */
class Follower : public rclcpp::Node {
    public:
        /**
         * @brief Constructor for the Follower class.
         */
        Follower();

        /**
         * @brief Checks if the node should exit.
         * @return True if the node should exit, false otherwise.
         */
        bool shouldExit();

        /**
         * @brief Arms the vehicle.
         */
        void arm();

        /**
         * @brief Disarms the vehicle.
         */
        void disarm();

        /**
         * @brief Gets the current latitude of the vehicle.
         * @return The current latitude.
         */
        float getLatitude();

        /**
         * @brief Gets the current longitude of the vehicle.
         * @return The current longitude.
         */
        float getLongitude();

        /**
         * @brief Gets the current altitude of the vehicle.
         * @return The current altitude.
         */
        float getAltitude();
    
    private:
        /**
         * @brief Callback function for the global position subscription.
         * @param msg The received message.
         */
        void subGlobalPosCallback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg);

        /**
         * @brief Callback function for the trajectory subscription.
         * @param msg The received message.
         */
        void subTrajectoryCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

        /**
         * @brief Callback function for the vehicle command acknowledgment subscription.
         * @param msg The received message.
         */
        void subVehicleCommandAckCallback(const px4_msgs::msg::VehicleCommandAck::SharedPtr msg);

        /**
         * @brief Timer callback function for publishing data.
         */
        void publishTimerCallback();

        /**
         * @brief Publishes the offboard control mode.
         */
        void pubOffboardControlMode();

        /**
         * @brief Publishes the trajectory setpoint.
         */
        void pubTrajectorySetpoint();

        /**
         * @brief Publishes a vehicle command.
         * @param command The command to be sent.
         * @param param1 Optional parameter 1.
         * @param param2 Optional parameter 2.
         * @param param3 Optional parameter 3.
         * @param param4 Optional parameter 4.
         * @param param5 Optional parameter 5.
         * @param param6 Optional parameter 6.
         * @param param7 Optional parameter 7.
         */
        void pubVehicleCommand(uint16_t command, float param1=0.0, float param2=0.0, float param3=0.0,
                            float param4=0.0, float param5=0.0, float param6=0.0, float param7=0.0);

        bool exit; ///< Flag indicating whether the node should exit.

        geometry_msgs::msg::PoseStamped::SharedPtr current_pose_; ///< Current pose of the vehicle.

        rclcpp::TimerBase::SharedPtr timer_; ///< Timer for periodic publishing.

        rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr globalPosition_sub_; ///< Subscription to global position.
        rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr vehicleCommandAck_sub_; ///< Subscription to vehicle command acknowledgment.

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr trajectory_sub_; ///< Subscription to trajectory.

        rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboardControlMode_pub_; ///< Publisher for offboard control mode.
        rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectorySetpoint_pub_; ///< Publisher for trajectory setpoint.
        rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicleCommand_pub_; ///< Publisher for vehicle commands.
};