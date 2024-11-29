#pragma once

#include <memory>
#include <thread>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_command_ack.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;

class Leader : public rclcpp::Node {
    public:
        Leader();

        bool shouldExit();
    
    private:
        void subVehicleCommandAckCallback(const px4_msgs::msg::VehicleCommandAck::SharedPtr msg);
        void subWaypointCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

        void publishTimerCallback();

        void pubVehicleCommand(uint16_t command, float param1=0.0, float param2=0.0, float param3=0.0,
                    float param4=0.0, float param5=0.0, float param6=0.0, float param7=0.0);

        void pubOffboardControlMode();

        bool exit;

        geometry_msgs::msg::PoseStamped::SharedPtr current_pose_;

        rclcpp::TimerBase::SharedPtr timer_;
        
        rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr vehicleCommandAck_sub_;

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_sub_;

        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr currentPose_pub_;
        rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboardControlMode_pub_;
        rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicleCommand_pub_;
};