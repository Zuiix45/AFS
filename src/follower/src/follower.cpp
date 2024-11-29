#include "follower/follower.hpp"

#define VehicleCommand px4_msgs::msg::VehicleCommand

Follower::Follower() : Node("follower"), exit(false) {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    globalPosition_sub_ = create_subscription<px4_msgs::msg::VehicleGlobalPosition>("/fmu/out/vehicle_global_position", 
    qos, std::bind(&Follower::subGlobalPosCallback, this, std::placeholders::_1));

    vehicleCommandAck_sub_ = create_subscription<px4_msgs::msg::VehicleCommandAck>("/fmu/out/vehicle_command_ack", 
    qos, std::bind(&Follower::subVehicleCommandAckCallback, this, std::placeholders::_1));

    offboardControlMode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectorySetpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    vehicleCommand_pub_ = create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

    timer_ = create_wall_timer(100ms, std::bind(&Follower::publishTimerCallback, this));

    current_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>();

    this->pubVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
    this->arm();

    this->pubVehicleCommand(VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 10);

    this->pubVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_CHANGE_SPEED, 1, 100.0, 100.0);
}

bool Follower::shouldExit() {
    return exit;
}

void Follower::arm() {
    this->pubVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1);

    RCLCPP_INFO(get_logger(), "Arming...");
}

void Follower::disarm() {
    this->pubVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0);

    RCLCPP_INFO(get_logger(), "Disarming...");
}

void Follower::subGlobalPosCallback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg) {
    current_pose_->header.stamp = this->get_clock()->now();
    current_pose_->pose.position.x = msg->lat;
    current_pose_->pose.position.y = msg->lon;
    current_pose_->pose.position.z = msg->alt;

    //RCLCPP_INFO(get_logger(), "Current position: %f, %f, %f", msg->lat, msg->lon, msg->alt);
}

void Follower::subTrajectoryCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    RCLCPP_INFO(get_logger(), "Trajectory received, position: %f, %f, %f", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}

void Follower::subVehicleCommandAckCallback(const px4_msgs::msg::VehicleCommandAck::SharedPtr msg) {
    RCLCPP_INFO(get_logger(), "Command ack received, id: %d, result: %d", msg->command, msg->result);

}

void Follower::pubOffboardControlMode() {
    px4_msgs::msg::OffboardControlMode msg;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.position = true;
    msg.velocity = true;
    msg.acceleration = true;
    msg.attitude = false;
    msg.body_rate = false;

    offboardControlMode_pub_->publish(msg);
}

void Follower::pubTrajectorySetpoint() {
    px4_msgs::msg::TrajectorySetpoint msg;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.yaw = -3.14;
    msg.position = {110.0, 200.0, 0.0};

    trajectorySetpoint_pub_->publish(msg);
}

void Follower::pubVehicleCommand(uint16_t command, float param1, float param2, float param3,
                            float param4, float param5, float param6, float param7) {

    VehicleCommand msg;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    msg.param1 = param1;
    msg.param2 = param2;
    msg.param3 = param3;
    msg.param4 = param4;
    msg.param5 = param5;
    msg.param6 = param6;
    msg.param7 = param7;

    msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;

    RCLCPP_INFO(get_logger(), "Sending command, id: %d", command);

    vehicleCommand_pub_->publish(msg);
}

void Follower::publishTimerCallback() {
    this->pubOffboardControlMode();
    this->pubTrajectorySetpoint();
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Follower>();

    std::thread node_thread([node]() {
        while (rclcpp::ok() && !node->shouldExit()) {
            rclcpp::spin_some(node);
        }
    });

    node_thread.join();
    rclcpp::shutdown();
    return 0;
}