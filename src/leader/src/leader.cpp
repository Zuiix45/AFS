#include "leader/leader.hpp"

#define VehicleCommand px4_msgs::msg::VehicleCommand

Leader::Leader() : Node("leader"), exit(false) {
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, 5), rmw_qos_profile_sensor_data);

    vehicleCommandAck_sub_ = create_subscription<px4_msgs::msg::VehicleCommandAck>("/fmu/out/vehicle_command_ack", 
    qos, std::bind(&Leader::subVehicleCommandAckCallback, this, std::placeholders::_1));

    waypoint_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("/current_waypoint", 
    qos, std::bind(&Leader::subWaypointCallback, this, std::placeholders::_1));

    currentPose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/leader_pose", 10);
    offboardControlMode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    vehicleCommand_pub_ = create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

    timer_ = create_wall_timer(100ms, std::bind(&Leader::publishTimerCallback, this));

    current_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>();
}

bool Leader::shouldExit() {
    return exit;
}

void Leader::publishTimerCallback() {
    currentPose_pub_->publish(*current_pose_);
}

void Leader::pubVehicleCommand(uint16_t command, float param1, float param2, float param3,
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

void Leader::pubOffboardControlMode() {
    px4_msgs::msg::OffboardControlMode msg;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.position = true;
    msg.velocity = true;
    msg.acceleration = true;
    msg.attitude = false;
    msg.body_rate = false;

    offboardControlMode_pub_->publish(msg);
}

void Leader::pubTrajectorySetpoint() {
    px4_msgs::msg::TrajectorySetpoint msg;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.yaw = -3.14;

    msg.position[0] = waypoint_->pose.position.x;
    msg.position[1] = waypoint_->pose.position.y;
    msg.position[2] = waypoint_->pose.position.z;

    trajectorySetpoint_pub_->publish(msg);
}

void Leader::subVehicleCommandAckCallback(const px4_msgs::msg::VehicleCommandAck::SharedPtr msg) {
    RCLCPP_INFO(get_logger(), "Command ack received, id: %d, result: %d", msg->command, msg->result);
}

void Leader::subWaypointCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    waypoint_ = msg;
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Leader>();

    std::thread node_thread([node]() {
        while (rclcpp::ok() && !node->shouldExit()) {
            rclcpp::spin_some(node);
        }
    });

    node_thread.join();
    rclcpp::shutdown();
    return 0;
}