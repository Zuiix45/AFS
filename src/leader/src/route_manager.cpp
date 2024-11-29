#include "leader/route_manager.hpp"

#define MAX_DISTANCE 10.0
#define ANGLE_LIMIT M_PI / 6

RouteManager::RouteManager() : Node("route_manager"), exit(false) {
    setToleranceFactor(0.00005);

    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, 5), rmw_qos_profile_sensor_data);

    currentPose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("/leader_pose", 
    qos, std::bind(&RouteManager::subCurrentPoseCallback, this, std::placeholders::_1));

    waypoint_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/current_waypoint", 10);

    timer_ = create_wall_timer(100ms, std::bind(&RouteManager::publishTimerCallback, this));

    current_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>();

    generateWaypoint(MAX_DISTANCE, ANGLE_LIMIT);

    srand(time(NULL));
}

bool RouteManager::shouldExit() {
    return exit;
}

float RouteManager::getDistanceToWaypoint() {
    auto currentWaypoint = waypoints.end()[-1];

    float dx = currentWaypoint.pose.position.x - current_pose_->pose.position.x;
    float dy = currentWaypoint.pose.position.y - current_pose_->pose.position.y;

    return std::sqrt(dx * dx + dy * dy);
}

float RouteManager::getAngleToWaypoint() {
    auto currentWaypoint = waypoints.end()[-1];

    float dx = currentWaypoint.pose.position.x - current_pose_->pose.position.x;
    float dy = currentWaypoint.pose.position.y - current_pose_->pose.position.y;

    return std::atan2(dy, dx);
}

void RouteManager::generateWaypoint(float maxDistance, float angleLimit) {
    if (waypoints.size() == 0) {
        waypoints.push_back(*current_pose_);
        return;
    }

    geometry_msgs::msg::PoseStamped newWaypoint;

    // Generate a random waypoint within the specified limits

    // 0 to maxDistance
    float distance = maxDistance * static_cast<float>(rand()) / RAND_MAX;

    // PI/2 - angleLimit to PI/2 + angleLimit
    float angle = M_PI/2 - angleLimit * static_cast<float>(rand() - RAND_MAX/2) / RAND_MAX / 2;

    newWaypoint.pose.position.x = waypoints.end()[-1].pose.position.x + distance * std::cos(angle);
    newWaypoint.pose.position.y = waypoints.end()[-1].pose.position.y + distance * std::sin(angle);

    waypoints.push_back(newWaypoint);
}

bool RouteManager::isOnPoint() {
    if (waypoints.size() == 0) {
        return false;
    }

    return getDistanceToWaypoint() < toleranceFactor;
}

void RouteManager::publishTimerCallback() {
    if (isOnPoint()) {
        generateWaypoint(MAX_DISTANCE, ANGLE_LIMIT);
    }

    waypoint_pub_->publish(waypoints.end()[-1]);
}

void RouteManager::subCurrentPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    current_pose_ = msg;
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<RouteManager>();

    std::thread node_thread([node]() {
        while (rclcpp::ok() && !node->shouldExit()) {
            rclcpp::spin_some(node);
        }
    });

    node_thread.join();
    rclcpp::shutdown();
    return 0;
}