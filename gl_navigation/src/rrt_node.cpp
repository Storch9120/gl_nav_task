#include <gl_navigation/rrt_planner.hpp>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RRTPlanner>();
    RCLCPP_INFO(node->get_logger(), "[RRTPlanner] Starting Node...");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}