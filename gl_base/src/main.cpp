#include <gl_base/frontier_detect.hpp>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FrontierDetect>();
    RCLCPP_INFO(node->get_logger(), "[FrontierGuidance] Starting Node...");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}