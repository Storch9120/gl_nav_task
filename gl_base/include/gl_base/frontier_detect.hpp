#include <memory>
#include <vector>
#include <queue>
#include <mutex>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::placeholders;

using Point = geometry_msgs::msg::Point;
using Pose = geometry_msgs::msg::Pose;
using OccGrid = nav_msgs::msg::OccupancyGrid;
using NavToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav2 = rclcpp_action::ClientGoalHandle<NavToPose>;
using Cell = std::tuple<int, int>;


struct Frontier {
    int size;
    Point centroid;
};


class FrontierDetect : public rclcpp::Node {
    public:
        FrontierDetect();

        /* * Main loop to run the frontier detection and navigation
         * 1. Pull latest map data
         * 2. Run expanding wavefront frontier detection
         * 3. Cluster frontiers to calc the centroid
         * 4. Select best frontier to go to
         * 5. Send goal to nav2
         * 6. Wait for goal completion and repeat
        */
        void tick();

    private:
        std::atomic<bool> ready_for_next_goal{true}, map_received{false};
        std::mutex map_mutex, tick_mutex;

        // * ROS Vars
        OccGrid::SharedPtr map;
        float map_res;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        Point last_robot_pose;

        // * ROS Pub, Sub, Clients
        rclcpp::Subscription<OccGrid>::SharedPtr map_sub;
        rclcpp_action::Client<NavToPose>::SharedPtr nav_client;
        rclcpp::TimerBase::SharedPtr tick_timer_;

        // * Callbacks
        void mapCb(const OccGrid::SharedPtr msg);

        // * Accessors
        OccGrid::SharedPtr getMap();

        // * Core functions
        std::vector<Frontier> detectFrontiers(const OccGrid::SharedPtr& map);
        Point targetedFrontier(const std::vector<Frontier>& clusters, const OccGrid::SharedPtr& map_ptr);

        // * Helper functions
        Point getRobotPose();
        Cell getCellFromPose(const Point& point);
        double distance2D(const Point& rp, const Point& centroid);

        static std::vector<Cell> neighborsWADX(
            const OccGrid::SharedPtr& mapdata,
            const Cell& cell
        );
        static std::vector<Cell> neighborsQWEADZXC(
            const OccGrid::SharedPtr& mapdata,
            const Cell& cell
        );

        static int fetchCellData(
            const OccGrid::SharedPtr& mapdata,
            const Cell& cell
        );

        static geometry_msgs::msg::Point cellPosToMap(
            const OccGrid::SharedPtr& mapdata,
            const Cell& grid_point
        );

        static Frontier growFrontier(
            const OccGrid::SharedPtr& mapdata,
            const Cell& initial_cell,
            std::map<Cell, bool>& is_frontier
        );

        static bool checkCellFrontier(
            const OccGrid::SharedPtr& mapdata,
            const Cell& cell,
            const std::map<Cell, bool>& is_frontier
        );

        void sendGoal(const Point& target);

};