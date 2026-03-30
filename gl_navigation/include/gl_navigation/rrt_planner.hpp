#include <atomic>
#include <cmath>
#include <memory>
#include <random>
#include <thread>
#include <vector>
#include <mutex>
#include <condition_variable>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::placeholders;

using Point = geometry_msgs::msg::Point;
using Pose = geometry_msgs::msg::Pose;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using Path = nav_msgs::msg::Path;
using OccGrid = nav_msgs::msg::OccupancyGrid;
using FollowPath = nav2_msgs::action::FollowPath;
using GoalHandleNav2 = rclcpp_action::ClientGoalHandle<FollowPath>;
using Cell = std::tuple<int, int>;


struct TNode {
    Cell cell;
    int parent_index;
};

class RRTPlanner : public rclcpp::Node {
    public:
        RRTPlanner();

    private:
        std::mutex map_mutex, goal_mutex;
        std::atomic<bool> new_goal_received{false}, goal_reached{false};
        std::condition_variable cv_;
        std::thread planner_thread;
        std::random_device rd;
        std::mt19937 generator;

        // * RRT State
        int steer_distance; // cells => resolution * 10 = 0.5m; tunable
        int goal_threshold; // cells => resolution * 3 = 0.15m; tunable
        int MAX_ITERATIONS;
        std::vector<TNode> tree;

        // * ROS Vars
        OccGrid::SharedPtr map;
        float map_res;
        PoseStamped goal_pose;

        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        // * ROS Pub, Sub, Clients
        rclcpp::Subscription<PoseStamped>::SharedPtr goal_pose_sub;
        rclcpp::Subscription<OccGrid>::SharedPtr map_sub;
        rclcpp::Publisher<Path>::SharedPtr plan_pub;
        rclcpp_action::Client<FollowPath>::SharedPtr nav_client;

        // * Callbacks
        void goalPoseCb(const PoseStamped::SharedPtr msg);
        void loadMapCb(const OccGrid::SharedPtr msg);

        // * Core functions
        bool makePlan();
        bool findRoute(const OccGrid::SharedPtr& mapdata, const Pose& start, const Pose& goal, Path& plan);

        Cell sampleRandomCell(const OccGrid::SharedPtr& mapdata);
        int findNearestNode(const Cell& cell);
        Cell steer(const Cell& from, const Cell& to);
        bool isCollisionFree(const OccGrid::SharedPtr& mapdata, const Cell& from, const Cell& to);
        bool isGoalReached(const Cell& cell, const Cell& c_goal);
        void extractPath(const OccGrid::SharedPtr& mapdata, Path& plan);

        // * Helper functions
        Pose getRobotPose();
        OccGrid::SharedPtr getMap();
        Cell getCellFromPose(const OccGrid::SharedPtr& mapdata, const Pose& pose);
        void sendGoal(const Path& plan);

};