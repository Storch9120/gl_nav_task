/**
 * @file rrt_planner.hpp
 * @brief Rapidly-exploring Random Tree (RRT) based global planner node for ROS2 Nav2.
 *
 * This node subscribes to a goal pose and occupancy grid map, generates a collision-free
 * path using the RRT algorithm, and sends the resulting path to Nav2's FollowPath action server.
 */

#pragma once

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

/**
 * @struct TNode
 * @brief Node representation for the RRT tree.
 */
struct TNode {
    Cell cell;              ///< Grid cell (x, y)
    int parent_index;       ///< Index of parent node in tree
};

/**
 * @class RRTPlanner
 * @brief ROS2 node implementing an RRT-based path planner.
 *
 * Responsibilities:
 * - Subscribe to goal pose and occupancy grid map
 * - Generate a path using RRT
 * - Publish the path
 * - Send the path to Nav2 FollowPath action server
 */
class RRTPlanner : public rclcpp::Node {
    public:
        RRTPlanner();

    private:

        std::mutex map_mutex, goal_mutex;
        std::atomic<bool> new_goal_received{false};///< Flag indicating a new goal
        std::atomic<bool> goal_reached{false};     ///< Flag indicating goal completion
        std::condition_variable cv_;               ///< Condition variable for planner thread
        std::thread planner_thread;                ///< Background planning thread

        std::random_device rd;                     ///< Random device
        std::mt19937 generator;                    ///< Random number generator

        // * RRT Parameters and State Vars
        int steer_distance;    ///< Max step size (in cells)
        int goal_threshold;    ///< Distance threshold to consider goal reached (in cells)
        int MAX_ITERATIONS;    ///< Maximum number of RRT iterations
        std::vector<TNode> tree; ///< RRT tree structure

        // * ROS Vars
        OccGrid::SharedPtr map; ///< Latest occupancy grid map
        float map_res;          ///< Map resolution (meters/cell)
        PoseStamped goal_pose;  ///< Current goal pose

        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        // * ROS Pub, Sub, Clients
        rclcpp::Subscription<PoseStamped>::SharedPtr goal_pose_sub; ///< Goal subscriber
        rclcpp::Subscription<OccGrid>::SharedPtr map_sub;           ///< Map subscriber
        rclcpp::Publisher<Path>::SharedPtr plan_pub;                ///< Path publisher
        rclcpp_action::Client<FollowPath>::SharedPtr nav_client;    ///< Nav2 action client

        // * Callbacks
        /**
         * @brief Callback for receiving goal pose.
         * @param msg Goal pose message
         */
        void goalPoseCb(const PoseStamped::SharedPtr msg);

        /**
         * @brief Callback for receiving occupancy grid map.
         * @param msg Occupancy grid message
         */
        void loadMapCb(const OccGrid::SharedPtr msg);

        // * Accessors
        /**
         * @brief Gets current robot pose using TF.
         * @return Robot pose
         */
        Pose getRobotPose();

        /**
         * @brief Returns latest map safely.
         * @return Shared pointer to map
         */
        OccGrid::SharedPtr getMap();

        // * Core functions
        /**
         * @brief Main planning loop triggered on new goal.
         * @return true if plan successfully generated
         */
        bool makePlan();

        /**
         * @brief Executes RRT algorithm to find a path.
         * @param mapdata Occupancy grid map
         * @param start Start pose
         * @param goal Goal pose
         * @param plan Output path
         * @return true if path found
         */
        bool findRoute(const OccGrid::SharedPtr& mapdata,
                    const Pose& start,
                    const Pose& goal,
                    Path& plan);

        /**
         * @brief Samples a random free cell in the map.
         * @param mapdata Occupancy grid map
         * @return Random cell
         */
        Cell sampleRandomCell(const OccGrid::SharedPtr& mapdata);

        /**
         * @brief Finds nearest node in tree to given cell.
         * @param cell Target cell
         * @return Index of nearest node
         */
        int findNearestNode(const Cell& cell);

        /**
         * @brief Steers from one cell towards another.
         * @param from Start cell
         * @param to Target cell
         * @return New cell after steering
         */
        Cell steer(const Cell& from, const Cell& to);

        /**
         * @brief Checks if path between two cells is collision-free.
         * @param mapdata Occupancy grid map
         * @param from Start cell
         * @param to End cell
         * @return true if no collision
         */
        bool isCollisionFree(const OccGrid::SharedPtr& mapdata,
                            const Cell& from,
                            const Cell& to);

        /**
         * @brief Checks if goal is reached.
         * @param cell Current cell
         * @param c_goal Goal cell
         * @return true if within `goal_threshold`
         */
        bool isGoalReached(const Cell& cell, const Cell& c_goal);

        /**
         * @brief Extracts final path from RRT tree.
         * @param mapdata Occupancy grid map
         * @param plan Output path
         */
        void extractPath(const OccGrid::SharedPtr& mapdata, Path& plan);

        // * Helper functions
        /**
         * @brief Converts world pose to grid cell.
         * @param mapdata Occupancy grid map
         * @param pose Input pose
         * @return Corresponding grid cell
         */
        Cell getCellFromPose(const OccGrid::SharedPtr& mapdata,
                            const Pose& pose);

        /**
         * @brief Sends computed path to Nav2 FollowPath action server.
         * @param plan Path to execute
         */
        void sendGoal(const Path& plan);
};