/**
 * @file frontier_detect.hpp
 * @brief Frontier exploration node for ROS2 with Nav2 integration.
 *
 * This node subscribes to an occupancy grid, identifies frontiers in the
 * explored map, clusters them, selects a target frontier, and sends NavigateToPose
 * goals to Nav2. It uses wavefront expansion and centroid selection to choose
 * next exploration targets.
 */


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

/**
 * @struct Frontier
 * @brief Represents a frontier region discovered in the occupancy map.
 */
struct Frontier {
    int size;        ///< Number of frontier cells in this region.
    Point centroid;  ///< Map coordinate estimate of the frontier cluster centroid.
};


/**
 * @class FrontierDetect
 * @brief ROS2 node that detects frontier regions from occupancy grid and commands Nav2.
 *
 * The node follows a periodic `tick()` cycle:
 * 1. Reads latest map update
 * 2. Computes frontier cells (unknown/free boundary)
 * 3. Grows clusters and computes centroids
 * 4. Selects best frontier target using size/distance heuristic
 * 5. Sends Nav2 NavigateToPose goal
 * 6. Waits for completion to proceed
 */
class FrontierDetect : public rclcpp::Node {
    public:
        FrontierDetect();

        /**
         * @brief Main exploration loop called by timer.
         *
         * Handles map availability, frontier detection, best frontier selection,
         * and dispatching navigation goals until no frontiers remain.
         */
        void tick();

    private:
        std::atomic<bool> ready_for_next_goal{true}, map_received{false};
        std::mutex map_mutex, tick_mutex;

        // * ROS Vars
        OccGrid::SharedPtr map;               ///< last received occupancy grid
        float map_res;                        ///< map resolution (m/cell)
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        Point last_robot_pose;                ///< cached robot pose on TF failure

        // * ROS Pub, Sub, Clients
        rclcpp::Subscription<OccGrid>::SharedPtr map_sub;
        rclcpp_action::Client<NavToPose>::SharedPtr nav_client;
        rclcpp::TimerBase::SharedPtr tick_timer_;

        // * Callbacks
        /**
         * @brief Map topic callback updates current occupancy grid state.
         * @param msg Shared pointer to received occupancy grid.
         */
        void mapCb(const OccGrid::SharedPtr msg);

        // * Accessors
        /**
         * @brief Thread-safe getter for latest map state.
         * @return Shared pointer to current occupancy grid.
         */
        OccGrid::SharedPtr getMap();

        // * Core functions
        /**
         * @brief Detects frontier regions in the global occupancy grid.
         * @param map Shared pointer to the occupancy grid to evaluate.
         * @return Vector of discovered frontier clusters.
         */
        std::vector<Frontier> detectFrontiers(const OccGrid::SharedPtr& map);

        /**
         * @brief Selects the best frontier based on size and distance scoring.
         * @param clusters Vector of candidate frontiers.
         * @param map_ptr Shared pointer to occupancy grid for validity checks.
         * @return Selected target point in map frame.
         */
        Point targetedFrontier(const std::vector<Frontier>& clusters, const OccGrid::SharedPtr& map_ptr);

        // * Helper functions
        /**
         * @brief Obtains current robot pose from TF.
         * @return Robot pose in map frame.
         */
        Point getRobotPose();

        /**
         * @brief Converts world point into map cell coordinates.
         * @param point Point in map frame.
         * @return Tuple (x, y) cell index.
         */
        Cell getCellFromPose(const Point& point);

        /**
         * @brief Euclidean distance between two points in 2D.
         * @param rp Robot point.
         * @param centroid Target point.
         * @return Distance in meters.
         */
        double distance2D(const Point& rp, const Point& centroid);

        /**
         * @brief 4-connected neighbor iterator.
         * @param mapdata Occupancy grid.
         * @param cell Query cell index.
         * @return Adjacent 4-connected cells.
         */
        static std::vector<Cell> neighborsWADX(
            const OccGrid::SharedPtr& mapdata,
            const Cell& cell
        );

        /**
         * @brief 8-connected neighbor iterator.
         */
        static std::vector<Cell> neighborsQWEADZXC(
            const OccGrid::SharedPtr& mapdata,
            const Cell& cell
        );

        /**
         * @brief Safely fetches occupancy value for a cell.
         * @param mapdata Occupancy grid.
         * @param cell Cell indices.
         * @return Occupancy value (-1 unknown, 0 free, 100 occupied)
         */
        static int fetchCellData(
            const OccGrid::SharedPtr& mapdata,
            const Cell& cell
        );

        /**
         * @brief Converts grid cell indices to map frame coordinates.
         */
        static geometry_msgs::msg::Point cellPosToMap(
            const OccGrid::SharedPtr& mapdata,
            const Cell& grid_point
        );

        /**
         * @brief Grow a frontier cluster from an initial frontier cell.
         * @param mapdata Occupancy grid.
         * @param initial_cell Seed cell.
         * @param is_frontier Map of frontier cells discovered so far.
         * @return Frontier struct with cluster size and centroid.
         */
        static Frontier growFrontier(
            const OccGrid::SharedPtr& mapdata,
            const Cell& initial_cell,
            std::map<Cell, bool>& is_frontier
        );

        /**
         * @brief Determines whether a candidate cell belongs to a frontier boundary.
         */
        static bool checkCellFrontier(
            const OccGrid::SharedPtr& mapdata,
            const Cell& cell,
            const std::map<Cell, bool>& is_frontier
        );

        /**
         * @brief Dispatches a NavigateToPose goal to Nav2 for the selected target point.
         * @param target Target position in map frame.
         */
        void sendGoal(const Point& target);

};