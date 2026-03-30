#include <gl_navigation/rrt_planner.hpp>

RRTPlanner::RRTPlanner() : Node("rrt_planner") {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    map_sub = this->create_subscription<OccGrid>(
        "/map", 10, std::bind(&RRTPlanner::loadMapCb, this, std::placeholders::_1)
    );
    goal_pose_sub = this->create_subscription<PoseStamped>(
        "rrt_goal_pose", 10, std::bind(&RRTPlanner::goalPoseCb, this, _1)
    );

    plan_pub = this->create_publisher<Path>("rrt_plan", 10);

    nav_client = rclcpp_action::create_client<FollowPath>(this, "follow_path");

    // seeding the random generator
    generator.seed(rd());

    declare_parameter<int>("steer_distance", 5);
    declare_parameter<int>("goal_threshold", 3);
    declare_parameter<int>("rrt_max_iterations", 10000);

    steer_distance = get_parameter("steer_distance").as_int();
    goal_threshold  = get_parameter("goal_threshold").as_int();
    MAX_ITERATIONS = get_parameter("rrt_max_iterations").as_int();


    RCLCPP_INFO(this->get_logger(), "[RRTPlanner] Holding makePlan...");

    // * Planner thread waits on a conditional variable to run makePlan when a new goal is received
    planner_thread = std::thread([this]() {
        while (rclcpp::ok()) {
            std::unique_lock<std::mutex> lock(goal_mutex);
            cv_.wait(lock, [this] { return new_goal_received.load() || !rclcpp::ok(); });

            if (!rclcpp::ok()) {
                break;
            }

            RCLCPP_INFO(this->get_logger(), "[RRTPlanner] makePlan activated");
            if (makePlan()) {
                RCLCPP_INFO(this->get_logger(), "[RRTPlanner] Plan created successfully");
            }
            else{
                RCLCPP_ERROR(this->get_logger(), "[RRTPlanner] makePlan failed. Please give a new goal.");
                new_goal_received.store(false);
            }
        }
    });

}

bool RRTPlanner::makePlan(){
    // * Fetch the map from map_server before planning
    RCLCPP_ERROR(this->get_logger(), "[RRTPlanner::makePlan] 111");
    OccGrid::SharedPtr latest_map;
    try {
        latest_map = std::make_shared<OccGrid>(*getMap());
        if (!latest_map) {
        RCLCPP_ERROR(this->get_logger(), "[RRTPlanner::makePlan] No map available for planning");
        return false;
        }
    } catch(const std::exception& e){
        RCLCPP_ERROR(this->get_logger(), "[RRTPlanner::makePlan] Exception while fetching map: %s", e.what());
        return false;
    }


    Path plan;
    Pose start = getRobotPose();
    Pose goal = goal_pose.pose;
    RCLCPP_ERROR(this->get_logger(), "[RRTPlanner::makePlan] 222");

    if (!findRoute(latest_map, start, goal, plan)) {
        RCLCPP_WARN(this->get_logger(), "[RRTPlanner::makePlan] RRT failed");
        return false;
    }
    RCLCPP_ERROR(this->get_logger(), "[RRTPlanner::makePlan] 333");

    sendGoal(plan);

    return true;
}

void RRTPlanner::goalPoseCb(const PoseStamped::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "[RRTPlanner::goalPoseCb] Received new goal pose");

    {
        std::lock_guard<std::mutex> lock(goal_mutex);
        goal_pose = *msg;
    }

    if (new_goal_received.load() && !goal_reached.load()) {
        RCLCPP_WARN(this->get_logger(), "[RRTPlanner::goalPoseCb] New goal received before previous goal was completed. Sending cancel req");
        // Preempting the last goal by sending a cancel req;
        auto cancel_result_future = nav_client->async_cancel_all_goals();

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), cancel_result_future) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "[RRTPlanner::goalPoseCb] Failed to cancel previous goal");
        }
    }
    else{
        RCLCPP_INFO(this->get_logger(), "[RRTPlanner::goalPoseCb] Ready to plan");
        new_goal_received.store(true);
        goal_reached.store(false);
        // Activate planner thread
        // conditional var to trigger
        cv_.notify_one();
    }
}

void RRTPlanner::loadMapCb(const OccGrid::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(map_mutex);
    map = msg;
    map_res = msg->info.resolution;
    // RCLCPP_INFO(this->get_logger(), "[RRTPlanner::loadMap] Map loaded with resolution %f", map->info.resolution);
}

OccGrid::SharedPtr RRTPlanner::getMap() {
    std::lock_guard<std::mutex> lock(map_mutex);
    return map;
}

Pose RRTPlanner::getRobotPose(){
    auto tf = tf_buffer_->lookupTransform(
        "map", "base_link", tf2::TimePointZero);

    Pose rp;
    rp.position.x = tf.transform.translation.x;
    rp.position.y = tf.transform.translation.y;
    rp.orientation = tf.transform.rotation;

    return rp;
}

Cell RRTPlanner::getCellFromPose(const OccGrid::SharedPtr& mapdata, const Pose& pose){
    Pose origin = mapdata->info.origin;

    int cell_x = (pose.position.x - origin.position.x)/map_res;
    int cell_y = (pose.position.y - origin.position.y)/map_res;

    return std::make_tuple(cell_x, cell_y);
}

Cell RRTPlanner::sampleRandomCell(const OccGrid::SharedPtr& mapdata){
    // limiting sampling within the bounds of the map
    int width = mapdata->info.width;
    int height = mapdata->info.height;

    std::uniform_int_distribution<> dis_w(0, width-1);
    std::uniform_int_distribution<> dis_h(0, height-1);

    int rand_x = dis_w(generator);
    int rand_y = dis_h(generator);

    return std::make_tuple(rand_x, rand_y);
}

int RRTPlanner::findNearestNode(const Cell& cell){
    int nearest_index = 0;
    double min_dist_yet = INFINITY;

    for (size_t i = 0; i < tree.size(); i++) {
        double dist = std::hypot(std::get<0>(tree[i].cell) - std::get<0>(cell), std::get<1>(tree[i].cell) - std::get<1>(cell));
        if (dist < min_dist_yet) {
            min_dist_yet = dist;
            nearest_index = i;
        }
    }
    return nearest_index;
}

Cell RRTPlanner::steer(const Cell& from, const Cell& to){
    // * Steer a fixed distance from 'from' towards 'to'
    double angle = std::atan2(std::get<1>(to) - std::get<1>(from), std::get<0>(to) - std::get<0>(from));

    int new_x = std::get<0>(from) + steer_distance * std::cos(angle);
    int new_y = std::get<1>(from) + steer_distance * std::sin(angle);

    return std::make_tuple(new_x, new_y);
}

bool RRTPlanner::isCollisionFree(const OccGrid::SharedPtr& mapdata, const Cell& from, const Cell& to){
    // * inspired from Bresenham's line algorithm
    int width = mapdata->info.width;
    int height = mapdata->info.height;

    int x0 = std::get<0>(from);
    int y0 = std::get<1>(from);
    int x1 = std::get<0>(to);
    int y1 = std::get<1>(to);

    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    // step direction decision
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    while (true) {
        // Check if current cell is within map bounds
        if (x0 < 0 || x0 >= width || y0 < 0 || y0 >= height) {
            return false; // Out of bounds
        }

        // Check if current cell is occupied
        int index = y0 * width + x0;
        if (mapdata->data[index] > 50) {
            return false; // Collision detected
        }

        if (x0 == x1 && y0 == y1) {
            break; // Reached the final cell
        }

        int err2 = err * 2;
        if (err2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (err2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
    return true; // No collision along the path
}

bool RRTPlanner::isGoalReached(const Cell& cell, const Cell& c_goal){
    double dist = std::hypot(std::get<0>(cell) - std::get<0>(c_goal), std::get<1>(cell) - std::get<1>(c_goal));
    return dist < goal_threshold; // tunable threshold in cells
}

void RRTPlanner::extractPath(const OccGrid::SharedPtr& mapdata, Path& plan){
    // * Backtrack from the goal node to the start node using parent indices to extract the path
    std::vector<geometry_msgs::msg::PoseStamped> poses;

    int current_index = tree.size() - 1; // goal node index
    while (current_index != -1) {
        Cell cell = tree[current_index].cell;
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.pose.position.x = std::get<0>(cell) * map_res + mapdata->info.origin.position.x;
        pose_stamped.pose.position.y = std::get<1>(cell) * map_res + mapdata->info.origin.position.y;
        pose_stamped.pose.orientation.w = 1.0; // better if i assign from dir of movement
        poses.push_back(pose_stamped);
        current_index = tree[current_index].parent_index;
    }

    // need to reverse from goal to start, so its start to goal for the controller
    std::reverse(poses.begin(), poses.end());
    plan.poses = poses;
}

bool RRTPlanner::findRoute(const OccGrid::SharedPtr& mapdata, const Pose& start, const Pose& goal, Path& plan){
    // * RRT implementation
    RCLCPP_INFO(this->get_logger(), "[RRTPlanner::findRoute] Finding route from (%f, %f) to (%f, %f)", start.position.x, start.position.y, goal.position.x, goal.position.y);

    // tree data structure to hold the RRT nodes

    Cell c_start = getCellFromPose(mapdata, start);
    Cell c_goal = getCellFromPose(mapdata, goal);

    tree.clear();
    tree.push_back(TNode{c_start, -1}); // root node has no parent


    for (int i = 0; i < MAX_ITERATIONS; i++){
        // * Sample random point in the map
        Cell rand_cell = sampleRandomCell(mapdata);

        // * Find nearest node in the tree to the random cell
        int nearest_index = findNearestNode(rand_cell);

        // * Steer from nearest node towards random cell, creating a new node
        Cell new_cell = steer(tree[nearest_index].cell, rand_cell);

        // * Check if the path from nearest node to new node is collision free
        if (isCollisionFree(mapdata, tree[nearest_index].cell, new_cell)) {
            // ! New node appended here
            tree.push_back({new_cell, nearest_index});

            // * Check if new node is close enough to goal to consider it reached
            if (isGoalReached(new_cell, c_goal)) {
                RCLCPP_INFO(this->get_logger(), "[RRTPlanner::findRoute] Goal found in %d iterations", i);
                extractPath(mapdata, plan);
                return true;
            }
        }
    }

    RCLCPP_WARN(this->get_logger(), "[RRTPlanner::findRoute] Failed to find a path to the goal after max iterations");
    return false; // failed to find a path
}

void RRTPlanner::sendGoal(const Path& plan){
    if (!nav_client->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "[RRTPlanner::sendGoal] Action server not available after waiting");
        rclcpp::shutdown();
    }

    auto goal_msg = FollowPath::Goal();

    goal_msg.path.header.frame_id = "map";
    goal_msg.path.header.stamp = this->get_clock()->now();

    goal_msg.path.poses = plan.poses;

    plan_pub->publish(goal_msg.path);

    RCLCPP_INFO(this->get_logger(), "[RRTPlanner::sendGoal] Sending goal");

    auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
    send_goal_options.goal_response_callback = [this](const GoalHandleNav2::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "[RRTPlanner::sendGoal] Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "[RRTPlanner::sendGoal] Goal accepted by server, waiting for result");
        }
    };

    send_goal_options.feedback_callback = [this](
    GoalHandleNav2::SharedPtr,
    const std::shared_ptr<const FollowPath::Feedback> feedback)
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[RRTPlanner::feedbackCb] Dist pending: %f", feedback->distance_to_goal);
    };

    send_goal_options.result_callback = [this](const GoalHandleNav2::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "[RRTPlanner::resultCb] Goal Reached; Awaiting next path...");
                goal_reached.store(true);
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "[RRTPlanner::resultCb] Goal was aborted");
                goal_reached.store(true);
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "[RRTPlanner::resultCb] Goal was canceled");
                goal_reached.store(true);
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "[RRTPlanner::resultCb] Unknown result code");
                return;
        }
    };

    nav_client->async_send_goal(goal_msg, send_goal_options);
    new_goal_received.store(false);
}