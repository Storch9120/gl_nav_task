#include <gl_base/frontier_detect.hpp>

FrontierDetect::FrontierDetect() : Node("frontier_detect") {
    map_sub = this->create_subscription<OccGrid>(
        "/map", 10, std::bind(&FrontierDetect::mapCb, this, std::placeholders::_1)
    );

    nav_client = rclcpp_action::create_client<NavToPose>(this, "navigate_to_pose");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // * Initialize timer for calling tick() at a fixed rate
    tick_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000), // 1 Hz
        std::bind(&FrontierDetect::tick, this)
    );

    last_robot_pose = Point();
}

void FrontierDetect::tick() {

    if (rclcpp::ok() && ready_for_next_goal.load()) {

        std::lock_guard<std::mutex> lock(tick_mutex);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[tick] Running");
        if (!map_received.load()){
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[tick] Waiting for 1st map update");
            return;
        }

        OccGrid::SharedPtr latest_map = std::make_shared<OccGrid>(*getMap());

        std::vector<Frontier> frontiers = detectFrontiers(latest_map);

        if (frontiers.empty()) {
            RCLCPP_WARN(this->get_logger(), "[tick] No more frontiers detected; Shutting down...");
            tick_timer_->cancel();
            rclcpp::shutdown();
            return;
        }

        Point best = targetedFrontier(frontiers, latest_map);

        sendGoal(best);
    }
}

void FrontierDetect::mapCb(const OccGrid::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(map_mutex);
    map = msg;
    map_res = msg->info.resolution;
    map_received.store(true);
}

OccGrid::SharedPtr FrontierDetect::getMap() {
    std::lock_guard<std::mutex> lock(map_mutex);
    return map;
}

Point FrontierDetect::getRobotPose(){
    Point rp;
    try{
        auto tf = tf_buffer_->lookupTransform(
            "map", "base_link", tf2::TimePointZero);
        rp.x = tf.transform.translation.x;
        rp.y = tf.transform.translation.y;
        last_robot_pose = rp;
    }
    catch (tf2::TransformException & ex) {
        RCLCPP_ERROR(this->get_logger(), "[getRobotPose] Could not get robot pose: %s", ex.what());
        rp = last_robot_pose;
    }

    return rp;
}

Cell FrontierDetect::getCellFromPose(const Point& point){
    Pose origin = getMap()->info.origin;
    float res = map_res;

    int cell_x = (point.x - origin.position.x)/res;
    int cell_y = (point.y - origin.position.y)/res;

    return std::make_tuple(cell_x, cell_y);
}

std::vector<Cell> FrontierDetect::neighborsWADX(
    const OccGrid::SharedPtr& mapdata,
    const Cell& cell
) {
    int x = std::get<0>(cell);
    int y = std::get<1>(cell);
    int width = mapdata->info.width;
    int height = mapdata->info.height;

    std::vector<Cell> neighbors;
    if (x > 0) neighbors.emplace_back(x-1, y);
    if (x < width - 1) neighbors.emplace_back(x+1, y);
    if (y > 0) neighbors.emplace_back(x, y-1);
    if (y < height - 1) neighbors.emplace_back(x, y+1);
    return neighbors;
}

std::vector<Cell> FrontierDetect::neighborsQWEADZXC(
    const OccGrid::SharedPtr& mapdata,
    const Cell& cell
) {
    int x = std::get<0>(cell);
    int y = std::get<1>(cell);
    int width = mapdata->info.width;
    int height = mapdata->info.height;

    std::vector<Cell> neighbors;
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) continue;
            int nx = x + dx;
            int ny = y + dy;
            if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                neighbors.emplace_back(nx, ny);
            }
        }
    }
    return neighbors;
}

int FrontierDetect::fetchCellData(
    const OccGrid::SharedPtr& mapdata,
    const Cell& cell
) {
    int x = std::get<0>(cell);
    int y = std::get<1>(cell);
    int index = y * mapdata->info.width + x;
    if (index < 0 || index >= static_cast<int>(mapdata->data.size())) {
        return -1; // out of bounds
    }
    return mapdata->data[index];
}

geometry_msgs::msg::Point FrontierDetect::cellPosToMap(
    const OccGrid::SharedPtr& mapdata,
    const Cell& grid_point
) {
    double x = std::get<0>(grid_point) * mapdata->info.resolution + mapdata->info.origin.position.x;
    double y = std::get<1>(grid_point) * mapdata->info.resolution + mapdata->info.origin.position.y;
    geometry_msgs::msg::Point point;
    point.x = x;
    point.y = y;
    point.z = 0.0;
    return point;
}

Frontier FrontierDetect::growFrontier(
    const OccGrid::SharedPtr& mapdata,
    const Cell& initial_cell,
    std::map<Cell, bool>& is_frontier
) {
    int size = 1;
    double centroid_x = std::get<0>(initial_cell);
    double centroid_y = std::get<1>(initial_cell);

    std::queue<Cell> queue;
    queue.push(initial_cell);

    while (!queue.empty()) {
        auto current = queue.front();
        queue.pop();

        for (const auto& neighbor : neighborsQWEADZXC(mapdata, current)) {
            if (checkCellFrontier(mapdata, neighbor, is_frontier)) {
                is_frontier[neighbor] = true;
                size++;
                centroid_x += std::get<0>(neighbor);
                centroid_y += std::get<1>(neighbor);
                queue.push(neighbor);
            }
        }
    }

    centroid_x /= size;
    centroid_y /= size;

    geometry_msgs::msg::Point centroid = cellPosToMap(mapdata, {static_cast<int>(centroid_x), static_cast<int>(centroid_y)});

    return Frontier{size, centroid};
}

bool FrontierDetect::checkCellFrontier(
    const OccGrid::SharedPtr& mapdata,
    const Cell& cell,
    const std::map<Cell, bool>& is_frontier
) {
    if (fetchCellData(mapdata, cell) != -1 || is_frontier.find(cell) != is_frontier.end()) {
        return false;
    }

    for (const auto& neighbor : neighborsWADX(mapdata, cell)) {
        int cell_val = fetchCellData(mapdata, neighbor);
        if (cell_val >= 0 && cell_val < 50) {
            return true;
        }
    }

    return false;
}

std::vector<Frontier> FrontierDetect::detectFrontiers(const OccGrid::SharedPtr& map_ptr){
    std::queue<Cell> queue;
    Cell start = getCellFromPose(getRobotPose());

    queue.push(start);

    std::map<Cell, bool> visited;
    std::map<Cell, bool> is_frontier;
    visited[start] = true;

    std::vector<Frontier> frontiers;
    std::vector<Cell> frontier_cells;


    while (!queue.empty()){
        auto current = queue.front();
        queue.pop();

        for (const auto& neighbor : neighborsWADX(map_ptr, current)){
            // neighbour value is 100 for blocked and 0 for free; -1 for unknown
            if (fetchCellData(map_ptr, neighbor) != -1 && visited.find(neighbor) == visited.end()){
                visited[neighbor] = true;
                queue.push(neighbor);
            } else if (checkCellFrontier(map_ptr, neighbor, is_frontier)){
                is_frontier[neighbor] = true;

                auto new_frontier = growFrontier(
                    map_ptr, neighbor, is_frontier
                );
                // tunable
                if (new_frontier.size >= 10){
                    frontiers.push_back(new_frontier);
                }
            }
        }
    }

    return frontiers;
}

double FrontierDetect::distance2D(const Point& rp, const Point& centroid){
    return std::hypot((rp.x - centroid.x), (rp.y - centroid.y));
}

Point FrontierDetect::targetedFrontier(const std::vector<Frontier>& clusters, const OccGrid::SharedPtr& map_ptr){
    Point selected;
    double min_dist_yet = INFINITY;
    std::vector<int> f_scores;
    double max_score_yet = -INFINITY;
    Point rp = getRobotPose();
    for (const auto& frontier : clusters){
        // * calc distance to the centroid from the robot
        double new_min_dist = distance2D(rp, frontier.centroid);
        double score = frontier.size - 2 *new_min_dist; // tunable; need to normalise this properly

        f_scores.push_back(score);

        if (score > max_score_yet){
            selected = frontier.centroid;
            min_dist_yet = new_min_dist;
            // * validating if the centroid is free or not;
            // * if not we take the nearest free cell
            Cell centroid_cell = getCellFromPose(frontier.centroid);
            if (fetchCellData(map_ptr, centroid_cell) != 0) {
                RCLCPP_WARN(this->get_logger(), "[targetedFrontier] Frontier centroid is not free; Searching for nearest free cell");
                std::queue<Cell> queue;
                queue.push(centroid_cell);

                std::map<Cell, bool> visited;
                visited[centroid_cell] = true;

                bool found_free_cell = false;

                while (!queue.empty() && !found_free_cell) {
                    auto current = queue.front();
                    queue.pop();

                    for (const auto& neighbor : neighborsWADX(map_ptr, current)) {
                        if (visited.find(neighbor) == visited.end()) {
                            visited[neighbor] = true;
                            if (fetchCellData(map_ptr, neighbor) == 0) {
                                selected = cellPosToMap(map_ptr, neighbor);
                                found_free_cell = true;
                                RCLCPP_INFO(this->get_logger(), "[targetedFrontier] Found nearest free cell to frontier centroid at %f, %f", selected.x, selected.y);
                                break;
                            }
                            queue.push(neighbor);
                        }
                    }
                }
            }
            max_score_yet = score;
            RCLCPP_INFO(this->get_logger(), "[targetedFrontier] Selected frontier with score: %f, mindist: %f", score, min_dist_yet);
        }

    }

    if (min_dist_yet < INFINITY){
        RCLCPP_INFO(this->get_logger(), "[targetedFrontier] MinDist: %f Heading to pt %f %f", min_dist_yet, selected.x, selected.y);
    }
    else{
        RCLCPP_ERROR(this->get_logger(), "[targetedFrontier] Error in deciding best frontier; Max score: %f", max_score_yet);
    }
    return selected;
}

void FrontierDetect::sendGoal(const Point& target) {
    if (!nav_client->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "[FrontierDetect::sendGoal] Action server not available after waiting");
        rclcpp::shutdown();
    }

    auto goal_msg = NavToPose::Goal();

    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->get_clock()->now();

    goal_msg.pose.pose.position = target;
    goal_msg.pose.pose.orientation.w = 1.0;
    // if ^ ends up being problematic then can do lse of the frontier
    //  points and take orientation perpendicular to that

    RCLCPP_INFO(this->get_logger(), "[FrontierDetect::sendGoal] Sending goal");

    auto send_goal_options = rclcpp_action::Client<NavToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback = [this](const GoalHandleNav2::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "[FrontierDetect::sendGoal] Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "[FrontierDetect::sendGoal] Goal accepted by server, waiting for result");
            ready_for_next_goal.store(false);
        }
    };

    send_goal_options.feedback_callback = [this](
    GoalHandleNav2::SharedPtr,
    const std::shared_ptr<const NavToPose::Feedback> feedback)
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[FrontierDetect::feedbackCb] Dist pending: %f", feedback->distance_remaining);
    };

    send_goal_options.result_callback = [this](const GoalHandleNav2::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "[FrontierDetect::resultCb] Goal Reached; Fetching next goal");
                ready_for_next_goal.store(true);
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "[FrontierDetect::resultCb] Goal was aborted");
                ready_for_next_goal.store(true);
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "[FrontierDetect::resultCb] Goal was canceled");
                ready_for_next_goal.store(true);
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "[FrontierDetect::resultCb] Unknown result code");
                return;
        }
    };

    nav_client->async_send_goal(goal_msg, send_goal_options);
}

