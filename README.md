# GL: Autonomy Nav Assignment

### Target Platform
* ROS2 Jazzy Jalisco
* Turtlebot3 Burger Simulation
* Gazebo Harmonic

### Setup

```bash
# Additional packages:
sudo apt-get -y install ros-jazzy-dynamixel-sdk ros-jazzy-tf2-geometry-msgs

# Create the ROS ws
cd ~
mkdir -p gl_ws/src
cd ~/gl_ws/src

# Clone the repository and its submods
git clone --recurse-submodules https://github.com/Storch9120/gl_nav_task.git .

# Build the packages
cd ~/gl_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Section 1: Exploration and SLAM

This section entails creation of node that accesses the online SLAM map data and automatically decides the next area to explore by detecting frontiers; selecting the best one based on distance and size and finally, navigating to it.
Current implemented method of frontier detection is more commonly known as Wavefront Frontier Detection. Plainly put,
- It detects frontier points
- Clusters the points using BFS
- Computes centroid for each cluster
Selection of best frontier is being made using 
- nearest distance (Fastest to reach)
- max cluster size (Explore big unknown regions first)
Score is computed using these two params.
Finally, a navigation goal is computed and sent to the action server, `navigate_to_pose`.

Also, since yaw alignment is not necessary here,
the goal_checker plugin was swapped to PositionGoalChecker.

see [nav_burger.yaml](gl_base/config/nav_burger.yaml#L115-119)
```yaml
...
controller_server:
    ...
    goal_checker:
      stateful: true
      plugin: "nav2_controller::PositionGoalChecker" #SimpleGoalChecker
      xy_goal_tolerance: 0.25
    ...
```
### Demo Video:
TODO

### Launch commands:
```bash
export TURTLEBOT3_MODEL=burger
```
> [!TIP]
> Add this ^ line to your ~/.bashrc file to skip putting this command 
```bash
ros2 launch gl_base gl_auto_slam.launch.py
ros2 run nav2_map_server map_saver_cli -f "blah_blah"
```

### Future Scope:
- Swap WFD with NaiveAA i guess

---

## Section 2:  Agentic Semantic Reasoning

In this section, robot is given few goals in the map to explore. The `tagger_node` tags these goals based on robot pose and creates a semantic map. This map consisting of label dicts with keys pose and embedded vector of the label text. This semantic map is read by `query_node` to handle text queries via service call `/send_query` and navigate to the queried label.

### Demo Video:
TODO

### Launch commands:
 
```bash
export TURTLEBOT3_MODEL=burger
```
> [!NOTE]
> Export model as `burger_cam` so we can use the burger model that comes with a camera. This image topic can be potentially fed to CLIP or some one shot recognition model to identify landmarks

```bash
ros2 launch gl_navigation gl_sem.launch.py

# To run the Tagger Node
ros2 run gl_navigation tagger_node.py

# To run the Query Node
ros2 run gl_navigation query_node.py
```

### Instructions:

1. Localise the bot using `2D Initial Pose` tool in rviz.
2. Use the `2D Goal Pose` tool to explore the map and populate the semantic map json using tagger_node.
3. Once the tagging process is complete, run the query_node.
4. Call the service `/send_query` and pop a query in the field.
```bash
ros2 service call /send_query gl_navigation/srv/Query "{query: 'go to the bathroom'}"
```

### Design Decisions:

- Decoupled semantic perception (tagger) from query reasoning (query node)
- Stored semantic map as JSON for simplicity and inspectability

### Future Scope:
- Use entire word embeddings instead of trivial character matching for robustness to synonyms
- Add in OpenAI embedding to replace current mock embedding using a fast, cheap and small model
```python
from openai import OpenAI

client = OpenAI()

def get_embedding(text: str) -> list[float]:
    response = client.embeddings.create(
        model="text-embedding-3-small",
        input=text
    )
    return response.data[0].embedding
```
- Run the tagger node in continuous/online mode and regularly update and expand the regions dict. Along with this, query_node needs to regularly reload `semantic_map.json`
- For my specific implementation, can maybe add higher weights to the colors when embedding.
- Use images from the camera, detect landmarks; get a label for the landmarks; embed the label and so on. This way, send_query can be expanded to accept visual input as well. For example, sending a pic of a toilet sends the bot to the bathroom.

---

## Section 3: Planner Task

Occupancy grid publisher node processes an image and gives an occupancy grid type map but not sure about the point of publishing from it as well. The job is redundant as nav2 already has its own map_server; if we want to force usage of our publisher then we must send an deactivate() lifecycle event to map_server then start publishing to /map topic.

### Demo Video:
TODO

### Launch commands:
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch gl_navigation gl_nav.launch.py
```

### Instructions:

1. Localise the bot using `2D Initial Pose` tool in rviz.
2. Use the `2D Goal Pose` tool.
3. Mark the arrow to send a goal pose to the RRT Planner Node.

### Future Scope:
- Upgrade RRT to RRT*
- OccGrid map creater could use more complex floor plan images which could be treated with some erosion dilation to keep the major structural parts intact while omitting the minor labellings, etc. Then pass through the rest of the existing pipeline to get the occ grid directly.