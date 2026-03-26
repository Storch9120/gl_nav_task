## GL: Autonomy Nav Assignment

```bash
export TURTLEBOT3_MODEL=burger

ros2 launch gl_base gl_auto_slam.launch.py

ros2 run nav2_map_server map_saver_cli -f "small_room"
```