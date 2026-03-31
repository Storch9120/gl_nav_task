### Directory Structure
```bash
gl_base/
├── CMakeLists.txt
├── config
│   ├── mapper_params_online_async.yaml
│   └── nav_burger.yaml
├── include
│   └── gl_base
│       └── frontier_detect.hpp
├── launch
│   ├── gl_auto_slam.launch.py
│   ├── gl_room.launch.py
│   └── gl_slam.launch.py
├── package.xml
├── rviz
│   └── gl_slam.rviz
├── src
│   ├── frontier_detect.cpp
│   └── main.cpp
└── worlds
    └── small_room.sdf
gl_navigation/
├── CMakeLists.txt
├── config
│   ├── map_server_params.yaml
│   ├── semantic_map.json
│   ├── small_room_map.pgm
│   ├── small_room_map.png
│   ├── small_room_map.png:Zone.Identifier
│   ├── small_room_map.yaml
│   ├── small_room_serial.data
│   └── small_room_serial.posegraph
├── include
│   └── gl_navigation
│       └── rrt_planner.hpp
├── launch
│   ├── gl_nav.launch.py
│   └── gl_sem.launch.py

├── package.xml
├── rviz
│   ├── gl_nav_main.rviz
│   └── gl_rrt.rviz
├── scripts
│   ├── __init__.py
│   ├── __pycache__
│   │   └── fake_VLM.cpython-312.pyc
│   ├── fake_VLM.py
│   ├── occupancy_grid_publisher.py
│   ├── query_node.py
│   └── tagger_node.py
├── src
│   ├── rrt_node.cpp
│   └── rrt_planner.cpp
└── srv
    └── Query.srv

18 directories, 36 files
```