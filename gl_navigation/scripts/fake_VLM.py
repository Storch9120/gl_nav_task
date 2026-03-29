#!/usr/bin/env python3

# xmin xmax ymin ymax 
MOCK_REGIONS = [
    ((-1.7, 1.9, -2.4, 0.5), 'red_shelf'),
    ((-1.7, 1.9, 0.5, 3.45), 'green_shelf'),
    ((1.9, 5.7, 0.5, 3.45), 'blue_shelf'),
    ((1.9, 5.7, -2.4, 0.5), 'teal_shelf'),
]

""" Return a label based on robot position in the simulated world."""
def mock_label_image(robot_x: float, robot_y: float) -> str | None:
    for (xmin, xmax, ymin, ymax), label in MOCK_REGIONS:
        if xmin <= robot_x <= xmax and ymin <= robot_y <= ymax:
            return label
    return None