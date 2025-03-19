# Kinova Arm and TurtleBot Integration

This package integrates the Kinova Gen3 Lite arm with a TurtleBot 3 to perform a coordinated pick-and-place  and transport task. The TurtleBot moves forward, the Kinova arm picks up an object and places it in a basket on the TurtleBot, and then the TurtleBot returns to its starting position.

## Prerequisites

- ROS 2 (Humble)
- TurtleBot 3 with ROS 2 packages installed
- Kinova Gen3 Lite arm with ROS 2 packages installed
- Python 3.8+

## Package Structure

```
turtlebot_kinova_transport/
├── package.xml
├── turtlebot_kinova_transport/
│   ├── __init__.py
│   ├── turtlebot_kinova_transport.py     # Main script for integration
│   ├── gen3lite_pymoveit2.py        # Kinova arm control module
│   └── data_final_demo.csv          # Pose data for pick and place
├── launch/
│   └── turtlebot_kinova.launch.py    # Launch file for the integrated task
└── README.md
```

## Installation

1. Clone this repository into your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/Po-Jui-Lin/turtlebot_kinova_transport.git
```

2. Install dependencies:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the package:

```bash
cd ~/ros2_ws
colcon build --packages-select turtlebot_kinova_transport
source install/setup.bash
```

## Usage

### Preparation

1. Position the TurtleBot and the Kinova arm appropriately:
   - The TurtleBot should be at the starting position
   - The Kinova arm should be near the object to be picked up and is fixed to the table
   - Ensure the path is clear for the TurtleBot to move 3.16 meters forward

2. Place the object at the location specified in the CSV file

### Running the Task

1. Launch the integrated task:

```bash
ros2 launch turtlebot_kinova_transport turtlebot_kinova_transport.launch.py
```

2. The task will execute automatically:
   - TurtleBot will move forward 3.16 meters
   - Kinova arm will pick up the object and place it in the basket
   - TurtleBot will return to the starting position

3. To stop the task at any time:

```bash
Ctrl+C
```

## Customization

### Modifying Pick and Place Positions

The pick and place positions are defined in the `data_final_demo.csv` file. The file contains the following pose definitions:

- `pick_position`: Position where the gripper picks up the object
- `mid_position`: Intermediate position for obstacle avoidance
- `place_position`: Position where the gripper places the object
- `approach_position` (optional): Position before approaching the pick position
- `retreat_position` (optional): Position after releasing the object

You can modify these positions by editing the CSV file.

### Changing the TurtleBot Movement Distance

By default, the TurtleBot moves 3.16 meters forward and then backward. To change this distance, modify the `move_linear()` distance parameter in the `turtlebot_kinova_transport.py` file.

## Troubleshooting

### Communication Issues

- Ensure both the TurtleBot and Kinova arm are on the same network
- Verify that ROS_DOMAIN_ID is set consistently on all machines
- Check that firewall settings allow ROS 2 communication

### Movement Issues

- If the TurtleBot doesn't move the correct distance, adjust the speed parameter in the `move_linear()` function
- If the Kinova arm fails to pick up the object, check the CSV file for correct pose information

## Contributing

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add some amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request