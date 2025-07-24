# pi0_franka_ros2

ROS2 integration of Pi-Zero (π0) vision-language-action models with Franka Emika Panda for autonomous manipulation.

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-brightgreen)](https://docs.ros.org/en/humble/)

## Features

- **Simulation-first**: Complete Gazebo environment with no hardware required
- **Pi-Zero integration**: Vision-language-action model for natural language control  
- **Random scene generation**: Automatic object spawning for training/testing
- **Real robot support**: Direct Franka hardware integration
- **Extensible**: Designed for multiple VLA models (OpenVLA planned)

## Installation

### Prerequisites
- Ubuntu 22.04 
- ROS2 Humble

### Build from Source

```bash
# Create workspace
mkdir -p ~/ws_pi0/src
cd ~/ws_pi0

# Import repositories  
vcs import src < src/pi0_franka_ros2/pi0_franka.repos

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build

# Source
source install/setup.bash
```

## Quick Start

### Simulation
```bash
# Launch simulation environment
ros2 launch pi0_franka_ros2 simulation.launch.py

# Send commands
ros2 topic pub /text_command std_msgs/String "data: 'Pick up the red cube'"

# Spawn objects
ros2 service call /spawn_objects pi0_franka_msgs/SpawnObjects "{num_objects: 3}"
```

### Real Robot
```bash
# Launch real robot (replace IP)
ros2 launch pi0_franka_ros2 real_robot.launch.py robot_ip:=192.168.1.100
```

## Package Structure

```
pi0_franka_ros2/
├── package.xml                    # ROS2 dependencies
├── CMakeLists.txt
├── pi0_franka.repos               # Source dependencies
├── requirements.txt               # Non-ROS Python packages
├── launch/
│   ├── simulation.launch.py       # Complete simulation
│   └── real_robot.launch.py       # Real hardware
├── config/
│   ├── robot_config.yaml
│   └── camera_config.yaml  
├── src/pi0_franka_ros2/
│   ├── pi0_controller.py          # Main VLA node
│   ├── franka_interface.py        # Robot control
│   ├── sensor_processor.py        # Camera processing
│   └── object_spawner.py          # Scene generation
└── worlds/
    └── manipulation_table.world
```

## Testing

```bash
# Build and test
colcon build --packages-select pi0_franka_ros2
colcon test --packages-select pi0_franka_ros2

# Integration test
ros2 launch pi0_franka_ros2 simulation.launch.py
# In another terminal:
python -m pytest src/pi0_franka_ros2/tests/
```

## Development

### Adding Dependencies

**ROS2 packages:** Add to `package.xml`, run `rosdep install`

**Non ROS2 Python packages:** Add to `requirements.txt`, run `pip install -r requirements.txt`

### Code Quality
```bash
black src/ && flake8 src/
```

## Contributing

1. Fork and create feature branch
2. Follow ROS2 conventions  
3. Add tests for new features
4. Submit PR

## License

Mine for now 

---

For detailed documentation, tutorials, and advanced usage, visit the [wiki](https://github.com/your-username/pi0_franka_ros2/wiki).