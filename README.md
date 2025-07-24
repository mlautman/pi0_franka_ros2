# pi0_franka_ros2

A ROS2 package for integrating Pi-Zero (π0) vision-language-action models with the Franka Emika Panda robotic arm for autonomous manipulation tasks.

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-brightgreen)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.8%2B-blue)](https://www.python.org/)

## Overview

This package provides a complete integration between Pi-Zero (π0) vision-language-action models and the Franka Emika Panda arm in ROS2. The system can understand natural language commands, process visual sensor input, and execute precise manipulation tasks autonomously.

**🎯 Simulation-First Design**: Run everything in simulation without any physical hardware required!

### Key Features

- **🚀 Complete Simulation Environment**: Full Gazebo simulation with physics, cameras, and interactive objects
- **🎲 Randomized Scene Generation**: Automatically spawns objects with varying shapes, colors, and positions
- **🤖 Virtual Robot & Sensors**: Simulated Franka Panda arm with RGB-D camera feeds
- **💬 Multi-modal Input Processing**: Combines simulated camera feeds, text commands, and robot state
- **🔧 Real Robot Support**: Direct integration with Franka ROS2 drivers when hardware is available
- **🔌 Extensible Architecture**: Designed for easy integration with other VLA models (OpenVLA support planned)
- **📖 Open Source**: Apache 2.0 licensed for community development

### 🎮 No Hardware Required
Get started immediately with our complete simulation environment - no robot, cameras, or special hardware needed!

## System Architecture

### 🎮 Simulation Mode (Default - No Hardware Required)
```
┌─────────────────────────────────────────────────────────────────┐
│                    Gazebo Simulation Environment                │
│  ┌─────────────┐  ┌──────────────┐  ┌─────────────────────────┐ │
│  │ Virtual     │  │ Simulated    │  │    Random Object        │ │
│  │ RGB-D       │  │ Franka Panda │  │    Spawner              │ │
│  │ Camera      │  │ Robot        │  │ (Cubes, Spheres, etc.)  │ │
│  └─────────┬───┘  └──────┬───────┘  └─────────────────────────┘ │
└───────────┼─────────────┼─────────────────────────────────────────┘
            │             │
┌───────────▼─────────────▼───────────┐    ┌──────────────────┐
│     Sensor Data & Robot State       │    │  Text Commands   │
│        (Simulated)                  │    │   (User Input)   │
└───────────┬─────────────────────────┘    └─────────┬────────┘
            │                                        │
            └────────────────┬───────────────────────┘
                             │
                ┌────────────▼─────────────┐
                │     Pi-Zero Model        │
                │  (Vision-Language-Action)│
                └────────────┬─────────────┘
                             │
                ┌────────────▼─────────────┐
                │    Action Processor      │
                │  (Trajectory Planning)   │
                └────────────┬─────────────┘
                             │
                ┌────────────▼─────────────┐
                │  Simulated Robot Control │
                │    (Gazebo Physics)      │
                └──────────────────────────┘
```

### 🤖 Real Robot Mode (Optional Hardware)
```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│  Real Camera    │    │  Text Commands   │    │ Real Robot      │
│     Feed        │    │   (User Input)   │    │     State       │
└─────────┬───────┘    └─────────┬────────┘    └─────────┬───────┘
          │                      │                       │
          └──────────────────────┼───────────────────────┘
                                 │
                    ┌────────────▼─────────────┐
                    │     Pi-Zero Model        │
                    │  (Vision-Language-Action)│
                    └────────────┬─────────────┘
                                 │
                    ┌────────────▼─────────────┐
                    │    Action Processor      │
                    │  (Trajectory Planning)   │
                    └────────────┬─────────────┘
                                 │
                    ┌────────────▼─────────────┐
                    │   Real Franka Control    │
                    │   (Physical Hardware)    │
                    └──────────────────────────┘
```

## Prerequisites

### System Requirements
- Ubuntu 22.04 LTS
- ROS2 Humble Hawksbill
- Python 3.8+
- CUDA-capable GPU (recommended)

### Hardware Requirements

#### 🎮 Simulation Mode (Recommended for Development)
- **No special hardware required!**
- Standard desktop/laptop with:
  - Ubuntu 22.04 LTS
  - 8GB+ RAM
  - Dedicated GPU recommended (for Pi-Zero inference)
  - OpenGL-capable graphics card (for Gazebo rendering)

#### 🤖 Real Robot Mode (Optional)
- Franka Emika Panda robot arm
- RGB-D camera (Intel RealSense D435/D455 recommended)
- Workstation with sufficient compute for real-time VLA inference
- Robot network connection and safety setup

## 🐳 Docker Development (Recommended)

#### 🚀 Daily Development Workflow
#### 1. Launch Complete Simulation Environment
# Activate your workspace (any time you open a new terminal)
cd ~/pi0_franka_ws
source scripts/activate_workspace.sh  # Sources ROS2, venv, and workspace

# Run simulation
ros2 launch pi0_franka_ros2 simulation.launch.py

# Run tests
./scripts/run_tests.sh

# Update dependencies (when requirements.txt changes)
pip install -r requirements.txt
```

### 🧪 Testing and CI/CD

#### Local Testing
```bash
# Run full test suite
./scripts/run_tests.sh

# Run specific test categories
pytest tests/unit/           # Unit tests
pytest tests/integration/    # Integration tests
pytest tests/simulation/     # Simulation-specific tests

# Run with coverage
pytest --cov=pi0_franka_ros2 tests/

# Linting and formatting
black src/ tests/            # Code formatting
flake8 src/ tests/           # Linting
mypy src/                    # Type checking
```

#### Docker Testing
```bash
# Test in clean Docker environment
docker build -f docker/Dockerfile.ci -t pi0_franka:test .
docker run --rm pi0_franka:test

# Test GPU functionality
docker run --gpus all --rm pi0_franka:gpu python -c "import torch; print(torch.cuda.is_available())"
```

#### GitHub Actions CI/CD
The repository includes automated testing workflows:

- **🔄 Continuous Integration**: Tests on every PR and push
  - Unit tests across Python 3.8, 3.9, 3.10
  - Integration tests in simulation
  - Code quality checks (black, flake8, mypy)
  - ROS2 build validation

- **🌙 Nightly Docker Builds**: Automatic Docker image builds
  - Base image (CPU-only)
  - GPU-enabled image
  - Multi-architecture support (x86_64, arm64)
  - Automatic pushes to Docker Hub

- **🧪 Test Matrix**: Multi-platform testing
  - Ubuntu 22.04 LTS
  - Different ROS2 versions
  - With/without GPU
  - Real robot integration tests (when available) with Docker
#### 1. Install ROS2 Humble
# Clone repository
git clone https://github.com/your-username/pi0_franka_ros2.git
cd pi0_franka_ros2

# Build and run with Docker Compose (includes GPU support if available)
docker-compose up --build

# Or run specific services
docker-compose up simulation  # Just simulation environment
docker-compose up pi0-dev     # Development environment with mounted code
```

### Manual Docker Build
```bash
# Build base image (CPU-only)
docker build -f docker/Dockerfile.base -t pi0_franka:base .

# Build GPU-enabled image
docker build -f docker/Dockerfile.gpu -t pi0_franka:gpu .

# Run interactive development container
docker run -it --rm \
  --gpus all \
  -v $(pwd):/workspace \
  -p 8080:8080 \
  pi0_franka:gpu bash
```

## 🛠️ Local Development Setup

### 1. Automated Setup (Recommended)
```bash
git clone https://github.com/your-username/pi0_franka_ros2.git
cd pi0_franka_ros2
./scripts/setup_environment.sh
```

### 2. Manual Setup
```bash
```bash
# Follow official ROS2 installation guide
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions
```

#### 2. Install Franka ROS2 Dependencies
```bash
sudo apt install ros-humble-franka-* ros-humble-moveit* ros-humble-gazebo-*
```

#### 3. Create Workspace and Clone Repository
```bash
mkdir -p ~/pi0_franka_ws/src
cd ~/pi0_franka_ws/src
git clone https://github.com/your-username/pi0_franka_ros2.git
cd ..
```

### 4. Setup Python Environment and Dependencies
```bash
# Create and activate Python virtual environment
python3 -m venv ~/pi0_franka_ws/venv
source ~/pi0_franka_ws/venv/bin/activate

# Install Python dependencies
cd ~/pi0_franka_ws/src/pi0_franka_ros2
pip install -r requirements.txt
pip install -e .  # Install package in development mode
```

### 5. Build the ROS2 Package
```bash
cd ~/pi0_franka_ws
source /opt/ros/humble/setup.bash
source venv/bin/activate  # Always source venv before building
colcon build --packages-select pi0_franka_ros2
source install/setup.bash
```

## Quick Start

## 🎮 Quick Start

### 🎮 Simulation Mode (Start Here!)
```bash
# Terminal 1: Launch everything at once (robot, world, cameras, objects)
ros2 launch pi0_franka_ros2 simulation.launch.py

# This automatically starts:
# - Gazebo with Panda robot
# - Simulated RGB-D camera
# - Manipulation table with bin
# - Random object spawning
# - Pi-Zero integration node
```

#### 2. Interact with the System
```bash
# Send text commands to the robot
ros2 topic pub /text_command std_msgs/String "data: 'Pick up the red cube and place it in the blue bin'"

# Generate new random objects
ros2 service call /spawn_objects pi0_franka_msgs/SpawnObjects "{num_objects: 5}"

# Reset the scene
ros2 service call /reset_scene std_srvs/Empty
```

#### 3. Monitor the System
```bash
# Watch camera feed
ros2 run rqt_image_view rqt_image_view /camera/image_raw

# View robot state
rviz2 -d $(ros2 pkg prefix pi0_franka_ros2)/share/pi0_franka_ros2/rviz/pi0_franka.rviz

# Monitor all topics
ros2 topic list
```

### 🤖 Real Robot Mode (Advanced)

#### 1. Setup Real Hardware
```bash
# Configure Franka network (replace with your robot IP)
export FRANKA_ROBOT_IP=192.168.1.100

# Launch real robot interface
ros2 launch pi0_franka_ros2 real_robot.launch.py robot_ip:=$FRANKA_ROBOT_IP
```

#### 2. Connect Camera
```bash
# Launch RealSense camera (if using Intel RealSense)
ros2 launch realsense2_camera rs_launch.py

# Or launch your camera driver and remap topics
```

#### 3. Start Pi-Zero Controller
```bash
ros2 run pi0_franka_ros2 pi0_controller --ros-args -p use_simulation:=false
```

## Package Structure

```
pi0_franka_ros2/
├── CMakeLists.txt
├── package.xml
├── README.md
├── setup.py                       # Python package setup
├── setup.cfg                      # Python package configuration
├── requirements.txt               # 🐍 Core Python dependencies
├── requirements-dev.txt           # 🛠️ Development dependencies
├── requirements-gpu.txt           # 🚀 GPU-specific dependencies (CUDA)
├── pyproject.toml                 # Modern Python project configuration
├── Dockerfile                     # 🐳 Docker container setup
├── docker-compose.yml             # 🐳 Multi-service Docker setup
├── .dockerignore                  # Docker ignore patterns
├── .github/
│   └── workflows/
│       ├── ci.yml                 # 🔄 Continuous integration
│       ├── docker-nightly.yml     # 🌙 Nightly Docker builds
│       └── test-matrix.yml        # 🧪 Multi-platform testing
├── launch/
│   ├── simulation.launch.py       # 🎮 Complete simulation setup (DEFAULT)
│   ├── real_robot.launch.py       # 🤖 Real robot configuration
│   ├── gazebo_world.launch.py     # Gazebo world and physics
│   ├── spawn_robot.launch.py      # Robot spawning in simulation
│   └── cameras.launch.py          # Camera setup (sim/real)
├── config/
│   ├── robot_config.yaml          # Robot parameters
│   ├── camera_config.yaml         # Camera calibration
│   └── pi0_config.yaml            # Model configuration
├── src/pi0_franka_ros2/
│   ├── __init__.py
│   ├── pi0_controller.py          # Main VLA integration node
│   ├── franka_interface.py        # Robot control (sim + real)
│   ├── sensor_processor.py        # Camera processing (sim + real)
│   ├── object_spawner.py          # 🎲 Simulation object generation
│   ├── action_processor.py        # Action space conversion
│   ├── simulation_manager.py      # 🎮 Gazebo interaction utilities
│   └── utils/
│       ├── __init__.py
│       ├── transforms.py          # Coordinate transformations
│       ├── vision_utils.py        # Computer vision utilities
│       └── model_loader.py        # Pi-Zero model management
├── worlds/
│   ├── manipulation_table.world   # 🎮 Main simulation world
│   ├── empty_world.world          # Minimal testing environment
│   └── objects/                   # Object mesh files and materials
├── urdf/
│   ├── table_setup.urdf           # Table and bin descriptions
│   ├── objects.urdf               # Spawnable object definitions
│   └── camera_mount.urdf          # Camera positioning
├── rviz/
│   └── pi0_franka.rviz           # RViz configuration
├── scripts/
│   ├── setup_environment.sh       # 🛠️ Complete environment setup
│   ├── activate_workspace.sh      # Quick workspace activation
│   ├── download_models.sh         # Model download script
│   ├── docker_build.sh           # 🐳 Docker build helper
│   └── run_tests.sh              # 🧪 Test execution script
├── docker/
│   ├── Dockerfile.base           # Base ROS2 + dependencies
│   ├── Dockerfile.gpu            # GPU-enabled version
│   ├── Dockerfile.ci             # CI/testing specific
│   └── entrypoint.sh             # Docker container entrypoint
└── tests/
    ├── test_pi0_controller.py
    ├── test_franka_interface.py
    └── test_integration.py
```

## ROS2 Interfaces

### Topics

#### 🎮 Simulation Topics
- `/camera/image_raw` (sensor_msgs/Image): Simulated RGB camera feed
- `/camera/depth/image_raw` (sensor_msgs/Image): Simulated depth camera feed
- `/camera/camera_info` (sensor_msgs/CameraInfo): Simulated camera parameters

#### 🤖 Robot Topics (Sim + Real)
- `/joint_states` (sensor_msgs/JointState): Current robot configuration
- `/panda_arm_controller/follow_joint_trajectory` (control_msgs/FollowJointTrajectory): Robot control

#### 💬 User Interface
- `/text_command` (std_msgs/String): Natural language commands
- `/pi0/status` (std_msgs/String): System status messages
- `/pi0/action_feedback` (std_msgs/String): Action execution feedback

### Services

#### 🎲 Simulation Services
- `/spawn_objects` (pi0_franka_msgs/SpawnObjects): Generate random objects in scene
- `/remove_object` (gazebo_msgs/DeleteEntity): Remove specific objects
- `/reset_scene` (std_srvs/Empty): Reset simulation environment
- `/pause_physics` (std_srvs/Empty): Pause/unpause Gazebo physics

#### 🤖 Robot Services
- `/execute_action` (pi0_franka_msgs/ExecuteAction): Execute computed actions
- `/home_robot` (std_srvs/Empty): Move robot to home position
- `/emergency_stop` (std_srvs/Empty): Emergency stop (real robot only)

### Parameters
- `use_simulation`: Enable simulation mode (default: true)
- `pi0_model_path`: Path to Pi-Zero model files
- `camera_frame`: Camera reference frame
- `robot_base_frame`: Robot base coordinate frame
- `workspace_bounds`: Manipulation workspace limits
- `object_spawn_rate`: Rate of random object generation (sim only)
- `physics_real_time_factor`: Gazebo physics speed (sim only)

## Configuration

## 📋 Python Dependencies

### Core Dependencies (`requirements.txt`)
```txt
# ROS2 Python bindings
rclpy>=3.3.0
sensor-msgs
geometry-msgs
std-msgs
tf2-ros
cv-bridge

# Computer Vision & ML
torch>=2.0.0
torchvision>=0.15.0
transformers>=4.30.0
pillow>=9.0.0
opencv-python>=4.5.0
numpy>=1.21.0

# Robotics
moveit-py
franka-ros2-py

# Vision-Language Models
huggingface-hub>=0.15.0
accelerate>=0.20.0

# Utilities
pyyaml
scipy
matplotlib
```

### Development Dependencies (`requirements-dev.txt`)
```txt
# Testing
pytest>=7.0.0
pytest-cov
pytest-mock
pytest-asyncio

# Code Quality
black
flake8
mypy
pre-commit

# Documentation
sphinx
sphinx-rtd-theme

# Development Tools
ipython
jupyter
```

### GPU Dependencies (`requirements-gpu.txt`)
```txt
# CUDA-enabled PyTorch (install based on your CUDA version)
--index-url https://download.pytorch.org/whl/cu118
torch>=2.0.0+cu118
torchvision>=0.15.0+cu118
torchaudio>=2.0.0+cu118

# Additional GPU utilities
nvidia-ml-py3
gpustat
```
## ⚙️ Configuration

### Robot Configuration (`config/robot_config.yaml`)
robot:
  name: "panda"
  base_frame: "panda_link0"
  ee_frame: "panda_hand"
  planning_group: "panda_arm"
  
workspace:
  x_min: 0.3
  x_max: 0.8
  y_min: -0.4
  y_max: 0.4
  z_min: 0.0
  z_max: 0.5
```

### Camera Configuration (`config/camera_config.yaml`)
```yaml
camera:
  frame_id: "camera_link"
  image_topic: "/camera/image_raw"
  depth_topic: "/camera/depth/image_raw"
  info_topic: "/camera/camera_info"
```

## Development

### Adding New VLA Models
The architecture is designed to support multiple VLA models. To add OpenVLA or other models:

1. Create a new model interface in `src/pi0_franka_ros2/models/`
2. Implement the `VLAModelInterface` abstract class
3. Update the model loader configuration

### Custom Object Types
Add new object types by modifying `config/objects.yaml` and updating the spawning logic.

### Extending Action Space
The action processor can be extended to support additional manipulation primitives beyond pick-and-place.

## Testing

Run the test suite:
```bash
colcon test --packages-select pi0_franka_ros2
```

Run integration tests:
```bash
# Start simulation first
ros2 launch pi0_franka_ros2 simulation.launch.py

# Run integration tests
python3 -m pytest tests/test_integration.py -v
```

## Troubleshooting

### Common Issues

**Model Loading Errors**
- Ensure Pi-Zero model files are properly downloaded
- Check CUDA/GPU availability for model inference
- Verify Python environment has all required packages

**Robot Connection Issues**
- For real robot: Check Franka network configuration
- For simulation: Verify Gazebo is properly launched
- Ensure all ROS2 nodes are running

**Camera Calibration**
- Run camera calibration routine if using real hardware
- Check camera frame transformations in TF tree

## Contributing

We welcome contributions! Please see [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

### Development Setup
```bash
# Clone and setup development environment
git clone https://github.com/your-username/pi0_franka_ros2.git
cd pi0_franka_ros2

# Run automated setup
./scripts/setup_environment.sh

# Install development dependencies
source venv/bin/activate
pip install -r requirements-dev.txt

# Setup pre-commit hooks for code quality
pre-commit install

# Run initial tests to verify setup
./scripts/run_tests.sh
```

### Docker Development
```bash
# Use Docker for isolated development
docker-compose up pi0-dev

# Or build specific configurations
docker build -f docker/Dockerfile.gpu -t pi0_franka:dev .
docker run -it --gpus all -v $(pwd):/workspace pi0_franka:dev
```

## 🚀 Roadmap

- [x] Basic ROS2 package structure with Python workspace management
- [x] Docker containerization and CI/CD pipeline
- [x] Franka simulation integration
- [x] Automated testing and code quality checks
- [ ] Pi-Zero model integration
- [ ] Real robot testing and validation
- [ ] OpenVLA support and model comparison
- [ ] Multi-robot coordination capabilities
- [ ] Advanced manipulation primitives
- [ ] Performance optimization and benchmarking
- [ ] Community tutorials and documentation
- [ ] ROS2 package release to apt repositories

## 🐳 Docker Hub Images

Pre-built Docker images are available:

```bash
# Pull latest stable image
docker pull your-username/pi0_franka:latest

# Pull GPU-enabled image
docker pull your-username/pi0_franka:gpu

# Pull development image with all tools
docker pull your-username/pi0_franka:dev
```

## Citation

If you use this package in your research, please cite:

```bibtex
@software{pi0_franka_ros2,
  title={Pi-Zero Franka ROS2 Integration},
  author={Your Name},
  year={2024},
  url={https://github.com/your-username/pi0_franka_ros2}
}
```

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- [Pi-Zero team](https://www.physicalintelligence.company/) for the vision-language-action model
- [Franka Emika](https://www.franka.de/) for the Panda robot
- [ROS2 community](https://ros.org/) for the robotics middleware
- [MoveIt2](https://moveit.ros.org/) for motion planning capabilities

## Support

- **Issues**: [GitHub Issues](https://github.com/your-username/pi0_franka_ros2/issues)
- **Discussions**: [GitHub Discussions](https://github.com/your-username/pi0_franka_ros2/discussions)
- **Documentation**: [Wiki](https://github.com/your-username/pi0_franka_ros2/wiki)