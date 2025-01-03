# 🤖 Mobile Robot Simulation Game (ROS 2 + Python)

An interactive educational game designed to supplement the "Mobile Robot" course, focusing on practical understanding of mobile robotics concepts through ROS 2 and Python 3.

![GitHub license](https://img.shields.io/badge/license-MIT-blue.svg)
![ROS 2](https://img.shields.io/badge/ROS%202-Humble-brightgreen)
![Python](https://img.shields.io/badge/Python-3.8%2B-blue)

## 📚 Educational Content

- Mobile Robot Modeling
- Kinematics (Forward & Inverse)
- Odometry
- Simple Control Algorithms

## 🎮 Game Overview

Learn mobile robotics interactively within a ROS 2 environment. Each mission challenges players to:

- Configure robot parameters (wheel distance, radius)
- Implement kinematics formulas
- Develop control algorithms
- Test solutions in real-time simulation

## 🎯 Mission Structure

### Mission 1: Robot Modeling & Basic Control
- Experiment with robot parameters (L, R)
- Learn differential-drive equations
- Practice basic ROS 2 velocity commands

### Mission 2: Forward Kinematics
- Calculate robot position (x, y, θ) from wheel velocities (vL, vR)
- Navigate to target positions
- Visualize results in Rviz

### Mission 3: Inverse Kinematics
- Compute wheel velocities (ωL, ωR) for desired trajectories
- Convert between linear/angular velocities (v, ω)
- Fine-tune parameters for accuracy

### Mission 4: Odometry
- Track robot position using wheel encoders
- Compare with ground truth
- Minimize drift through calibration

### Mission 5: Control Algorithms
- Implement line following
- Develop obstacle avoidance
- Create custom navigation solutions

## 🛠️ Installation

### Prerequisites
- ROS 2 (Foxy, Galactic, or Humble)
- Python 3.8+

### Setup Steps

1. **Create Workspace**
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```

2. **Clone Repository**
   ```bash
   git clone https://github.com/YOUR_USERNAME/MobileRobotGame.git
   ```

3. **Install Dependencies**
   ```bash
   pip install -r requirements.txt
   ```

4. **Build Workspace**
   ```bash
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```

## 🎲 Running the Game

1. **Launch Simulation Environment**
   ```bash
   ros2 launch mobile_robot_simulation gazebo_world.launch.py
   ```

2. **Start Mission**
   ```bash
   ros2 run mobile_robot_simulation game_mission1
   ```

## 📁 Project Structure

```
MobileRobotGame/
├── launch/                  # Launch files
│   ├── gazebo_world.launch.py
│   └── rviz_config.rviz
├── worlds/                  # Gazebo worlds
├── urdf/                    # Robot models
├── mobile_robot_simulation/ # Core game logic
│   ├── game_mission1.py
│   ├── kinematics.py
│   ├── odometry.py
│   └── controller.py
├── config/                  # Configuration files
└── docs/                    # Documentation
```

## 📊 Scoring System

- **Accuracy**: Position error vs target
- **Efficiency**: Time and collision metrics
- **Implementation**: Code quality and parameter optimization
- **Progress**: Number of attempts per mission

## 🌟 Features

- Real-time simulation feedback
- Progressive difficulty curve
- Multiple challenge environments
- Performance metrics tracking
- Customizable robot parameters

## 🤝 Contributing

We welcome contributions! Please follow these steps:

1. Fork the repository
2. Create your feature branch
3. Commit your changes
4. Push to the branch
5. Submit a pull request

## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 📧 Contact

- Email: youremail@example.com
- GitHub: [Your GitHub Profile](https://github.com/YOUR_USERNAME)
- Issue Tracker: [GitHub Issues](https://github.com/YOUR_USERNAME/MobileRobotGame/issues)

## 🙏 Acknowledgments

- Course Instructors
- ROS 2 Community
- Contributors and Testers

---
Made with ❤️ for robotics education
