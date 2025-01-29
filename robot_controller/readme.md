# คู่มือการใช้งาน Pure Pursuit Controller (ROS2)

## การติดตั้ง Lib และ Package สำหรับการเตรียม Enviroment
ให้นักศึกษาเตรียมติดตั้ง Lib / Package ตามขั้นตอนต่อไปนี้

### 1. การติดตั้ง ROS (Humble)

```bash
sudo apt update
sudo apt install ros-humble-desktop
source /opt/ros/humble/setup.bash
```

### 2. การติดตั้ง Dependencies ที่จำเป็น
#### 1. การติดตั้ง Package ROS2

```bash
sudo apt install \
  ros-humble-nav2-msgs \
  ros-humble-tf2-ros \
  ros-humble-robot-localization \
  ros-humble-slam-toolbox \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-nav2-* \
  ros-humble-turtlebot3-*
```

#### 2. การติดตั้ง Lib ของ Python

```bash
pip install numpy scipy matplotlib
```

#### 3. การใช้งาน rosdep สำหรับการจัดการ dependencies แบบอัตโนมัติ

```bash
cd ~/[your_workspace]
rosdep install --from-paths src --ignore-src -r -y
```

#### 4. การ Compile

```bash
colcon build --symlink-install
```

## การทดสอบ Enviroments

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

```bash
ros2 topic list

Output
/clock
/cmd_vel    "ใช้สำหรับการสั่งงานหุ่นยนต์ให้เครื่อนที่ตามคำสั่งแบบ Twist"
/imu
/joint_states
/odom       "ใช้สำหรับการรับค่า Pose ของหุ่นยนต์"
/parameter_events
/performance_metrics
/robot_description
/rosout
/scan       "ใช้สำหรับการรับค่า Laser Scan"
/tf
/tf_static
```
