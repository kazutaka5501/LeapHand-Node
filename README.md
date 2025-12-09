# 🖐️ LeapHand-Node (ROS1 Add-On Control Package)

This repository provides additional control modes for the **LEAP Hand** and is intended to be used **on top of the official LEAP Hand ROS module**.  

⚠️ **Before using this package, you must install the official LEAP Hand ROS module located here:**

👉 https://github.com/leap-hand/LEAP_Hand_API/tree/main/ros_module

This package does **not** replace the original driver — instead, it offers teleoperation modes, pose-based behaviors, and manual soft-actuation control for research and experimentation.

---

## 🚀 Features

- ✔️ Manual joint control with near-zero PD stiffness (physically manipulable hand posture recording)
- ✔️ Pose-based playback (open-loop position execution)
- ✔️ Full pose teleoperation (closed-loop interpolation and execution)
- ✔️ Basic direct teleoperation interface
- ✔️ ROS1 Noetic compatible Python nodes
- ✔️ Extendable for glove-based teleoperation (CyberGlove, Manus MetaGlove, DG-5F, etc.)

---

## 📦 Package Contents

| File | Launch | Description |
|------|--------|-------------|
| `leaphand_node_manual.py` | `manual.launch` | Manual physical manipulation mode — PD gains minimized to softly move the hand and record joint states |
| `leaphand_node_posenoloop.py` | `posenoloop.launch` | Predefined manipulation sequences executed using **pure position control (no feedback loop)** |
| `leaphand_node_teleop.py` | `teleop.launch` | Basic teleoperation mode — device → LEAP joint mapping |
| `leaphand_node_teleop_pose_full.py` | `teleop_full.launch` | Full teleoperation with pose-based control (device input + target pose interpolation) |

LeapHand-Node/
├─ leaphand_node_manual.py
├─ leaphand_node_posenoloop.py
├─ leaphand_node_teleop.py
├─ leaphand_node_teleop_pose_full.py
├─ manual.launch
├─ posenoloop.launch
├─ teleop.launch
└─ teleop_full.launch


---

## 🧰 Requirements

| Item | Version |
|------|---------|
| Ubuntu | **20.04** |
| ROS | **Noetic** |
| Python | 3.8 or higher |
| LEAP Hand ROS Module | Required |

---

## 🔧 Install the Official LEAP Hand ROS Module First

```bash
sudo apt install ros-noetic-desktop-full



(Optional virtual environment — recommended)

python3 -m venv leap_env
source leap_env/bin/activate

Create workspace:

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

Clone or copy the official ros_module folder from:

👉 https://github.com/leap-hand/LEAP_Hand_API/tree/main/ros_module

Then build:

pip install empy==3.3.4 catkin_pkg pyyaml rospkg
cd ~/catkin_ws
catkin_make

Setup environment:

echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

Install dependencies:

pip install dynamixel_sdk numpy
sudo chmod 777 /dev/serial/by-id/*
cd ~/catkin_ws/src/ros_module
chmod +x leaphand_node.py

Test official module:

roslaunch example.launch
rostopic list
rosservice list
