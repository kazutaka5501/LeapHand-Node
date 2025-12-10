# 🖐️ LeapHand-Node (ROS1 Add-On Control Package)

This repository provides additional control modes for the **LEAP Hand** and is intended to be used **on top of the official LEAP Hand ROS module**.  

 **Before using this package, you must install the official LEAP Hand ROS module located here:**

👉 https://github.com/leap-hand/LEAP_Hand_API/tree/main/ros_module

This package does **not** replace the original driver — instead, it provides enhanced teleoperation, pose-based execution, and soft manual manipulation modes suitable for research and experimentation.

---

## 🚀 Features

- ✔️ Manual joint movement mode with reduced PD gains (allowing physical pose shaping)
- ✔️ Predefined manipulation motion execution (position-only, no feedback loop)
- ✔️ CyberGlove-based teleoperation interface
- ✔️ ROS1 Noetic compatible
- ✔️ Modular Python code for extension


| Mode | Node | Launch File | Description |
|------|------|-------------|-------------|
| Manual Control | `leaphand_node_manual.py` | `manual.launch` | Soft actuation mode. The hand becomes compliant and can be physically moved to record joint poses. |
| Pose (No Loop) | `leaphand_node_posenoloop.py` | `posenoloop.launch` | Executes predefined hand manipulation motions using pure positional commands. |
| CyberGlove Teleoperation | `leaphand_node_teleop.py` | `teleop.launch` | Teleoperation mode where CyberGlove joint angles are mapped directly to LEAP hand commands. |
| Full Pose Teleoperation | `leaphand_node_teleop_pose_full.py` | `teleop_full.launch` | CyberGlove-based teleoperation with additional pose targeting and closed-loop refinement. |

---

## Requirements

| Item | Version |
|------|---------|
| Ubuntu | **20.04** |
| ROS | **Noetic** |
| Python | 3.8+ |
| LEAP Hand ROS Module | Required |
| CyberGlove driver | Required for teleoperation modes |

---

## 🔧 Install the Official LEAP Hand ROS Module First

```bash
sudo apt install ros-noetic-desktop-full

(Optional venv)

python3 -m venv leap_env
source leap_env/bin/activate

Clone required module:

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/leap-hand/LEAP_Hand_API.git
mv LEAP_Hand_API/ros_module .

Build:

pip install empy==3.3.4 catkin_pkg pyyaml rospkg
cd ~/catkin_ws
catkin_make

Setup:

echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
