#!/usr/bin/env python3
import numpy as np
import rospy
import threading
import sys
import select
import termios
import tty
import subprocess
import datetime
import os
import signal

from sensor_msgs.msg import JointState
from leap_hand_utils.dynamixel_client import *
import leap_hand_utils.leap_hand_utils as lhu
from leap_hand.srv import *

class LeapNode:
    def __init__(self):
        #### parameters
        self.kP = float(rospy.get_param('/leaphand_node/kP', 800.0))
        self.kI = float(rospy.get_param('/leaphand_node/kI', 0.0))
        self.kD = float(rospy.get_param('/leaphand_node/kD', 350.0))
        self.curr_lim = float(rospy.get_param('/leaphand_node/curr_lim', 350.0))

        self.motors = list(range(16))
        self.active_control = False
        self.rosbag_proc = None

        # Dynamixel接続
        self.dxl_client = None
        for port in ['/dev/ttyUSB0','/dev/ttyUSB1','/dev/ttyUSB2']:
            try:
                self.dxl_client = DynamixelClient(self.motors, port, 4000000)
                self.dxl_client.connect()
                rospy.loginfo(f"Connected to LEAP Hand on {port}")
                break
            except Exception:
                continue
        if self.dxl_client is None:
            rospy.logerr("Failed to connect to LEAP Hand motors!")
            sys.exit(1)

        # PID設定
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * 5, 11, 1)
        self.dxl_client.set_torque_enabled(self.motors, True)
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.kP, 84, 2)
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.kI, 82, 2)
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.kD, 80, 2)
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.curr_lim, 102, 2)

        # 固定ポーズ
        self.fixed_pose = np.array([
            5.129632, 3.14, 3.805806, 5.09435,
            4.784486, 3.14, 4.212311, 5.194059,
            4.50,     3.14, 3.959204, 2.802583,
            4.70, 3.084835, 4.700369, 1.1934377
        ])

        # 制御可能モーター
        self.control_ids = [8, 9, 10, 11, 12, 13]
        self.lower_limits = {i: self.fixed_pose[i] for i in self.control_ids}
        self.range_limits = {
            12: (self.fixed_pose[12] - 0.611, self.fixed_pose[12] + 0.611),
            13: (self.fixed_pose[13] - 0.611, self.fixed_pose[13] + 0.611)
        }

        self.home_pose = np.copy(self.fixed_pose)
        self.dxl_client.write_desired_pos(self.motors, self.home_pose)
        rospy.loginfo("Ready. Press 'a' → enter episode number → teleop start.")

        # Publisher
        self.pub_state   = rospy.Publisher("/leaphand_node/state", JointState, queue_size=10)
        self.pub_command = rospy.Publisher("/leaphand_node/command", JointState, queue_size=10)

        # Mirror publishers
        self.pub_cmd_leap    = rospy.Publisher("/leaphand_node/cmd_leap_mirror", JointState, queue_size=10)
        self.pub_cmd_allegro = rospy.Publisher("/leaphand_node/cmd_allegro_mirror", JointState, queue_size=10)
        self.pub_cmd_ones    = rospy.Publisher("/leaphand_node/cmd_ones_mirror", JointState, queue_size=10)

        # Subscriber
        rospy.Subscriber("/leaphand_node/cmd_leap", JointState, self._receive_pose)
        rospy.Subscriber("/leaphand_node/cmd_allegro", JointState, self._receive_allegro)
        rospy.Subscriber("/leaphand_node/cmd_ones", JointState, self._receive_ones)

        # Services
        rospy.Service('leap_position', leap_position, self.pos_srv)
        rospy.Service('leap_velocity', leap_velocity, self.vel_srv)
        rospy.Service('leap_effort', leap_effort, self.eff_srv)

        # 状態配信タイマ
        hz = float(rospy.get_param('~state_publish_hz', 100.0))
        self.timer = rospy.Timer(rospy.Duration(1.0 / hz), self._publish_state)

        # キーボードスレッド
        threading.Thread(target=self._keyboard_listener, daemon=True).start()

    # rosbag制御
    def start_rosbag(self, episode):
        today = datetime.date.today().strftime("%Y%m%d")
        base_dir = os.path.expanduser(f"~/catkin_ws/src/ros_module/data_logger/cyberglove/{today}")
        os.makedirs(base_dir, exist_ok=True)
        bag_path = os.path.join(base_dir, f"episode_{episode}.bag")
        cmd = [
            "rosbag", "record", "-O", bag_path,
            "/raw_cyberglove_joint_states",
            "/leaphand_node/cmd_leap_mirror",
            "/leaphand_node/cmd_allegro_mirror",
            "/leaphand_node/cmd_ones_mirror",
            "/leaphand_node/command",
            "/leaphand_node/state",
        ]
        self.rosbag_proc = subprocess.Popen(cmd, preexec_fn=os.setsid)
        rospy.loginfo(f"Started rosbag recording: {bag_path}")

    def stop_rosbag(self):
        if self.rosbag_proc:
            try:
                os.killpg(os.getpgid(self.rosbag_proc.pid), signal.SIGINT)
                self.rosbag_proc.wait(timeout=5.0)
            except Exception:
                self.rosbag_proc.terminate()
            self.rosbag_proc = None
            rospy.loginfo("Stopped rosbag recording.")

    # キーボード監視
    def _keyboard_listener(self):
        old_attr = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        try:
            while not rospy.is_shutdown():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1).lower()
                    if key == 'a':
                        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
                        episode = input("Enter episode number: ")
                        tty.setcbreak(sys.stdin.fileno())
                        self.start_rosbag(episode)
                        self.active_control = True
                        rospy.loginfo(f"Teleop ENABLED after episode {episode} + recording started.")
                    elif key == 's':
                        self.active_control = False
                        self.stop_rosbag()
                        self.dxl_client.write_desired_pos(self.motors, self.home_pose)
                        rospy.loginfo("Teleop DISABLED. Returned to fixed pose.")
                    elif key == 'q':
                        self.stop_rosbag()
                        rospy.loginfo("Exiting by 'q' key.")
                        rospy.signal_shutdown("User requested shutdown.")
                        break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)

    # 共通処理（オフセット含む）
    def _merge_with_fixed(self, input_array):
        desired = np.copy(self.fixed_pose)
        for i in self.control_ids:
            val = max(input_array[i], self.lower_limits[i])

            # --- オフセット適用 ---
            if i == 13:
                val = val - np.deg2rad(15)   # ID13 → -15°
            if i == 9:
                val = val + np.deg2rad(10)   # ID9  → +10°
            if i == 8:
                val = val - np.deg2rad(5)    # ID8  → -5°

            # 可動域制約（ID12,13のみ）
            if i in self.range_limits:
                lo, hi = self.range_limits[i]
                val = min(max(val, lo), hi)

            desired[i] = val
        return desired

    # 状態Publish
    def _publish_state(self, event):
        try:
            pos = self.dxl_client.read_pos()
            vel = self.dxl_client.read_vel()
            cur = self.dxl_client.read_cur()

            js = JointState()
            js.header.stamp = rospy.Time.now()
            js.name = [f"joint_{i}" for i in range(16)]
            js.position = list(pos)
            js.velocity = list(vel)
            js.effort = list(cur)
            self.pub_state.publish(js)
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"Failed to publish state: {e}")

    # 指令Publish
    def _publish_command(self, desired):
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name = [f"joint_{i}" for i in range(16)]
        js.position = list(desired)
        self.pub_command.publish(js)

    # Callbacks
    def _receive_pose(self, pose):
        if not self.active_control:
            return
        desired = self._merge_with_fixed(np.array(pose.position))
        self.dxl_client.write_desired_pos(self.motors, desired)
        self.pub_cmd_leap.publish(pose)
        self._publish_command(desired)

    def _receive_allegro(self, pose):
        if not self.active_control:
            return
        converted = lhu.allegro_to_LEAPhand(pose.position, zeros=False)
        desired = self._merge_with_fixed(converted)
        self.dxl_client.write_desired_pos(self.motors, desired)
        self.pub_cmd_allegro.publish(pose)
        self._publish_command(desired)

    def _receive_ones(self, pose):
        if not self.active_control:
            return
        converted = lhu.sim_ones_to_LEAPhand(np.array(pose.position))
        desired = self._merge_with_fixed(converted)
        self.dxl_client.write_desired_pos(self.motors, desired)
        self.pub_cmd_ones.publish(pose)
        self._publish_command(desired)

    # Services
    def pos_srv(self, req): return {"position": self.dxl_client.read_pos()}
    def vel_srv(self, req): return {"velocity": self.dxl_client.read_vel()}
    def eff_srv(self, req): return {"effort": self.dxl_client.read_cur()}

def main():
    rospy.init_node("leaphand_node")
    node = LeapNode()
    rospy.spin()

if __name__ == "__main__":
    main()

