#!/usr/bin/env python3
import numpy as np
import rospy
import threading
import sys
import select
import termios
import tty

from sensor_msgs.msg import JointState
from std_msgs.msg import String

from leap_hand_utils.dynamixel_client import *
import leap_hand_utils.leap_hand_utils as lhu
from leap_hand.srv import *

class LeapNode:
    def __init__(self):
        #### parameters
        self.kP = float(rospy.get_param('/leaphand_node/kP', 800.0))
        self.kI = float(rospy.get_param('/leaphand_node/kI', 0.0))
        self.kD = float(rospy.get_param('/leaphand_node/kD', 200.0))
        self.curr_lim = float(rospy.get_param('/leaphand_node/curr_lim', 350.0))

        self.motors = [i for i in range(16)]
        self.active_control = False   # ← フラグ: a押下後に有効化

        # Dynamixel 接続
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
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors))*5, 11, 1)
        self.dxl_client.set_torque_enabled(self.motors, True)
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.kP, 84, 2)
        self.dxl_client.sync_write([0,4,8], np.ones(3) * (self.kP * 0.75), 84, 2)
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.kI, 82, 2)
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.kD, 80, 2)
        self.dxl_client.sync_write([0,4,8], np.ones(3) * (self.kD * 0.75), 80, 2)
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.curr_lim, 102, 2)

        # 初期ポーズ（オープン）
        self.home_pose = lhu.allegro_to_LEAPhand(np.zeros(16))
        self.dxl_client.write_desired_pos(self.motors, self.home_pose)
        rospy.loginfo("Moved to initial home pose. Press 'a' to enable teleop control.")

        # Subscriber
        rospy.Subscriber("/leaphand_node/cmd_leap", JointState, self._receive_pose)
        rospy.Subscriber("/leaphand_node/cmd_allegro", JointState, self._receive_allegro)
        rospy.Subscriber("/leaphand_node/cmd_ones", JointState, self._receive_ones)

        # Services
        rospy.Service('leap_position', leap_position, self.pos_srv)
        rospy.Service('leap_velocity', leap_velocity, self.vel_srv)
        rospy.Service('leap_effort', leap_effort, self.eff_srv)

        # 別スレッドでキーボード監視
        threading.Thread(target=self._keyboard_listener, daemon=True).start()

    def _keyboard_listener(self):
        """ キーボード入力で制御モードを切り替える """
        old_attr = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        try:
            while not rospy.is_shutdown():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1).lower()

                    if key == 'a':  # Teleop ON
                        self.active_control = True
                        rospy.loginfo("Teleop control ENABLED (press 's' to stop).")

                    elif key == 's':  # Teleop OFF + 初期位置へ戻す
                        self.active_control = False
                        self.dxl_client.write_desired_pos(self.motors, self.home_pose)
                        rospy.loginfo("Teleop control DISABLED. Returned to home pose.")

                    elif key == 'q':  # 終了
                        rospy.loginfo("Exiting by 'q' key.")
                        rospy.signal_shutdown("User requested shutdown.")
                        break

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)


    # --- Callbacks ---
    def _receive_pose(self, pose):
        if not self.active_control: return
        self.dxl_client.write_desired_pos(self.motors, np.array(pose.position))

    def _receive_allegro(self, pose):
        if not self.active_control: return
        converted = lhu.allegro_to_LEAPhand(pose.position, zeros=False)
        self.dxl_client.write_desired_pos(self.motors, np.array(converted))

    def _receive_ones(self, pose):
        if not self.active_control: return
        converted = lhu.sim_ones_to_LEAPhand(np.array(pose.position))
        self.dxl_client.write_desired_pos(self.motors, np.array(converted))

    # --- Services ---
    def pos_srv(self, req): return {"position": self.dxl_client.read_pos()}
    def vel_srv(self, req): return {"velocity": self.dxl_client.read_vel()}
    def eff_srv(self, req): return {"effort": self.dxl_client.read_cur()}

# init
def main():
    rospy.init_node("leaphand_node")
    node = LeapNode()
    rospy.spin()

if __name__ == "__main__":
    main()

