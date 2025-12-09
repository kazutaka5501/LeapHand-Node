#!/usr/bin/env python3
import numpy as np
import rospy

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
        self.ema_amount = 0.2
        self.prev_pos = self.pos = self.curr_pos = np.array([
            6.00, 3.14, 3.811942, 5.075943,
            6.00, 3.14, 3.762855, 5.190991,
            4.60, 3.14, 3.00, 3.0,
            4.712, 2.90, 4.70, 3.0
        ])

        # Subscribers
        rospy.Subscriber("/leaphand_node/cmd_leap", JointState, self._receive_pose)
        rospy.Subscriber("/leaphand_node/cmd_allegro", JointState, self._receive_allegro)
        rospy.Subscriber("/leaphand_node/cmd_ones", JointState, self._receive_ones)

        # Services
        rospy.Service('leap_position', leap_position, self.pos_srv)
        rospy.Service('leap_velocity', leap_velocity, self.vel_srv)
        rospy.Service('leap_effort', leap_effort, self.eff_srv)

        # Dynamixel client setup
        self.motors = motors = list(range(16))
        try:
            self.dxl_client = DynamixelClient(motors, '/dev/ttyUSB0', 4000000)
            self.dxl_client.connect()
        except Exception:
            try:
                self.dxl_client = DynamixelClient(motors, '/dev/ttyUSB1', 4000000)
                self.dxl_client.connect()
            except Exception:
                self.dxl_client = DynamixelClient(motors, '/dev/ttyUSB2', 4000000)
                self.dxl_client.connect()

        # Mode and torque
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * 5, 11, 1)
        self.dxl_client.set_torque_enabled(motors, True)

        # Gain settings
        normal_ids = list(range(0, 8))   # Index + Middle
        weak_ids   = list(range(8, 16))  # Ring + Thumb

        # Pゲイン
        self.dxl_client.sync_write(normal_ids, np.ones(len(normal_ids)) * self.kP, 84, 2)
        self.dxl_client.sync_write([0, 4], np.ones(2) * (self.kP * 0.75), 84, 2)
        self.dxl_client.sync_write(weak_ids, np.zeros(len(weak_ids)), 84, 2)

        # Iゲイン (weak_ids は 0.1)
        self.dxl_client.sync_write(normal_ids, np.ones(len(normal_ids)) * self.kI, 82, 2)
        self.dxl_client.sync_write(weak_ids, np.ones(len(weak_ids)) * 0.1, 82, 2)

        # Dゲイン
        self.dxl_client.sync_write(normal_ids, np.ones(len(normal_ids)) * self.kD, 80, 2)
        self.dxl_client.sync_write([0, 4], np.ones(2) * (self.kD * 0.75), 80, 2)
        self.dxl_client.sync_write(weak_ids, np.zeros(len(weak_ids)), 80, 2)

        # Current limit
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.curr_lim, 102, 2)

        # 初期姿勢
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)

        # === Publisher for state ===
        self.pub_state = rospy.Publisher("/leaphand_node/state", JointState, queue_size=10)
        # 100Hz タイマー
        self.timer = rospy.Timer(rospy.Duration(0.01), self._publish_state)

        rospy.spin()

    # Publish current state
    def _publish_state(self, event):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = [f"joint_{i}" for i in range(16)]
        msg.position = self.dxl_client.read_pos().tolist()
        msg.velocity = self.dxl_client.read_vel().tolist()
        msg.effort   = self.dxl_client.read_cur().tolist()
        self.pub_state.publish(msg)

    # disable PID for IDs 8-15
    def _disable_pid_8_to_15(self):
        disable_ids = list(range(8, 16))
        zeros = np.zeros(len(disable_ids))
        self.dxl_client.sync_write(disable_ids, zeros, 84, 2)   # P = 0
        self.dxl_client.sync_write(disable_ids, np.ones(len(disable_ids)) * 0.1, 82, 2)  # I = 0.1
        self.dxl_client.sync_write(disable_ids, zeros, 80, 2)   # D = 0

    # callbacks
    def _receive_pose(self, pose):
        pose = pose.position
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
        self._disable_pid_8_to_15()

    def _receive_allegro(self, pose):
        pose = lhu.allegro_to_LEAPhand(pose.position, zeros=False)
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
        self._disable_pid_8_to_15()

    def _receive_ones(self, pose):
        pose = lhu.sim_ones_to_LEAPhand(np.array(pose.position))
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
        self._disable_pid_8_to_15()

    # services
    def pos_srv(self, req):
        return {"position": self.dxl_client.read_pos()}
    def vel_srv(self, req):
        return {"velocity": self.dxl_client.read_vel()}
    def eff_srv(self, req):
        return {"effort": self.dxl_client.read_cur()}
    def pos_vel_srv(self, req):
        output = self.dxl_client.read_pos_vel()
        return {"position": output[0], "velocity": output[1], "effort": np.zeros_like(output[1])}
    def pos_vel_eff_srv(self, req):
        output = self.dxl_client.read_pos_vel_cur()
        return {"position": output[0], "velocity": output[1], "effort": output[2]}

def main(**kwargs):
    rospy.init_node("leaphand_node")
    leaphand_node = LeapNode()

if __name__ == "__main__":
    main()

