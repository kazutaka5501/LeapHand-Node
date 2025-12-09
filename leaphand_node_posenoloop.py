#!/usr/bin/env python3
import numpy as np
import rospy
import time
import os
import subprocess
from datetime import datetime
from sensor_msgs.msg import JointState

from leap_hand_utils.dynamixel_client import *
import leap_hand_utils.leap_hand_utils as lhu

# -------------------------
# あらかじめ定義するポーズ
# -------------------------
poses = [
    np.array([  # pose1
        5.129632, 3.14, 3.805806, 5.09435,
        4.784486, 3.14, 4.212311, 5.194059,
        4.50, 3.14, 3.959204, 2.802583,
        4.70, 3.14, 4.350369, 1.1934377
    ]),
    np.array([  # pose2
        5.30, 3.14, 3.805806, 5.09435,
        5.00, 3.14, 4.212311, 5.194059,
        4.50, 3.14, 3.959204, 2.802583,
        4.70, 3.14, 4.350369, 1.193437
    ]),
    np.array([  # pose3
        6.00, 3.14, 3.805806, 5.09435,
        6.00, 3.14, 4.212311, 5.194059,
        5.50, 3.14, 3.959204, 3.50,
        4.70, 3.14, 4.350369, 2
    ]),
    np.array([  # pose4
        6.00, 3.14, 3.805806, 5.09435,
        6.00, 3.14, 4.212311, 5.194059,
        5.00, 4.50, 3.959204, 3.50,
        4.60, 3.14, 4.350369, 2
    ]),
    np.array([  # pose5
        6.00, 3.14, 3.805806, 5.09435,
        6.00, 3.14, 4.212311, 5.194059,
        4.00, 3.14, 3.959204, 3.146195,
        4.70, 3.14, 4.350369, 2
    ]),
    np.array([  # pose6
        6.00, 3.14, 3.805806, 5.09435,
        6.00, 3.14, 4.212311, 5.194059,
        6.00, 3.14, 3.959204, 4.00,
        4.70, 3.14, 4.350369, 3.00
    ])
]

# -------------------------
# Rosbag Logger
# -------------------------
class EpisodeRosbagLogger:
    def __init__(self):
        self.process = None
        self.base_dir = "/home/kazutaka/catkin_ws/src/ros_module/data_logger/position_control"

        # 記録対象トピック一覧
        self.topics = [
            "/camera/color/camera_info",
            "/camera/color/image_raw",
            "/camera/color/image_raw/compressed",
            "/camera/color/image_raw/compressed/parameter_descriptions",
            "/camera/color/image_raw/compressed/parameter_updates",
            "/camera/color/image_raw/compressedDepth",
            "/camera/color/image_raw/compressedDepth/parameter_descriptions",
            "/camera/color/image_raw/compressedDepth/parameter_updates",
            "/camera/color/image_raw/theora",
            "/camera/color/image_raw/theora/parameter_descriptions",
            "/camera/color/image_raw/theora/parameter_updates",
            "/camera/color/metadata",
            "/camera/depth/camera_info",
            "/camera/depth/image_rect_raw",
            "/camera/depth/image_rect_raw/compressed",
            "/camera/depth/image_rect_raw/compressed/parameter_descriptions",
            "/camera/depth/image_rect_raw/compressed/parameter_updates",
            "/camera/depth/image_rect_raw/compressedDepth",
            "/camera/depth/image_rect_raw/compressedDepth/parameter_descriptions",
            "/camera/depth/image_rect_raw/compressedDepth/parameter_updates",
            "/camera/depth/image_rect_raw/theora",
            "/camera/depth/image_rect_raw/theora/parameter_descriptions",
            "/camera/depth/image_rect_raw/theora/parameter_updates",
            "/camera/depth/metadata",
            "/camera/extrinsics/depth_to_color",
            "/camera/realsense2_camera_manager/bond",
            "/camera/rgb_camera/auto_exposure_roi/parameter_descriptions",
            "/camera/rgb_camera/auto_exposure_roi/parameter_updates",
            "/camera/rgb_camera/parameter_descriptions",
            "/camera/rgb_camera/parameter_updates",
            "/camera/stereo_module/auto_exposure_roi/parameter_descriptions",
            "/camera/stereo_module/auto_exposure_roi/parameter_updates",
            "/camera/stereo_module/parameter_descriptions",
            "/camera/stereo_module/parameter_updates",
            "/diagnostics",
            "/gelsight2/raspicam_node/camera_info",
            "/gelsight2/raspicam_node/image/compressed",
            "/gelsight2/raspicam_node/parameter_descriptions",
            "/gelsight2/raspicam_node/parameter_updates",
            "/leaphand_node/command",
            "/leaphand_node/state"
        ]

    def start_episode(self, episode_id):
        if self.process:
            rospy.logwarn("Episode already recording. Stop it first.")
            return

        date_str = datetime.now().strftime("%Y%m%d")
        save_dir = os.path.join(self.base_dir, date_str, "rosbag")
        os.makedirs(save_dir, exist_ok=True)

        bagfile = os.path.join(save_dir, f"episode_{episode_id}.bag")

        cmd = ["rosbag", "record", "-O", bagfile] + self.topics
        self.process = subprocess.Popen(cmd)
        rospy.loginfo(f"Episode {episode_id} recording started -> {bagfile}")

    def stop_episode(self):
        if self.process:
            self.process.terminate()
            self.process.wait()
            rospy.loginfo("Episode recording stopped.")
            self.process = None

    def shutdown(self):
        self.stop_episode()

# -------------------------
# LeapNode
# -------------------------
class LeapNode:
    def __init__(self):
        self.kP = float(rospy.get_param('/leaphand_node/kP', 800.0))
        self.kI = float(rospy.get_param('/leaphand_node/kI', 0.0))
        self.kD = float(rospy.get_param('/leaphand_node/kD', 200.0))
        self.curr_lim = float(rospy.get_param('/leaphand_node/curr_lim', 350.0))

        self.curr_pos = lhu.allegro_to_LEAPhand(np.zeros(16))
        self.motors = list(range(16))
        self.dxl_client = DynamixelClient(self.motors, '/dev/ttyUSB0', 4000000)
        self.dxl_client.connect()

        # 初期設定
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * 5, 11, 1)
        self.dxl_client.set_torque_enabled(self.motors, True)
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.kP, 84, 2)
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.kI, 82, 2)
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.kD, 80, 2)
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.curr_lim, 102, 2)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)

        self.pub_cmd = rospy.Publisher("/leaphand_node/command", JointState, queue_size=10)
        self.pub_state = rospy.Publisher("/leaphand_node/state", JointState, queue_size=10)
        rospy.Timer(rospy.Duration(0.01), self.publish_state)

    def move_in_steps(self, target_pose, steps=100, delay=0.01):
        start_pose = np.array(self.curr_pos, dtype=float)
        target_pose = np.array(target_pose, dtype=float)
        diff = target_pose - start_pose
        increment = diff / steps
        pose = start_pose.copy()

        for _ in range(steps):
            pose += increment
            self.dxl_client.write_desired_pos(self.motors, pose)

            js_cmd = JointState()
            js_cmd.header.stamp = rospy.Time.now()
            js_cmd.name = [f"joint_{i}" for i in self.motors]
            js_cmd.position = pose.tolist()
            self.pub_cmd.publish(js_cmd)

            time.sleep(delay)

        self.curr_pos = target_pose

    def publish_state(self, event=None):
        pos = self.dxl_client.read_pos()
        vel = self.dxl_client.read_vel()
        cur = self.dxl_client.read_cur()

        js_state = JointState()
        js_state.header.stamp = rospy.Time.now()
        js_state.name = [f"joint_{i}" for i in self.motors]
        js_state.position = pos.tolist()
        js_state.velocity = vel.tolist()
        js_state.effort = cur.tolist()
        self.pub_state.publish(js_state)


# -------------------------
# Main
# -------------------------
def main():
    rospy.init_node("leaphand_episode_runner")
    node = LeapNode()
    logger = EpisodeRosbagLogger()
    rospy.on_shutdown(logger.shutdown)

    try:
        while not rospy.is_shutdown():
            rospy.loginfo("Moving to pose 1...")
            node.move_in_steps(poses[0], steps=100)
            time.sleep(1)

            ans = input("次のポーズに進みますか？ (y/n): ").strip().lower()
            if ans != "y":
                rospy.loginfo("ユーザ入力で終了しました。")
                break

            ep = input("エピソード番号を入力してください: ")
            logger.start_episode(ep)

            for i in range(1, len(poses)):
                rospy.loginfo(f"Moving to pose {i+1}...")
                node.move_in_steps(poses[i], steps=100)
                time.sleep(1)

            logger.stop_episode()
            rospy.loginfo("1シーケンス終了しました。")

            loop_ans = input("pose1からもう一度loopしますか？ (y/n): ").strip().lower()
            if loop_ans != "y":
                rospy.loginfo("全シーケンス終了しました。")
                break

    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()

