#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
import time
from nav_msgs.msg import Path
import numpy as np
import pandas as pd
import threading
import os


class SlamPosePublisher(Node):
    def __init__(self):
        super().__init__('slam_pose_publisher')

        self.drones = ['Alpha', 'Bravo', 'Charlie']
        # ——————————————————————————————————————————————————————————
        # 1) Pre‐process all logs so SLAM⇔mocap only share 2‐decimal timestamps
        #    (this overwrites your .txt files in‐place)
        self._match_two_decimal()
        # ——————————————————————————————————————————————————————————

        self.initial_offsets = {drone: None for drone in self.drones}
        self.slam_initials   = {drone: None for drone in self.drones}

        self.pose_publishers = {
            drone: self.create_publisher(PoseStamped, f'{drone}/slam_pose', 10)
            for drone in self.drones
        }

        self.slam_paths = {drone: Path() for drone in self.drones}
        self.slam_path_publishers = {
            drone: self.create_publisher(Path, f'{drone}/slam_path', 10)
            for drone in self.drones
        }

        self.mocap_paths = {drone: Path() for drone in self.drones}
        self.mocap_path_publishers = {
            drone: self.create_publisher(Path, f'{drone}/mocap_path', 10)
            for drone in self.drones
        }

        self.file_paths = {
            drone: f'KeyFrameTrajectory_{drone.lower()}.txt' for drone in self.drones
        }

        self.mocap_file_paths = {
            drone: f'MocapTrajectory_{drone}.txt' for drone in self.drones
        }

        self.slam_arrow_publishers = {
            drone: self.create_publisher(Marker, f'{drone}/slam_arrow', 10)
            for drone in self.drones
        }

        self.mocap_arrow_publishers = {
            drone: self.create_publisher(Marker, f'{drone}/mocap_arrow', 10)
            for drone in self.drones
        }

        for drone in self.drones:
            threading.Thread(target=self.read_mocap_file_once, args=(drone,), daemon=True).start()
            threading.Thread(target=self.read_slam_file_once, args=(drone,), daemon=True).start()

    def _match_two_decimal(self):
        
        COLS = ['t','x','y','z','qx','qy','qz','qw']
        for drone in self.drones:
            slam_in  = f'KeyFrameTrajectory_{drone.lower()}.txt'
            mocap_in = f'MocapTrajectory_{drone}.txt'
            if not os.path.exists(slam_in) or not os.path.exists(mocap_in):
                self.get_logger().warn(f"Missing files for {drone}, skipping match")
                continue
            df_s = pd.read_csv(slam_in,  delim_whitespace=True, names=COLS)
            df_m = pd.read_csv(mocap_in, delim_whitespace=True, names=COLS)
            df_s['t2'] = df_s['t'].round(2)
            df_m['t2'] = df_m['t'].round(2)
            df  = pd.merge(df_s, df_m, on='t2', how='inner',
                           suffixes=('_slam','_mocap'))
            if df.empty:
                self.get_logger().warn(f"{drone}: no 2-decimal matches, leaving files untouched")
                continue
            # rebuild two matched DataFrames
            slam_matched = df[['t2','x_slam','y_slam','z_slam',
                               'qx_slam','qy_slam','qz_slam','qw_slam']]
            mocap_matched= df[['t2','x_mocap','y_mocap','z_mocap',
                               'qx_mocap','qy_mocap','qz_mocap','qw_mocap']]
            slam_matched.columns  = COLS
            mocap_matched.columns = COLS
            # overwrite originals
            slam_matched.to_csv(slam_in,  sep=' ', index=False,
                                header=False, float_format='%.2f')
            mocap_matched.to_csv(mocap_in, sep=' ', index=False,
                                 header=False, float_format='%.2f')
            # self.get_logger().info(f"{drone}: matched {len(slam_matched)} lines")

    def read_mocap_file_once(self, drone):
        path = self.mocap_file_paths[drone]
        while rclpy.ok():
            if not os.path.exists(path):
                self.get_logger().warn(f"[{drone}] Mocap file not found: {path}")
                return

            with open(path, 'r') as f:
                lines = f.readlines()

            for line in lines:
                data = line.strip().split()
                if len(data) != 8:
                    continue

                timestamp, x, y, z, qx, qy, qz, qw = map(float, data)

                if self.initial_offsets[drone] is None:
                    self.initial_offsets[drone] = np.array([x, y, z])
                    # self.get_logger().info(f"[{drone}] Initial mocap offset set to {self.initial_offsets[drone]}")

                msg = PoseStamped()
                msg.header.frame_id = 'map'
                msg.header.stamp.sec = int(timestamp)
                msg.header.stamp.nanosec = int((timestamp - int(timestamp)) * 1e9)
                msg.pose.position.x = x
                msg.pose.position.y = y
                msg.pose.position.z = z
                msg.pose.orientation.x = qx
                msg.pose.orientation.y = qy
                msg.pose.orientation.z = qz
                msg.pose.orientation.w = qw

                self.mocap_paths[drone].header = msg.header
                self.mocap_paths[drone].poses.append(msg)
                self.mocap_path_publishers[drone].publish(self.mocap_paths[drone])

                arrow = self.make_arrow_marker(drone, msg, namespace='mocap', marker_id=0)
                self.mocap_arrow_publishers[drone].publish(arrow)

        self.get_logger().info(f"[{drone}] Finished publishing mocap path")

    def read_slam_file_once(self, drone):
        # 1) wait until we've read the first mocap sample
        while self.initial_offsets[drone] is None and rclpy.ok():
            self.get_logger().warn(f"[{drone}] Waiting for initial offset...")
            time.sleep(0.1)

        path = self.file_paths[drone]
        while rclpy.ok():
            if not os.path.exists(path):
                self.get_logger().warn(f"[{drone}] SLAM file not found: {path}")
                return

            with open(path, 'r') as f:
                lines = f.readlines()

            for line in lines:
                data = line.strip().split()
                if len(data) != 8:
                    continue

                timestamp, x_s, y_s, z_s, qx_s, qy_s, qz_s, qw_s = map(float, data)

                # 2) SLAM→ROS axis conversion
                x_r =  z_s
                y_r = -x_s
                z_r = -y_s

                # # 3) add mocap origin (already in ROS axes)
                # offset = self.initial_offsets[drone]
                # shifted = np.array([x_r, y_r, z_r]) + offset

                # 3) normalize by first SLAM, then add mocap origin
                rotated = np.array([x_r, y_r, z_r])

                # --- record first SLAM sample as zero ---
                if self.slam_initials[drone] is None:
                    self.slam_initials[drone] = rotated.copy()
                    self.get_logger().info(
                         f"[{drone}] SLAM zero set to {self.slam_initials[drone]}"
                    )

                # now shift everything so that first→origin
                normalized = rotated - self.slam_initials[drone]
                shifted    = normalized + self.initial_offsets[drone]

                # 4) pack into a PoseStamped
                msg = PoseStamped()
                msg.header.frame_id = 'map'
                msg.header.stamp.sec = int(timestamp)
                msg.header.stamp.nanosec = int((timestamp - int(timestamp)) * 1e9)

                msg.pose.position.x = float(shifted[0])
                msg.pose.position.y = float(shifted[1])
                msg.pose.position.z = float(shifted[2])

                # 5) (optional) rotate the SLAM quaternion into ROS
                #    q_conv represents the same rotation R above
                q_conv = (0.0, 0.0, 0.0, np.pi)  # (x,y,z,w)
                qx, qy, qz, qw = self._quat_mul(q_conv, (qx_s, qy_s, qz_s, qw_s))

                msg.pose.orientation.x = qx
                msg.pose.orientation.y = qy
                msg.pose.orientation.z = qz
                msg.pose.orientation.w = qw

                # 6) publish exactly as before
                self.pose_publishers[drone].publish(msg)
                self.slam_paths[drone].header = msg.header
                self.slam_paths[drone].poses.append(msg)
                self.slam_path_publishers[drone].publish(self.slam_paths[drone])

                arrow = self.make_arrow_marker(drone, msg, namespace='slam', marker_id=0)
                self.slam_arrow_publishers[drone].publish(arrow)

        self.get_logger().info(f"[{drone}] Finished publishing SLAM path")

    def _quat_mul(self, q1, q2):
        # quaternion multiplication q1 * q2
        x1,y1,z1,w1 = q1
        x2,y2,z2,w2 = q2
        x =  w1*x2 + x1*w2 + y1*z2 - z1*y2
        y =  w1*y2 - x1*z2 + y1*w2 + z1*x2
        z =  w1*z2 + x1*y2 - y1*x2 + z1*w2
        w =  w1*w2 - x1*x2 - y1*y2 - z1*z2
        return x, y, z, w


    def make_arrow_marker(self, drone, pose_msg, namespace, marker_id):
        marker = Marker()
        marker.header = pose_msg.header
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = pose_msg.pose
        marker.scale.x = 0.3
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        if namespace == 'slam':
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 1.0
        else:
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 1.0
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = SlamPosePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
