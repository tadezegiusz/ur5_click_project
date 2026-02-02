#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import cv2
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class ClickUR5Node(Node):
    def __init__(self):
        super().__init__('ur5_click_node')

        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint',
        ]

        self.q = None
        self.step = 0.3
        self.click_direction = None

        self.joint_states_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_states_callback, 10
        )

        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )

        self.window_name = 'control_ur'
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 640, 480)

        self.ui_timer = self.create_timer(0.05, self.ui_tick)

        self.get_logger().info('UR5 Click Node started')

    def joint_states_callback(self, msg):
        if self.q is not None:
            return
        q = [0.0] * 6
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                q[i] = msg.position[msg.name.index(name)]
        self.q = q
        self.get_logger().warning(f'[INIT] q = {[round(v, 3) for v in self.q]}')

    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.click_direction = 1.0 if y < 240 else -1.0
            self.get_logger().warning(f'[CLICK] {"up" if y < 240 else "down"} | y={y}')

    def ui_tick(self):
        if self.click_direction is not None and self.q is not None:
            self.q[1] += self.click_direction * self.step
            self.q[1] = max(min(self.q[1], 0.0), -2.3)
            self.click_direction = None
            self.send_trajectory(self.q)
            self.get_logger().warning(f'[MOVE] q[1] = {self.q[1]:.3f}')

        img = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.line(img, (0, 240), (640, 240), (0, 255, 255), 2)

        cv2.imshow(self.window_name, img)
        cv2.waitKey(1)
        cv2.setMouseCallback(self.window_name, self.on_mouse)

    def send_trajectory(self, q):
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = self.joint_names

        pt = JointTrajectoryPoint()
        pt.positions = list(q)
        pt.velocities = [0.0] * 6
        pt.time_from_start = Duration(sec=2, nanosec=0)
        traj.points = [pt]

        self.traj_pub.publish(traj)
        self.get_logger().warning(f'[PUB] {[round(v, 3) for v in q]}')

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ClickUR5Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
