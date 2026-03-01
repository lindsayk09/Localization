import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from mocap4r2_msgs.msg import RigidBodies
from filterpy.kalman import ExtendedKalmanFilter
import numpy as np
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import math

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('localization_node')

        # Publishers
        self.publisher = self.create_publisher(Odometry, '/odom', 10)
    
        # Subscriber
        self.subscription = self.create_subscription(RigidBodies, '/pose_modelcars', self.pose_callback, 10)

        # Timer
        self.timer = self.create_timer(1/30, self.publish_odom)

        # EKF Setup
        self.dt = 1/30
        self.ekf = ExtendedKalmanFilter(dim_x=6, dim_z=3)
        self.ekf.x = np.zeros(6)
        self.ekf.P = np.eye(6) * 0.1
        self.ekf.Q = np.eye(6) * 0.01
        self.ekf.R = np.eye(3) * 0.05

        self.prev_pose = None

    def hx(self, x):
        """ Measurement function: maps EKF state to expected sensor reading. """
        return np.array([x[0], x[1], x[4]]) # x, y, yaw

    def pose_callback(self, msg):
        for rigidbody in msg.rigidbodies:
            if rigidbody.rigid_body_name == '11':
                new_x = rigidbody.pose.position.x
                new_y = rigidbody.pose.position.y
                quat = [0.0, 0.0, rigidbody.pose.orientation.z, rigidbody.pose.orientation.w]
                _, _, new_theta = euler_from_quaternion(quat)

                if self.prev_pose is not None:
                    measurement = np.array([new_x, new_y, new_theta])

                    # EKF prediction
                    self.ekf.F = np.array([
                        [1, 0, self.dt, 0, 0, 0],
                        [0, 1, 0, self.dt, 0, 0],
                        [0, 0, 1, 0, 0, 0],
                        [0, 0, 0, 1, 0, 0],
                        [0, 0, 0, 0, 1, self.dt],
                        [0, 0, 0, 0, 0, 1],
                    ])
                    self.ekf.predict()

                    H = np.array([
                        [1, 0, 0, 0, 0, 0],
                        [0, 1, 0, 0, 0, 0],
                        [0, 0, 0, 0, 1, 0],
                    ])

                    self.ekf.update(measurement, HJacobian=lambda x: H, Hx=self.hx)

                self.prev_pose = np.array([new_x, new_y, new_theta])

    def publish_odom(self):
        x=self.ekf.x
        
        # Odometry message
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.child_frame_id = "car_11/base_link"

        msg.pose.pose.position.x = x[0]
        msg.pose.pose.position.y = x[1]
        msg.pose.pose.position.z = 0.0  

        quat = quaternion_from_euler(0.0, 0.0, x[4])  # [x, y, z, w]
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]

        msg.twist.twist.linear.x = x[2]
        msg.twist.twist.linear.y = x[3]
        msg.twist.twist.angular.z = x[5]

        self.publisher.publish(msg)
        self.get_logger().info(f'Position -> x: {x[0]:.2f}, y: {x[1]:.2f}, yaw: {math.degrees(x[4])} | Velocity -> vx: {x[2]:.2f}, vy: {x[3]:.2f} | Turn Rate: {x[5]:.2f}')

def main():
    rclpy.init()
    node = OdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
