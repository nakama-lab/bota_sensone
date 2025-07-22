import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, WrenchStamped
from sensor_msgs.msg import Imu
from openpyxl import Workbook
from datetime import datetime
from threading import Lock

class DataRecorder(Node):
    def __init__(self):
        super().__init__('data_recorder_node')
        self.get_logger().info("Starting DataRecorder node...")

        # Subscriptions
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/franka_robot_state_broadcaster/current_pose',
            self.pose_callback,
            10
        )
        self.imu_sub = self.create_subscription(
            Imu,
            '/Senseone_eth/imu',
            self.imu_callback,
            10
        )
        self.wrench_sub = self.create_subscription(
            WrenchStamped,
            '/Senseone_eth/wrench',
            self.wrench_callback,
            10
        )

        # Latest data
        self.latest_pose = None
        self.latest_imu = None
        self.latest_wrench = None
        self.lock = Lock()
        self.data_ready = False

        # Excel setup
        self.filename = f"robot_sensor_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.xlsx"
        self.wb = Workbook()
        self.ws = self.wb.active
        self.ws.title = "Sensor Data"
        self.ws.append([
            "Timestamp",
            "Pose_X", "Pose_Y", "Pose_Z",
            "Pose_QX", "Pose_QY", "Pose_QZ", "Pose_QW",
            "IMU_Orientation_X", "IMU_Orientation_Y", "IMU_Orientation_Z", "IMU_Orientation_W",
            "IMU_Angular_X", "IMU_Angular_Y", "IMU_Angular_Z",
            "IMU_Linear_X", "IMU_Linear_Y", "IMU_Linear_Z",
            "Wrench_Force_X", "Wrench_Force_Y", "Wrench_Force_Z",
            "Wrench_Torque_X", "Wrench_Torque_Y", "Wrench_Torque_Z"
        ])

        # Timer at 10Hz (0.1s interval)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Subscriptions and timer created. Ready to record.")

    def pose_callback(self, msg):
        with self.lock:
            self.latest_pose = msg
        self.get_logger().debug("Pose received")

    def imu_callback(self, msg):
        with self.lock:
            self.latest_imu = msg
        self.get_logger().debug("IMU received")

    def wrench_callback(self, msg):
        with self.lock:
            self.latest_wrench = msg
        self.get_logger().debug("Wrench received")

    def timer_callback(self):
        with self.lock:
            missing_topics = []

            if not self.latest_pose:
                missing_topics.append("/franka_robot_state_broadcaster/current_pose")
            if not self.latest_imu:
                missing_topics.append("/Senseone_eth/imu")
            if not self.latest_wrench:
                missing_topics.append("/Senseone_eth/wrench")

            if missing_topics:
                self.get_logger().warn(
                    f"Waiting for all sensor data... Missing: {', '.join(missing_topics)}"
                )
                return

            self.data_ready = True

            # Use ROS time
            now = self.get_clock().now().to_msg()
            timestamp_str = f"{now.sec}.{now.nanosec:09d}"

            row = [timestamp_str]

            # Pose
            pose = self.latest_pose.pose
            row += [pose.position.x, pose.position.y, pose.position.z]
            row += [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

            # IMU
            imu = self.latest_imu
            row += [imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w]
            row += [imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z]
            row += [imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z]

            # Wrench
            wrench = self.latest_wrench.wrench
            row += [wrench.force.x, wrench.force.y, wrench.force.z]
            row += [wrench.torque.x, wrench.torque.y, wrench.torque.z]

            self.ws.append(row)
            self.get_logger().info(f"Data recorded at {timestamp_str}")

def main(args=None):
    rclpy.init(args=args)
    node = DataRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received, shutting down...")
    finally:
        node.wb.save(node.filename)
        node.get_logger().info(f"Excel file saved to {node.filename}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
