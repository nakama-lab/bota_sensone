import ctypes
import struct
from collections.abc import Callable
from typing import TYPE_CHECKING, NamedTuple

import pysoem
import rclpy
import rclpy.qos
from geometry_msgs.msg import WrenchStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from sensor_msgs.msg import Imu, Temperature
from std_msgs.msg import Header

if TYPE_CHECKING:
    from rclpy.time import Time
    from rclpy.timer import Timer


class SlaveSet(NamedTuple):
    """Entry for EtherCAT slave mapping."""

    slave_name: str
    """Name of this slave."""

    product_code: int
    """Product code of this slave."""

    config_func: Callable[[int], None]
    """Function to call to set up this slave. Takes one integer argument as the slave position."""


class BotaEthercatNode(Node):
    """EtherCAT SensONE Force Sensor node."""

    # Constants
    BOTA_VENDOR_ID = 0xB07A
    BOTA_PRODUCT_CODE = 0x00000001

    # Parameter default values
    DEFAULT_SINC_LENGTH = 32
    # DEFAULT_FIR_DISABLED = False
    # DEFAULT_FAST_ENABLED = False
    # DEFAULT_CHOP_ENABLED = False

    def __init__(self) -> None:
        """Initialize the EtherCAT SensONE Force Sensor node."""
        super().__init__("bota_sensor_node")

        # Declare all ROS connections
        self._declare_all_parameters()
        self._declare_all_publishers()

        # Set up a timer to trigger the loop
        # The timer is started in the start() method
        self.sampling_rate = 0.0
        self.loop_timer: Timer | None = None

        # EtherCAT objects
        self._master = pysoem.Master()
        self._expected_slave_mapping = {
            0: SlaveSet("BFT-MEDS-ECAT-M8", self.BOTA_PRODUCT_CODE, self._bota_sensor_setup)
        }

        # Node is initialized
        self.get_logger().info("Bota Sensor Node initialized and connected")

    # --------------- EtherCAT connections ---------------

    def connect(self) -> None:
        """Connect to the EtherCAT device."""
        self._master.open(self.eth_interface_name_param.get_parameter_value().string_value)

        if self._master.config_init() == 0:
            err_msg = "EtherCAT slaves not found"
            self.get_logger().error(err_msg)
            raise ConnectionError(err_msg)

        for i, slave in enumerate(self._master.slaves):
            if slave.man != self.BOTA_VENDOR_ID:
                err_msg = f"Wrong vendor ID (got {slave.man}, expected {self.BOTA_VENDOR_ID})"
                self.get_logger().error(err_msg)
                raise ConnectionError(err_msg)
            if slave.id != self._expected_slave_mapping[i].product_code:
                err_msg = (
                    f"Wrong product code (got {slave.id}, expected {self._expected_slave_mapping[i].product_code})"
                )
                self.get_logger().error(err_msg)
                raise ConnectionError(err_msg)

            slave.config_func = self._expected_slave_mapping[i].config_func

        self._master.config_map()

        if self._master.state_check(pysoem.SAFEOP_STATE, 50000) != pysoem.SAFEOP_STATE:
            err_msg = "Not all slaves reached SAFEOP state"
            self.get_logger().error(err_msg)
            raise ConnectionError(err_msg)

        self._master.state = pysoem.OP_STATE
        self._master.write_state()

        if self._master.state_check(pysoem.OP_STATE, 50000) != pysoem.OP_STATE:
            err_msg = "Not all slaves reached OP state"
            self.get_logger().error(err_msg)
            raise ConnectionError(err_msg)

        # The sampling rate should be set now
        if not self.sampling_rate > 0.0:
            err_msg = "Sensor is connected but sampling rate was not read!"
            self.get_logger().error(err_msg)
            raise RuntimeError(err_msg)

        # Start the timer
        self.loop_timer = self.create_timer(
            timer_period_sec=1.0 / self.sampling_rate,
            callback=self._loop_once,
        )
        self.get_logger().info(f"Loop started: reading Bota SensONE Ethercat at {self.sampling_rate} Hz")

    def disconnect(self) -> None:
        """Close the EtherCAT master connection."""
        self._master.state = pysoem.INIT_STATE
        self._master.write_state()
        self._master.close()

    # --------------- ROS connections ---------------

    def _declare_all_parameters(self) -> None:
        """Declare all parameters for the node."""
        # [REQUIRED] Ethernet interface parameter
        self.eth_interface_name_param = self.declare_parameter(
            "eth_interface_name",
            "",
            descriptor=ParameterDescriptor(
                description="[REQUIRED] Ethernet interface name to connect to EtherCAT.",
                read_only=True,  # Can only be set on startup
            ),
        )
        if self.eth_interface_name_param.get_parameter_value().string_value == "":  # Validate that param was set
            msg = "Missing required parameter: 'eth_interface_name'"
            self.get_logger().error(msg)
            raise RuntimeError(msg)

        # SINC filter length
        self.sinc_length_param = self.declare_parameter(
            "sinc_length",
            self.DEFAULT_SINC_LENGTH,
            descriptor=ParameterDescriptor(
                description="SINC filter length. Consult Table 5 of Bota SensONE User Manual for valid values.",
                read_only=True,  # Can only be set on startup
            ),
        )

    def _declare_all_publishers(self) -> None:
        """Declare all publishers."""
        self.publisher_wrench = self.create_publisher(
            msg_type=WrenchStamped,
            topic="~/wrench",
            qos_profile=10,
            # qos_profile=rclpy.qos.qos_profile_sensor_data,
        )

        self.publisher_imu = self.create_publisher(
            msg_type=Imu,
            topic="~/imu",
            qos_profile=10,
            # qos_profile=rclpy.qos.qos_profile_sensor_data,
        )

        self.publisher_temperature = self.create_publisher(
            msg_type=Temperature,
            topic="~/temperature",
            qos_profile=10,
            # qos_profile=rclpy.qos.qos_profile_sensor_data,
        )

    # --------------- Bota configuration ---------------

    def _bota_sensor_setup(self, slave_pos: int) -> None:
        """Set up the Bota SensONE sensor.

        This function should be passed to the slave mapping in the master configuration step.
        """
        slave = self._master.slaves[slave_pos]

        slave.sdo_write(0x8010, 1, bytes(ctypes.c_uint8(1)))  # calibration matrix active
        slave.sdo_write(0x8010, 2, bytes(ctypes.c_uint8(0)))  # temperature compensation
        slave.sdo_write(0x8010, 3, bytes(ctypes.c_uint8(0)))  # IMU active

        # Filter parameters
        slave.sdo_write(0x8006, 2, bytes(ctypes.c_uint8(1)))  # FIR disable
        slave.sdo_write(0x8006, 3, bytes(ctypes.c_uint8(0)))  # FAST enable
        slave.sdo_write(0x8006, 4, bytes(ctypes.c_uint8(0)))  # CHOP enable

        # Sinc filter size (configured with ROS parameter)
        slave.sdo_write(0x8006, 1, bytes(ctypes.c_uint16(self.sinc_length_param.get_parameter_value().integer_value)))

        # Obtain the rate that the sensor is producing results in and match that rate for this node
        sampling_rate: int = struct.unpack("h", slave.sdo_read(0x8011, 0))[0]
        self.sampling_rate = sampling_rate

    # --------------- Loop ---------------

    def _loop_once(self) -> None:
        """One loop of the data reading."""
        processdata_timeout_us = 1_000_000 / self.sampling_rate  # The data can at most be 1 sample late
        self._master.send_processdata()
        self._master.receive_processdata(timeout=processdata_timeout_us)

        # Get the current time right after receiving the new message
        now: Time = self.get_clock().now()

        sensor_input_as_bytes = self._master.slaves[0].input
        status = struct.unpack_from("B", sensor_input_as_bytes, 0)[0]
        warningsErrorsFatals = struct.unpack_from("I", sensor_input_as_bytes, 1)[0]

        # WrenchStamped
        wrench_stamped = WrenchStamped()
        wrench_stamped.header = Header()
        wrench_stamped.header.stamp = now.to_msg()
        wrench_stamped.header.frame_id = "wrench_sensor_frame"  # Change if needed
        wrench_stamped.wrench.force.x = struct.unpack_from("f", sensor_input_as_bytes, 5)[0]
        wrench_stamped.wrench.force.y = struct.unpack_from("f", sensor_input_as_bytes, 9)[0]
        wrench_stamped.wrench.force.z = struct.unpack_from("f", sensor_input_as_bytes, 13)[0]
        wrench_stamped.wrench.torque.x = struct.unpack_from("f", sensor_input_as_bytes, 17)[0]
        wrench_stamped.wrench.torque.y = struct.unpack_from("f", sensor_input_as_bytes, 21)[0]
        wrench_stamped.wrench.torque.z = struct.unpack_from("f", sensor_input_as_bytes, 25)[0]
        self.publisher_wrench.publish(wrench_stamped)

        # # IMU
        # imu_data = Imu()
        # imu_data.angular_velocity.x = struct.unpack_from("f", sensor_input_as_bytes, 44)[0]
        # imu_data.angular_velocity.y = struct.unpack_from("f", sensor_input_as_bytes, 48)[0]
        # imu_data.angular_velocity.z = struct.unpack_from("f", sensor_input_as_bytes, 52)[0]
        # imu_data.linear_acceleration.x = struct.unpack_from("f", sensor_input_as_bytes, 31)[0]
        # imu_data.linear_acceleration.y = struct.unpack_from("f", sensor_input_as_bytes, 35)[0]
        # imu_data.linear_acceleration.z = struct.unpack_from("f", sensor_input_as_bytes, 39)[0]
        # self.publisher_imu.publish(imu_data)

        # # Temperature
        # temperature_data = Temperature()
        # temperature_data.temperature = struct.unpack_from("f", sensor_input_as_bytes, 57)[0]
        # self.publisher_temperature.publish(temperature_data)


def main(args: list[str] | None = None) -> None:
    """Start the node."""
    # Initialize ROS
    rclpy.init(args=args)

    # Initialize the node
    node = BotaEthercatNode()
    node.connect()  # Connect the node to the sensor
    try:
        # Run this node until ROS closes or the program is interrupted
        rclpy.spin(node)

    except KeyboardInterrupt:
        # Gracefully shut down when a Ctrl+C is pressed.
        # Other errors are shown with a backtrace in the terminal.
        pass

    finally:
        # Clean up the node
        node.disconnect()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
