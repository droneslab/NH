from typing import Any, Optional, Union
import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rcl_interfaces.msg import (
    FloatingPointRange,
    IntegerRange,
    ParameterDescriptor,
    SetParametersResult
)
from std_msgs.msg import Int32, Float32
from lifecycle_msgs.msg import Transition

import serial
import time
import sys

from rclpy.qos import QoSProfile, DurabilityPolicy


class PWMDriver:
    
    @staticmethod
    def connect_serial(serial_port, baud_rate=9600):
        """
        Establishes a serial connection to the microcontroller.
        """
        try:
            ser = serial.Serial(serial_port, baud_rate, timeout=1)
            print(f"Connected to {serial_port} at {baud_rate} baud.")
            return ser
        except serial.SerialException as e:
            print(f"Error: {e}")
            return None

    def __init__(self, port, baud, min_power=0, max_power=255):
        self.port = port
        self.baud = baud
        self._MIN_POWER = min_power
        self._MAX_POWER = max_power
        self._serial_connection = PWMDriver.connect_serial(self.port, self.baud)
        self._intensity = None
        
    def get_serial(self):
        return self._serial_connection

    @staticmethod
    def send_value(serial_conn, value):
        if serial_conn and serial_conn.is_open:
            try:
                serial_conn.write(f"{value}\n".encode('utf-8'))
            except serial.SerialException as e:
                print(f"Error writing to serial: {e}")
        else:
            print("Serial connection is not open.")

    @property
    def intensity(self):
        return self._intensity
    
    @intensity.setter
    def intensity(self, intensity: float = 0):
        self._intensity = int(self._MIN_POWER + (self._MAX_POWER - self._MIN_POWER) * intensity)
        PWMDriver.send_value(self._serial_connection, self._intensity)
        time.sleep(0)

    def stop(self):
        self._intensity = 0
        PWMDriver.send_value(self._serial_connection, self._intensity)


class LightControlRos(LifecycleNode):

    def __init__(self):
        super().__init__("light_control_ros")

        self.METER_INTEGRATION_TIME_MS = self.declare_parameter("METER_INTEGRATION_TIME_MS", 200).value

        self.qos = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE)

        self.auto_reconfigurable_params = []
        self.parameters_callback = None
        self.subscriber = None
        self.publisher = None
        self.led_subscriber = None

        self._active = False

        # Declare parameters (to be used during configuration)
        self.port = self.declareAndLoadParameter(
            name="led_port",
            param_type=rclpy.Parameter.Type.STRING,
            description="Serial port for LED light",
            default="/dev/ttyACM1"
        )
        self.baud = self.declareAndLoadParameter(
            name="led_baud",
            param_type=rclpy.Parameter.Type.INTEGER,
            description="Baud rate for LED light",
            default=115200
        )

        self.led = PWMDriver(self.port, self.baud)
        self.get_logger().info("LED Driver Initialized.")
        self.led.stop()  # Ensure LED starts turned off

    def declareAndLoadParameter(self,
                                name: str,
                                param_type: rclpy.Parameter.Type,
                                description: str,
                                default: Optional[Any] = None,
                                add_to_auto_reconfigurable_params: bool = True,
                                is_required: bool = False,
                                read_only: bool = False,
                                from_value: Optional[Union[int, float]] = None,
                                to_value: Optional[Union[int, float]] = None,
                                step_value: Optional[Union[int, float]] = None,
                                additional_constraints: str = "") -> Any:
        """Declares and loads a ROS parameter"""
        param_desc = ParameterDescriptor()
        param_desc.description = description
        param_desc.additional_constraints = additional_constraints
        param_desc.read_only = read_only
        if from_value is not None and to_value is not None:
            if param_type == rclpy.Parameter.Type.INTEGER:
                step_value = step_value if step_value is not None else 1
                param_desc.integer_range = [IntegerRange(from_value=from_value, to_value=to_value, step=step_value)]
            elif param_type == rclpy.Parameter.Type.DOUBLE:
                step_value = step_value if step_value is not None else 1.0
                param_desc.floating_point_range = [FloatingPointRange(from_value=from_value, to_value=to_value, step=step_value)]
            else:
                self.get_logger().warn(f"Parameter type of parameter '{name}' does not support specifying a range")
        # Use positional default value instead of default_value keyword
        self.declare_parameter(name, default, descriptor=param_desc)

        try:
            param = self.get_parameter(name).value
            self.get_logger().info(f"Loaded parameter '{name}': {param}")
        except rclpy.exceptions.ParameterUninitializedException:
            if is_required:
                self.get_logger().fatal(f"Missing required parameter '{name}', exiting")
                raise SystemExit(1)
            else:
                self.get_logger().warn(f"Missing parameter '{name}', using default value: {default}")
                param = default
                self.set_parameters([rclpy.Parameter(name=name, value=param)])

        if add_to_auto_reconfigurable_params:
            self.auto_reconfigurable_params.append(name)

        return param

    def parametersCallback(self, parameters: rclpy.Parameter) -> SetParametersResult:
        """Handles reconfiguration when a parameter value is changed"""
        for param in parameters:
            if param.name in self.auto_reconfigurable_params:
                setattr(self, param.name, param.value)
                self.get_logger().info(f"Reconfigured parameter '{param.name}' to: {param.value}")
        result = SetParametersResult()
        result.successful = True
        return result

    def setup(self):
        """Sets up subscriptions, publishers, etc."""
        self.parameters_callback = self.add_on_set_parameters_callback(self.parametersCallback)

        self.subscriber = self.create_subscription(
            Int32,
            "~/input",
            self.topicCallback,
            qos_profile=10
        )
        self.get_logger().info(f"Subscribed to '{self.subscriber.topic_name}'")

        self.publisher = self.create_publisher(
            Int32,
            "~/output",
            qos_profile=10
        )
        self.get_logger().info(f"Publishing to '{self.publisher.topic_name}'")

        self.led_subscriber = self.create_subscription(
            Float32,
            'led_driver/value',
            self.led_callback,
            self.qos
        )
        self.get_logger().info("Subscribed to 'led_driver/value'")

    def topicCallback(self, msg: Int32):
        """Processes messages received by a subscriber"""
        self.get_logger().info(f"Message received: '{msg.data}'")

    def led_callback(self, msg: Float32):
        """Handles LED intensity updates"""
        intensity = msg.data
        if 0.0 <= intensity <= 1.0:
            if self._active:
                self.get_logger().info(f"Setting LED intensity to {intensity}")
                self.led.intensity = intensity
                time.sleep(self.METER_INTEGRATION_TIME_MS / 1000)
            else:
                self.get_logger().info(f"Node is set to {self._active}")
            
        else:
            self.get_logger().warn("Received invalid LED intensity. Must be between 0 and 1.")

    # --- Lifecycle Callbacks ---

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Configuring LightControlRos node")
        self.setup()
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Activating LightControlRos node")
        # If you use a true lifecycle publisher, activate it here.
        self._active = True
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Deactivating LightControlRos node")
        # Deactivate any publishers if needed.
        self.led.stop()
        self._active = False
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Cleaning up LightControlRos node")
        if self.subscriber:
            self.destroy_subscription(self.subscriber)
            self.subscriber = None
        if self.led_subscriber:
            self.destroy_subscription(self.led_subscriber)
            self.led_subscriber = None
        if self.publisher:
            self.destroy_publisher(self.publisher)
            self.publisher = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Shutting down LightControlRos node")
        self.led.stop()
        return TransitionCallbackReturn.SUCCESS


def main(args=None):
    rclpy.init(args=args)
    node = LightControlRos()

    # Simulate lifecycle transitions manually:
    node.on_configure(None)
    node.on_activate(None)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_deactivate(None)
        node.on_cleanup(None)
        node.on_shutdown(None)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()