#!/usr/bin/env python3
import sys, os, time, subprocess, cv2, numpy as np, torch
from PIL import Image
from tqdm import tqdm

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float32, Float32MultiArray, String
from builtin_interfaces.msg import Time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy


from skopt import gp_minimize
from skopt.space import Real
from skopt.callbacks import DeltaYStopper
from skopt.utils import use_named_args

from multiprocessing import shared_memory

from nighthawk_ros import score as scr


# Global constants for optimization and scoring
# OP_MAX_CALLS = 50
# OP_INITIAL_POINTS = 10
# OP_ACQ_FUNC = "EI"
# OP_NUM_JOBS = 6
# OP_STOPPING_DELTA = 0.01
# OP_STOPPING_NBEST = 10

# METER_INTEGRATION_TIME_MS = 200  # integration time in ms after settings update
# NIGHTHAWK_SCORE_THRESHOLD = 15   # percentage threshold for re-optimization
# NIGHTHAWK_REQUIRED_WARNINGS = 3
# NIGHTHAWK_CURRENT_WARNINGS = 0

# IMAGE_WIDTH = 640
# IMAGE_HEIGHT = 400


# Camera exposure space parameters (example values)
# SHUTTER_SPEEDS = [1000, 800, 640, 500, 400, 320, 250, 200, 160, 125]
# EXPOSURE_TIMES = [1/x * 1000 for x in SHUTTER_SPEEDS]
# OP_SEARCH_SPACE = [
    # Real(min(EXPOSURE_TIMES), max(EXPOSURE_TIMES), name="exposure_time_ms"),
    # Real(0, 1, name="light_intensity")
# ]

# Timer decorator (for logging duration)
def timer(func):
    def wrapper(*args, **kwargs):
        start = time.time()
        result = func(*args, **kwargs)
        end = time.time()
        print(f"{func.__name__} took {end - start:.2f} seconds")
        return result
    return wrapper

def compute_change(current_score, prev_optimal_score):
    """Compute the difference between current and optimal scores."""
    if current_score < prev_optimal_score:
        return prev_optimal_score - current_score
    else:
        return 0

def check_change(change, prev_optimal_score,score_threshold, current_warning_count, required_warnings):
    """
    Check if the change exceeds the threshold.
    Returns True if re-optimization is needed.
    """
    if change < (score_threshold / 100 * prev_optimal_score):
        return False, 0
    else:
        # Require N consecutive violations before switching
        if current_warning_count <= required_warnings:
            current_warning_count += 1
            return False, current_warning_count
        else:
            return True, current_warning_count

def get_optimal(res):
    """
    Extract the optimal exposure_time, LED intensity, and best score
    from the optimization result.
    """
    optimal_exposure_time = res.x[0]
    optimal_light_intensity = res.x[1]
    best_score = -res.fun  # because we minimized the negative score
    return optimal_exposure_time, optimal_light_intensity, best_score

# ------------------ NighthawkRos Node ------------------ #
class NighthawkRos(Node):
    def __init__(self):
        super().__init__("nighthawk_ros")

        self.OP_MAX_CALLS = self.declare_parameter("OP_MAX_CALLS", 25).value
        self.OP_INITIAL_POINTS = self.declare_parameter("OP_INITIAL_POINTS", 10).value
        self.OP_ACQ_FUNC = self.declare_parameter("OP_ACQ_FUNC", "EI").value
        self.OP_NUM_JOBS = self.declare_parameter("OP_NUM_JOBS", 6).value
        self.OP_STOPPING_DELTA = self.declare_parameter("OP_STOPPING_DELTA", 0.01).value
        self.OP_STOPPING_NBEST = self.declare_parameter("OP_STOPPING_NBEST", 10).value

        self.METER_INTEGRATION_TIME_MS = self.declare_parameter("METER_INTEGRATION_TIME_MS", 200).value
        self.NIGHTHAWK_SCORE_THRESHOLD = self.declare_parameter("NIGHTHAWK_SCORE_THRESHOLD", 15).value
        self.NIGHTHAWK_REQUIRED_WARNINGS = self.declare_parameter("NIGHTHAWK_REQUIRED_WARNINGS", 3).value
        self.NIGHTHAWK_CURRENT_WARNINGS = self.declare_parameter("NIGHTHAWK_CURRENT_WARNINGS", 0).value

        self.IMAGE_WIDTH = self.declare_parameter("IMAGE_WIDTH", 640).value
        self.IMAGE_HEIGHT = self.declare_parameter("IMAGE_HEIGHT", 400).value

        SHUTTER_SPEEDS = self.declare_parameter("SHUTTER_SPEEDS", [1000, 800, 125]).value
        self.EXPOSURE_TIMES = [1.0 / x * 1000 for x in SHUTTER_SPEEDS]  # Convert dynamically

        self.OP_SEARCH_SPACE = [
            Real(min(self.EXPOSURE_TIMES), max(self.EXPOSURE_TIMES), name="exposure_time_ms"),
            Real(0, 1, name="light_intensity")
        ]

        self.R2D2_CHECKPOINT = self.declare_parameter("R2D2_CHECKPOINT", "/home/spottop/yt-nighthawk/r2d2/models/faster2d2_WASF_N16.pt").value
        self.R2D2_GPU_INDICES = self.declare_parameter("R2D2_GPU_INDICES", [0]).value

        self.SHARED_MEMORY_NAME = self.declare_parameter("SHARED_MEMORY_NAME", "shared_img").value
        self.IMAGE_SHAPE = tuple(self.declare_parameter("IMAGE_SHAPE", [1200, 1920, 3]).value)
        self.IMAGE_DTYPE = np.uint8  # Assuming uint8, as retrieved types are typically strings

        # Define a QoS profile with durability set to transient local
        latched_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,  # Keep only the last message
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        # Publisher to send optimal settings as [LED intensity, exposure_time, score]
        self.optimal_pub = self.create_publisher(Float32MultiArray, "nighthawk/optimal", latched_qos)
        # Publisher for the current state ("NORMAL" or "OPTIMIZING")
        self.state_pub = self.create_publisher(String, "nighthawk/state", latched_qos)
        # Subscriber to receive current score (in NORMAL mode only)
        self.score_sub = self.create_subscription(Float32, "nighthawk/score", self.score_callback, 10)
        self.current_score = None


        # Starting state with optimizing and publishing (first state)
        self.state = "OPTIMIZING"
        state_msg = String()
        state_msg.data = self.state
        self.state_pub.publish(state_msg)

        # Setup R2D2 model for scoring
        self.R2D2_MODEL, self.R2D2_DEVICE = scr.setup_r2d2(checkpoint_path=self.R2D2_CHECKPOINT, gpu_indices=self.R2D2_GPU_INDICES)

        # Create an LED publisher (if not already created)
        self.led_pub = self.create_publisher(Float32, "/led_driver/value", latched_qos)


        CONTROL_LOOP_TIMER = self.declare_parameter("CONTROL_LOOP_TIMER", 0.5).value

        # Main control loop timer (runs every 2 seconds; adjust as needed)
        self.timer = self.create_timer(CONTROL_LOOP_TIMER, self.main_loop)
        self.get_logger().info("NighthawkRos node initialized in state: " + self.state)


        self.current_warning_count = 0

        self.optimization_count = 0
        self.prev_optimal_score = None
        self.prev_optimal_P = None
        self.prev_optimal_delta_t = None

    def score_callback(self, msg: Float32):
        """Callback to update the current score (NORMAL state)."""
        self.current_score = msg.data
        self.get_logger().info(f"Received score: {self.current_score}")

    def set_exposure(self, exposure_time_ms):
        """Set the camera exposure via a ROS parameter call (using subprocess)."""
        try:
            subprocess.run(
                ["ros2", "param", "set", "/flir_camera", "exposure_time", str(int(exposure_time_ms * 1000))],
                check=True
            )
            self.get_logger().info(f"Set exposure_time to {exposure_time_ms * 1000} µs")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Failed to set exposure_time: {str(e)}")

    def update_settings(self, exposure_time_ms, light_intensity):
        """Apply the settings by updating exposure and LED intensity."""
        self.get_logger().info(f"Updating settings: exposure_time={exposure_time_ms}, LED intensity={light_intensity}")
        self.set_exposure(exposure_time_ms)
        led_msg = Float32(data=light_intensity)
        self.led_pub.publish(led_msg)

    def get_image_from_shared_memory(self):
        """
        Retrieve the image from shared memory.
        Assumes shared memory name is "shared_img" and that image shape/type are known.
        """
        try:
            shm = shared_memory.SharedMemory(name=self.SHARED_MEMORY_NAME, create=False)
            image_shape = self.IMAGE_SHAPE
            image_dtype = self.IMAGE_DTYPE
            image = np.ndarray(image_shape, dtype=image_dtype, buffer=shm.buf).copy()  # Copy to avoid dependency on shared memory

            # Explicitly close and unlink shared memory
            shm.close()
            return image
        except Exception as e:
            self.get_logger().error("Failed to access shared memory: " + str(e))
            return None

    @timer
    def optimize_nighthawk(self):
        """
        Run the optimization routine using skopt.gp_minimize.
        This method uses the shared memory image to compute the score, applies settings,
        and returns the optimal exposure, LED intensity, and best score.
        """
        self.get_logger().info("Starting optimization...")

        # TODO : Pubslish iteration num

        iteration = 0

        @use_named_args(self.OP_SEARCH_SPACE)
        def objective(exposure_time_ms, light_intensity):
            nonlocal iteration
            self.get_logger().info(f"Optimization number {self.optimization_count} and iteration {iteration}: exposure={exposure_time_ms}, LED={light_intensity}")
            self.update_settings(exposure_time_ms, light_intensity)
            time.sleep(self.METER_INTEGRATION_TIME_MS / 1000.0)  # Wait for settings to take effect
            image = self.get_image_from_shared_memory()
            if image is None:
                self.get_logger().error("No image retrieved from shared memory during optimization.")
                return 0.0
            score = scr.compute_score(image, self.R2D2_MODEL, self.R2D2_DEVICE, resize=(self.IMAGE_WIDTH,self.IMAGE_HEIGHT))
            # self.get_logger().info(f"Score: {score} for exposure={exposure_time_ms}, LED={light_intensity}")
            self.get_logger().info(f"\033[31mScore: {score}\033[0m for \033[32mexposure={exposure_time_ms}\033[0m, \033[34mLED={light_intensity}\033[0m")
            iteration += 1
            return -score  # Negative score for minimization

        stopper = DeltaYStopper(delta=self.OP_STOPPING_DELTA, n_best=self.OP_STOPPING_NBEST)
        res = gp_minimize(objective,
                          self.OP_SEARCH_SPACE,
                          n_calls=self.OP_MAX_CALLS,
                          n_initial_points=self.OP_INITIAL_POINTS,
                          n_jobs=self.OP_NUM_JOBS,
                          acq_func=self.OP_ACQ_FUNC,
                          callback=[stopper])
        optimal_exposure, optimal_led, optimal_score = get_optimal(res)
        self.get_logger().info(f"Optimization converged: exposure={optimal_exposure}, LED={optimal_led}, score={optimal_score}")
        return optimal_exposure, optimal_led, optimal_score

    def main_loop(self):
        """
        Main control loop that alternates between two states:
          - In OPTIMIZING: run the optimizer (using shared memory image) and update optimal parameters.
          - In NORMAL: use the externally published score (from the score topic) to decide if re-optimization is needed.
        In both states, the node publishes the current optimal LED intensity, exposure time, score, and state.
        """
        opt_msg = Float32MultiArray()
        state_msg = String()
        if self.state == "OPTIMIZING":
            self.optimization_count += 1
            self.current_warning_count = 0 # Reset warning counter
            self.get_logger().info("State: OPTIMIZING")
            optimal_exposure, optimal_led, optimal_score = self.optimize_nighthawk()

            # Apply the optimal settings
            self.update_settings(optimal_exposure, optimal_led)
            self.prev_optimal_score = optimal_score
            self.prev_optimal_P = optimal_led
            self.prev_optimal_delta_t = optimal_exposure
            self.state = "NORMAL"

            # Publishing data
            opt_msg.data = [
                float(self.prev_optimal_P),
                float(self.prev_optimal_delta_t),
                float(self.prev_optimal_score),
                float(self.optimization_count)
            ]
            self.optimal_pub.publish(opt_msg)
            state_msg.data = self.state
            self.state_pub.publish(state_msg)
            self.get_logger().info(f"Publishing: P={self.prev_optimal_P}, delta_t={self.prev_optimal_delta_t}, Score={self.prev_optimal_score}, State={self.state}")

            # Adding delay for old settings to take effect
            time.sleep(self.METER_INTEGRATION_TIME_MS / 1000.0)

        elif self.state == "NORMAL":
            self.get_logger().info("\033[34mState: NORMAL\033[0m")

            if self.current_score is not None:
                change = compute_change(self.current_score, self.prev_optimal_score)

                self.get_logger().info(
                    f"\033[33mChecking score...\033[0m \n"
                    f"  🔹 Current Score: \033[31m{self.current_score}\033[0m \n"
                    f"  🔹 Optimal Score: \033[32m{self.prev_optimal_score}\033[0m \n"
                    f"  🔹 Change: \033[35m{change}\033[0m \n"
                    f"  🔹 Change Count: \033[31m{self.current_warning_count}\033[0m"
                )

                change_bool, change_count  = check_change(change, self.prev_optimal_score,self.NIGHTHAWK_SCORE_THRESHOLD, self.current_warning_count,self.NIGHTHAWK_REQUIRED_WARNINGS)
                self.current_warning_count = change_count
                if change_bool:
                    self.get_logger().info("\033[31mDetected significant score deviation. Switching to OPTIMIZING state.\033[0m")
                    self.state = "OPTIMIZING"

                
                # Publishing data
                opt_msg.data = [
                    float(self.prev_optimal_P),
                    float(self.prev_optimal_delta_t),
                    float(self.prev_optimal_score),
                    float(self.optimization_count)
                ]
                self.optimal_pub.publish(opt_msg)
                state_msg.data = self.state
                self.state_pub.publish(state_msg)
                self.get_logger().info(f"Publishing: P={self.prev_optimal_P}, delta_t={self.prev_optimal_delta_t}, Score={self.prev_optimal_score}, State={self.state}")


            else:
                self.get_logger().warn("\033[33m⚠ No current score received yet in NORMAL state.\033[0m")


def main(args=None):
    rclpy.init(args=args)
    node = NighthawkRos()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received. Shutting down gracefully...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        try:
            shm = shared_memory.SharedMemory(name="shared_img", create=False)
            shm.close()
        except FileNotFoundError:
            pass

if __name__ == "__main__":
    main()