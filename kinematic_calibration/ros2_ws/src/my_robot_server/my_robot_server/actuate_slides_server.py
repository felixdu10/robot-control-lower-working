import time

import rclpy
import serial
from rclpy.node import Node
from rclpy.action import ActionServer

from my_robot_interfaces.action import ActuateSlides


class ActuateSlidesServer(Node):

    def __init__(self):
        super().__init__('actuate_slides_server')
        self.get_logger().info("### NEW SERVER VERSION WITH HOME_LOWER + Y/A ONLY ###")

        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

        self.ser.setDTR(False)
        time.sleep(0.1)
        self.ser.setDTR(True)

        time.sleep(2)

        self._action_server = ActionServer(
            self,
            ActuateSlides,
            'actuate_slides',
            self.execute_callback
        )

        self.current_position = None
        self.ready = False
        self.create_timer(0.1, self.check_ready)

    def check_ready(self):
        if self.ready:
            return

        if self.ser.in_waiting:
            line = self.ser.readline().decode(errors='replace').strip()
            if line:
                self.get_logger().info(f"Arduino boot: {line}")

                if line == "READY":
                    self.get_logger().info("Arduino is READY")
                    self.ready = True

    def _read_serial_responses(
        self, goal_handle, target_positions, feedback_msg, timeout_s=300.0
    ):
        """Read lines until DONE. Sleeps briefly when no bytes are waiting to avoid busy-waiting."""
        deadline = time.monotonic() + timeout_s
        while True:
            if time.monotonic() > deadline:
                self.get_logger().error(
                    'Timed out waiting for Arduino DONE (possible disconnect or crash).'
                )
                return False

            if self.ser.in_waiting:
                response = self.ser.readline().decode(errors='replace').strip()
                if not response:
                    continue

                self.get_logger().info(f"Arduino says: {response}")

                if response.startswith("PROGRESS:"):
                    try:
                        progress_percent = float(response.split(":", 1)[1])
                    except ValueError:
                        self.get_logger().warn(f"Invalid progress message: {response}")
                        continue

                    feedback_msg.current_positions = [float(x) for x in target_positions]
                    feedback_msg.state = "MOVING"
                    feedback_msg.progress = max(0.0, min(1.0, progress_percent / 100.0))
                    feedback_msg.limit_switches = [False, False, False, False]
                    feedback_msg.error_code = 0
                    goal_handle.publish_feedback(feedback_msg)
                    continue

                if response == "DONE" or response == "HOMING DONE":
                    feedback_msg.current_positions = [float(x) for x in target_positions]
                    feedback_msg.state = "DONE"
                    feedback_msg.progress = 1.0
                    feedback_msg.limit_switches = [False, False, False, False]
                    feedback_msg.error_code = 0
                    goal_handle.publish_feedback(feedback_msg)
                    return True

                continue

            time.sleep(0.01)

    def execute_callback(self, goal_handle):
        if not self.ready:
            self.get_logger().warn("Arduino not ready yet!")
            goal_handle.abort()
            result = ActuateSlides.Result()
            result.success = False
            result.message = "Arduino not ready"
            return result

        self.get_logger().info('Received goal')

        self.get_logger().info('STEP 1')
        command = goal_handle.request.command.strip().upper()
        self.get_logger().info(f"RAW COMMAND DEBUG: {repr(command)}")
        self.get_logger().info('STEP 2')
        target_positions = goal_handle.request.target_positions
        speed = goal_handle.request.speed

        self.get_logger().info(f'Command: {command}')
        self.get_logger().info(f'Target positions: {target_positions}')
        self.get_logger().info(f'Speed: {speed}')

        feedback_msg = ActuateSlides.Feedback()
        result = ActuateSlides.Result()

        if command == "MOVE":
            x, y, z, a = target_positions

            # Lower-base mode only controls Y and A.
            if x != 0.0 or z != 0.0:
                goal_handle.abort()
                result.success = False
                result.message = (
                    "Lower-base mode only allows X=0 and Z=0. Use [0, Y, 0, A]."
                )
                result.final_positions = target_positions
                return result

            if y < 0.0 or y > 25.0 or a < 0.0 or a > 25.0:
                goal_handle.abort()
                result.success = False
                result.message = "Safe range exceeded. Y and A must stay within [0, 25]."
                result.final_positions = target_positions
                return result

            steps_per_mm = 100

            steps = [0, int(y * steps_per_mm), 0, int(a * steps_per_mm)]
            cmd_str = (
                f"MOVE,{steps[0]},{steps[1]},{steps[2]},{steps[3]},{float(speed)}\n"
            )

            self.get_logger().info(f"Sending to Arduino: {cmd_str}")
            self.ser.write(cmd_str.encode())
            self.ser.flush()

            safe_positions = [0.0, y, 0.0, a]

            if not self._read_serial_responses(
                goal_handle, safe_positions, feedback_msg
            ):
                goal_handle.abort()
                result.success = False
                result.message = "Timed out waiting for Arduino"
                result.final_positions = [float(x) for x in safe_positions]
                return result

            goal_handle.succeed()
            self.current_position = safe_positions
            result.success = True
            result.message = "Lower-base motion completed"
            result.final_positions = [float(x) for x in safe_positions]
            return result

        elif command == "HOME":
            self.get_logger().info('STEP 3 - entering HOME block')

            cmd_str = "HOME_LOWER\n"
            self.get_logger().info(f"Sending to Arduino: {cmd_str.strip()}")
            time.sleep(0.1)
            self.ser.write(cmd_str.encode())
            self.ser.flush()
            self.get_logger().info('STEP 4 - sent HOME')
            home_positions = [0.0, 0.0, 0.0, 0.0]
            if not self._read_serial_responses(
                goal_handle, home_positions, feedback_msg
            ):
                goal_handle.abort()
                result.success = False
                result.message = "Timed out waiting for Arduino during HOME"
                result.final_positions = target_positions
                return result

            goal_handle.succeed()
            self.current_position = [0.0, 0.0, 0.0, 0.0]
            result.success = True
            result.message = "Lower-base homing completed"
            result.final_positions = home_positions
            return result

        else:
            goal_handle.abort()
            result.success = False
            result.message = f"Unsupported command: {command}"
            result.final_positions = target_positions
            return result


def main(args=None):
    rclpy.init(args=args)

    node = ActuateSlidesServer()
    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
