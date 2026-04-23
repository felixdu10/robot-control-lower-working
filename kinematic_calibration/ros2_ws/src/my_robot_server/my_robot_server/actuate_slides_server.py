import math
import time

import rclpy
import serial
from rclpy.action import ActionServer
from rclpy.node import Node

from my_robot_interfaces.action import ActuateSlides


class LowerBaseGeometry:
    """Geometry model for the lower 4-link mechanism.

    Units are millimeters.

    Assumptions used here (based on the discussion so far):
    - Lower fixed base points are 20 mm apart.
    - Each lower link (base point -> left/right upper point) is 90 mm.
    - H-link length is 110 mm.
    - H-link attaches at the midpoint of the 90 mm lower link.
    - Each slider rail is horizontally 36 mm outward from its nearby lower base point.
    - HOME slider coordinates, relative to the nearby lower base point, are:
      left  = (-36, -88) mm
      right = (+36, -88) mm
    - Positive commanded motion moves the slider upward from HOME.

    What this class does:
    - Converts commanded Y/A (mm from HOME) into actual slider heights.
    - Solves left/right lower-link angles from H-link distance constraints.
    - Computes left/right upper point positions and their Euclidean distance.
    - Optionally checks that distance against configured min/max limits.
    """

    def __init__(
        self,
        base_span_mm: float = 20.0,
        lower_link_mm: float = 90.0,
        h_link_mm: float = 110.0,
        h_attach_fraction: float = 0.5,
        slider_x_offset_mm: float = 36.0,
        home_slider_y_mm: float = -88.0,
        positive_moves_slider_up: bool = True,
        safe_lr_distance_min_mm: float | None = None,
        safe_lr_distance_max_mm: float | None = None,
    ):
        self.base_span_mm = base_span_mm
        self.lower_link_mm = lower_link_mm
        self.h_link_mm = h_link_mm
        self.h_attach_fraction = h_attach_fraction
        self.attach_len_mm = lower_link_mm * h_attach_fraction
        self.slider_x_offset_mm = slider_x_offset_mm
        self.home_slider_y_mm = home_slider_y_mm
        self.positive_moves_slider_up = positive_moves_slider_up
        self.safe_lr_distance_min_mm = safe_lr_distance_min_mm
        self.safe_lr_distance_max_mm = safe_lr_distance_max_mm

        # Fixed lower base points. Use left base as x=0 if you prefer mentally;
        # here we keep a symmetric frame for convenience.
        self.bl = (-base_span_mm / 2.0, 0.0)
        self.br = (+base_span_mm / 2.0, 0.0)
        # Fixed slider rail x locations. Sliders are OUTSIDE the two lower base points.
        self.left_slider_x = self.bl[0] - slider_x_offset_mm
        self.right_slider_x = self.br[0] + slider_x_offset_mm

    def slider_heights_from_command(self, y_cmd_mm: float, a_cmd_mm: float) -> tuple[float, float]:
        sign = 1.0 if self.positive_moves_slider_up else -1.0
        y_slider = self.home_slider_y_mm + sign * y_cmd_mm
        a_slider = self.home_slider_y_mm + sign * a_cmd_mm
        return y_slider, a_slider

    def _solve_theta_left(self, slider_y_mm: float) -> float:
        return self._solve_theta(
            base=self.bl,
            slider=(self.left_slider_x, slider_y_mm),
            side="left",
        )

    def _solve_theta_right(self, slider_y_mm: float) -> float:
        return self._solve_theta(
            base=self.br,
            slider=(self.right_slider_x, slider_y_mm),
            side="right",
        )

    def _solve_theta(self, base: tuple[float, float], slider: tuple[float, float], side: str) -> float:
        bx, by = base
        sx, sy = slider
        r = self.attach_len_mm
        hx = sx - bx
        hy = sy - by

        # Midpoint-on-link must satisfy distance to slider = h_link_mm.
        # midpoint = base + r * [cos(theta), sin(theta)]   (left)
        # midpoint = base + r * [-cos(theta), sin(theta)]  (right)
        # Solve with dot product identity: A cos(theta) + B sin(theta) = C.
        if side == "left":
            a = -2.0 * r * hx
        elif side == "right":
            a = +2.0 * r * hx
        else:
            raise ValueError(f"Unknown side: {side}")
        b = -2.0 * r * hy
        c = self.h_link_mm**2 - (hx**2 + hy**2 + r**2)

        rho = math.hypot(a, b)
        if rho < 1e-9:
            raise ValueError("Degenerate geometry while solving theta.")
        ratio = c / rho
        if ratio < -1.0 or ratio > 1.0:
            raise ValueError(
                f"No real solution: H-link cannot reach slider position (ratio={ratio:.4f})."
            )

        phi = math.atan2(b, a)
        alpha = math.acos(max(-1.0, min(1.0, ratio)))
        candidates = [phi + alpha, phi - alpha]

        valid = []
        for th in candidates:
            # Normalize to [0, pi]
            while th < 0:
                th += 2.0 * math.pi
            while th >= 2.0 * math.pi:
                th -= 2.0 * math.pi
            if 0.0 <= th <= math.pi:
                valid.append(th)

        # Prefer the physically meaningful open-upward solution.
        scored = []
        for th in valid:
            if side == "left":
                mx = bx + r * math.cos(th)
            else:
                mx = bx - r * math.cos(th)
            my = by + r * math.sin(th)
            if my < -1e-6:
                continue
            if side == "left":
                lx = bx + self.lower_link_mm * math.cos(th)
            else:
                lx = bx - self.lower_link_mm * math.cos(th)
            ly = by + self.lower_link_mm * math.sin(th)
            scored.append((ly, th, mx, my, lx))

        if not scored:
            raise ValueError("No upward physical solution for theta.")

        # Choose the branch with higher upper-point y (the open upward branch).
        scored.sort(reverse=True)
        return scored[0][1]

    def state_from_command(self, y_cmd_mm: float, a_cmd_mm: float) -> dict:
        y_slider, a_slider = self.slider_heights_from_command(y_cmd_mm, a_cmd_mm)

        th_l = self._solve_theta_left(y_slider)
        th_r = self._solve_theta_right(a_slider)

        blx, bly = self.bl
        brx, bry = self.br
        L = (
            blx + self.lower_link_mm * math.cos(th_l),
            bly + self.lower_link_mm * math.sin(th_l),
        )
        R = (
            brx - self.lower_link_mm * math.cos(th_r),
            bry + self.lower_link_mm * math.sin(th_r),
        )
        Lmid = (
            blx + self.attach_len_mm * math.cos(th_l),
            bly + self.attach_len_mm * math.sin(th_l),
        )
        Rmid = (
            brx - self.attach_len_mm * math.cos(th_r),
            bry + self.attach_len_mm * math.sin(th_r),
        )
        d = math.hypot(R[0] - L[0], R[1] - L[1])
        theta_left_deg = math.degrees(th_l)
        theta_right_deg = math.degrees(th_r)
        opening_left_deg = self.display_opening_deg(theta_left_deg)
        opening_right_deg = self.display_opening_deg(theta_right_deg)

        ok = True
        reason = "OK"
        if self.safe_lr_distance_min_mm is not None and d < self.safe_lr_distance_min_mm:
            ok = False
            reason = (
                f"Left-right point distance {d:.2f} mm is below min "
                f"{self.safe_lr_distance_min_mm:.2f} mm."
            )
        if self.safe_lr_distance_max_mm is not None and d > self.safe_lr_distance_max_mm:
            ok = False
            reason = (
                f"Left-right point distance {d:.2f} mm is above max "
                f"{self.safe_lr_distance_max_mm:.2f} mm."
            )

        return {
            "y_slider_mm": y_slider,
            "a_slider_mm": a_slider,
            "theta_left_deg": theta_left_deg,
            "theta_right_deg": theta_right_deg,
            "opening_left_deg": opening_left_deg,
            "opening_right_deg": opening_right_deg,
            "left_point": L,
            "right_point": R,
            "left_mid": Lmid,
            "right_mid": Rmid,
            "lr_distance_mm": d,
            "ok": ok,
            "reason": reason,
        }

    @staticmethod
    def display_opening_deg(theta_deg: float) -> float:
        return 180.0 - theta_deg


class ActuateSlidesServer(Node):
    def __init__(self):
        super().__init__('actuate_slides_server')
        self.get_logger().info("### SERVER WITH ACCUMULATED LOWER-BASE GEOMETRY CHECK ###")

        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.ser.setDTR(False)
        time.sleep(0.1)
        self.ser.setDTR(True)
        time.sleep(2)

        self._action_server = ActionServer(
            self,
            ActuateSlides,
            'actuate_slides',
            self.execute_callback,
        )

        self.current_position = [0.0, 0.0, 0.0, 0.0]
        self.ready = False
        self.create_timer(0.1, self.check_ready)

        self.steps_per_mm = 100

        # ===== Geometry parameters you can tune after one or two bench checks =====
        self.geom = LowerBaseGeometry(
            base_span_mm=20.0,
            lower_link_mm=90.0,
            h_link_mm=110.0,
            h_attach_fraction=0.5,
            slider_x_offset_mm=38.0,
            home_slider_y_mm=-88.0,
            positive_moves_slider_up=True,
            safe_lr_distance_min_mm=36.0,
            safe_lr_distance_max_mm=180.0,
        )

    def check_ready(self):
        if self.ready:
            return
        if self.ser.in_waiting:
            line = self.ser.readline().decode(errors='replace').strip()
            if line:
                self.get_logger().info(f"Arduino boot: {line}")
                if line == 'READY':
                    self.get_logger().info('Arduino is READY')
                    self.ready = True

    def _read_serial_responses(self, goal_handle, target_positions, feedback_msg, timeout_s=300.0):
        deadline = time.monotonic() + timeout_s
        while True:
            if time.monotonic() > deadline:
                self.get_logger().error('Timed out waiting for Arduino DONE.')
                return False

            if self.ser.in_waiting:
                response = self.ser.readline().decode(errors='replace').strip()
                if not response:
                    continue

                self.get_logger().info(f"Arduino says: {response}")

                if response.startswith('PROGRESS:'):
                    try:
                        progress_percent = float(response.split(':', 1)[1])
                    except ValueError:
                        self.get_logger().warn(f"Invalid progress message: {response}")
                        continue

                    feedback_msg.current_positions = [float(x) for x in target_positions]
                    feedback_msg.state = 'MOVING'
                    feedback_msg.progress = max(0.0, min(1.0, progress_percent / 100.0))
                    feedback_msg.limit_switches = [False, False, False, False]
                    feedback_msg.error_code = 0
                    goal_handle.publish_feedback(feedback_msg)
                    continue

                if response in ('DONE', 'HOMING DONE'):
                    feedback_msg.current_positions = [float(x) for x in target_positions]
                    feedback_msg.state = 'DONE'
                    feedback_msg.progress = 1.0
                    feedback_msg.limit_switches = [False, False, False, False]
                    feedback_msg.error_code = 0
                    goal_handle.publish_feedback(feedback_msg)
                    return True

            time.sleep(0.01)

    def execute_callback(self, goal_handle):
        if not self.ready:
            self.get_logger().warn('Arduino not ready yet!')
            goal_handle.abort()
            result = ActuateSlides.Result()
            result.success = False
            result.message = 'Arduino not ready'
            return result

        command = goal_handle.request.command.strip().upper()
        target_positions = goal_handle.request.target_positions
        speed = goal_handle.request.speed

        self.get_logger().info(f'Command: {command}')
        self.get_logger().info(f'Target positions: {target_positions}')
        self.get_logger().info(f'Speed: {speed}')

        feedback_msg = ActuateSlides.Feedback()
        result = ActuateSlides.Result()

        if command == 'MOVE':
            x, y, z, a = target_positions

            if x != 0.0 or z != 0.0:
                goal_handle.abort()
                result.success = False
                result.message = 'Lower-base mode only allows X=0 and Z=0. Use [0, dY, 0, dA].'
                result.final_positions = [float(v) for v in self.current_position]
                return result

            # In this server, MOVE is treated as an incremental command for debug use:
            #   [0, dY, 0, dA]
            # The Arduino already behaves incrementally, so we keep that behavior and
            # accumulate the software-side state before checking geometry.
            delta_y = float(y)
            delta_a = float(a)

            current_y = float(self.current_position[1])
            current_a = float(self.current_position[3])
            proposed_y = current_y + delta_y
            proposed_a = current_a + delta_a

            # Keep the accumulated pose inside the allowed travel window relative to HOME.
            if proposed_y < -30.0 or proposed_y > 30.0 or proposed_a < -30.0 or proposed_a > 30.0:
                goal_handle.abort()
                result.success = False
                result.message = (
                    'Accumulated safe range exceeded. '
                    f'Current=[0,{current_y:.2f},0,{current_a:.2f}], '
                    f'Delta=[0,{delta_y:.2f},0,{delta_a:.2f}], '
                    f'Proposed=[0,{proposed_y:.2f},0,{proposed_a:.2f}] must stay within [-30,30].'
                )
                result.final_positions = [0.0, current_y, 0.0, current_a]
                return result

            # ----- Geometry guard on the accumulated target pose -----
            try:
                geom_state = self.geom.state_from_command(proposed_y, proposed_a)
            except ValueError as exc:
                goal_handle.abort()
                result.success = False
                result.message = (
                    f'Geometry check failed at proposed accumulated pose '
                    f'[0,{proposed_y:.2f},0,{proposed_a:.2f}]: {exc}'
                )
                result.final_positions = [0.0, current_y, 0.0, current_a]
                return result

            self.get_logger().info(
                'Geom check | '
                f"current=[0,{current_y:.2f},0,{current_a:.2f}] | "
                f"delta=[0,{delta_y:.2f},0,{delta_a:.2f}] | "
                f"proposed=[0,{proposed_y:.2f},0,{proposed_a:.2f}] | "
                f"thetaL_raw={geom_state['theta_left_deg']:.2f} deg, "
                f"thetaR_raw={geom_state['theta_right_deg']:.2f} deg, "
                f"openingL={geom_state['opening_left_deg']:.2f} deg, "
                f"openingR={geom_state['opening_right_deg']:.2f} deg, "
                f"dLR={geom_state['lr_distance_mm']:.2f} mm, "
                f"ySlider={geom_state['y_slider_mm']:.2f} mm, "
                f"aSlider={geom_state['a_slider_mm']:.2f} mm"
            )

            if not geom_state['ok']:
                goal_handle.abort()
                result.success = False
                result.message = (
                    f"Geometry unsafe at proposed accumulated pose [0,{proposed_y:.2f},0,{proposed_a:.2f}]: "
                    f"{geom_state['reason']}"
                )
                result.final_positions = [0.0, current_y, 0.0, current_a]
                return result

            # Send ONLY the incremental motion to Arduino.
            steps = [0, int(delta_y * self.steps_per_mm), 0, int(delta_a * self.steps_per_mm)]
            cmd_str = f"MOVE,{steps[0]},{steps[1]},{steps[2]},{steps[3]},{float(speed)}\n"

            self.get_logger().info(f'Sending to Arduino: {cmd_str.strip()}')
            self.ser.write(cmd_str.encode())
            self.ser.flush()

            accumulated_positions = [0.0, proposed_y, 0.0, proposed_a]
            if not self._read_serial_responses(goal_handle, accumulated_positions, feedback_msg):
                goal_handle.abort()
                result.success = False
                result.message = 'Timed out waiting for Arduino'
                result.final_positions = [float(v) for v in self.current_position]
                return result

            goal_handle.succeed()
            self.current_position = accumulated_positions
            result.success = True
            result.message = (
                'Lower-base incremental motion completed | '
                f"current=[0,{proposed_y:.2f},0,{proposed_a:.2f}] | "
                f"dLR={geom_state['lr_distance_mm']:.2f} mm, "
                f"openingL={geom_state['opening_left_deg']:.2f} deg, "
                f"openingR={geom_state['opening_right_deg']:.2f} deg"
            )
            result.final_positions = [float(v) for v in accumulated_positions]
            return result

        if command == 'HOME':
            y_dir_high = 1 if self.current_position[1] >= 0.0 else 0
            a_dir_high = 1 if self.current_position[3] >= 0.0 else 0
            cmd_str = f"HOME_LOWER_SMART,{y_dir_high},{a_dir_high}\n"
            self.get_logger().info(f'Sending to Arduino: {cmd_str.strip()}')
            time.sleep(0.1)
            self.ser.write(cmd_str.encode())
            self.ser.flush()

            home_positions = [0.0, 0.0, 0.0, 0.0]
            if not self._read_serial_responses(goal_handle, home_positions, feedback_msg):
                goal_handle.abort()
                result.success = False
                result.message = 'Timed out waiting for Arduino during HOME'
                result.final_positions = target_positions
                return result

            goal_handle.succeed()
            self.current_position = [0.0, 0.0, 0.0, 0.0]
            result.success = True
            result.message = (
                'Lower-base homing completed | '
                f'smart_dirs=[{y_dir_high},{a_dir_high}]'
            )
            result.final_positions = home_positions
            return result

        goal_handle.abort()
        result.success = False
        result.message = f'Unsupported command: {command}'
        result.final_positions = target_positions
        return result


def main(args=None):
    rclpy.init(args=args)
    node = ActuateSlidesServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
