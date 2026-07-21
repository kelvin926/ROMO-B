import errno
import os
import pathlib
import pty
import tty
from time import monotonic

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

from .protocol import (
    CommandParser,
    ackermann_feedback,
    encode_feedback,
    four_wis_feedback,
    pivot_feedback,
)


class PcuSimulator(Node):
    def __init__(self):
        super().__init__("romo_b_pcu_simulator")
        self.declare_parameter("symlink_path", "/tmp/romo_b_pcu")
        self.declare_parameter("feedback_hz", 20.0)
        self.declare_parameter("auto_switch", True)
        self.declare_parameter("command_timeout_sec", 0.30)
        self.feedback_enabled = True
        self.alive_enabled = True

        self.master_fd, self.slave_fd = pty.openpty()
        tty.setraw(self.slave_fd)
        self.slave_name = os.ttyname(self.slave_fd)
        os.set_blocking(self.master_fd, False)
        link = pathlib.Path(self.get_parameter("symlink_path").value)
        if link.exists() or link.is_symlink():
            if not link.is_symlink():
                raise RuntimeError(f"Refusing to replace non-symlink {link}")
            link.unlink()
        link.symlink_to(self.slave_name)
        self.link = link

        self.parser = CommandParser()
        self.command = {
            "auto_mode": False,
            "estop": False,
            "steer_mode": 0,
            "speed_mps": 0.0,
            "steer_deg": 0.0,
            "alive": 0,
        }
        self.last_command = monotonic()
        self.pcu_alive = 0
        self.create_timer(0.005, self.read_commands)
        self.create_timer(1.0 / float(self.get_parameter("feedback_hz").value), self.write_feedback)
        self.create_service(SetBool, "/romo_b_sim/feedback", self.set_feedback)
        self.create_service(SetBool, "/romo_b_sim/alive", self.set_alive)
        self.get_logger().info(f"PTY PCU ready: {self.link} -> {self.slave_name}")

    def set_feedback(self, request, response):
        self.feedback_enabled = bool(request.data)
        response.success = True
        response.message = "feedback enabled" if self.feedback_enabled else "feedback paused"
        return response

    def set_alive(self, request, response):
        self.alive_enabled = bool(request.data)
        response.success = True
        response.message = "alive enabled" if self.alive_enabled else "alive frozen"
        return response

    def read_commands(self):
        while True:
            try:
                data = os.read(self.master_fd, 4096)
            except BlockingIOError:
                return
            except OSError as error:
                if error.errno == errno.EIO:
                    return
                raise
            if not data:
                return
            for command in self.parser.push(data):
                self.command = command
                self.last_command = monotonic()

    def write_feedback(self):
        if not self.feedback_enabled:
            return
        fresh = monotonic() - self.last_command <= float(
            self.get_parameter("command_timeout_sec").value
        )
        enabled = fresh and self.command["auto_mode"] and not self.command["estop"]
        speed = self.command["speed_mps"] if enabled else 0.0
        steer = self.command["steer_deg"] if enabled else 0.0
        if self.command["steer_mode"] == 2:
            speeds, angles = pivot_feedback(speed)
        elif self.command["steer_mode"] == 1:
            speeds, angles = four_wis_feedback(speed, steer)
        else:
            speeds, angles = ackermann_feedback(speed, steer)
        auto_mode = bool(self.get_parameter("auto_switch").value) and self.command["auto_mode"]
        frame = encode_feedback(
            auto_mode,
            self.command["estop"],
            self.command["steer_mode"],
            speeds,
            angles,
            self.pcu_alive,
        )
        try:
            os.write(self.master_fd, frame)
        except OSError as error:
            if error.errno != errno.EIO:
                raise
        if self.alive_enabled:
            self.pcu_alive = (self.pcu_alive + 1) % 256

    def destroy_node(self):
        try:
            if self.link.is_symlink():
                self.link.unlink()
            os.close(self.master_fd)
            os.close(self.slave_fd)
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PcuSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
