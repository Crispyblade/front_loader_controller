#!/usr/bin/env python3

# Hanford Dry Retrieval Project Front-Loader Main Control Program

import rclpy
from rclpy.node import Node
from enum import IntEnum
import serial
import time
import sys
import select
import tty
import termios
import signal
import threading

class ServoChannel(IntEnum):
    TURNING = 0
    ARM = 1
    BUCKET = 2
    PUMP = 3
    MOVEMENT = 4

class AutonomousFrontloader(Node):
    NEUTRAL = 6000
    PUMP_ACTIVE = 8000
    STEP_SIZE = 150
    MIN_POS = 4000
    MAX_POS = 8000

    def __init__(self):
        super().__init__('autonomous_frontloader')
        self._shutting_down = False
        self.serial_port = self._initialize_serial()
        self.old_terminal_settings = None
        self._configure_terminal()
        
        self._current_positions = {
            ServoChannel.ARM: self.NEUTRAL,
            ServoChannel.BUCKET: self.NEUTRAL,
            ServoChannel.TURNING: self.NEUTRAL,
            ServoChannel.MOVEMENT: self.NEUTRAL,
            ServoChannel.PUMP: self.PUMP_ACTIVE
        }
        self._initialize_servos()
        self.get_logger().info("System initialized - Manual control active")

    def _initialize_serial(self):
        try:
            port = serial.Serial('/dev/ttyAMA0', 9600, timeout=1)
            self.get_logger().info("Connected to Pololu Mini Maestro")
            return port
        except serial.SerialException as e:
            self.get_logger().error(f"Serial connection failed: {e}")
            raise SystemExit(1)

    def _configure_terminal(self):
        self.old_terminal_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    def _initialize_servos(self):
        for channel, position in self._current_positions.items():
            self._set_servo_position(channel, position)
        time.sleep(1)
        self.get_logger().info("Servos positioned to defaults")

    def _set_servo_position(self, channel, position):
        clamped_pos = max(self.MIN_POS, min(position, self.MAX_POS))
        command = bytearray([
            0x84, channel,
            clamped_pos & 0x7F,
            (clamped_pos >> 7) & 0x7F
        ])
        try:
            self.serial_port.write(command)
            self._current_positions[channel] = clamped_pos
        except serial.SerialException as e:
            if not self._shutting_down:
                self.get_logger().error(f"Servo command failed: {e}")

    def _manual_control_documentation(self):
        help_text = """
*********************
Controls:
t/g - Arm up/down
u/j - Bucket up/down
w/s - Drive forward/backward
a/d - Turn left/right
i/o - Reset arm/bucket
p - Toggle pump
space - Stop movement
q - Quit
h - Show help
*********************"""
        self.get_logger().info(help_text)

    def run_arm_sequence(self):
        self.get_logger().info("Resetting ARM...")
        self._smooth_movement(ServoChannel.ARM, self.MAX_POS, 0.05)
        self._smooth_movement(ServoChannel.ARM, self.MIN_POS, 0.05)
        self._smooth_movement(ServoChannel.ARM, self.NEUTRAL, 0.05)

    def run_bucket_sequence(self):
        self.get_logger().info("Resetting BUCKET...")
        self._smooth_movement(ServoChannel.BUCKET, self.MIN_POS, 1)
        self._smooth_movement(ServoChannel.BUCKET, self.MAX_POS, 0.05)
        self._smooth_movement(ServoChannel.BUCKET, self.NEUTRAL, 0.05)

    def _smooth_movement(self, channel, target, step_delay):
        current = self._current_positions[channel]
        step = self.STEP_SIZE if target > current else -self.STEP_SIZE
        while current != target and not self._shutting_down:
            current += step
            if (step > 0 and current > target) or (step < 0 and current < target):
                current = target
            self._set_servo_position(channel, current)
            time.sleep(step_delay)

    def _update_position(self, channel, delta):
        new_pos = self._current_positions[channel] + delta
        self._set_servo_position(channel, new_pos)

    def _toggle_pump(self):
        current = self._current_positions[ServoChannel.PUMP]
        new_pos = self.NEUTRAL if current == self.PUMP_ACTIVE else self.PUMP_ACTIVE
        self._set_servo_position(ServoChannel.PUMP, new_pos)
        self.get_logger().info(f"Pump {'ON' if new_pos == self.PUMP_ACTIVE else 'OFF'}")

    def manual_control(self):
        key_actions = {
            't': lambda: self._update_position(ServoChannel.ARM, self.STEP_SIZE),
            'g': lambda: self._update_position(ServoChannel.ARM, -self.STEP_SIZE),
            'u': lambda: self._update_position(ServoChannel.BUCKET, self.STEP_SIZE),
            'j': lambda: self._update_position(ServoChannel.BUCKET, -self.STEP_SIZE),
            'w': lambda: self._update_position(ServoChannel.MOVEMENT, self.STEP_SIZE),
            's': lambda: self._update_position(ServoChannel.MOVEMENT, -self.STEP_SIZE),
            'a': lambda: self._update_position(ServoChannel.TURNING, self.STEP_SIZE),
            'd': lambda: self._update_position(ServoChannel.TURNING, -self.STEP_SIZE),
            'i': self.run_arm_sequence,
            'o': self.run_bucket_sequence,
            'p': self._toggle_pump,
            ' ': lambda: self._set_servo_position(ServoChannel.MOVEMENT, self.NEUTRAL),
            'b': lambda: self._set_servo_position(ServoChannel.BUCKET, self.NEUTRAL),
            'm': lambda: self._set_servo_position(ServoChannel.ARM, self.NEUTRAL),
            'x': lambda: self._set_servo_position(ServoChannel.TURNING, self.NEUTRAL),
            'h': self._manual_control_documentation,
        }

        self._manual_control_documentation()
        try:
            while rclpy.ok() and not self._shutting_down:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1).lower()
                    if key == 'q':
                        self.destroy_node()

                    if key in key_actions:
                        key_actions[key]()
        finally:
            self._cleanup_terminal()

    def _cleanup_terminal(self):
        if self.old_terminal_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_terminal_settings)

    def safe_shutdown(self):
        self._shutting_down = True
        self.get_logger().info("Performing safe shutdown...")
        self._set_servo_position(ServoChannel.MOVEMENT, self.NEUTRAL)
        self._set_servo_position(ServoChannel.PUMP, self.NEUTRAL)
        try:
            self.serial_port.close()
        except Exception as e:
            self.get_logger().error(f"Error closing serial port: {e}")
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    loader = AutonomousFrontloader()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(loader)

    # Start manual control in separate thread
    control_thread = threading.Thread(target=loader.manual_control)
    control_thread.start()

    def sigint_handler(sig, frame):
        loader.get_logger().info("Ctrl+C received, initiating shutdown...")
        executor.shutdown()
        loader.safe_shutdown()
        control_thread.join() # Wait for control thread to finish
        rclpy.try_shutdown()

    signal.signal(signal.SIGINT, sigint_handler)

    try:
        executor.spin()
    finally:
        if rclpy.ok():
            executor.shutdown()
            loader.destroy_node()
            rclpy.shutdown()
        control_thread.join()

if __name__ == '__main__':
    main()
