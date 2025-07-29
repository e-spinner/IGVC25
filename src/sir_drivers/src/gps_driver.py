#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import pynmea2
from builtin_interfaces.msg import Time
from sir_msgs.msg import GPSFeedback
import time

class GarminGPSNode(Node):
    def __init__(self):
        super().__init__('garmin_gps_node')
        self.publisher_ = self.create_publisher(GPSFeedback, 'gps/fix', 10)
        self.serial_port = serial.Serial('/dev/ttyUSB0', baudrate=4800, timeout=1)
        self.timer = self.create_timer(0.1, self.read_serial)

    def read_serial(self):
        line = self.serial_port.readline().decode(errors='ignore').strip()
        try:
            msg = pynmea2.parse(line)
            if isinstance(msg, pynmea2.types.talker.GGA):
                gps_msg = GPSFeedback()
                gps_msg.latitude = msg.latitude
                gps_msg.longitude = msg.longitude

                now = self.get_clock().now().to_msg()
                gps_msg.tov = now
                self.publisher_.publish(gps_msg)

        except pynmea2.ParseError:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = GarminGPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
