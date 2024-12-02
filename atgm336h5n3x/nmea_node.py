#!/usr/bin/env python3
from atgm336h5n3x.atgm336h_serial import ATGM336H_Serial
from rclpy.node import Node
import rclpy
from sensor_msgs.msg import NavSatFix
import numpy as np
import argparse

class GPS(Node):
    def __init__(self, atgm366h: ATGM336H_Serial, dev: str):
        super().__init__('gps_node')
        self.navsatfix_pub = self.create_publisher(NavSatFix, 'gps/fix', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback_gps)
        self.atgm = atgm366h
        self.atgm.connect(dev)

    def timer_callback_gps(self):
        if self.navsatfix_pub.get_subscription_count() > 0:
            data = self.atgm.read_data("$GNGGA")
            if data and len(data) == 14:
                navsatfix = NavSatFix()
                navsatfix.header.stamp = self.get_clock().now().to_msg()
                navsatfix.header.frame_id = 'gps'
                navsatfix.latitude = float(data[1])/100 if data[1] else np.nan
                navsatfix.longitude = float(data[3])/100 if data[3] else np.nan
                navsatfix.altitude = float(data[8])/100 if data[8] else np.nan
                # Set the position covariance if available
                navsatfix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
                self.navsatfix_pub.publish(navsatfix)

def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--dev', default='/dev/myserial', help='Serial device path')
    args_ = parser.parse_args()
    dev = args_.dev

    rclpy.init(args=args)
    atgm = ATGM336H_Serial()
    gps_publisher = GPS(atgm366h=atgm, dev=dev)
    rclpy.spin(gps_publisher)
    atgm.disconnect()
    gps_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
