#!/usr/bin/env python3
"""
    * Author: nipun.dhananjaya@gmail.com
    * Created: 25.05.2023
"""

from atgm336h5n3x.atgm336h_serial import ATGM336H_Serial
from rclpy.node import Node
import rclpy
from nmea_msgs.msg import Gpgga
from time import time
import argparse
import numpy as np
from  std_msgs.msg import Int8MultiArray


"""
$GPGSA: GPS DOP and Active Satellites

$GPGSV: GPS Satellites in View

$GPTXT: General Purpose Text Transmission

$GNGLL: Geographic Position - Latitude/Longitude
     | $GNGLL | lat | lat dir | lon | lon gir | utc | data status | mode ind | *xx | [CR][LF] |

$GNGGA: Global Navigation Satellite System Fix Data

$GNRMC: Recommended Minimum Specific GPS/Transit Data

$GNVTG: Course Over Ground and Ground Speed

$GNZDA: Time and Date

$BDGSA: Beidou DOP and Active Satellites

$BDGSV: Beidou Satellites in View

"""

class GPS(Node):
    def __init__(self, atgm366h:ATGM336H_Serial, dev:str):
        super().__init__('gps_node')
        self.gpgga_pub = self.create_publisher(Gpgga, 'gps/fix', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback_gps)
        self.atgm = atgm366h
        self.atgm.connect(dev)

    def timer_callback_gps(self):
        if self.gpgga_pub.get_subscription_count() > 0:
            gpgga = Gpgga()
            data = self.atgm.read_data("$GNGGA") 
            if data != False and len(data) ==14:
                gpgga.header.stamp = self.get_clock().now().to_msg()
                gpgga.header.frame_id = 'gps'
                gpgga.utc_seconds = np.float64(data[0]) if data[0] != '' else np.nan
                gpgga.lat = np.float64(data[1]) if data[1] != '' else np.nan
                gpgga.lat_dir = data[2] 
                gpgga.lon = np.float64(data[3]) if data[3] != '' else np.nan
                gpgga.lon_dir = data[4] 
                if not data[5] == '': gpgga.gps_qual = int(data[5])
                if not data[6] == '': gpgga.num_sats = int(data[6])
                gpgga.hdop = float(data[7]) if data[7] != '' else np.nan
                gpgga.alt = float(data[8]) if data[8] != '' else np.nan
                gpgga.altitude_units = data[9]
                gpgga.undulation = float(data[10]) if data[10] != '' else np.nan
                gpgga.undulation_units = data[11]
                if not data[12] == '': gpgga.diff_age = int(data[12])
                gpgga.station_id = data[13]
                self.gpgga_pub.publish(gpgga)



def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--dev')
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
