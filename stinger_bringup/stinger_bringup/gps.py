'''
GT-U7 GPS Module
This node is publishing topic /stinger/gps/fix

Resources:
1. https://receiverhelp.trimble.com/alloy-gnss/en-us/NMEA-0183messages_MessageOverview.html
^ Look into GNS Fix data

Example output: $GPGGA,210621.00,3346.39300,N,08423.80898,W,1,03,3.03,127.4,M,-31.2,M,,*61
'''

import rclpy
from rclpy.node import Node
import serial
from sensor_msgs.msg import NavSatFix

class GpsPublisher(Node):
    def __init__(self):
        super().__init__('gps_serial_reader')
        
        # Serial port configuration
        self.declare_parameter('port', '/dev/serial0') # replace with actual port
        self.declare_parameter('baudrate', 9600)
        
        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        self.serial_port = serial.Serial(port, baudrate, timeout=1)
        self.publisher_ = self.create_publisher(NavSatFix, '/stinger/gps/fix', 10)
        
        self.timer = self.create_timer(0.5, self.read_serial_data)

    def read_serial_data(self):
        if self.serial_port.in_waiting > 0:
            line = self.serial_port.readline().decode('ascii', errors='ignore').strip()
            if line.startswith('$GNGGA') or line.startswith('$GPGGA'): # for our case, it's GPGGA
                self.parse_nmea_and_publish(line)

    def parse_nmea_and_publish(self, nmea_sentence):
        # Check output when plugged in: to see the datatype
        # TODO: modify this!
        # https://receiverhelp.trimble.com/alloy-gnss/en-us/NMEA-0183messages_MessageOverview.html
        try:
            parts = nmea_sentence.split(',')
            if len(parts) < 10 or parts[6] == '0':  # Check for valid fix
                return

            latitude = self.nmea_to_decimal(parts[2], parts[3])
            longitude = self.nmea_to_decimal(parts[4], parts[5])
            altitude = float(parts[9])

            msg = NavSatFix()
            msg.latitude = latitude
            msg.longitude = longitude
            msg.altitude = altitude
            msg.header.frame_id = 'gps_frame'
            msg.header.stamp = self.get_clock().now().to_msg()

            # self.get_logger().info(f'GPS Fix:: Lat={latitude}, Long={longitude}, Alt={altitude}')
            
            self.publisher_.publish(msg)
        except ValueError:
            self.get_logger().error('Failed to parse NMEA sentence')

    def nmea_to_decimal(self, nmea_value, direction):
        """
        for our case, lat DDMM.MMMM, long DDDMM.MMMM
        TODO: change the logic here if that's not the case for you
        """
        if not nmea_value:
            return 0.0

        if direction in ['N', 'S']:  # Latitude (DDMM.MMMM)
            degrees = float(nmea_value[:2])
            minutes = float(nmea_value[2:])
        else:  # Longitude (DDDMM.MMMM)
            degrees = float(nmea_value[:3])
            minutes = float(nmea_value[3:])

        decimal = degrees + minutes / 60.0
        if direction in ['S', 'W']:
            decimal *= -1 # make sure negative for s and w
        return decimal

def main(args=None):
    rclpy.init(args=args)
    gps_serial_reader = GpsPublisher()
    rclpy.spin(gps_serial_reader)
    gps_serial_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
