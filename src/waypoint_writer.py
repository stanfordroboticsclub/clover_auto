#!/usr/bin/env python
import rospy
import serial
from sensor_msgs.msg import NavSatFix
from std_srvs.msg import Trigger


class WaypointWriter():
    WAYPOINT_FILE = '/home/nvidia/waypoints.txt'
    PATH = '/dev/ttyUSB2'

    def __init__(self):
        self.sub = rospy.Subscriber('fix', NavSatFix, self.fix_callback)
        self.serv =  rospy.Service('record', None, self.record_callback)
        self.ser = serial.Serial(self.PATH)

    def fix_callback(self, fix):
        self.fix = fix

    def record_callback(self):
        with open(self.WAYPOINT_FILE, 'a') as f:
            f.write(self.get_data_string())
        trigger = Trigger()
        trigger.success = True
        return trigger

    def get_data_string(self):
        serial_line = self.ser.readline()
        theta = float(serial_line.split(',')[1])
        data_string = ''.join((self.fix.latitude, self.fix.longitude, theta))
        return data_string + '\n'


def main():
    rospy.init_node('waypoint_writer', anonymous=True)
    ww = WaypointWriter()
    try:
        rospy.spin()
    except rospy.ROSInterruptException, KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()

