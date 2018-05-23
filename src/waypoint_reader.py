#!/usr/bin/env python
import rospy
from sensor_msgs.msg import NavSatFix


class WaypointReader():
    WAYPOINT_FILE = '/home/nvidia/waypoints.txt'
    THRESHOLD = 10  # meters

    def __init__(self):
        with open(self.WAYPOINT_FILE, 'r') as f:
            self.waypoints = [self.convert_to_fix(t) for t in 
                              f.read().splitlines()]

        self.pub = rospy.Publisher('waypoint', NavSatFix, queue_size=10)
        self.sub = rospy.Subscriber('fix', NavSatFix, self.fix_callback)
        self.waypoint = self.waypoints.pop(0)

    @staticmethod
    def convert_to_fix(waypoint_text):
        lat, lon = [float(val) for val in waypoint_text.split(',')]
        fix = NavSatFix()
        fix.latitude = lat
        fix.longitude = lon
        return fix

    def at_goal(self, fix):
        delta_lat = fix.latitude - self.waypoint.latitude
        delta_lon = fix.longitude - self.waypoint.longitude
        dist = (delta_lat**2 + delta_lon**2)**0.5
        return dist < self.THRESHOLD

    def fix_callback(self, fix):
        if self.at_goal(fix) and len(self.waypoints) > 0:
            self.waypoint = convert_to_fix(self.waypoints.pop(0))
        self.pub.publish(self.waypoint) 


def main():
    rospy.init_node('waypoint_reader', anonymous=True)
    wr = WaypointReader()
    try:
        rospy.spin()
    except rospy.ROSInterruptException, KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()

