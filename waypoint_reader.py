#!/usr/bin/env python
import rospy
from sensor_msgs.msg import NavSatFix


class WaypointReader():
    WAYPOINT_FILE = '/home/nvidia/waypoints.txt'
    THRESHOLD = 10  # meters

    def __init__(self):
        with open(self.WAYPOINT_FILE, 'r') as f:
            waypoints = [self.convert_to_fix(t) for t in f.read().splitlines()]

        self.pub = rospy.Publisher('waypoint', NavSatFix, queue_size=10)
        self.sub = rospy.Subscriber('fix', NavSatFix, self.fix_callback)
        self.rate = rospy.Rate(10)
        self.waypoint = waypoints.pop()

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
        while not rospy.is_shutdown():
            if self.at_goal(fix) and len(self.waypoints) > 0:
                self.waypoint = convert_to_fix(self.waypoints.pop())
            self.pub.publish(waypoint) 
            self.rate.sleep()


def main():
    wr = WaypointReader()
    rospy.init_node('waypoint_reader', anonymous=True)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()

