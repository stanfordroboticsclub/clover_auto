#!/usr/bin/env python
import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Pose2D


class WaypointReader():
    WAYPOINT_FILE = '/home/nvidia/waypoints.txt'
    THRESHOLD = 10 # approx meters

    def __init__(self):
        with open(self.WAYPOINT_FILE, 'r') as f:
            self.waypoints = [self.convert_to_pose(t) for t in
                              f.read().splitlines()]

        self.pub = rospy.Publisher('waypoint', Pose2D, queue_size=10)
        self.sub = rospy.Subscriber('fix', NavSatFix, self.fix_callback)
        self.waypoint = self.waypoints.pop(0)

    @staticmethod
    def convert_to_pose(waypoint_text):
        lat, lon, head = [float(val) for val in waypoint_text.split(',')]
        pose = Pose2D()
        pose.x = lat
        pose.y = lon
        pose.theta = head
        return pose

    def at_goal(self, fix):
        delta_lat = 85000 * (fix.latitude - self.waypoint.x)
        delta_lon = 85000 * (fix.longitude - self.waypoint.y)
        dist = (delta_lat**2 + delta_lon**2)**0.5
        return dist < self.THRESHOLD

    def fix_callback(self, fix):
        if self.at_goal(fix) and len(self.waypoints) > 0:
            self.waypoint = convert_to_pose(self.waypoints.pop(0))
        self.pub.publish(self.waypoint)


def main():
    rospy.init_node('waypoint_reader', anonymous=False)
    wr = WaypointReader()
    try:
        rospy.spin()
    except rospy.ROSInterruptException, KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()

