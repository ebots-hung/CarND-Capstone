#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 70   # Number of waypoints we will publish. You can change this number
MAX_DECEL = 0.5      # Max deceleration for slowing at traffic lights
CONSTANT_DECEL = 1/LOOKAHEAD_WPS # Smooth braking

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # TODO: Add other member variables you need below
        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoints_tree = None
        self.stopline_wp_idx = -1

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoints', Int32, self.traffic_cb)

        # Publisher
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.loop()

    def loop(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints:

                # if pose and waypoints exist, call the publisher
                self.publish_waypoints()

            rate.sleep()

    def publish_waypoints(self):
        # Call lane generator
        final_lane = self.generate_lane()

        # Publish generated lane
        self.final_waypoints_pub.publish(final_lane)

    def generate_lane(self):
        lane = Lane()

        # Reuse header - Not mandatory
        lane.header = self.base_waypoints.header

        # Define the base waypoints from the closest to the farthest
        closest_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        base_waypoints = self.base_waypoints.waypoints[closest_idx:farthest_idx]

        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farthest_idx):
            # rospy.loginfo("Closest waypoynt idx :{0}".format(closest_idx))
            # rospy.loginfo("Farthest waypoynt idx :{0}".format(farthest_idx))
            # rospy.loginfo("Normal lane generated")

            # Case of no stopline in sight - publish base waypoints
            lane.waypoints = base_waypoints
        else:
            # rospy.loginfo("Closest waypoynt idx :{0}".format(closest_idx))
            # rospy.loginfo("Farthest waypoynt idx :{0}".format(farthest_idx))
            # rospy.loginfo("Decelerating lane generated")

            # In case a stopline IS in sight, generate decelerating waypoints
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)

        return lane

    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoints_tree.query([x, y], 1)[1]

        # Check if closest wp is ahead or behind the vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx - 1]

        # Define the Hyperplane through closest coordinates
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        val = np.dot(cl_vect-prev_vect, cl_vect-pos_vect)

        if (val > 0):
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)

        return closest_idx

    def decelerate_waypoints(self, waypoints, closest_idx):
        temp = []

        # rospy.loginfo("Length base wp :{0}".format(len(waypoints)))
        stop_idx = max(self.stopline_wp_idx - closest_idx - 3, 0)

        for i, wp in enumerate(waypoints):

            p = Waypoint()
            p.pose = wp.pose

            # Define a deceleration profile based on distance of the current wp from stop line
            dist = self.distance(waypoints, i, stop_idx)
            vel = math.sqrt(2 * MAX_DECEL * dist) + (i * CONSTANT_DECEL)

            if vel <= 0.1:
                vel = 0.0

            p.twist.twist.linear.x = min(vel, p.twist.twist.linear.x)
            temp.append(p)

        return temp

    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        # Store waypoints and define their KDTree
        self.base_waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoints_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        # rospy.loginfo("subscribing stopline_wp_idx :{0}".format(msg.data))

        # Store index of closer stop line
        self.stopline_wp_idx = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. Not implemented in this project
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
