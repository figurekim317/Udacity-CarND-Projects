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

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        # These variables should be initialize before the callback is called.
        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.stopline_wp_idx = None

        self.loop()

    def loop(self):
        # Frequency should be 50Hz.
        rate = rospy.Rate(50)
        # main loop (instead of rospy.spin())
        while not rospy.is_shutdown(): 
            # This is to prevent crash if waypoint_tree were not loaded before first loop.
            if self.pose and self.waypoint_tree:
                # Get closest waypoint
                closest_waypoint_idx = self.get_closest_waypoint_idx()
                self.publish_waypoints(closest_waypoint_idx)
            rate.sleep()

    def get_closest_waypoint_idx(self):
        # Query KDTree closest waypoint
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_dist, closest_idx = self.waypoint_tree.query([x, y], 1)

        # Check if closest waypoint is ahead or behind the vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx-1]

        # Equation for hyperplane through closest_coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])
        
        # If dot product value is positive, the waypoint will be behind the car.
        # cl_vect-prev_vect is reference heading direction
        # pos_vect-cl_vect is relative car position to closest waypoint.
        # The value will be positive if car is ahead of closest waypoint.
        val = np.dot(cl_vect-prev_vect, pos_vect-cl_vect)
        # Then, simply take the next one.
        if val > 0:
            closest_idx = (closest_idx+1) % len(self.waypoints_2d)

        return closest_idx

    def publish_waypoints(self, closest_idx):
        # lane = Lane()
        # lane.header = self.base_waypoints.header
        # lane.waypoints = self.base_waypoints.waypoints[closest_idx:closest_idx + LOOKAHEAD_WPS]
        final_lane = self.generate_lane()
        self.final_waypoints_pub.publish(final_lane)

    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg

    # This is a latched subscriber.
    # Waypoints does not change once called.
    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_waypoints = waypoints
        rospy.loginfo("base_waypoints set")
        if not self.waypoints_2d:
            self.waypoints_2d = []
            for waypoint in waypoints.waypoints:
                x = waypoint.pose.pose.position.x
                y = waypoint.pose.pose.position.y
                self.waypoints_2d.append([x, y])
            #Use KDTree for efficient search on 2D space
            self.waypoint_tree = KDTree(self.waypoints_2d) 

            rospy.loginfo("waypoints_2d set")
    
    def generate_lane(self):
        lane = Lane()
        
        closest_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        base_lane = self.base_waypoints.waypoints[closest_idx:farthest_idx]

        # Prevent crash when waypoints_cb is triggered before traffic_cb is updated.
        if self.stopline_wp_idx:
            # if no stop light found or far ahead
            if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farthest_idx):
                # just follow the base lane
                lane.waypoints = base_lane
            else:
                # otherwise, if stop lights is ahead, then decelerate.
                lane.waypoints = self.decelerate_waypoints(base_lane, closest_idx)

        return lane


    def decelerate_waypoints(self, waypoints, closest_idx):
        # creating delerated waypoints
        temp = []
        for i, wp in enumerate(waypoints):
            # Position of waypoint remains same.
            p = Waypoint()
            p.pose = wp.pose

            # car center is 2 points behind the stopline
            stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0)
            # line distance from closest waypoint to stop line
            dist = self.distance(waypoints, i, stop_idx)

            # square root characteristic of velocity when deceleration
            vel = math.sqrt(2 * 0.5 * dist)
            if vel < 1.0: # smoothing
                vel = 0.

            # deceleration could be larger than speed limit when far away.
            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            temp.append(p)

        return temp



    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stopline_wp_idx = msg.data
        print(self.stopline_wp_idx)

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
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
