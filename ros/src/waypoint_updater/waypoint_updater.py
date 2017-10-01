#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLightArray, Intersection
from geometry_msgs.msg import TwistStamped

import math
import tf
import numpy as np

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

LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. You can change this number

TARGET_SPEED = 5.0
SAMPLE_RATE = 100
ENABLE_TL = 1
class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        #rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb, queue_size=1)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        self.base_wp_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)
        rospy.Subscriber('/traffic_waypoint', Intersection, self.traffic_waypoint_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        # Subscribe to ground truth traffic light array for development

        # TODO: Add other member variables you need below
        self.car_x = None
        self.car_y = None
        self.car_yaw = None
        self.car_pose = None
        self.car_velo = 0.0
        self.car_closest_wp = None
        self.first_waypoint = None
        self.base_waypoints = None
        self.base_waypoints_np = None
        self.update_intr = 0


        self.tl_list = None # list of all traffic lights
        self.tl_X = None # closest traffic light X
        self.tl_Y = None # closest traffic light Y
        self.tl_S = None # closest traffic light state
        self.tl_wp = None # nearest Waypoint to the next light
        self.tl_list = []

        self.target_velo = TARGET_SPEED

        self.time_out()

        rospy.spin()


    def time_out(self):

        rate = rospy.Rate(SAMPLE_RATE)
        while not rospy.is_shutdown():
            self.update_intr = 1

            rate.sleep()

    def pose_cb(self, msg):
        self.car_x = msg.pose.position.x
        self.car_y = msg.pose.position.y
        self.car_pose = msg.pose
        #need to know euler yaw angle for car orientation relative to waypoints
        #for quaternion transformation using https://answers.ros.org/question/69754/quaternion-transformations-in-python/
        quaternion = [msg.pose.orientation.x,
                        msg.pose.orientation.y,
                        msg.pose.orientation.z,
                        msg.pose.orientation.w]
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.car_yaw = euler[2]
        self.car_closest_wp = self.get_closest_waypoint(self.car_x, self.car_y)
        self.check_tl()
        self.publishFinalWaypoints()

    def publishFinalWaypoints(self):
        #if not received base_waypoints message, not able to update final_waypoints
        if self.base_waypoints is None:
            return
        #checking if closest waypoint is not the same as from the last final_waypoints update
        #then publishing updated final_waypoints message
        closestWaypoint = self.car_closest_wp #self.get_closest_waypoint(self.car_x, self.car_y)
        car_velocity = self.car_velo
        target_velocity = self.target_velo
        rospy.loginfo("Current car velo = %s", car_velocity)
        if self.update_intr == 1 or (closestWaypoint != self.first_waypoint):
            #updating final_waypoints
            rospy.loginfo("updating waypoints")
            rospy.loginfo("publishing velocity %s", target_velocity)
            self.first_waypoint = closestWaypoint
            lenWaypoints = len(self.base_waypoints)
            final_waypoints_msg = Lane()
            for i in range(LOOKAHEAD_WPS):
                wp = self.base_waypoints[(closestWaypoint + i) % lenWaypoints]
                new_final_wp = Waypoint()
                new_final_wp.pose = wp.pose
                #currently using constant speed to get car moving
                #if (self.target_velo == 0.0): # Brake reqd
                #    new_final_wp.twist.twist.linear.x = 0.0
                #    #rospy.loginfo("braking")
                #else:
                if (target_velocity == 0):
                    new_final_wp.twist.twist.linear.x = 0
                else:
                    new_final_wp.twist.twist.linear.x =  wp.twist.twist.linear.x #min(car_velocity + ((100*((target_velocity - car_velocity)/target_velocity)/LOOKAHEAD_WPS)*(i+1)),target_velocity)
                #rospy.loginfo("velo %s = %s, car velo = %s",i,new_final_wp.twist.twist.linear.x, car_velocity)
                final_waypoints_msg.waypoints.append(new_final_wp)

            #if ENABLE_TL: final_waypoints_msg = self.calc_tl(final_waypoints_msg, closestWaypoint)
            self.final_waypoints_pub.publish(final_waypoints_msg)
            self.update_intr = 0

    def waypoints_cb(self, msg):
        #updating base_waypoints
        if self.base_waypoints is None:
            rospy.loginfo("rcvd base waypoints")
            self.base_waypoints = msg.waypoints
            #creating numpy arrya which contains only x & y coordinates
            self.base_waypoints_np = np.array([[msg.waypoints[j].pose.pose.position.x, msg.waypoints[j].pose.pose.position.y] for j in range(len(msg.waypoints))])
            self.base_wp_sub.unregister()

    def traffic_waypoint_cb(self,msg):
         self.tl_X = msg.next_light.pose.pose.position.x
         self.tl_Y = msg.next_light.pose.pose.position.y
         self.tl_S = msg.next_light_detection
         self.tl_wp = msg.next_light_waypoint
         self.tl_stop_wp = msg.stop_line_waypoint
         #self.check_tl()


    def traffic_cb(self, msg):
         #TODO: Callback for /traffic_waypoint message. Implement
        #updating traffic light waypoints
        self.tl_list = msg.lights
        #rospy.loginfo("updated state = %s of TL 0 @ x = %s, y = %s", self.tl_list[0].state, self.tl_list[0].pose.pose.position.x, self.tl_list[0].pose.pose.position.y)
        #rospy.loginfo("updated state = %s of TL 1 @ x = %s, y = %s", self.tl_list[1].state, self.tl_list[1].pose.pose.position.x, self.tl_list[1].pose.pose.position.y)
        #rospy.loginfo("updated state = %s of TL 2 @ x = %s, y = %s", self.tl_list[2].state, self.tl_list[2].pose.pose.position.x, self.tl_list[2].pose.pose.position.y)
        #rospy.loginfo("updated state = %s of TL 3 @ x = %s, y = %s", self.tl_list[3].state, self.tl_list[3].pose.pose.position.x, self.tl_list[3].pose.pose.position.y)

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def get_closest_waypoint(self, X, Y):
        deltas = self.base_waypoints_np - [X, Y]
        dist_2 = np.einsum('ij,ij->i', deltas, deltas)
        closestWaypoint = np.argmin(dist_2)
        closest_wp = self.base_waypoints_np[closestWaypoint]
        heading = math.atan2(closest_wp[1] - Y,
                                closest_wp[0] - X)
        angle = abs(self.car_yaw - heading)
        if (angle > math.pi/4):
            closestWaypoint += 1
            closestWaypoint %= len(self.base_waypoints)
        return closestWaypoint

    def get_closest_tl(self):
        closestLen = 100000
        closestTL = -1
        for i in range(len(self.tl_list)):
            tl = self.tl_list[i]
            dist = math.sqrt((self.car_x - tl.pose.pose.position.x)**2
                                + (self.car_y - tl.pose.pose.position.y)**2)
            if dist < closestLen:
                closestLen = dist
                closestTL = i

        closest_tl = self.tl_list[closestTL]
        heading = math.atan2(closest_tl.pose.pose.position.y - self.car_y,
                                closest_tl.pose.pose.position.x - self.car_x)
        angle = abs(self.car_yaw - heading)
        if (angle > math.pi/4):
            closestTL += 1
            closestTL %= len(self.tl_list)
            closest_tl = self.tl_list[closestTL]
            closestLen = math.sqrt((self.car_x - closest_tl.pose.pose.position.x)**2
                                + (self.car_y - closest_tl.pose.pose.position.y)**2)
            rospy.loginfo("changing TL to %s", closestTL)
        self.tl_Y = closest_tl.pose.pose.position.y
        self.tl_X = closest_tl.pose.pose.position.x
        self.tl_S = closest_tl.state
        return closestTL

    def check_tl(self):
        #rate = rospy.Rate(SAMPLE_RATE)
        #while not rospy.is_shutdown():
           if self.car_x is not None and self.tl_wp is not None:
                closestWaypoint = self.tl_wp
                rospy.loginfo("tl_x = %s, tl_y = %s, state = %s, WP = %s", self.tl_X, self.tl_Y, self.tl_S, self.tl_wp)
                #rospy.loginfo("state = %s", self.tl_S)
                #rospy.loginfo("I am at X = %s, Y = %s", self.car_x, self.car_y)
                dist = self.distance(self.base_waypoints, self.car_closest_wp, closestWaypoint)
                rospy.loginfo("closest visible tl at %s distance", dist)
                # Our traffic_waypoint publishes only when the next light is red/orange or unknown.
                if dist < 35 and dist > 18: ### STOP!!!
                    self.update_intr = 1
                    self.target_velo = 0.0
                #elif dist < 40 and dist > 34:
                #    #self.update_intr = 1
                #    self.target_velo = 1.8
                else: ## FULL THROTTLE!!
                    self.target_velo = TARGET_SPEED
                    #rospy.loginfo("Setting velo to %s",self.target_velo)
            #rate.sleep()

    def dist(self, p1, p2):
        return math.sqrt(pow(p1.x-p2.x,2) + pow(p1.y-p2.y,2))

    def current_velocity_cb(self, msg):
        self.car_velo = msg.twist.linear.x

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
