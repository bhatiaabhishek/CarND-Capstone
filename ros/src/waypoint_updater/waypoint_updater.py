#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLightArray
from geometry_msgs.msg import TwistStamped

import math
import tf

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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number

TARGET_SPEED = 5
SAMPLE_RATE = 50
ENABLE_TL = 1
class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb, queue_size=10)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        self.base_wp_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        # Subscribe to ground truth traffic light array for development

        # TODO: Add other member variables you need below
        self.car_x = None
        self.car_y = None
        self.car_yaw = None
        self.car_pose = None
        self.car_velo = None
        self.first_waypoint = None
        self.base_waypoints = None


        self.tl_list = None # list of all traffic lights 
        self.tl_X = None # closest traffic light X
        self.tl_Y = None # closest traffic light Y
        self.tl_S = None # closest traffic light state

        self.target_velo = TARGET_SPEED

        rospy.spin()

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
        self.publishFinalWaypoints()

    def publishFinalWaypoints(self):
        #if not received base_waypoints message, not able to update final_waypoints
        if self.base_waypoints is None:
            return
        #checking if closest waypoint is not the same as from the last final_waypoints update
        #then publishing updated final_waypoints message
        closestWaypoint = self.get_closest_waypoint(self.car_x, self.car_y)
        if self.tl_list is not None:
            #updating final_waypoints
            rospy.loginfo("updating waypoints")
            self.first_waypoint = closestWaypoint
            lenWaypoints = len(self.base_waypoints)
            final_waypoints_msg = Lane()
            for i in range(LOOKAHEAD_WPS):
                wp = self.base_waypoints[(closestWaypoint + i) % lenWaypoints]
                new_final_wp = Waypoint()
                new_final_wp.pose = wp.pose
                #currently using constant speed to get car moving
                new_final_wp.twist.twist.linear.x = self.car_velo + (((self.target_velo - self.car_velo)/LOOKAHEAD_WPS)*(i+1))
                final_waypoints_msg.waypoints.append(new_final_wp)
            
            if ENABLE_TL: final_waypoints_msg = self.calc_tl(final_waypoints_msg, closestWaypoint)
            self.final_waypoints_pub.publish(final_waypoints_msg)

    def waypoints_cb(self, msg):
        #updating base_waypoints
        if self.base_waypoints is None:
            rospy.loginfo("rcvd base waypoints")
            self.base_waypoints = msg.waypoints
            self.base_wp_sub.unregister()


    def traffic_cb(self, msg):
         #TODO: Callback for /traffic_waypoint message. Implement
        #updating traffic light waypoints
        self.tl_list = msg.lights
        rospy.loginfo("updated state = %s of TL 0 @ x = %s, y = %s", self.tl_list[0].state, self.tl_list[0].pose.pose.position.x, self.tl_list[0].pose.pose.position.y)
        rospy.loginfo("updated state = %s of TL 1 @ x = %s, y = %s", self.tl_list[1].state, self.tl_list[1].pose.pose.position.x, self.tl_list[1].pose.pose.position.y)
        rospy.loginfo("updated state = %s of TL 2 @ x = %s, y = %s", self.tl_list[2].state, self.tl_list[2].pose.pose.position.x, self.tl_list[2].pose.pose.position.y)
        rospy.loginfo("updated state = %s of TL 3 @ x = %s, y = %s", self.tl_list[3].state, self.tl_list[3].pose.pose.position.x, self.tl_list[3].pose.pose.position.y)

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
        closestLen = 100000
        closestWaypoint = 0
        for i in range(len(self.base_waypoints)):
            wp = self.base_waypoints[i]
            dist = math.sqrt((X - wp.pose.pose.position.x)**2
                                + (Y - wp.pose.pose.position.y)**2)
            if dist < closestLen:
                closestLen = dist
                closestWaypoint = i

        closest_wp = self.base_waypoints[closestWaypoint]
        heading = math.atan2(wp.pose.pose.position.y - Y,
                                wp.pose.pose.position.x - X)
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
    
    def calc_tl(self, final_waypoints_msg, car_closest_wp):
        if self.car_x is not None and self.tl_list is not None:
            closest_tl = self.get_closest_tl()
            closestWaypoint = self.get_closest_waypoint(self.tl_X, self.tl_Y)
            rospy.loginfo("tl_x = %s, tl_y = %s", self.tl_X, self.tl_Y) 
            rospy.loginfo("state = %s of TL %s", self.tl_S, closest_tl) 
            rospy.loginfo("I am at X = %s, Y = %s", self.car_x, self.car_y)
            dist = self.distance(self.base_waypoints, car_closest_wp, closestWaypoint)          
            rospy.loginfo("closest visible tl at %s distance", dist)
            if (self.tl_S == 0) and (dist < 40):
                for i in range(len(final_waypoints_msg.waypoints)):
                    final_waypoints_msg.waypoints[i].twist.twist.linear.x = 0
                rospy.loginfo("traffic light ahead %s ahead", dist)
            else:
                rospy.loginfo("setting target velo back to normal")
                self.target_velo = TARGET_SPEED
        return final_waypoints_msg
                
    def dist(self, p1, p2):
        return math.sqrt(pow(p1.x-p2.x,2) + pow(p1.y-p2.y,2))

    def current_velocity_cb(self, msg):
        self.car_velo = msg.twist.linear.x

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

