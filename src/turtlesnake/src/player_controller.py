#!/usr/bin/env python
import roslib
import rospy

import math
import turtlesim.msg
import geometry_msgs.msg
from turtlesnake.srv import GoTo


class PlayerController():
    # Interval to accept the current orientation
    orientation_deviation = 0.2
    distance_deviation = 0.2
    linear_speed = 1
    angular_speed = 4

    def __init__(self):
        self.current_x = 0
        self.current_y = 0
        self.current_theta = 0
        self.target_x = 0
        self.target_y = 0
        self.target_theta = 0
        self.vel_x = 0
        self.vel_theta = 0

        rospy.init_node('player_controller')
        self.turtlename = rospy.get_param('~turtle')
        # Subscriber to the player position
        rospy.Subscriber('/%s/pose' % self.turtlename, turtlesim.msg.Pose, self.handle_turtle_pose)
        # Publisher to the player speed
        self.player_vel_topic = rospy.Publisher(self.turtlename + '/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
        # Advertise the service go to position
        rospy.Service('/go_to_pos', GoTo, self.go_to_position)
        self.rate = rospy.Rate(10)

    def run(self):
        while not rospy.is_shutdown():
            self.update_velocities()
            self.rate.sleep()

    def update_velocities(self):
        self.target_theta = math.atan2(self.target_y - self.current_y, self.target_x - self.current_x)
        orientation_difference = self.target_theta - self.current_theta
        linear_distance = math.sqrt((self.target_x - self.current_x) ** 2 + (self.target_y - self.current_y) ** 2)
        rospy.logdebug('Target theta: ' + str(self.target_theta))
        rospy.logdebug('Orientation difference: ' + str(orientation_difference))

        if linear_distance > self.distance_deviation:
            # Orientate the turtle first
            if abs(orientation_difference) > self.orientation_deviation:
                self.vel_x = 0
                self.vel_theta = self.angular_speed * orientation_difference
            else:
                self.vel_theta = 0
                self.vel_x = self.linear_speed * linear_distance
        else:
            self.vel_x = 0
            self.vel_theta = 0

        cmd = geometry_msgs.msg.Twist()
        cmd.linear.x = self.vel_x
        cmd.angular.z = self.vel_theta
        self.player_vel_topic.publish(cmd)

    def handle_turtle_pose(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_theta = msg.theta

    def go_to_position(self, req):
        self.target_x = req.x
        self.target_y = req.y

        return True

if __name__ == '__main__':
    player_controller = PlayerController()
    player_controller.run()
