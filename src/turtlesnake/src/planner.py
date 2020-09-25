#!/usr/bin/env python
import roslib
import rospy

import math
import turtlesim.msg
import geometry_msgs.msg
from std_srvs.srv import Empty
from turtlesnake.srv import GoTo


class Point:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def distance_to_point(self, other_point):
        return math.sqrt((self.x - other_point.x) ** 2 + (self.y - other_point.y) ** 2)


class Target:
    def __init__(self, name):
        self.position = Point()
        # Subscriber to the turtle position
        rospy.Subscriber('/%s/pose' % name, turtlesim.msg.Pose, self.handle_target_pose)

    def handle_target_pose(self, req):
        self.position = Point(req.x, req.y)


class Planner:
    catching_distance = 0.5
    maximum_turtles = 30
    def __init__(self):
        self.targets = []
        self.player_position = Point()

        rospy.init_node('planner')
        # Create 10 turtles
        for i in range(self.maximum_turtles):
            new_target = Target('turtle' + str(i+2))
            self.targets.append(new_target)
        # Subscriber to the player position
        rospy.Subscriber('/%s/pose' % 'turtle1', turtlesim.msg.Pose, self.handle_turtle_pose)

        # ServiceProxy to the go_to_pos from the player controller
        rospy.wait_for_service('/go_to_pos')
        self.go_to_service = rospy.ServiceProxy('/go_to_pos', GoTo)

        # Advertise the service to catch all the turtles
        rospy.Service('/catchem_all', Empty, self.catch_them_all)

        self.rate = rospy.Rate(10)
        rospy.spin()

    def catch_them_all(self, req):
        trajectory = self.get_closest_trajectory()
        while trajectory:
            rospy.logdebug(str(len(trajectory)) + ' points remaining.')
            if self.player_position.distance_to_point(trajectory[0]) < self.catching_distance:
                trajectory.pop(0)
                if not trajectory:
                    return True
                rospy.logdebug('Go to ' + str(trajectory[0].x) + ' ' + str(trajectory[0].y))
                self.go_to_service(trajectory[0].x, trajectory[0].y)
            self.rate.sleep()

    def get_closest_trajectory(self):
        trajectory_points = [self.player_position]
        while self.targets:
            rospy.logdebug(str(len(self.targets)) + ' targets left.')
            # Higher value than possible
            index = min(range(len(self.targets)), key=lambda i: trajectory_points[-1].distance_to_point(self.targets[i].position))
            trajectory_points.append(self.targets[index].position)
            self.targets.pop(index)

        return trajectory_points

    def handle_turtle_pose(self, msg):
        self.player_position = Point(msg.x, msg.y)


if __name__ == '__main__':
    planner = Planner()
