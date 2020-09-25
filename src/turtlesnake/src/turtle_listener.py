#!/usr/bin/env python  

import rospy
import roslaunch
import math
import random
import tf
import geometry_msgs.msg
import turtlesim.srv
from std_srvs.srv import Empty


class Turtle:
    """
    Class that defines everything necessary for the individual turtles that follow the player.
    The player turtle doesnt belong to this class.
    """
    def __init__(self, name, parent, x=None, y=None, theta=None):
        """
        Inits a new turtle member
        :param name: Name of the turtle
        :param parent: Name of the parent. Used to know which one to follow
        :param x,y,theta: Parameters for the spawning points
        """
        rospy.loginfo("Spawning new turtle.")

        self.name = name
        self.turtle_vel = rospy.Publisher(self.name + '/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)

        # Flag to know if the turtle is following its parent
        self.following = False
        self.parent = parent

        # Generate random position if not given
        if x is None:
            x = random.uniform(1, 10)
        if y is None:
            y = random.uniform(1, 10)
        if theta is None:
            theta = random.uniform(-3.14, 3.14)

        # Spawn the turtle in the simulator
        rospy.wait_for_service('spawn')
        spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
        spawner(x, y, theta, name)

        # Launch broadcaster node that publishes the tf of the turtle
        node = roslaunch.core.Node('learning_tf_python', 'turtle_tf_broadcaster.py', name=name + '_tf_broadcaster',
                                   args='_turtle:=' + name)
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()
        launch.launch(node)

    def set_parent(self, parent):
        self.parent = parent


class TurtleListenerNode:
    """
    Class that manages the tf listener node. It calculates and sends the velocities for all the turtles
    """

    # Square distance to catch the new turtle
    following_distance = 0.5
    # Linear speed of followers
    following_speed = 2
    # Number of spawning turtles
    turtle_number = 30

    def __init__(self):
        rospy.init_node('turtle_tf_listener')
        rospy.loginfo("Starting turtle listener node.")

        # Advertise the service start turtlesim
        rospy.Service('/start_turtlesim_snake', Empty, self.start_turtlesim_snake)

        # Whether the snake game has started or not
        self.playing_turtlesnake = False
        # List with all the turtles of the body, player not included
        self.turtles = []
        # List with the following turtles
        self.following_turtles = []

        self.rate = rospy.Rate(10)

        self.listener = tf.TransformListener()

        random.seed()

    def run(self):
        while not rospy.is_shutdown():
            rospy.logdebug_once("Node running.")
            if self.playing_turtlesnake is True:
                rospy.logdebug_once("Turtlesnake running.")

                # Spawn all the turtles in the beginning
                if not self.turtles:
                    rospy.logdebug("Turtle list empty.")
                    # First turtle of body
                    new_turtle = Turtle('turtle2', '/turtle1')
                    self.turtles.append(new_turtle)
                    # Rest
                    for i in range(self.turtle_number-1):
                        new_turtle = Turtle('turtle' + str(len(self.turtles) + 2), self.turtles[-1].name)
                        self.turtles.append(new_turtle)

                # If every turtle is following create a new one
                if all(turtle.following is True for turtle in self.turtles):
                    new_turtle = Turtle('turtle' + str(len(self.turtles) + 2), self.turtles[-1].name)
                    self.turtles.append(new_turtle)

                for turtle in self.turtles:
                    # If not following get tf respect the head and check if it is close
                    if turtle.following is False:
                        rospy.logdebug("Checking if close.")
                        try:
                            (trans, rot) = self.listener.lookupTransform('/' + turtle.name, '/turtle1', rospy.Time(0))
                        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
                            continue

                        if math.sqrt(trans[0] ** 2 + trans[1] ** 2) < self.following_distance:
                            rospy.loginfo("New turtle follows.")
                            turtle.following = True
                            # Follow the head or the previous turtle
                            if not self.following_turtles:
                                turtle.set_parent('/turtle1')
                            else:
                                turtle.set_parent(self.following_turtles[-1])

                            self.following_turtles.append('/' + turtle.name)

                    # If following get tf respect the previous turtle and follow
                    if turtle.following is True:
                        try:
                            (trans, rot) = self.listener.lookupTransform('/' + turtle.name, turtle.parent,
                                                                         rospy.Time(0))
                        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
                            continue
                        angular = 4 * math.atan2(trans[1], trans[0])
                        linear = self.following_speed * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
                        cmd = geometry_msgs.msg.Twist()
                        cmd.linear.x = linear
                        cmd.angular.z = angular
                        turtle.turtle_vel.publish(cmd)

            self.rate.sleep()

    def start_turtlesim_snake(self, req):
        '''
        Service callback
        '''
        rospy.loginfo("Starting turtlesim snake.")
        self.playing_turtlesnake = True
        return []


def main():
    turtle_listener = TurtleListenerNode()
    turtle_listener.run()


if __name__ == '__main__':
    main()
