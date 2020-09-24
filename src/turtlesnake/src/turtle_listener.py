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
	def __init__(self, name, parent, x=None, y=None, theta=None):
		rospy.loginfo("Spawning new turtle.")
		rospy.wait_for_service('spawn')

		node = roslaunch.core.Node('learning_tf_python', 'turtle_tf_broadcaster.py', args='_turtle:='+name)
		launch = roslaunch.scriptapi.ROSLaunch()
		launch.start()
		process = launch.launch(node)

		self.name = name
		self.turtle_vel = rospy.Publisher(self.name + '/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)

		# Flag to know if the other turtle is following
		self.following = False
		self.parent = parent

		spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
		if x is None:
			x = random.uniform(1, 10)
		if y is None:
			y = random.uniform(1, 10)
		if theta is None:
			theta = random.uniform(-3.14, 3.14)
		spawner(x, y, theta, name)


class TurtleListenerNode:
	# Square distance to catch the new turtle
	following_distance = 0.5

	def __init__(self):
		rospy.init_node('turtle_tf_listener')
		rospy.loginfo("Starting turtle listener node.")

		# Advertise the service start turtlesim
		rospy.Service('/start_turtlesim_snake', Empty, self.start_turtlesim_snake)

		# Whether the snake game has started or not
		self.turtlesim_snake = False
		self.turtles = []

		self.rate = rospy.Rate(10)

		self.listener = tf.TransformListener()

		random.seed()

	def run(self):
		while not rospy.is_shutdown():
			rospy.logdebug_once("Node running.")
			if self.turtlesim_snake is True:
				rospy.logdebug_once("Turtlesnake running.")
				# If there isnt any turtles spawn the first one
				if not self.turtles:
					rospy.logdebug("Turtle list empty.")
					new_turtle = Turtle('turtle2', '/turtle1')
					self.turtles.append(new_turtle)

				# If every turtle is following spawn a new one
				if all(turtle.following is True for turtle in self.turtles):
					new_turtle = Turtle('turtle' + str(len(self.turtles)+2), self.turtles[-1].name)
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

					# If following get tf respect the previous turtle and follow
					if turtle.following is True:
						try:
							(trans, rot) = self.listener.lookupTransform('/' + turtle.name, turtle.parent, rospy.Time(0))
						except (tf.Exception, tf.LookupException, tf.ConnectivityException):
							continue
						angular = 4 * math.atan2(trans[1], trans[0])
						linear = 2 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
						cmd = geometry_msgs.msg.Twist()
						cmd.linear.x = linear
						cmd.angular.z = angular
						turtle.turtle_vel.publish(cmd)

			self.rate.sleep()

	def start_turtlesim_snake(self, req):
		rospy.loginfo("Starting turtlesim snake.")
		self.turtlesim_snake = True
		return []


def main():
	turtle_listener = TurtleListenerNode()
	turtle_listener.run()


if __name__ == '__main__':
	main()
