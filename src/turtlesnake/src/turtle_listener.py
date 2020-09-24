#!/usr/bin/env python  

import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv
from turtlesnake.srv import SpawnTurtle


class TurtleListenerNode:
	# Square distance to catch the new turtle
	following_distance = 0.5

	def __init__(self):
		rospy.init_node('turtle_tf_listener')
		rospy.loginfo("Starting turtle listener node.")

		# Advertise the service start turtlesim
		rospy.Service('/start_turtlesim_snake', SpawnTurtle, self.start_turtlesim_snake)

		# Init some variables for the second turtle
		# Flag to know if the other turtle is following
		self.following = False
		# Whether the snake game has started or not
		self.turtlesim_snake = False
		self.listener = None
		self.turtle_vel = None
		self.rate = rospy.Rate(10)

	def run(self):
		while not rospy.is_shutdown():
			if self.turtlesim_snake is True:
				# Get transform
				try:
					(trans, rot) = self.listener.lookupTransform('/turtle2', '/turtle1', rospy.Time(0))
				except (tf.Exception, tf.LookupException, tf.ConnectivityException):
					continue

				if self.following is False:
					if math.sqrt(trans[0] ** 2 + trans[1] ** 2) < self.following_distance:
						rospy.loginfo("Now turtle follows.")
						self.following = True

				if self.following is True:
					angular = 4 * math.atan2(trans[1], trans[0])
					linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
					cmd = geometry_msgs.msg.Twist()
					cmd.linear.x = linear
					cmd.angular.z = angular
					self.turtle_vel.publish(cmd)

			self.rate.sleep()

	def start_turtlesim_snake(self, req):
		rospy.loginfo("Starting turtlesim snake.")

		self.listener = tf.TransformListener()

		rospy.loginfo("Spawning new turtle.")
		rospy.wait_for_service('spawn')
		spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
		spawner(req.x, req.y, req.theta, 'turtle2')

		self.turtle_vel = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)

		self.turtlesim_snake = True
		return True


def main():
	turtle_listener = TurtleListenerNode()
	turtle_listener.run()


if __name__ == '__main__':
	main()
