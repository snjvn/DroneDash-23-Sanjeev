#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np

class ImageSubscriber:
	def __init__(self):
		# Create the subscribers
		self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)
		self.rgb_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.rgb_callback)

	def depth_callback(self, msg):
		try:
			# Convert the image message to a NumPy array
			img_np = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width, 1)
		except Exception as e:
			rospy.logerr(e)
			return

		# Display the image using OpenCV
		cv2.imshow('Depth Image', cv2.normalize(img_np, None, 1, 0, cv2.NORM_MINMAX))
		cv2.waitKey(1)

	def rgb_callback(self, msg):
		try:
			# Convert the image message to a NumPy array
			img_np = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
		except Exception as e:
			rospy.logerr(e)
			return

		# Display the image using OpenCV
		cv2.imshow('RGB Image', cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR))
		cv2.waitKey(1)

if __name__ == '__main__':
	# Initialize the node
	rospy.init_node('image_subscriber', anonymous=True)
	# Create the subscriber
	image_subscriber = ImageSubscriber()
	# Spin and prevent exit
	rospy.spin()