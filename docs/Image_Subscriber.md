# Image Subscriber Guide
This guide will walk you through the process of creating a subscriber node that subscribes to a the camera topic.

## Import the Required Packages
First be sure to import the required packages.
```python
import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
```

## Create a Subscriber Node
Now we need to create a subscriber node. This node will subscribe to the `/camera/depth/image_raw` topic and call the callback function whenever a new message on this topic is received.
```python
image_sub = rospy.Subscriber('/camera/depth/image_raw', Image, callback)
```

> You can change the topic to any other image topic that you want to subscribe to, for example `/camera/color/image_raw`.

## Create a Callback Function
Now we need to create the callback function that will be called whenever a new message is received on the topic. This function will convert the image message to a NumPy array and display it using OpenCV.

The image message format is `sensor_msgs/Image`, details about this message can be found [here](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html).

We take the data field of the message and convert it to a NumPy array using `np.frombuffer`. Then we reshape the array to the correct dimensions. Finally we convert the image from BGR to RGB (ROS uses RGB whereas OpenCV uses BGR) and display it using OpenCV.
```python
def callback(msg):
	try:
		# Convert the image message to a NumPy array
		img_np = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
		img_np = cv2.cvtColor(img_np, cv2.COLOR_BGR2RGB)
	except Exception as e:
		rospy.logerr(e)
		return

	# Display the image using OpenCV
	cv2.imshow('Image', img_np)
	cv2.waitKey(1)
```

## Create the Node
Finally we need to create the node and initialize it.
```python
rospy.init_node('image_subscriber')
```

Don't forget to add the following line to the end of the file to keep the node running.
```python
rospy.spin()
```

## Full Code
Here is the full code for a full subscriber node.

```python
#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np

class ImageSubscriber:
    def __init__(self):
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.callback)

    def callback(self, msg):
        try:
            # Convert the image message to a NumPy array
            img_np = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
            img_np = cv2.cvtColor(img_np, cv2.COLOR_BGR2RGB)
        except Exception as e:
            rospy.logerr(e)
            return

        # Display the image using OpenCV
        cv2.imshow('Image', img_np)
        cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('image_subscriber', anonymous=True)
    image_subscriber = ImageSubscriber()
    rospy.spin()
```