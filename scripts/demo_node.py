#! /usr/bin/env python

import rospy
from math import sqrt, pi
import numpy as np
import cv2
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from sensor_msgs.msg import Image
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
current_state = State()
def state_cb(msg):
    global current_state
    current_state = msg

img_np = np.ndarray((480, 640, 1))
q1 = float()
q2 = float()
q3 = float()
q4 = float()
mid_u = float()
mid_d = float()
def c_img_cb(msg):
    global mid_u
    global mid_d
    global q1
    global q2
    global q3
    global q4
    
    img_np = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width, 1)
    img_np= cv2.normalize(img_np, None, 1, 0, cv2.NORM_MINMAX, dtype=cv2.CV_32FC1)
    mid_u = np.mean(img_np[:240, 260:380])
    mid_d = np.mean(img_np[240:, 260:380])
    q1 = np.mean(img_np[:240, 320:])
    q2 = np.mean(img_np[:240, :320])
    q3 = np.mean(img_np[240:, :320])
    q4 = np.mean(img_np[240:, 320:])
    
    #rospy.loginfo(mid)
    #cv2.imshow('Depth', img_np[80:400, 260:380])
    #cv2.waitKey(1)
    

if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=1)
    
    c_img_sub = rospy.Subscriber("/camera/depth/image_raw", Image, callback = c_img_cb)
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)


    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose = PoseStamped()

    pose.pose.position.x = 0 
    pose.pose.position.y = 0
    pose.pose.position.z = 2
    #pose.pose.orientation.w = sqrt(0.5)
    #pose.pose.orientation.z = sqrt(0.5)
    

    # Send a few setpoints before starting
    for i in range(100):
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True
    first = True
    last_req = rospy.Time.now()

    while(not rospy.is_shutdown()):
        # First set the mode to offboard (refer to PX4 Flight Modes)
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")

            last_req = rospy.Time.now()

        # Armed the vehicle
        elif(not current_state.armed and ((rospy.Time.now() - last_req) > rospy.Duration(5.0))):
            if(arming_client.call(arm_cmd).success == True):
                rospy.loginfo("Vehicle armed")

        # Move the vehicle
        elif(current_state.armed and ((rospy.Time.now() - last_req) > rospy.Duration(4.0))):
            #mid = np.mean(img_np[80:400, 280:360])
            #left = np.mean(img_np[80:400, 0:320])
            #right = np.mean(img_np[80:400, 320:])
            #full = np.mean(img_np)
            #rospy.loginfo(right)

            if first:
                last_req = rospy.Time.now()
                first = False
                continue
            if max(q1, q2, q3, q4) == q1:
                pose.pose.position.x += 1
                pose.pose.position.y -= 1
                if pose.pose.position.z < 3:
                    pose.pose.position.z += 1

            elif max(q1, q2, q3, q4) == q2:
                pose.pose.position.x += 1
                pose.pose.position.y += 1
                if pose.pose.position.z < 3:
                    pose.pose.position.z += 1

            elif max(q1, q2, q3, q4) == q3:
                pose.pose.position.x += 1
                pose.pose.position.y += 1
                if pose.pose.position.z > 1:
                    pose.pose.position.z -= 1

            elif max(q1, q2, q3, q4) == q4:
                pose.pose.position.x += 1
                pose.pose.position.y -= 1
                if pose.pose.position.z > 1:
                    pose.pose.position.z -= 1     

            last_req = rospy.Time.now()


        local_pos_pub.publish(pose)

        rate.sleep() 
