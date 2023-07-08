#! /usr/bin/env python

import rospy
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

d_img_np = np.ndarray((480, 640, 1))
depth_matrix = np.ndarray((3, 3))

def d_img_cb(msg):

    global depth_matrix
    
    d_img_np = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width, 1)
    d_img_np= cv2.normalize(d_img_np, None, 1, 0, cv2.NORM_MINMAX, dtype=cv2.CV_32FC1)
    #mid_u = np.mean(img_np[:240, 260:380])
    #mid_d = np.mean(img_np[240:, 260:380])
    depth_matrix[0, 0] = np.mean(d_img_np[:160, :240])
    depth_matrix[0, 1] = np.mean(d_img_np[:160, 240:400])
    depth_matrix[0, 2] = np.mean(d_img_np[:160, 400:])

    depth_matrix[1, 0] = np.mean(d_img_np[160:320, :240])
    depth_matrix[1, 1] = np.mean(d_img_np[160:320, 240:400])
    depth_matrix[1, 2] = np.mean(d_img_np[160:320, 400:])

    depth_matrix[2, 0] = np.mean(d_img_np[320:, :240])
    depth_matrix[2, 1] = np.mean(d_img_np[320:, 240:400])
    depth_matrix[2, 2] = np.mean(d_img_np[320:, 400:])

    #(corners, ids, rejected) = cv2.aruco.detectMarkers(img_np, aruco_Dictionary, parameters=aruco_Parameters)
    #rospy.loginfo(ids)
    #cv2.imshow('Depth', img_np[80:400, 260:380])
    #cv2.waitKey(1)


if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=1)
    
    d_img_sub = rospy.Subscriber("/camera/depth/image_raw", Image, callback = d_img_cb)
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
    move = 5e-2
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
        elif(current_state.armed):
            #mid = np.mean(img_np[80:400, 280:360])
            #left = np.mean(img_np[80:400, 0:320])
            #right = np.mean(img_np[80:400, 320:])
            #full = np.mean(img_np)
            #rospy.loginfo(right)
            
            if first:
                first = False
                rospy.loginfo("waiting for complete take off...")
                last_req = rospy.Time.now()
                while ((rospy.Time.now() - last_req) < rospy.Duration(5.0)):
                    local_pos_pub.publish(pose)
                    rate.sleep()
                continue

            if np.max(depth_matrix) == depth_matrix[0,0]:
                pose.pose.position.x += move
                pose.pose.position.y += move
                if pose.pose.position.z < 3:
                    pose.pose.position.z += 2*move
                rospy.loginfo('1')

            elif np.max(depth_matrix) == depth_matrix[0,1]:
                pose.pose.position.x += move
                if pose.pose.position.z < 3:
                    pose.pose.position.z += 2*move
                rospy.loginfo('2')

            elif np.max(depth_matrix) == depth_matrix[0,2]:
                pose.pose.position.x += move
                pose.pose.position.y -= move
                if pose.pose.position.z > 1:
                    pose.pose.position.z += 2*move
                rospy.loginfo('3')

            elif np.max(depth_matrix) == depth_matrix[1,0]:
                pose.pose.position.x += move
                pose.pose.position.y += move
                rospy.loginfo('4')
                

            elif np.max(depth_matrix) == depth_matrix[1,1]:
                pose.pose.position.x += move
                rospy.loginfo('5')

            elif np.max(depth_matrix) == depth_matrix[1,2]:
                pose.pose.position.x += move
                pose.pose.position.y -= move
                rospy.loginfo('6')

            elif np.max(depth_matrix) == depth_matrix[2,0]:
                pose.pose.position.x += move
                pose.pose.position.y += move
                if pose.pose.position.z > 1:
                    pose.pose.position.z -= 2*move
                rospy.loginfo('7')

            elif np.max(depth_matrix) == depth_matrix[2,1]:
                pose.pose.position.x += move
                if pose.pose.position.z > 1:
                    pose.pose.position.z -= 2*move
                rospy.loginfo('8')

            elif np.max(depth_matrix) == depth_matrix[2,2]:
                pose.pose.position.x += move
                pose.pose.position.y -= move
                if pose.pose.position.z > 1:
                    pose.pose.position.z -= 2*move
                rospy.loginfo('9')  
            
            if pose.pose.position.z > 3:
                pose.pose.position.z -= 2*move

            #last_req = rospy.Time.now()


        local_pos_pub.publish(pose)

        rate.sleep() 
