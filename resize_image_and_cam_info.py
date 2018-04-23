import os
import argparse
import cv2
import numpy as np
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from copy import deepcopy

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--in_bag_file", help="input ROS bag")
    parser.add_argument("--out_bag_file", help="output ROS bag")
    parser.add_argument("--image_topic", help="image topic")
    parser.add_argument("--cam_info_topic", help="camera info topic")
    parser.add_argument("--desired_width", type=int)
    parser.add_argument("--desired_height", type=int)

    args = parser.parse_args()
    in_bag = rosbag.Bag(args.in_bag_file, "r")
    bridge = CvBridge()
    count = 0

    orig_width = None
    orig_height = None
    desired_width = args.desired_width
    desired_height = args.desired_height

    # figure out orig_width and orig_height first
    for topic, msg, t in in_bag.read_messages(args.cam_info_topic):
        orig_width = msg.width
        orig_height = msg.height
        break

    # scaling factors for camera_info in x and y directions
    s_x = desired_width / float(orig_width) 
    s_y = desired_height / float(orig_height)

    with rosbag.Bag(args.out_bag_file, 'w') as outbag:
        for topic, msg, t in in_bag.read_messages():
            # resize camera_info msgs' K and P accordingly
            if (topic == args.cam_info_topic):
                orig_K = msg.K
                orig_P = msg.P
                new_msg = deepcopy(msg)
                new_msg.width = desired_width
                new_msg.height = desired_height
                new_msg.K = [s_x*orig_K[0], orig_K[1], s_x*orig_K[2],\
                         orig_K[3], s_y*orig_K[4], s_y*orig_K[5],\
                         orig_K[6], orig_K[7], orig_K[8]]
                new_msg.P = [s_x*orig_P[0], orig_P[1], s_x*orig_P[2], orig_P[3],\
                        orig_P[4], s_y*orig_P[5], s_y*orig_P[6], orig_P[7],\
                        orig_P[8], orig_P[9], orig_P[10], orig_P[11]]
                # print new_msg
                outbag.write(topic, new_msg, t)

            # resize images accordingly
            elif (topic == args.image_topic):
                # read bayered image as a long vector
                bayer_img = np.frombuffer(msg.data, dtype='uint8')
                # resize bayered image to 2D
                bayer_img = np.reshape(bayer_img, (orig_height, orig_width, 1))
                # debayer the raw image
                color_img = cv2.cvtColor(bayer_img, cv2.COLOR_BAYER_BG2BGR)
                # resize the debayered image
                color_img = cv2.resize(color_img, (desired_width, desired_height))
                # cv2.imshow("color_img", color_img)
                # cv2.waitKey(1)
                # conver back to imgmsg
                new_msg = bridge.cv2_to_imgmsg(color_img, encoding='bgr8')
                outbag.write(topic, new_msg, t)

            else:
                outbag.write(topic, msg, t)
        in_bag.close()

if __name__=='__main__':
    main()