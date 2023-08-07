#!/usr/bin/python
# -*- coding: utf-8 -*-

# Copyright 2016 Massachusetts Institute of Technology

"""Extract images from a rosbag."""

import os
import argparse
import rospy
import cv2

import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import os

class getCustomArgs():
    def __init__(self):
        import rospkg
        rospack = rospkg.RosPack()  # get an instance of RosPack with the default search paths
        self.path = rospack.get_path('dualcamera_sync')   # get the package location where the bagfiles are. In my case it is within a package named dualcamera_sync.
        self.bag_folder_path = f"{self.path}/bags"
        self.bag_filenames = []
        self.output_dir = ''
        self.image_topics = ['/camera/color/image_raw','/kinect2/hd/image_color']   # MOdify these topic names as per your usecase.
        self.getfilenames()
        self.setOutputDir()
        
    def getfilenames(self):
        self.bag_filenames = [
            filename
            for filename in os.listdir(self.bag_folder_path)
            if filename.endswith(".bag")
        ]
        
    def setOutputDir(self):
        self.output_dir = f'{self.path}/extracted_bags' # Modify this as needed.
    
def extractAndSave(bagfile, foldername, args):
    
    bag_folder = args.bag_folder_path
    print(f"Extracting images from {bag_folder}/{bagfile} on topic(s) {args.image_topics} into {args.output_dir}/{foldername}")
    bag = rosbag.Bag(f'{bag_folder}/{bagfile}', "r")
    cvb = CvBridge()
    for count, (topic, msg, t) in enumerate(bag.read_messages(topics=args.image_topics)):
        try:   
            try:
                cv_image = cvb.imgmsg_to_cv2(msg, "rgb8")
                # print(msg.encoding)
            except CvBridgeError as e:
                print(e)

            image_normal = np.array(cv_image)
            image_normal = cv2.cvtColor(image_normal, cv2.COLOR_BGR2RGB)
            
            if topic == '/camera/color/image_raw':      # Modify if needed
                folder_path = f'{args.output_dir}/{foldername}/agent_0/'    # Modify if needed
            elif topic == '/kinect2/hd/image_color':    # Modify if needed
                folder_path = f'{args.output_dir}/{foldername}/agent_1/'    # Modify if needed
            else:
                folder_path = f'{args.output_dir}/{foldername}/other/'
                
            if not os.path.exists(folder_path):
                os.makedirs(folder_path)
                
            cv2.imwrite(os.path.join(folder_path, f"frame{count}.jpg"), image_normal)
            print(f"Wrote image {count} from {topic} in {folder_path}")
            
        except rospy.ROSInterruptException or KeyboardInterrupt:
            bag.close()
            rospy.signal_shutdown("Shutting down")
            cv2.destroyAllWindows()
    bag.close()
    return

def main():
    """Extract a folder of images from a rosbag.
    """
    # parser = argparse.ArgumentParser(
    #     description="Extract images from a ROS bag.")
    # parser.add_argument("--bagfile", help="Input ROS bag.")
    # parser.add_argument("--output_dir", help="Output directory.")
    # parser.add_argument("--image_topics", default=[''], help="Image topic.")

    # args = parser.parse_args()
    
    args = getCustomArgs()
    
    rospy.init_node("bag_to_rgb")
    
    # print(args.bag_filenames)

    # rate = rospy.Rate(1)
    rospy.loginfo("Running Bag to RGB extractor")
    # while not rospy.is_shutdown():
    j = 1
    foldername = 'myfolder'
    for i, bag in enumerate(sorted(args.bag_filenames)):
        # You typically wouldn't need the following lines until the function call extractAndSave(). You can safely comment it out.
        if bag in ['20230806-174810.bag', '20230806-175039.bag', '20230806-174701.bag', '20230806-174538.bag', '20230806-174921.bag']:
            if i > 4:
                foldername = f'fatigued_{j}'
                j += 1
        elif bag in ['20230806-173548.bag', '20230806-173802.bag', '20230806-173859.bag', '20230806-174135.bag', '20230806-174304.bag']:
            foldername = f'unfatigued_{i+1}'
        extractAndSave(bag, foldername, args)
        rospy.sleep(1)

if __name__ == '__main__':
    main()
