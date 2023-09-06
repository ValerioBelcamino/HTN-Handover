#!/usr/bin/env python
import pyzed.sl as sl
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import os

class Zed2Saver():
    def __init__(self):
        rospy.init_node('zed2_saver', anonymous=True)
        self.experimentID = rospy.get_param('~experiment_ID', 'default')
        rospy.logerr("Experiment ID: " + str(self.experimentID))
        self.isSaving = rospy.get_param('~save', 'default')
        rospy.logerr("isSaving: " + str(self.isSaving))
        self.base_saving_path = rospy.get_param('~base_saving_path', 'default')
        rospy.logerr("base_saving_path: " + str(self.base_saving_path))
        self.saving_path = os.path.join(self.base_saving_path, str(self.experimentID))
    
        self.image_sub = rospy.Subscriber("/zed2_driver", Image, self.save_img_callback)

        self.bridge = CvBridge()

    def Start(self):
        if self.isSaving:
            if not os.path.exists(self.base_saving_path):
                os.makedirs(self.base_saving_path)
            self.saving_path = os.path.join(self.saving_path, 'Zed')
            if not os.path.exists(self.saving_path):
                os.makedirs(self.saving_path)
                rospy.logerr("Saving path: " + self.saving_path + " created")
            else:
                rospy.logerr("SAVING PATH: " + self.saving_path + " ALREADY EXISTS")
                exit(-1)
                
            self.image_sub = rospy.Subscriber("/zed2_driver", Image, self.save_img_callback)
        rospy.spin()
        return


    
    def save_img_callback(self, msg):
        image_ocv = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if self.isSaving:
                image_resize = cv2.resize(image_ocv, (1280, 720))
                cv2.imwrite(os.path.join(self.saving_path, str(rospy.Time.now()) + '.png'), image_resize)
        

if __name__ == '__main__':
    ZS = Zed2Saver()
    ZS.Start()