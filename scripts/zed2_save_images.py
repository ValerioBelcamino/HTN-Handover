#!/usr/bin/env python
import pyzed.sl as sl
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import os
import threading as th
import numpy as np
import time 

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
        self.current_image = np.zeros((1920, 1080, 3), np.uint8)
        self.lock = th.Lock()
        # self.image_sub = rospy.Subscriber("/zed2_driver", Image, self.save_img_callback)
        self.thread = th.Thread(target=self.save_img, args=(self.current_image,self.saving_path))
        self.stop_sleeping_sig = th.Event()
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

    def save_img(self):
        while True:
            # if self.current_image is not None:
            if self.stop_sleeping_sig.is_set():
                self.stop_sleeping_sig.clear()
            self.stop_sleeping_sig.wait()
            self.lock.acquire()
            tmp = np.array(self.current_image)
            # self.current_image = None
            self.lock.release()

            np.save(os.path.join('/externalSSD/htn_experiment/0/Zed', str(rospy.Time.now()) + '.npy'), tmp)
        return
    
    
    
    def save_img_callback(self, msg):
        if self.isSaving:
            image_ocv = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            image_resize = cv2.resize(image_ocv, (1920, 1080))
            self.lock.acquire()
            self.current_image = image_resize
            # if not self.stop_sleeping_sig.is_set():
            self.stop_sleeping_sig.set()
            # thread = th.Thread(target=cv2.imwrite, args=([os.path.join(self.saving_path, str(rospy.Time.now()) + '.png'),self.current_image]))
            # thread.start()
            # thread.join()
            #cv2.imwrite(os.path.join(self.saving_path, str(rospy.Time.now()) + '.png'), self.current_image)
            self.lock.release()
            # if self.first:
            #     self.thread.start()
            #     self.first = False
            # cv2.imwrite(os.path.join(self.saving_path, str(rospy.Time.now()) + '.png'), image_resize)
            # cv2.waitKey(0)
            # rospy.logerr("Saved image")


if __name__ == '__main__':
    ZS = Zed2Saver()
    thread = th.Thread(target=ZS.save_img).start()
    ZS.Start()