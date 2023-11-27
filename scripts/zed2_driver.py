#!/usr/bin/env python
import pyzed.sl as sl
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import os
import threading as th
import numpy as np

class Zed2Driver():
    def __init__(self, isSaving = False, saving_path = '/externalSSD/htn_experiment/'):
        rospy.init_node('zed2_driver', anonymous=True)
        self.image_pub = rospy.Publisher("/zed2_driver", Image, queue_size=10)
        self.isSaving = isSaving
        self.saving_path = saving_path
        self.zed, self.image_size, self.image_zed, self.depth_zed = self.init_zed(sl.RESOLUTION.HD2K)
        self.bridge = CvBridge()
        self.stop_looping = False

        self.qr = cv2.QRCodeDetector()
        self.marker_size = 0.6

        self.calibration_matrix_path = "/home/index1/index_ws/src/zed_cv2/zed2_calibration_matrix.npy"
        self.calibration_matrix = np.load(self.calibration_matrix_path)   

        self.distortion_path = "/home/index1/index_ws/src/zed_cv2/zed2_distortion.npy"
        self.distortion = np.load(self.distortion_path)



    def init_zed(self, resolution):
        # Create a ZED camera object
        zed = sl.Camera()

        # Set configuration parameters
        input_type = sl.InputType()

        init = sl.InitParameters(input_t=input_type)
        init.camera_resolution = resolution
        init.depth_mode = sl.DEPTH_MODE.QUALITY
        init.coordinate_units = sl.UNIT.MILLIMETER

        # Open the camera
        err = zed.open(init)
        if err != sl.ERROR_CODE.SUCCESS :
            print(repr(err))
            zed.close()
            exit(1)

        # Set runtime parameters after opening the camera
        runtime = sl.RuntimeParameters()
        runtime.sensing_mode = sl.SENSING_MODE.STANDARD
        image_size = zed.get_camera_information().camera_resolution
        print(image_size.width, image_size.height)
        image_zed = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)
        depth_zed = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.F32_C1)
        return zed, image_size, image_zed, depth_zed


    def grab_zed_frame(self, zed, image_size, depth_zed, image_zed):
        if zed.grab() == sl.ERROR_CODE.SUCCESS :
            # Retrieve the left image in sl.Mat
            zed.retrieve_image(image_zed, sl.VIEW.LEFT, sl.MEM.CPU, image_size)
            # Use get_data() to get the numpy array
            image_ocv = image_zed.get_data()
            
            zed.retrieve_measure(depth_zed, sl.MEASURE.DEPTH, sl.MEM.CPU, image_size)
            depth_ocv = depth_zed.get_data()
            return image_ocv, depth_ocv

    
    def loop(self):
        rospy.loginfo("I am listening to the camera")
        # print(zed.get_camera_information())
        real_ids = []
        iterations = 0
        tries = 0

        while not self.stop_looping or not rospy.is_shutdown():
            image_ocv, depth_map = self.grab_zed_frame(self.zed, self.image_size, self.depth_zed, self.image_zed)
            # print(image_ocv.shape)

            ret_qr, points = self.qr.detect(image_ocv)

            if points is not None:
                print(ret_qr)
                print(points)
                print(points.shape)

                marker_points = np.array([[-self.marker_size / 2, self.marker_size / 2, 0],
                                    [self.marker_size / 2, self.marker_size / 2, 0],
                                    [self.marker_size / 2, -self.marker_size / 2, 0],
                                    [-self.marker_size / 2, -self.marker_size / 2, 0]], dtype=np.float32)
                
                ret, rvec, tvec = cv2.solvePnP(marker_points, points, self.calibration_matrix, self.distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
                
                unitv_points = np.array([[0,0,0], [1,0,0], [0,1,0], [0,0,1]], dtype = 'float32').reshape((4,1,3))

                points, jac = cv2.projectPoints(unitv_points, rvec, tvec, self.calibration_matrix, self.distortion)

                colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (0,0,0)]

                if len(points) > 0:
                    points = points.reshape((4,2))

                    origin = (int(points[0][0]),int(points[0][1]) )

                    for p, c in zip(points[1:], colors[:3]):
                        p = (int(p[0]), int(p[1]))

                        #Sometimes qr detector will make a mistake and projected point will overflow integer value. We skip these cases. 
                        if origin[0] > 5*image_ocv.shape[1] or origin[1] > 5*image_ocv.shape[1]:break
                        if p[0] > 5*image_ocv.shape[1] or p[1] > 5*image_ocv.shape[1]:break

                        cv2.line(image_ocv, origin, p, c, 5)

                cv2.imshow('frame', image_ocv)

                image_message = self.bridge.cv2_to_imgmsg(image_ocv, encoding="passthrough")
                # print(image_message.height, image_message.width)
                # print(image_message.encoding)
                self.image_pub.publish(image_message)

                if self.isSaving:
                    cv2.imwrite(os.path.join(self.saving_path, str(rospy.Time.now()) + '.png'), image_ocv)
                    iterations += 1

                # image_ocv = bridge.imgmsg_to_cv2(image_message, desired_encoding="passthrough")
                # print(image_ocv.shape)
                image_resize = cv2.resize(image_ocv, (1280, 720))
                cv2.imshow("Image", image_resize)

                # print(os.path.join(self.saving_path, str(rospy.Time.now()) + '.png'))
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

                rospy.on_shutdown(self.on_shutdown_hook)


    def on_shutdown_hook(self):
        self.stop_looping = True
        self.zed.close()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    HD = Zed2Driver(isSaving=False, saving_path='/externalSSD/htn_experiment/')
    HD.loop()