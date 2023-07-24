#!/usr/bin/env python
import pyzed.sl as sl
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import os

class Zed2Driver():
    def __init__(self, isSaving = False, saving_path = '/home/index1/index_ws/src/baxter_moveit/data'):
        rospy.init_node('zed2_driver', anonymous=True)
        self.image_pub = rospy.Publisher("/zed2_driver", Image, queue_size=10)
        self.isSaving = isSaving
        self.saving_path = saving_path
        self.zed, self.image_size, self.image_zed, self.depth_zed = self.init_zed(sl.RESOLUTION.HD2K)
        self.bridge = CvBridge()


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

        while True:
            image_ocv, depth_map = self.grab_zed_frame(self.zed, self.image_size, self.depth_zed, self.image_zed)
            # print(image_ocv.shape)
            
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
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            rospy.on_shutdown(self.on_shutdown_hook)

        cv2.destroyAllWindows()

    def on_shutdown_hook(self):
        self.zed.close()
        cv2.destroyAllWindows()
        exit(0)

if __name__ == '__main__':
    HD = Zed2Driver(isSaving=False, saving_path='/home/index1/index_ws/src/baxter_moveit/data')
    HD.loop()