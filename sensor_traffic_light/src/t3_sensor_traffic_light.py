import rospy
import cv2
from sensor_msgs.msg import Image as Imageros
from t3_msgs import traffic_light_data
from t3_msgs import traffic_light_image
import numpy as np
from cv_bridge import CvBridge

NAME = "traffic_light"
SUB_TOPIC_CAM = "usb_cam/image_raw"
SUB_TOPIC_TRAFFIC = "traffic_light_image"
PUB_TOPIC = "traffic_light_data"
lower_red = (160, 50, 50)
upper_blue = (180,255,255)
WIDTH = 640
HEIGHT = 480

bridge = CvBridge()
cam_image = np.empty(shape=[0])


def callbackcam(msg):
    cam_image = bridge.imgmsg_to_cv2(msg, "bgr8")

def callbacktraffic(msg):
    pass

def detect_traffic():
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish()
        rate.sleep()

        if cam_image.shape[0] == 0:
                continue
        
if __name__ == '__main__':
    
    pub = rospy.Publisher(PUB_TOPIC, 1, queue_size=1)
    sub_cam = rospy.Subscriber(SUB_TOPIC_CAM,Imageros,callbackcam)
    sub_traffic = rospy.Subscriber(SUB_TOPIC_TRAFFIC,traffic_light_image,callbacktraffic)
    
    rospy.init_node(NAME, anonymous=True)
    detect_traffic()
    