#!/usr/bin/python
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2

dispW=320
dispH=240
flip=2
camSet='nvarguscamerasrc !  video/x-raw(memory:NVMM), width=3264, height=2464, format=NV12, framerate=21/1 ! nvvidconv flip-method='+str(flip)+' ! video/x-raw, width='+str(dispW)+', height='+str(dispH)+', format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'

def publish_image():
    """Capture frames from a camera and publish it to the topic /image_raw
    """
    image_pub = rospy.Publisher("image_raw", Image, queue_size=10)
    bridge = CvBridge()
    capture = cv2.VideoCapture(camSet)

    while not rospy.is_shutdown():
        # Capture a frame
        ret, img = capture.read()
        if not ret:
            rospy.ERROR("Could not grab a frame!")
            break
        # Publish the image to the topic image_raw
        try:
            img_msg = bridge.cv2_to_imgmsg(img, "bgr8")
            image_pub.publish(img_msg)
        except CvBridgeError as error:
            print(error)

if __name__=="__main__":
    rospy.init_node("my_cam", anonymous=True)
    print("Image is being published to the topic /image_raw ...")
    publish_image()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down!")