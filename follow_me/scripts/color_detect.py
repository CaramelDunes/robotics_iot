#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool

bridge = CvBridge()

pub=rospy.Publisher('color_detected', Bool, queue_size=10)

def image_callback(msg):
    # print("Received an image!")
        try:
        # Convert your ROS Image message to OpenCV2
                img = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
                print(e)
    # else:
        # Save your OpenCV2 image as a jpeg 
        # cv2.imwrite('camera_image.jpeg', cv2_img)
        # cv2.imshow('test',cv2_img)
        
        status =False

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        #defining the range of Yellow color
        yellow_lower = np.array([22,60,200],np.uint8)
        yellow_upper = np.array([60,255,255],np.uint8)
        #defining the range of Blue color
        blue_lower = np.array([60,100,100])
        blue_upper = np.array([130,255,255])

        #finding the range yellow colour in the image
        yellow = cv2.inRange(hsv, yellow_lower, yellow_upper)
        blue = cv2.inRange(hsv, blue_lower, blue_upper)

        #Morphological transformation, Dilation         
        kernal = np.ones((5 ,5), "uint8")

        # blue=cv2.dilate(yellow, kernal)

        res = cv2.bitwise_and(img, img, mask = yellow)
        res2 = cv2.bitwise_and(img, img, mask = blue)

        #Tracking Colour (Yellow) 
        (_,contours,hierarchy)=cv2.findContours(yellow,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        (_,contours2,hierarchy2)=cv2.findContours(blue,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        
        # for pic, contour in enumerate(contours):
        #         area = cv2.contourArea(contour)
        #         if(area>5000):
        #                 x,y,w,h = cv2.boundingRect(contour)     
        #                 img = cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,255),3)
        #                 # print("Yellow at (x,y)= (" +str(x+w/2) +","+ str(y+h/2) +")")
        #                 status=True

        for pic2, contour2 in enumerate(contours2):
                area2 = cv2.contourArea(contour2)
                if(area2>5000):
                        x,y,w,h = cv2.boundingRect(contour2)     
                        img = cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),3)
                        # print("blue at (x,y)= " +str(x+w/2) +","+ str(y+h/2) )
                        status=True

        pub.publish(status)                
                        
        cv2.imshow("Color Tracking",img)
        cv2.waitKey(1)
        # img = cv2.flip(img,1)
        # cv2.imshow("Yellow",res)
        # cv2.imshow("Blue", res2)
                               
        if cv2.waitKey(10) & 0xFF == 27:
                # cap.release()
                cv2.destroyAllWindows()
                # rospy.signal_shutdown('hek 7orr')

def main():
    # rospy.init_node('image_listener')
    rospy.init_node('Color_detector')
    image_topic = "/camera/image_raw"

    rospy.Subscriber(image_topic, Image, image_callback)
    

    # while not rospy.is_shutdown():
    rospy.spin()

if __name__ == '__main__':
    main()