#! /usr/bin/env python3

#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
cv2.__version__
import numpy as np
from geometry_msgs.msg import Twist


class image_converter:

  def __init__(self):
    #self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/rrbot/camera1/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)


    # gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
    # color = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    _, img_bin = cv2.threshold(cv_image, 119, 255, cv2.THRESH_BINARY)


    foundLine = True
    rowOfCircle = 200
    startOfLine = 0
    endOfLine = 0

    # The x-position of the ends of the line is detected:

    for j in range(799):

      for i in range(799):
          if (img_bin[799 - j][i][1] < 50):
            endOfLine = i
            if (foundLine):
                startOfLine = i
                foundLine = False
          
      if endOfLine != 0:
        rowOfCircle = j
        break


    # The x-position of the midpoint of the line is calculated:
    colOfCircle = int(startOfLine + (endOfLine - startOfLine)/2)


    # A circle is added to each frame of the video on the midpoint
    cv2.circle(img=cv_image, center = (colOfCircle,798 - rowOfCircle), radius =20, color =(255,0,0), thickness=-1)


    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    rate = rospy.Rate(2)
    move = Twist()

    move.angular.z = 0
    if (colOfCircle > 350 and colOfCircle < 450):
        move.angular.z = 0
    
    if (colOfCircle < 350):
        move.angular.z = 2

    if (colOfCircle > 450):
        move.angular.z = -2

    move.linear.x = 0.5

    pub.publish(move)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)