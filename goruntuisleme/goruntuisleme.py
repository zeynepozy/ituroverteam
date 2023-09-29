#!/usr/bin/env python

import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage


class ImageProcessingNode:
    def __init__(self):
        rospy.init_node('image_processing_node')

        self.image_sub = rospy.Subscriber('/video_topic/compressed', CompressedImage, self.image_callback)
        self.filter_sub = rospy.Subscriber('/filter', String, self.filter_callback)
        self.image_pub = rospy.Publisher('/rover_view/compressed', CompressedImage, queue_size=1)

        self.filter_option = 'GRAY' 

 rospy.Timer(rospy.Duration(2), self.random_filter_callback) #iki saniye       

    def image_callback(self, data):
        try:
         
            np_arr = np.fromstring(data.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

          
            if self.filter_option == 'GRAY':
                processed_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            elif self.filter_option == 'RGB':
                processed_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                #ikiye katlar
            elif self.filter_option == 'RESIZE_UP':
                processed_image = cv2.resize(image, None, fx=2, fy=2)
                #ikiye böler
            elif self.filter_option == 'RESIZE_DOWN':
                processed_image = cv2.resize(image, None, fx=0.5, fy=0.5)
            else:
                processed_image = image
n
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.data = np.array(cv2.imencode('.jpg', processed_image)[1])

            self.image_pub.publish(msg)

        except Exception as e:
            rospy.logerr("Hata: {}".format(e))

    def filter_callback(self, data):
      
        self.filter_option = data.data

if __name__ == '__main__':
    try:
        image_processing_node = ImageProcessingNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
