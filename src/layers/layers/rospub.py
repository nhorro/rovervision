from .videoprocessinglayers import VideoProcessingLayer
import numpy as np
import cv2

class ROSPublisher(VideoProcessingLayer):
    def __init__(self, image_pub_topic):
        VideoProcessingLayer.__init__(self)
        self.image_pub = rospy.Publisher(image_pub_topic,Image)        
        
    def setup(self, ctx):
        pass
    
    def process(self, ctx):                
      try:
          self.image_pub.publish(self.bridge.cv2_to_imgmsg(ctx["frame"], "bgr8"))
      except CvBridgeError as e:
        print(e)    

    def release(self, ctx):
        pass