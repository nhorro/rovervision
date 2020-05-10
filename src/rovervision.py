#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('rovervision')
import argparse
import numpy as np
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image,CompressedImage
from cv_bridge import CvBridge, CvBridgeError

from layers import VideoProcessingLayer

# --- begin Layers - TODO ordenar --------------------------------------------------------

# -----------------------------------------------------------------------------------

class BackgroundExtractor2(VideoProcessingLayer):
    def __init__(self):
        VideoProcessingLayer.__init__(self)
        self.subtractor = cv2.createBackgroundSubtractorMOG2()
        
    def setup(self, ctx):
        pass

    def process(self, ctx):                
        mask = self.subtractor.apply(ctx["frame"])
        ctx["frame"] = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

    def release(self, ctx):
        pass

# -----------------------------------------------------------------------------------

class BackgroundExtractor1(VideoProcessingLayer):
    def __init__(self):
        VideoProcessingLayer.__init__(self)
        self.count = 0
        
    def setup(self, ctx):
        pass

    def process(self, ctx):                
        gray = cv2.cvtColor(ctx["frame"], cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5,5), 0)
        if self.count > 0:          
          difference = cv2.absdiff(gray,  self.prev_frame)
          _, difference = cv2.threshold(difference, 25, 255, cv2.THRESH_BINARY )
          ctx["frame"] = cv2.cvtColor(difference, cv2.COLOR_GRAY2BGR)
          #ctx["difference"] = difference
          #cv2.imshow('Difference',difference)
        #else:

        self.prev_frame = gray
        self.count+=1
        
        
    def release(self, ctx):
        pass

# -----------------------------------------------------------------------------------

class VideoDisplay(VideoProcessingLayer):
    def __init__(self):
        VideoProcessingLayer.__init__(self)
        
    def setup(self, ctx):
        cv2.namedWindow('Vision')            
    def process(self, ctx):              
        cv2.imshow('Vision',ctx["frame"])
    
    def release(self, ctx):
        pass


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

class RoverHUD(VideoProcessingLayer):
    def __init__(self):
        VideoProcessingLayer.__init__(self)
        self.img_overlay = cv2.imread("../data/hud.png", cv2.IMREAD_COLOR)
        if type(self.img_overlay) == None:
            raise ValueError("Could not load HUD overlay")

        # HUD        
        self.font = cv2.FONT_HERSHEY_SIMPLEX   
        self.font_scale = 0.5
        self.colors = [ (0, 255, 0), (0, 0, 255) ]
        self.thickness = 2

        self.variable_overlays = [
          [ "AHRS     [OK]", "AHRS [FAIL]"   , ( 32,  20), 1 ],
          [ "GPS      [OK]", "GPS  [FAIL]"   , (500,  20), 1 ],
          [ "SERIAL   [OK]", "SERIAL [FAIL]" , ( 32, 470), 0 ],           
          [ "ARMED        ", "DISARMED"      , (500, 470), 1 ]
        ]            

    def setup(self, ctx):
        pass

    def process(self, ctx):                
        # Draw HUD
        ctx["frame"] = cv2.addWeighted(ctx["frame"],1.0,self.img_overlay,1.0,0)
        for v in self.variable_overlays:
            state = v[3]
            cv2.putText(
                ctx["frame"], 
                v[0+state], 
                v[2],
                self.font,  
                self.font_scale, 
                self.colors[state], 
                self.thickness, cv2.LINE_AA)

        # Crosshair
        cv2.putText(
                ctx["frame"], 
                "+",                 
                (315,244),
                self.font,  
                self.font_scale, 
                (0,255,0), 
                1, cv2.LINE_AA)  

    def release(self, ctx):
        pass

# -----------------------------------------------------------------------------------

class RoverVision:

  def __init__(self, node_name, image_sub_topic, log_level):
    self.node_name = node_name
    rospy.init_node(node_name, log_level=log_level)
    rospy.loginfo("Starting RoverVision.")    
    
    
    self.bridge = CvBridge()
    self.ctx = {}

    self.layers = [
      BackgroundExtractor1(),
      BackgroundExtractor2(),
      RoverHUD(),
      VideoDisplay()
    ]

    # Default layer state
    self.layers[0].enable(False)
    self.layers[1].enable(False)

    rospy.loginfo("Initializing layers...")
    for l in self.layers:
      l.setup(self.ctx)

    self.image_sub = rospy.Subscriber(image_sub_topic,CompressedImage,self.on_frame)            

  def on_frame(self,image_message):    
      np_arr = np.fromstring(image_message.data, np.uint8)      
      self.ctx["frame"] = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
      rows,cols,channels = self.ctx["frame"].shape
      self.ctx["rows"] = rows
      self.ctx["cols"] = cols
      self.ctx["channels"] = channels

      for l in self.layers:
        if l.is_enabled():
          l.process(self.ctx)

      key_pressed = cv2.waitKey(1)
      if key_pressed >= ord('1') and key_pressed<=ord('9'):        
        layer = key_pressed-ord('1')
        if(layer < len(self.layers)):
          new_state = not self.layers[layer].is_enabled()
          rospy.loginfo("Setting layer %d to %d" % (layer, new_state) )
          self.layers[layer].enable( new_state )


  def test_overlay(self):
      self.img_overlay = cv2.imread("../data/hud.png", cv2.IMREAD_COLOR)
      print(self.img_overlay.shape)


  def run(self):
    try:
      rospy.spin()
    except KeyboardInterrupt:
      print("Shutting down")
    cv2.destroyAllWindows()    
    for l in self.layers:
      l.release(self.ctx)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Rover Vision')
    #parser.add_argument('--vehicle_id', type=str, default=DEFAULT_ROVER_ID, help='rover identifier')    
    parser.add_argument('--loglevel', type=int, default=rospy.DEBUG, help='loglevel (0=trace, 6=critical)' )
    args = parser.parse_args()
    vision = RoverVision(
      node_name="rovervision",
      image_sub_topic="/usb_cam/image_raw/compressed",
      log_level=args.loglevel)
    #vision.test_overlay()    
    vision.run()    
