#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('rovervision')
import argparse
import numpy as np
import sys
import os
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image,CompressedImage
from roverbridge.msg import general_tmy
from cv_bridge import CvBridge, CvBridgeError

from pipeline.videoprocessinglayers import VideoProcessingLayer

from pipeline.videoprocessinglayers.background import BackgroundExtractor1, BackgroundExtractor2
from pipeline.videoprocessinglayers.roverhud import RoverHUD
from pipeline.videoprocessinglayers.display import VideoDisplay
from pipeline.videoprocessinglayers.writer import VideoWriter
from pipeline.videoprocessinglayers.tensorflowobjectdetection import ObjectDetectionLayer, DrawBoundingBoxesLayer

OBJECT_DETECTION_SERVICE_ENDPOINT = 'localhost:8500'
OBJECT_DETECTION_MODEL_SPEC = 'ssd_mobilenet_v2_coco'
OBJECT_DETECTION_LABELS = "../data/tensorflowmodels/labels/coco-spanish.pb.txt"

DEFAULT_CAMERA_TOPIC_RAW = "/camera/image_raw"
DEFAULT_CAMERA_TOPIC_COMPRESSED = "/camera/image_raw/compressed"

class RoverVision:

  def __init__(self, node_name, image_sub_topic, log_level):
    self.node_name = node_name
    rospy.init_node(node_name, log_level=log_level)
    rospy.loginfo("Starting RoverVision.")    
        
    self.bridge = CvBridge()
    self.ctx = {}

    self.hud = RoverHUD(640,480)
    self.video_writer = VideoWriter("captura",640,480,25)
    self.layers = [
      BackgroundExtractor1(), # 0
      BackgroundExtractor2(), # 1
      
      # Tensorflow # 2,3     
      ObjectDetectionLayer(
          OBJECT_DETECTION_SERVICE_ENDPOINT,
          OBJECT_DETECTION_MODEL_SPEC
      ),
      DrawBoundingBoxesLayer(OBJECT_DETECTION_LABELS),
      # End WIP

      self.hud, # 4

      VideoDisplay(), # 5

       # 6
       self.video_writer
    ]

    # Default layer state
    layer_states = [
      False,  # 0
      False,  # 1
      True,   # 2 
      True,   # 3
      True,   # 4
      True,   # 5
      True,   # 6
    ]
    
    for l in range(len(layer_states)):
      self.layers[l].enable(layer_states[l])
      self.hud.set_pipeline_stage_state(l, layer_states[l] )

    rospy.loginfo("Initializing layers...")
    for l in self.layers:
      l.setup(self.ctx)

    self.image_sub = rospy.Subscriber(image_sub_topic,CompressedImage,self.on_frame_compressed, queue_size=1)

  def on_frame_compressed(self,image_message):    
      np_arr = np.fromstring(image_message.data, np.uint8)            
      self.ctx["frame"] = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
      rows,cols,channels = self.ctx["frame"].shape
      self.ctx["rows"] = rows
      self.ctx["cols"] = cols
      self.ctx["channels"] = channels
      self.run_pipeline()

  def run_pipeline(self):    
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
          self.hud.set_pipeline_stage_state(layer, new_state)
      elif key_pressed == ord('r'):
        self.video_writer.start_recording()
      elif key_pressed == ord('t'):
        self.video_writer.stop_recording()


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
    parser.add_argument('--name', type=str, default="rovervision", help='Node name' )
    parser.add_argument('--loglevel', type=int, default=rospy.DEBUG, help='Loglevel (0=trace, 6=critical)' )
    parser.add_argument('--camera_topic', type=str, default=DEFAULT_CAMERA_TOPIC_COMPRESSED, help='Image topic' )
    args, unknown = parser.parse_known_args()
    vision = RoverVision(
      node_name=args.name,
      image_sub_topic=args.camera_topic,
      log_level=args.loglevel)
    vision.run()    
