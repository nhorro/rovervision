from . import VideoProcessingLayer
import numpy as np
import cv2

class VideoDisplay(VideoProcessingLayer):
    def __init__(self):
        VideoProcessingLayer.__init__(self)
        self.window_name = "rovervision"
        
    def setup(self, ctx):
        cv2.namedWindow(self.window_name)            
    def process(self, ctx):              
        cv2.imshow(self.window_name,ctx["frame"])
    
    def release(self, ctx):
        pass