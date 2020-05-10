from .videoprocessinglayers import VideoProcessingLayer
import numpy as np
import cv2

class ResizerLayer(VideoProcessingLayer):
    def __init__(self, output_w, output_h):
        VideoProcessingLayer.__init__(self)        
        self.output_w = output_w
        self.output_h = output_h
    
    def setup(self, ctx):
        pass
    
    def process(self, ctx):
        ctx["OUTPUT_FRAME"] = cv2.resize(
            ctx["OUTPUT_FRAME"],(self.output_w,self.output_h),interpolation=cv2.INTER_AREA)
        
    def release(self, ctx):
        pass