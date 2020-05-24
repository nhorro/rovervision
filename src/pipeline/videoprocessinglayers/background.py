from . import VideoProcessingLayer
import numpy as np
import cv2

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