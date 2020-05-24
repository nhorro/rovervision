from . import VideoProcessingLayer
import numpy as np
import cv2

class VideoDisplay(VideoProcessingLayer):
    def __init__(self):
        VideoProcessingLayer.__init__(self)
        
    def setup(self, ctx):
        cv2.namedWindow('Vision')            
    def process(self, ctx):              
        cv2.imshow('Vision',ctx["frame"])
    
    def release(self, ctx):
        pass