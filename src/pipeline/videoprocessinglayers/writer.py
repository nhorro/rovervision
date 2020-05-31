from . import VideoProcessingLayer
import numpy as np
import cv2
import rospy
import datetime

class VideoWriter(VideoProcessingLayer):
    def __init__(self, filename, w, h, fps):
        VideoProcessingLayer.__init__(self)
        self.filename_prefix = filename
        self.filename = ""
        self.is_recording = False
        self.w = w
        self.h = h
        self.fps = fps
    
    def setup(self, ctx):
        print("Instancing video writer")              

    def process(self, ctx):
        if self.is_recording:
            self.out.write(ctx["frame"])
        pass

    def start_recording(self):
        if self.is_recording == False:
            now = datetime.datetime.now()
            self.filename = self.filename_prefix + now.strftime("_%m%d%Y_%H%M%S") + ".avi"
            self.fourcc = cv2.VideoWriter_fourcc('H', '2', '6', '4')
            self.out = cv2.VideoWriter(
                self.filename, 
                cv2.CAP_FFMPEG, 
                self.fourcc, 
                self.fps, 
                ( self.w , self.h )
            )                      
            if False == self.out.isOpened():
                raise ValueError("Could not open WriterLayer output")    
            else:
                self.is_recording = True
                rospy.loginfo("Recording %s" % self.filename )
            
    def stop_recording(self):
        if self.is_recording:            
            self.is_recording = False
            self.out.release()    
            rospy.loginfo("Saved video %s" % self.filename )

    def is_recording(self):
        return self.is_recording
    
    def release(self, ctx):
        self.stop_recording()        
        pass