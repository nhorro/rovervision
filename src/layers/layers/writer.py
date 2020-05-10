from .videoprocessinglayers import VideoProcessingLayer
import numpy as np
import cv2

class WriterLayer(VideoProcessingLayer):
    def __init__(self, filename, w, h, fps):
        VideoProcessingLayer.__init__(self)
        self.filename = filename
        self.w = w
        self.h = h
        self.fps = fps
    
    def setup(self, ctx):
        print("Instancing video writer")
        
        ctx["OUTPUT_FILENAME"] = self.filename
        ctx["OUTPUT_FPS"] = self.fps
        ctx["OUTPUT_W"] = self.w
        ctx["OUTPUT_H"] = self.h
                
        self.fourcc = cv2.VideoWriter_fourcc('H', '2', '6', '4')
        self.out = cv2.VideoWriter(
            ctx["OUTPUT_FILENAME"], 
            cv2.CAP_FFMPEG, 
            self.fourcc, 
            ctx["OUTPUT_FPS"], 
            (ctx["OUTPUT_W"],ctx["OUTPUT_H"])
        )
        
        if False == self.out.isOpened():
            raise ValueError("Could not open WriterLayer output")
        
        pass
    
    def process(self, ctx):
        self.out.write(ctx["OUTPUT_FRAME"])
        pass
    
    def release(self, ctx):
        self.out.release()    
        print("Generated video: ", ctx["OUTPUT_FILENAME"])
        pass