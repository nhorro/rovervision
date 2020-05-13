from .videoprocessinglayers import VideoProcessingLayer
import numpy as np
import cv2

class RoverHUD(VideoProcessingLayer):
    def __init__(self):
        VideoProcessingLayer.__init__(self)

        abspath=os.path.dirname(os.path.abspath(__file__))    
        # FIXME
        self.img_overlay = cv2.imread(abspath+"../../data/hud.png", cv2.IMREAD_COLOR)
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