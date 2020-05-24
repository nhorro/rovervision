from . import VideoProcessingLayer
import numpy as np
import cv2
import os
import cv2

import rospy
from roverbridge.msg import general_tmy

class RoverHUD(VideoProcessingLayer):
    def __init__(self):
        VideoProcessingLayer.__init__(self)

        abspath=os.path.dirname(os.path.abspath(__file__))    
        # FIXME
        self.img_overlay = cv2.imread(abspath+"/hud.png", cv2.IMREAD_COLOR)
        if type(self.img_overlay) == None:
            raise ValueError("Could not load HUD overlay")

        # HUD        
        self.font = cv2.FONT_HERSHEY_SIMPLEX   
        self.font_scale = 0.5
        self.colors = [ (0, 255, 0), (0, 0, 255) ]
        self.thickness = 2

        self.variable_overlays_bool = [
          [ "AHRS     [OK]", "AHRS [FAIL]  " , ( 32,  20), 1 ],
          [ "GPS      [OK]", "GPS  [FAIL]  " , (500,  20), 1 ],
          [ "SERIAL   [OK]", "SERIAL [FAIL]" , ( 32, 470), 0 ],           
          [ "ARMED        ", "DISARMED     " , (500, 470), 1 ],
        ]            

        self.variable_overlays_number = [
          [ "PITCH: ", (40, 390), 0 ],
          [ " ROLL: ", (40, 410), 0 ],
          [ "  YAW: ", (40, 430), 0 ]
        ]

        self.tmy_sub = rospy.Subscriber("/rover/tmy",general_tmy,self.on_tmy, queue_size=1)  

    def on_tmy(self, msg):
        self.variable_overlays_bool[0][3] = msg.ahrs_fail
        self.variable_overlays_bool[1][3] = msg.gps_fail
        self.variable_overlays_bool[2][3] = msg.serial_fail
        self.variable_overlays_bool[3][3] = msg.armed
        self.variable_overlays_number[0][2] = msg.pitch
        self.variable_overlays_number[1][2] = msg.roll
        self.variable_overlays_number[2][2] = msg.yaw


    def setup(self, ctx):
        pass

    def process(self, ctx):                
        # Draw HUD
        ctx["frame"] = cv2.addWeighted(ctx["frame"],1.0,self.img_overlay,1.0,0)
        for v in self.variable_overlays_bool:
            state = v[3]
            cv2.putText(
                ctx["frame"], 
                v[0+state], 
                v[2],
                self.font,  
                self.font_scale, 
                self.colors[state], 
                self.thickness, cv2.LINE_AA)

        for v in self.variable_overlays_number:
            cv2.putText(
                ctx["frame"], 
                v[0]+'{0:0.2f}'.format(v[2]), 
                v[1],
                self.font,  
                self.font_scale, 
                (0,100,255), 
                2, cv2.LINE_AA)            

        # Crosshair
        """
        cv2.putText(
                ctx["frame"], 
                "+",                 
                (315,244),
                self.font,  
                self.font_scale, 
                (0,255,0), 
                1, cv2.LINE_AA)  
        """                

    def release(self, ctx):
        pass