from . import VideoProcessingLayer
import numpy as np
import cv2
import os
import cv2

import rospy
from roverbridge.msg import general_tmy


class Sprite:
    x = 400
    y = 400
    cx = 0
    cy = 0
    angle = 0

    def __init__(self,spritesheet_img,x0,y0,w,h):
        self.img = spritesheet_img[y0:y0+h,x0:x0+w]
        # Extract the alpha mask of the RGBA image, convert to RGB 
        b,g,r,a = cv2.split(self.img)
        self.overlay_color = cv2.merge((b,g,r))
        
        # Apply some simple filtering to remove edge noise
        self.mask = cv2.medianBlur(a,5)
        self.h, self.w, _ = self.overlay_color.shape
        self.cx = self.w/2
        self.cy = self.h/2
        
    def blit(self, dst):
        x = self.x - self.cx
        y = self.y - self.cy

        roi = dst[y:y+self.h, x:x+self.w]

        # Black-out the area behind the logo in our original ROI
        img1_bg = cv2.bitwise_and(roi.copy(),roi.copy(),mask = cv2.bitwise_not(self.mask))
        
        # Mask out the logo from the logo image.
        img2_fg = cv2.bitwise_and(self.overlay_color,self.overlay_color,mask = self.mask)

        # Update the original image with our new ROI
        dst[y:y+self.h, x:x+self.w] = cv2.add(img1_bg, img2_fg)


    def rotate(self,angle):
        self.angle += angle
        self.update_rotation()

    def set_rotation(self,angle):
        self.angle = angle
        self.update_rotation()


    def update_rotation(self):
        if self.angle >= 360:
            self.angle -= 360
        elif self.angle < 0:
            self.angle += 360
        M = cv2.getRotationMatrix2D((self.cx,self.cy),self.angle,1)
        self.img_rotated = cv2.warpAffine(self.img,M,(self.w,self.h))        
        b,g,r,a = cv2.split(self.img_rotated)
        self.overlay_color_rotated = cv2.merge((b,g,r))
        self.mask_rotated = cv2.medianBlur(a,5)
        self.hr,self.wr, _ = self.overlay_color_rotated.shape


    def blit_rotated(self,dst):
        x = self.x - self.cx
        y = self.y - self.cy
        roi = dst[y:y+self.hr, x:x+self.wr]
        img1_bg = cv2.bitwise_and(roi.copy(),roi.copy(),mask = cv2.bitwise_not(self.mask_rotated))
        img2_fg = cv2.bitwise_and(self.overlay_color_rotated,self.overlay_color_rotated,mask = self.mask_rotated)

        # Update the original image with our new ROI
        dst[y:y+self.hr, x:x+self.wr] = cv2.add(img1_bg, img2_fg)

class RoverHUD(VideoProcessingLayer):
    pitch = 0
    roll = 0
    yaw = 0
    yaw_declination = -90-41
    def __init__(self, w, h):
        VideoProcessingLayer.__init__(self)

        abspath=os.path.dirname(os.path.abspath(__file__))    
        # FIXME
        #self.img_overlay = cv2.imread(abspath+"/hud.png", cv2.IMREAD_COLOR)
        #if type(self.img_overlay) == None:
        #    raise ValueError("Could not load HUD overlay")

        self.hud_sprites = cv2.imread(abspath+"/hud_ahrs.png",-1)

        # AHRS HUD
        # ----------------------------------------------------------------

        # Crosshair
        self.crosshair = Sprite(self.hud_sprites, 60,80, 200, 120)
        self.crosshair.x = w/2
        self.crosshair.y = h/2
        self.crosshair.rotate(0)

        # Compass
        self.compass = Sprite(self.hud_sprites, 300,0, 120, 120)
        self.compass.x = w-self.compass.w/2-20
        self.compass.y = h-self.compass.h/2-20    

        # Compass :: needle # Acomodar!!!
        self.needle = Sprite(self.hud_sprites, 280, 180, 40, 40)
        self.needle.cy = 20
        self.needle.x = self.compass.x   
        self.needle.y = self.compass.y 
        #self.needle.rotate(0)

        # Variable overlays HUD
        # ----------------------------------------------------------------

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

        # Pipeline Config
        # ----------------------------------------------------------------
        self.pipeline_text = [ 
            ["0|MOTDET1", False],
            ["1|MOTDET2", False],
            ["2|OBJDET", False],
            ["3|OBJDET", False],
            ["4|   HUD", False],
            ["5|DISPLAY", False],
            ["6|FILEREC", False]
        ]
        self.pipeline_text_y0 = 20
        self.pipeline_text_x0 = 170
        self.pipeline_text_w = 100
        self.pipeline_text_h = 16
        self.pipeline_text_colors = [ (100,100,100), (255,255,255) ] 

        self.tmy_sub = rospy.Subscriber("/rover/tmy",general_tmy,self.on_tmy, queue_size=1) 

    def set_pipeline_stage_state(self, stage, state):
        self.pipeline_text[stage][1] = state

    def on_tmy(self, msg):
        self.variable_overlays_bool[0][3] = msg.ahrs_fail
        self.variable_overlays_bool[1][3] = msg.gps_fail
        self.variable_overlays_bool[2][3] = msg.serial_fail
        self.variable_overlays_bool[3][3] = msg.armed
        self.variable_overlays_number[0][2] = msg.pitch
        self.variable_overlays_number[1][2] = msg.roll
        self.variable_overlays_number[2][2] = msg.yaw

        self.pitch = msg.pitch
        self.roll = msg.roll
        self.yaw = msg.yaw + self.yaw_declination

    def setup(self, ctx):
        pass

    def overlay_tmy(self,ctx):
        #ctx["frame"] = cv2.addWeighted(ctx["frame"],1.0,self.img_overlay,1.0,0)
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
        
        x = self.pipeline_text_x0
        y = self.pipeline_text_y0
        i = 0
        for p in self.pipeline_text:
            cv2.putText( ctx["frame"], p[0], (x, y), self.font,  self.font_scale, 
                self.pipeline_text_colors[p[1]], 1, cv2.LINE_AA)    
            x+=self.pipeline_text_w
            i+=1
            if i == 3:
                x = self.pipeline_text_x0
                y += self.pipeline_text_h
                i = 0
        
    def overlay_ahrs(self, ctx):
        self.crosshair.y = int(240 + (self.pitch/90.0)*200)
        self.crosshair.set_rotation(self.roll)
        self.needle.set_rotation(self.yaw)
        self.crosshair.blit_rotated(ctx["frame"])
        self.compass.blit(ctx["frame"])
        self.needle.blit_rotated(ctx["frame"])


    def process(self, ctx):                
        cv2.line(ctx["frame"], (0,240), (640,240), (0,55,20), 1) 
        cv2.line(ctx["frame"], (320,0), (320,480), (0,55,20), 1) 

        self.overlay_tmy(ctx)
        self.overlay_ahrs(ctx)

    def release(self, ctx):
        pass