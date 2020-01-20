import libjevois as jevois
import cv2
import numpy as np
import math
import json

## detect high goal... better
#
# Add some description of your module here.
#
# @author frc4079
# 
# @videomapping YUYV 640 480 20 YUYV 640 480 20 FRC DetectHighGoal2
# @email 
# @address 123 first street, Los Angeles CA 90012, USA
# @copyright Copyright (C) 2018 by frc4079
# @mainurl 
# @supporturl 
# @otherurl 
# @license 
# @distribution Unrestricted
# @restrictions None
# @ingroup modules
class DetectHighGoal2:
    def __init__(self):
        self.hue = [0.0, 255.0]
        self.sat = [150.0, 255.0]
        self.val = [75.0, 255.0]
        
        self.image_width = 640
        self.image_height = 480
        self.focal_length = 696.195
        self.actual_width = 39
        
        self.centerX = 0
        self.centerY = 0
        
        
    
    def process(self, inframe, outframe):
        img = inframe.getCvBGR()
        
        hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        hsv_threshold = cv2.inRange(hsv_image, (self.hue[0], self.sat[0], self.val[0]), (self.hue[1], self.sat[1], self.val[1]))
        
        contours, hierarchy = cv2.findContours(hsv_threshold, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_TC89_KCOS)
        filtered_contours = []
        
        for contour in contours:
            if(contours != None and (len(contours) > 0)):
                area = cv2.contourArea(contour)
                if (area > 400):
                    filtered_contours.append(contour)
                    
        #for contour in filtered_contours:
        if(filtered_contours != None and len(filtered_contours) > 0):
            cont = filtered_contours[0]
            rect = cv2.minAreaRect(cont)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            cv2.drawContours(img, [box], 0, (227, 5, 216), 4)
        
            self.centerX = rect[0][0]
            self.centerY = rect[0][1]
            jevois.sendSerial("CenterX: " + str(self.centerX))
            jevois.sendSerial("CenterY: " + str(self.centerY))
        
            cv2.circle(img, (int(self.centerX), int(self.centerY)), 5, (255, 255, 255), -1)
            
            pixel_width = rect[1][0]
            pixel_height = rect[1][1]
            if pixel_height > pixel_width:
                temp = pixel_height
                pixel_height = pixel_width
                pixel_width = temp
            
            offset_angle = self.get_offset_angle(self.centerX)
            distance = self.calculate_distance(self.focal_length, pixel_width, self.actual_width) - 4
            
            vision_data = {"Distance:":distance, "Offset_angle:": offset_angle}
            json_vision_data = json.dumps(vision_data)
            
            jevois.sendSerial(json_vision_data)
        else:
          vision_data = {"Error":"No targets found"}
          json_vison_data = json.dumps(vision_data)

          jevois.sendSerial(json_vison_data)
            
        outframe.sendCv(img)
            
    def calculate_distance(self, focal_len, pix_width, act_width):
        return (focal_len*act_width)/pix_width
    
    def get_offset_angle(self, offset_position):
        ## Returns an angle in degrees
        return math.degrees(math.atan((offset_position - (self.image_width / 2)) / self.focal_length))
