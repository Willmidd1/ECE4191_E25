import cv2 
import numpy as np
import serial

class vision: 
    def __init__(self):
        self.tennisball_lower_yellow = np.array([27,70,100])
        self.tennisball_upper_yellow = np.array([32,255,255])
        #self.box_lower_yellow = np.array([13,35,0])
        #self.box_upper_yellow = np.array([26,255,255])
        # Convert the frame to the HSV color space
        self.mask_width = None
        self.top_left = None
        self.bottom_right = None
        self.boundary_line = None
        self.box_boundary = 0

    def detect_tennisball(self, frame):

        # Define the lower and upper boundaries of the "tennis ball" in the HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.tennisball_lower_yellow, self.tennisball_upper_yellow)
        mask = cv2.erode(mask, np.ones((3, 3)), iterations=1)
        mask = cv2.dilate(mask, np.ones((5, 5)), iterations=1)
        height, width = mask.shape[:2]
        self.mask_width = width
        boundary_line = 0
        self.boundary_line = boundary_line
        mask[:boundary_line, :] = 0
        
        B_mask = np.zeros((height, width), dtype=np.uint8)
        top_left = (0, int(height * 1/4)+20)
        bottom_right = (int(width), int(height))
        B_mask[top_left[1]:bottom_right[1], top_left[0]:bottom_right[0]] = 255
        mask = cv2.bitwise_and(B_mask, mask)
        self.top_left = top_left
        self.bottom_right = bottom_right
        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        bottom_half = []
        if len(contours) > 0:
            # Find the largest contour in the mask, then use it to compute the minimum enclosing circle and centroid
            #c = max(contours, key=cv2.contourArea)
            bottom_half = []
            for c in contours:
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                if radius > 10:
                    bottom_half.append(c)

        if bottom_half:
            c = max(bottom_half, key=cv2.contourArea)
            M = cv2.moments(c)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            center = int(x) , int(y)

            return [center], [int(radius)]

        return [], []

    def detect_box(self, frame):
        x = 0
        y = 0
        h = 0
        w = 0
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 定义三个不同光照条件下的 HSV 颜色范围
        # 正常光
        lower_normal = np.array([15, 100, 0])
        upper_normal = np.array([25, 255, 255])

        # 弱光
        lower_low_light = np.array([13, 35, 0])
        upper_low_light = np.array([26, 100, 255])

        # 强光
        lower_bright_light = np.array([12, 36, 197])
        upper_bright_light = np.array([36, 255, 255])

        # 创建三个掩码
        mask_normal = cv2.inRange(hsv, lower_normal, upper_normal)
        mask_low_light = cv2.inRange(hsv, lower_low_light, upper_low_light)
        mask_bright_light = cv2.inRange(hsv, lower_bright_light, upper_bright_light)

        # 使用 bitwise_or 将三个掩码合并
        mask = cv2.bitwise_or(mask_normal, mask_low_light)
        mask = cv2.bitwise_or(mask, mask_bright_light)
        ####
        
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, np.ones((5, 5)), iterations=2)
        height, width = mask.shape[:2]
        boundary = (height//4)+25
        mask[:boundary , :] = 0
        self.box_boundary = boundary
        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        bottom_half = []
        if len(contours) > 0:
            # Find the largest contour in the mask, then use it to compute the minimum enclosing circle and centroid
            #c = max(contours, key=cv2.contourArea)
            bottom_half = []
            for c in contours:
                
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                x, y, w, h = cv2.boundingRect(c)
                if w > width//10:
                    bottom_half.append(c)

        if bottom_half:
            c = max(bottom_half, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(c)
            center = int(x+w/2),int(y+h/2)
            print(w)
            if w > 30:
                return x, y, w, h
            else:
                return 0, 0, 0, 0

        # return nothing if no box detected
        return x, y, w, h
    
