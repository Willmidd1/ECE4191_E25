from flask import Flask, Response
import cv2 as cv
from vision import vision
import time
import numpy as np
from serial import Serial

class Robot:
    def __init__(self, cap, port=8080): 
        self.cap = cap
        self.top_left = None
        self.bottom_right = None
        self.boundary_line = None
        self.mask_width = None
        self.x_box = 0
        self.y_box = 0
        self.w_box = 0
        self.h_box = 0
        self.port = port
        self.app = Flask(__name__)
        self.app_route()
        self.vision = vision()
        self.C = []
        self.R = []
        self.ser = Serial('/dev/serial0', 9600, timeout=1) # start serial instance for Robot

    def encode_frame(self, frame):
        success, buffer = cv.imencode('.jpg', frame)
        if not success:
            print("Failed to encode")
            return None, False
        return buffer.tobytes(), True

    def app_route(self):
        @self.app.route('/')
        def index():
            print("Page requested")
            return '''
            <!doctype html>
            <html>
            <body>
                <img src="/video_feed">
            </body>
            </html>
            '''
        
        @self.app.route('/video_feed')
        def video_feed():
            frame_packet = self.generate_frame()
            return Response(frame_packet, mimetype='multipart/x-mixed-replace; boundary=frame')

    def draw_shapes(self, frame):
        for i in range(len(self.R)):
            radius = self.R[i]
            x,y = self.C[i]
            cv.circle(frame, (x,y), radius, (0,255,255), 2)
            cv.circle(frame, (x,y), 5, (0,0,255), -1)
        cv.rectangle(frame, self.vision.top_left, self.vision.bottom_right, (0,255,0), 2)
        cv.line(frame, (0,self.vision.boundary_line), (self.vision.mask_width,self.vision.boundary_line), (0,0,200), 2)
        
        if self.w_box > 0:
            cv.rectangle(frame, (self.x_box, self.y_box), (self.x_box+self.w_box, self.y_box+self.h_box), (0, 170, 255), 2)
            x = self.x_box + (self.w_box/2)
            y = self.y_box + (self.h_box/2)
            cv.circle(frame, (x,y), 5, (0,0,255), -1)
            print("box detected")
        else:
            print("no box detected...")
            
        return frame
    
    def generate_object_coordinates(self):
        #while True:
        #time.sleep(0.1)
        success,frame = self.cap.read()
        if success: 
            self.C,self.R = self.vision.detect_tennisball(frame)
            self.x_box, self.y_box, self.w_box, self.h_box = self.vision.detect_box(frame)
            #print(self.C)
            #print(self.R)
        # spam update object coordinates
            
    def cast_frame(self,frame): 
        # Decrease the brightness by subtracting the value
        value = 0
        # Convert the image to float32 to prevent clipping issues
        frame = frame.astype(np.float32)
        # Decrease the brightness by subtracting the value
        frame -= value
        # Clip the values to be in the valid range [0, 255] and convert back to uint8
        frame = np.clip(frame, 0, 255).astype(np.uint8)
        return frame 
        
    
    def generate_frame(self):
        while True:
            #print("writing frame")
            #reduce the frame rate significantly to reduce CPU strain. Can do this because it is just  GUI and not important for the robots function
            time.sleep(0.2)
            success, frame = self.cap.read()
            if not success or frame is None:
                print("Could not get frame. Continuing...")
                continue
            # cast the frame into a standard size 
            frame = self.cast_frame(frame)
            # draw the object detection shapes on the frame
            frame = self.draw_shapes(frame)
            #encode the frame 
            frame_encoded, success = self.encode_frame(frame)
            if success:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_encoded + b'\r\n')
            else:
                print("Could not generate frame packet")

    def run_server(self):
        print("Starting server")
        self.app.run(host='0.0.0.0', port=self.port)
        
        
    def movement_control(self): 
        # init
        l_count = 0
        r_count = 0 
        f_count = 0
        s_count = 0
        # counter for num instances a ball has been detected in a row
        inst = 0
        image_center_threshold = 120
        ball_close_threshold = 70
        frame_avg_count = 5

        while True: 
            self.generate_object_coordinates()

            inst += 1
            #If there is at least one ball detected 
            if len(self.R)>0:
                # get the radius of the closest ball (by default only the closest is saved in this array)
                radius = self.R[0]
                # print(radius)
                # get ball center x coordinate
                center_x, center_y = self.C[0]
                #print(center_y)
                #if center_y <= 185:
                #    s_count += 2

                # get pixel dist from center 
                dist_from_center = center_x - 320
       
                #======================================================================
                
                # if the ball is close latch the center threshold to be finer 
                if radius > ball_close_threshold:
                    image_center_threshold = 35
                    uart_data = "CCCCCCCCCCCCC"
                    self.ser.write(uart_data.encode('utf-8'))
                    print("close")

                #=====================================================================

                # take a vote between L, R, F commands 
                if dist_from_center < -image_center_threshold:
                    # vote that the robot drives left
                    l_count+=1
                elif dist_from_center > image_center_threshold:
                    # vote that the robot drives right 
                    r_count+=1
                else:
                    # vote that the robot drives forward 
                    f_count+=1
            else:
                s_count += 1

            #======================================================================

            # While loop has detected a ball 3 instances in a row, and now the vote takes place on which direction to move
            if inst >= frame_avg_count:
                # choose the direction that has the highest votes 
                counts = [l_count, r_count, f_count, s_count]
                directions = ["LLLLLLLLLLL", "RRRRRRRRRRR", "FFFFFFFFFFF","SSSSSSSSSS"]
                max_idx = np.argmax(counts)
                direction = directions[max_idx]

                # drive toward that direction  
                self.ser.write(direction.encode('utf-8')) #send the command over UART to the STM
                print(direction[0])

                # reset closeness flag when ball is no longer in the camera
                if direction[0] == "S":
                    image_center_threshold = 120

                # reset all counters to 0
                l_count = r_count = f_count = s_count = 0
                inst = 0

            #=====================================================
            # sleep for 20ms to save resources 
            time.sleep(0.03)

            
        
        
            
            
            
        
