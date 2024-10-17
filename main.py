import cv2
from robot import Robot
import threading

def init_camera():
    for index in range(10):
        cap = cv2.VideoCapture(index)
        if cap.isOpened():
            print(f"Camera opened successfully at index {index}.")
            return cap
        cap.release()  
    return None

if __name__ == "__main__":
    cap = init_camera()
    if cap:
        robot = Robot(cap, port=8080)

        #--------- ONLY USE THE THREADS THAT ARE NECESSARY. TO SAVE COMPUTING POWER---------

        # ---------- you can also adjust the sleep timers in robot.py as necessary---


        # spam generate object coordinates in one thread
        #thread1 = threading.Thread(target=robot.generate_object_coordinates, daemon=True)
        #thread1.start()

        # spam generate movement commands based on detected object coordinates from thread1. 
        thread2 = threading.Thread(target=robot.movement_control, daemon=True) 
        thread2.start()
        
        # spam generate and serve frames in another thread, drawing bounding boxes from the coordinates generated in thread1. 
        thread3 = threading.Thread(target=robot.run_server, daemon=True)
        thread3.start()
        
        thread3.join()
    else: 
        print("Failed to initialise camera [outside of index range?]")
        
    
