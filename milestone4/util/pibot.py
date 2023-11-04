# access each wheel and the camera onboard of Alphabot

import numpy as np
import requests
import cv2 
import time as Time

class Alphabot:
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.wheel_vel = [0, 0]

    ##########################################
    # Change the robot velocity here
    # tick = forward speed
    # turning_tick = turning speed
    ########################################## 
    def set_velocity(self, command, tick, time=0): 
        l_vel = command[0]*tick[0] + command[1]*tick[0]
        r_vel = command[0]*tick[1] + command[1]*tick[1]
        self.wheel_vel = [l_vel, r_vel]
        if time == 0:
            requests.get(
                f"http://{self.ip}:{self.port}/robot/set/velocity?value="+str(l_vel)+","+str(r_vel))
        else:
            assert (time > 0), "Time must be positive."
            assert (time < 30), "Time must be less than network timeout (20s)."
            requests.get(
                "http://"+self.ip+":"+str(self.port)+"/robot/set/velocity?value="+str(l_vel)+","+str(r_vel)
                            +"&time="+str(time))
        # Fixees slam registering velocity offset when reality it shouldn't
        if l_vel == 23.6:
            l_vel0 = 25
        elif l_vel == -23.6:
            l_vel0 = -25
        elif l_vel == -14.7:
            l_vel0 = -15
        elif l_vel == 10 or l_vel == -10:
            l_vel0 = l_vel
        else:
            l_vel0 = 0
        return l_vel0, r_vel, time
        
    def get_image(self):
        try:
            r = requests.get(f"http://{self.ip}:{self.port}/camera/get")
            img = cv2.imdecode(np.frombuffer(r.content,np.uint8), cv2.IMREAD_COLOR)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        except (requests.exceptions.ConnectTimeout, requests.exceptions.ConnectionError, requests.exceptions.ReadTimeout) as e:
            print("Image retrieval timed out.")
            img = np.zeros((240,320,3), dtype=np.uint8)
        return img
    
    def get_IR(self):
        DL = 0
        DR = 0
        try:
            IR = requests.get(f"http://{self.ip}:{self.port}/IR/get")
        except (requests.exceptions.ConnectTimeout, requests.exceptions.ConnectionError, requests.exceptions.ReadTimeout) as e:
            print("IR retrieval timed out.")
        IR = np.frombuffer(IR.content, np.uint8)
        DL = IR[0]
        DR = IR[4]
        return DL, DR
    
    def set_Buzzer(self, setting):
        requests.get(
                "http://"+self.ip+":"+str(self.port)+"/robot/set/buzzer?value="+str(setting))
