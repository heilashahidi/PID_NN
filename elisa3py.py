from controller import Robot, Motor
from controller import Display
import math
import numpy as np
import pandas as pd
from utils import get_angle,run_exp



class ElisaRobot(Robot):
    TIME_STEP = 32
    MAX_SPEED = 0.6 #60 cm per second
    R = 0.0045 # wheels radius = 4.5 mm
    L = 0.0408
    
    def __init__(self) -> None:
        super().__init__()
        self.left_motor = self.getDevice('left wheel motor')
        self.right_motor = self.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
        self.gps = self.getDevice('gps')
        self.gps.enable(self.TIME_STEP)
        self.iu = self.getDevice('iu')
        self.iu.enable(self.TIME_STEP)

    #get pose info    
    def get_pose(self):
        x,y = self.gps.getValues()[0:2]
        yaw = self.iu.getRollPitchYaw()[2]
        return x,y,yaw

    # to run the robot using vl and vr
    def run(self,vl,vr):
        self.left_motor.setVelocity(vl)
        self.right_motor.setVelocity(vr)

    def stop(self):
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)


class PIDController():

    def __init__(self,kp,ki,kd) -> None:
        self.old_error=0
        self.total_error = 0
        self.kp = kp
        self.ki = ki
        self.kd = kd
    
    def calc(self,error):
        self.e_dot = error - self.old_error
        self.total_error = self.total_error+error
        self.u=self.kp * error + self.kd * self.e_dot + self.ki * self.total_error
        self.old_error = error
        return self.u

    def reset(self):
        self.old_error=0
        self.total_error = 0


# main function
#main parameter
no_of_experiments =1000

floor_width,floor_height = (5.9,5.9)

    
#get angel between two point


robot = ElisaRobot()

#controller
kp=20
ki=0
kd=0
w_pid = PIDController(kp,ki,kd)

#initialization display 
#display = Display('display')


print("started!")

data_pd = pd.DataFrame()

cols=['step','exp_no','x','y','yaw','Error_dot','xd','yd','U','vl','vr']

while robot.step(robot.TIME_STEP) != -1:
    
    for exp in range(no_of_experiments):
        #generate random goal from the center of the floor
        goal_x = np.random.uniform(-floor_width/2.0,floor_width/2.0)
        goal_y = np.random.uniform(-floor_height/2.0,floor_height/2.0)
        goal = (goal_x,goal_y)
        #display.drawOval(1,1,0.05,0.05)
        #display.setColor(goal,255)

        exp_data = run_exp(robot,w_pid,goal,exp)
        exp_data = pd.DataFrame.from_dict(exp_data,orient='index').reset_index()
        data_pd = pd.concat([data_pd,exp_data],ignore_index=True)
        #data_pd.columns = cols
        print(data_pd)
        print(robot.TIME_STEP)
        w_pid.reset()

    data_pd.columns = cols
    data_pd.to_csv('out1000.csv')
    print('experiements finished!')
    break
