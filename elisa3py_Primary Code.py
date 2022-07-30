from controller import Robot, Motor
import math
import numpy as np

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
        theta = self.iu.getRollPitchYaw()[2]
        return x,y,theta

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


# main function

TIME_STEP = 32
    
#get angel between two point
def get_angle(xd,yd,x,y):
    #x,y,_ = get_robot_loc()
    
    theta = math.atan2((yd-y),(xd-x))
   
    return theta

robot1 = ElisaRobot()
r = robot1.R
l = robot1.L

# goal position
xd= 0.25
yd= 0.25
#PID for w
kp=10
ki=0
kd=0
w_pid = PIDController(kp,ki,kd)
while robot1.step(TIME_STEP) != -1:
   
    x,y,theta = robot1.get_pose()
    #get goal angle
    theta_d = get_angle(xd,yd,x,y)
    #print(x,y)
    #error
    e_theta = theta_d - theta
    EE1=math.sin(e_theta )
    EE2=math.cos(e_theta)
    Error_dot=math.atan2(EE1,EE2)
    e_distance = math.sqrt((yd-y)**2+(xd-x)**2)   

    if abs(e_distance)<0.002:
        robot1.stop()            
        break
    #Calculate w and v
    w = w_pid.calc(Error_dot)

    max_w=5
    if w > max_w:
           w = max_w
    elif w < -(max_w):
           w = -(max_w)
    print(w)
    
    V=robot1.MAX_SPEED

    vr = ((2*V)+(w*l))/(2*r)
    vl = ((2*V)-(w*l))/(2*r)
    
    # Scale the speed
    max_speed = max(vl,vr)
    vl = (vl/max_speed)*10
    vr = (vr/max_speed)*10
    #print('vr = {}, vl= {}'.format(vr,vl))
    robot1.run(vl,vr)
