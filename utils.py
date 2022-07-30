import math

def get_angle(xd,yd,x,y):
    #x,y,_ = get_robot_loc()
    theta = math.atan2((yd-y),(xd-x))
    return theta

def run_exp(robot,controller,goal_pos,exp_num,dist_margin=0.002):
    
    xd,yd = goal_pos
    data_dic={}
    Step = 0
    print('Exp No.{} started! the goal is {}'.format(exp_num,goal_pos))
    print('robot pos is {}'.format(robot.get_pose()))
    x,y,yaw = robot.get_pose()
    theta_d = get_angle(xd,yd,x,y)
    print('Theta_D is {}'.format(theta_d))

    while True:
        x,y,yaw = robot.get_pose()
        #get goal angle
        theta_d = get_angle(xd,yd,x,y)
       
        #error
        e_theta = theta_d - yaw
        EE1=math.sin(e_theta )
        EE2=math.cos(e_theta)
        Error_dot=math.atan2(EE1,EE2) 
        w = controller.calc(Error_dot)
        #print(w)
        v=robot.MAX_SPEED
        r = robot.R
        l = robot.L

        #Setting upper and lower boundaries on W
        max_w=10
        if w > max_w:
           w = max_w
        elif w < -(max_w):
             w = -(max_w)
        print(w)
        vr = ((2*v)+(w*l))/(2*r)
        vl = ((2*v)-(w*l))/(2*r)
        # Scale the speed
        max_speed = max(vl,vr)
        vl = (vl/max_speed)*10.0
        vr = (vr/max_speed)*10.0
        robot.run(vl,vr)
        #save data
        data_dic[Step] = [exp_num,x,y,yaw,Error_dot,xd,yd,w,vl,vr]
        # check if the robot is reached the goal
        e_dist = math.sqrt((yd-y)**2+(xd-x)**2)
        if abs(e_dist)<dist_margin:
            robot.stop()
            break
        Step = Step+32
        robot.step(robot.TIME_STEP)
    return data_dic