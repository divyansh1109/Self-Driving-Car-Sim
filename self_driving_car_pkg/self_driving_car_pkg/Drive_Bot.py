import cv2
from collections import deque
from numpy import interp

from .config import config
from .Detection.Lanes.lane_det import detect_lanes

class Control():

    def __init__(self):
        self.prev_Mode = "Detection"


        self.prev_Mode_LT = "Detection"
        self.car_speed = 80.0
        self.angle_of_car = 0

        self.Left_turn_iterations = 0
        self.Frozen_Angle = 0
        self.Detected_LeftTurn = False
        self.Activat_LeftTurn = False

        self.TrafficLight_iterations = 0
        self.GO_MODE_ACTIVATED = False
        self.STOP_MODE_ACTIVATED = False

        # [NEW]: Deque member variable created for emulating rolling average filter to get smoothed Lane's ASsist
        self.angle_queue = deque(maxlen=10)

    def follow_Lane(self,Max_Sane_dist,distance,curvature):

        IncreaseTireSpeedInTurns = False

        # if((Tracked_class!=0) and (self.prev_Mode == "Tracking") and (Mode == "Detection")):
        #     if  (Tracked_class =="speed_sign_30"):
        #         self.car_speed = 30
        #     elif(Tracked_class =="speed_sign_60"):
        #         self.car_speed = 60
        #     elif(Tracked_class =="speed_sign_90"):
        #         self.car_speed = 90
        #     elif(Tracked_class =="stop"):
        #         self.car_speed = 0

        # self.prev_Mode = Mode 

        Max_turn_angle_neg = -90
        Max_turn_angle = 90

        CarTurn_angle = 0

        if( (distance > Max_Sane_dist) or (distance < (-1 * Max_Sane_dist) ) ):
            if(distance > Max_Sane_dist):
                CarTurn_angle = Max_turn_angle + curvature
            else:
                CarTurn_angle = Max_turn_angle_neg + curvature
        else:
            Turn_angle_interpolated = interp(distance,[-Max_Sane_dist,Max_Sane_dist],[-90,90])
            CarTurn_angle = (0.65*Turn_angle_interpolated) + (0.35*curvature)

        if( (CarTurn_angle > Max_turn_angle) or (CarTurn_angle < (-1 *Max_turn_angle) ) ):
            if(CarTurn_angle > Max_turn_angle):
                CarTurn_angle = Max_turn_angle
            else:
                CarTurn_angle = -Max_turn_angle

        angle = interp(CarTurn_angle,[-90,90],[-60,60])

        curr_speed = self.car_speed

        # if (IncreaseTireSpeedInTurns and (Tracked_class !="left_turn")):
        #     if(angle>30):
        #         car_speed_turn = interp(angle,[30,45],[80,100])
        #         curr_speed = car_speed_turn
        #     elif(angle<-30):
        #         car_speed_turn = interp(angle,[-45,-30],[100,80])
        #         curr_speed = car_speed_turn


        return angle, curr_speed
    
    def drive(self,Current_State):

        [Distance, Curvature, frame_disp] = Current_State

        current_speed = 0.0

        if((Distance != -1000) and (Curvature != -1000)):
            self.angle_of_car , current_speed = self.follow_Lane(int(frame_disp.shape[1]/2), Distance,Curvature)
        else:
            self.car_speed = 0.0
        
        # config.angle_orig = self.angle_of_car

        # self.angle_queue.append(self.angle_of_car)
        # self.angle_of_car = (sum(self.angle_queue)/len(self.angle_queue))
        # config.angle = self.angle_of_car
       
        # if Inc_LT:
        #     self.angle_of_car,current_speed, Detected_LeftTurn, Activat_LeftTurn = self.Obey_LeftTurn(self.angle_of_car,current_speed)
        # else:
        #     Detected_LeftTurn = False
        #     Activat_LeftTurn = False

        # if Inc_TL:
        #     self.angle_of_car,current_speed = self.OBEY_TrafficLights(self.angle_of_car,current_speed)


        return self.angle_of_car,current_speed

class Car():

    def __init__(self):

        self.Control_ = Control()
        # self.Inc_TL = Inc_TL
        # self.Inc_LT = Inc_LT
        # # [NEW]: Containers to Keep track of current state of Signs and Traffic Light detection
        # self.Tracked_class = "Unknown"
        # self.Traffic_State = "Unknown"

    def display_state(self,frame_disp,angle_of_car,current_speed):

        if (angle_of_car <-10):
            direction_string="[ Left ]"
            color_direction=(120,0,255)
        elif (angle_of_car >10):
            direction_string="[ Right ]"
            color_direction=(120,0,255)
        else:
            direction_string="[ Straight ]"
            color_direction=(0,255,0)

        if(current_speed>0):
            direction_string = "Moving --> "+ direction_string
        else:
            color_direction=(0,0,255)


        cv2.putText(frame_disp,str(direction_string),(20,40),cv2.FONT_HERSHEY_DUPLEX,0.4,color_direction,1)

        # angle_speed_str = "[ Angle ,Speed ] = [ " + str(int(angle_of_car)) + "deg ," + str(int(current_speed)) + "mph ]"
        # cv2.putText(frame_disp,str(angle_speed_str),(20,20),cv2.FONT_HERSHEY_DUPLEX,0.4,(0,0,255),1)

        # cv2.putText(frame_disp,"Traffic Light State = [ "+Traffic_State+" ] ",(20,60),cv2.FONT_HERSHEY_COMPLEX,0.35,255)

        # if (Tracked_class=="left_turn"):
        #     font_Scale = 0.32
        #     if (Detected_LeftTurn):
        #         Tracked_class = Tracked_class + " : Detected { True } "
        #     else:
        #         Tracked_class = Tracked_class + " : Activated { "+ str(Activat_LeftTurn) + " } "
        # else:
        #     font_Scale = 0.37
        # cv2.putText(frame_disp,"Sign Detected ==> "+str(Tracked_class),(20,80),cv2.FONT_HERSHEY_COMPLEX,font_Scale,(0,255,255),1)

    def drive_car(self, frame):
        img = frame[0:640, 238:1042]
        #resize to minimize computation power
        img = cv2.resize(img,(320,240))

        distance, Curvature = detect_lanes(img)

        Current_State = [distance, Curvature, img]

        self.Control_.drive(Current_State)

        return self.Control_.angle_of_car, self.Control_.car_speed