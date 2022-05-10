#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math


class LOSController():

    def __init__(self, U_max=3.0, U_min=0.0, X_max=math.radians(45.0), Y_max=20.0,
                       yaw_Kp=6.5,   yaw_Ki=0.0,   yaw_Kd=0.0,   yaw_Kf=0.0, surge_kf=30.0):

        self.surge_kf = surge_kf
        self.U_max = U_max
        self.U_min = U_min
        self.X_max = X_max
        self.Y_max = Y_max

        self.yaw_Kp = yaw_Kp
        self.yaw_Ki = yaw_Ki # do not use
        self.yaw_Kd = yaw_Kd # do not use
        self.yaw_Kf = yaw_Kf # do not use

        # self.integral_error = 0.0


    def execute(self, cross_track_error, angle_error, vehicle_yaw):

        desired_u = self.get_speed_assignment(cross_track_error, angle_error)
        desired_yaw = angle_error + vehicle_yaw


        yaw_signal = angle_error*self.yaw_Kp + desired_yaw * self.yaw_Kf

        # self.integral_error += angle_error

        control_signal = self.allocate_thrusters(desired_u, yaw_signal)
        
        return control_signal, desired_u, desired_yaw



    def allocate_thrusters(self, surge_signal, yaw_signal):

        ####### TODO why is this reverse
        
        left_motor = surge_signal - yaw_signal
        right_motor = surge_signal + yaw_signal

        return [self.surge_kf*left_motor, self.surge_kf*right_motor]



    def get_speed_assignment(self, cross_track_error, angle_error):
        
        n_cross_track = abs(cross_track_error) / self.Y_max # normalized cross track error
        n_angle_error = abs(angle_error) / self.X_max # normalized angle error

        term = 1 - n_cross_track - n_angle_error

        u_desired = max(self.U_max*term + self.U_min, self.U_min)

        return u_desired

