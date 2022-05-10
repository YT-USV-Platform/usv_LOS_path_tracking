#!/usr/bin/env python
# -*- coding: utf-8 -*-
from random import randrange
import matplotlib.pyplot as plt
from scipy.interpolate import pchip_interpolate
import numpy as np
import math



class WayPoint():

    def __init__(self, x, y):
        self.x_pos = x
        self.y_pos = y


    def __str__(self):
        return "[WayPoint] -- x: {}, y: {}".format(self.x_pos, self.y_pos)


class LOSPathTracking:

    def __init__(self, x_initial=0.4, y_initial=0.345, psi_initial=math.atan(math.radians(60)), sideslip_initial=8.0):

        self.x_current = x_initial # vehicle x_location
        self.y_current = y_initial # vehicle y_location

        self.get_current_pose(vehicle_x=self.x_current, vehicle_y=self.y_current, vehicle_psi=psi_initial, vehicle_sideslip=sideslip_initial)
        
        print("Psi: ", self.psi)
        print("Sideslip angle: ", self.sideslip_angle)
        print("Course angle: ", self.course_angle)

        self.delta_min = 0.15  # meters
        self.delta_max = 0.3  # meters
        self.delta_k = 50 # design parameter


    # self iptal test
    def pchip_interpolation(self, wp_list):

        min_x_wp = min(wp_list, key=lambda x: x.x_pos)
        max_x_wp = max(wp_list, key=lambda x: x.x_pos)

        x = np.linspace(min_x_wp.x_pos, max_x_wp.x_pos, 100)
        y = pchip_interpolate([wp.x_pos for wp in wp_list], [wp.y_pos for wp in wp_list], x)

        return x, y


    def get_current_pose(self, vehicle_x=None, vehicle_y=None, vehicle_psi=None, vehicle_sideslip=None):

        if vehicle_x is None:
            self.x_current = self.x_current+0.01
        else:
            self.x_current = vehicle_x

        if vehicle_y is None:
            self.y_current = self.y_current+0.01
        else:
            self.y_current = vehicle_y

        if vehicle_psi is None:
            self.psi = self.psi+0.01
        else:
            self.psi = vehicle_psi

        if vehicle_sideslip is None:
            self.sideslip_angle = self.sideslip_angle+0.01
        else:
            self.sideslip_angle = vehicle_sideslip

        self.course_angle = vehicle_psi + vehicle_sideslip
    


    def test_get_path(self):

        x_list = np.linspace(0.0, 100.0, 11)
        
        wp_list = []
        for i in range(11):
            # wp_list.append(WayPoint(x_list[i], math.sin(math.radians(180*x_list[i]))))
            wp_list.append(WayPoint(x_list[i], 2*x_list[i]))


        return wp_list


    def execute(self, vehicle_x, vehicle_y, vehicle_psi, vehicle_sideslip):

        self.wp_list = self.test_get_path()

        self.get_current_pose(vehicle_x, vehicle_y, vehicle_psi, vehicle_sideslip)

        self.x, self.y = self.pchip_interpolation(self.wp_list)

        error_list = np.zeros_like(self.x)
        min_dist = np.inf
        min_index = 0
        for i in range(len(self.x)):
            error_list[i] = math.sqrt((self.x[i] - self.x_current)**2 + (self.y[i] - self.y_current)**2)
            if error_list[i] < min_dist:
                min_dist = error_list[i]
                min_index = i


        cross_track_error = error_list[min_index]
        print("Cross track error: ", cross_track_error)


        self.x_closest = self.x[min_index]
        self.y_closest = self.y[min_index]


        spline_slope = (self.y[min_index]-self.y[min_index+1]) / (self.x[min_index]-self.x[min_index+1])
        self.los_angle = math.atan(spline_slope)


        delta = (self.delta_max-self.delta_min)*math.exp(-1*self.delta_k*cross_track_error**2)+self.delta_min
        print("Varying delta: ", delta)

        self.los_point_x = self.x_closest + delta * math.cos(self.los_angle)
        self.los_point_y = self.y_closest + delta * math.sin(self.los_angle)

        print("Spline slope: ", spline_slope)
        print("LOS angle: ", self.los_angle)

        self.angle_error = self.course_angle - self.los_angle
        print("Angle error: ", self.angle_error)
        
        return self.angle_error, cross_track_error



    def visualize_tracker(self):

        plt.clf()

        plt.plot([wp.x_pos for wp in self.wp_list], [wp.y_pos for wp in self.wp_list], 'ro')
            
        plt.plot(self.x, self.y, label='pchip')

        # print("min_index: ", min_index)
        print("Closest point: ", self.x_closest, self.y_closest)

        plt.plot(self.x_current, self.y_current, 'bo')

        # plot line between current position and the closest point
        plt.plot([self.x_current, self.x_closest], [self.y_current, self.y_closest], 'b--')

        # plot line of sight
        plt.plot([self.x_closest, self.los_point_x], [self.y_closest, self.los_point_y], 'r--')
        # plot line of sight point
        plt.plot(self.los_point_x, self.los_point_y, 'go')

        plt.show(block=False)
        plt.pause(0.05)











