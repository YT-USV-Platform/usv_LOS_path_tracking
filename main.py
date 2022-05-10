#!/usr/bin/env python
# -*- coding: utf-8 -*-

from re import S
from LOSTracker import LOSPathTracking
from LOSController import LOSController
from OtterSimulator.otter import Otter 


sample_time = 0.01

if __name__ == "__main__":

    tracker = LOSPathTracking()

    controller = LOSController(U_max=3.0, U_min=0.0, X_max=45.0, Y_max=20.0,
                               yaw_Kp=0.5,   yaw_Ki=0.0,   yaw_Kd=0.0,   yaw_Kf=0.0)


    simulator = Otter()
    simulator.initialize_otter(sample_time)

    vehicle_x = 0.4
    vehicle_y = 0.345
    vehicle_psi = 15.0
    vehicle_sideslip = 0.0

    current_motor_speed = [0.0, 0.0]

    for i in range(10000):

        print("-------------------------------")

        angle_error, cross_track_error = tracker.execute(vehicle_x, vehicle_y, vehicle_psi, vehicle_sideslip)

        control_signal, desired_u, desired_yaw = controller.execute(cross_track_error, angle_error, vehicle_psi)

        control_signal[0] += current_motor_speed[0]
        control_signal[1] += current_motor_speed[1]

        nu, eta = simulator.update_motors(control_signal)

        vehicle_x = eta[0] 
        vehicle_y = eta[1]
        vehicle_psi = eta[5] 
        vehicle_sideslip = vehicle_psi # TODO ENVIRONMENT EFFECT WILL BE ADDED SOON.

        tracker.visualize_tracker()

