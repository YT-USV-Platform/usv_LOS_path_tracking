#!/usr/bin/env python
# -*- coding: utf-8 -*-





import numpy as np
import matplotlib.pyplot as plt
import scipy.integrate
from scipy.optimize import minimize,fsolve
from scipy.integrate  import quad       
import scipy.misc
import sympy as sym
import math

class LOSTracking():

    def __init__(self):
        self.Wpx=0.0
        self.Wpy=0.0
        self.sideslip=0.0
        #look heading parameters
        self.delta_min=4
        self.delta_max=8
        self.delta_k=2
        self.U_max =1
        self.U_min=0.6
        self.y_max=80
        self.chi_max=5

    def execute(self,eta,Wpx): 

        for m in range(len(Wpx)-1):
            x_v=eta[0] # current vehicle position x
            y_v=eta[1] # current vehicle position y
    
            def f(x):
                return 0.01*x**2#40*math.sin(1/23*x)#

            Wpy=[f(Wpx[0]),f(Wpx[1])]
            print('Wpx---',Wpx)
            print('Wpy---',Wpy)
            print('Wpx[0]---',Wpx[0])
            self.x_curve=np.linspace(Wpx[0],Wpx[1],100*Wpx[1])
            self.y_curve=f(self.x_curve)

            print('******-------------********')


            ###################################################################################
            P = (x_v, y_v)

            def objective(X):
                x, y = X
                return np.sqrt((x - P[0]) ** 2 + (y - P[1]) ** 2)

            def c1(X):
                x, y = X
                return f(x)-y

            c1={'type':'eq','fun':c1}

            X = minimize(objective, x0 = [0, 0],method='SLSQP',constraints= c1)
            self.x_closest = X.x[0]  # closest point on curve -x
            self.y_closest = X.x[1]  # closest point on curve -y 
            ####################################################################################
            curve_slope_angle=math.atan(scipy.misc.derivative(f,self.x_closest,dx=1e-6)) 
            print('curve slope',curve_slope_angle)
            #cross-track error
            y_e=-(x_v-self.x_closest )*math.sin(curve_slope_angle)+(y_v- self.y_closest)*math.cos(curve_slope_angle) 
            print('y_e',y_e)
            self.var_lookhead_distance=(self.delta_max-self.delta_min)*math.exp(-self.delta_k*y_e**2)+self.delta_min

            def integrand(t):
                return math.sqrt(1+(f(t))**2)   # f matrix takip edilen path'in fonsksiyonudur.
            
            def function(x):
                return quad(integrand,0,x)[0]-self.var_lookhead_distance
            
            x_upper_limit = fsolve(function,1.0)
            self.x_los = x_upper_limit+self.x_closest
            self.y_los = f(self.x_los)






        # self.x_los =  self.x_closest+ self.var_lookhead_distance*math.cos(curve_slope_angle)
        # self.y_los =  self.y_closest + self.var_lookhead_distance*math.sin(curve_slope_angle)

        self.chi_r=math.atan(-y_e/abs(self.var_lookhead_distance)) # los angle 

        self.chi_d=curve_slope_angle+self.chi_r # ref açı
        error_angle=self.chi_d-eta[5] # açı hatası

        # referans hız değeri
        term1=abs(y_e)/self.y_max
        term2=abs(error_angle)/self.chi_max
        U_desired = max(self.U_max*(1-term1-term2)+self.U_min,self.U_min)
        return error_angle, U_desired, y_e,self.chi_d

    def los_simulation(self,eta):
        plt.clf()

        # plt.plot(self.Wpx,self.Wpy,'ro',label='Waypoints')
        
        plt.plot(self.x_curve,self.y_curve,'b-')
        
        plt.plot(eta[0],eta[1],'bo',label='vahicle position')

        plt.plot(self.x_closest,self.y_closest,'co')
        
        plt.plot([eta[0],self.x_closest],[eta[1],self.y_closest],'g--',label='cross track error ')

        plt.plot([self.x_closest,self.x_los],[self.y_closest,self.y_los],'m--',label='Varing Delta')

        plt.plot(self.x_los,self.y_los,'ko')

        plt.plot([eta[0],eta[0]+5*math.cos(eta[5])],[eta[1],eta[1]+5*math.sin(eta[5])])

        plt.show(block=False)

        plt.pause(0.02)











