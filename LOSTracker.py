

from operator import ge
import numpy as np
import matplotlib.pyplot as plt
from CubicCurve import generate_curve
from R_calculation import R_Calculator
import math


class LineofSight():

    def __init__(self):

        self.sideslip=0.0
        #look heading parameters
        self.delta_min=3
        self.delta_max=6
        self.delta_k=2
        self.U_max =1
        self.U_min=0.6
        self.y_max=20
        self.chi_max=30
        self.k=0
        self.x_int=0.0
        self.Wpx =[]
        self.Wpy=[]
        self.R_calculator =R_Calculator()
        self.Wp_y_init = []
        self.min_index = 0


    def path_generate(self,Wpx,Wpy):
        ds = 0.01
        coeff ,self.x_init, self.y_init,self.Wp_x_init,self.Wp_y_init= generate_curve(Wpx,Wpy,ds)
        return coeff

    def closest_point(self,eta,x_list,y_list):
        x_v = eta[0]
        y_v = eta[1]

        dx = [x_v-icx for icx in x_list]
        dy = [y_v-icy for icy in y_list]

        distance = [np.sqrt(idx**2 + idy**2) for (idx,idy) in zip(dx,dy)]
        min_value = min(distance)
        self.min_index = distance.index(min_value)

        x_closest = x_list[self.min_index]
        y_closest = y_list[self.min_index]

        return x_closest , y_closest


   

    def execute(self,eta,Wpx,Wpy,coeff): 

        x_v = eta[0]
        y_v = eta[1]
        m = 4 # Araç etrafındaki 2 gemi boyundaki çamber
       
        dist = np.sqrt((eta[0]-Wpx[self.k+1])**2+(eta[1]-Wpy[self.k+1])**2)

        if dist<=m:
            self.k+=1
            print('******')
            print('K değeri',self.k)
            if self.k>=len(Wpx):
                pass

        if (self.k) >= (len(self.Wp_x_init)-1):
            self.k = (len(self.Wp_x_init)-1)
            self.init_x_curve = self.Wp_x_init[self.k]
            self.init_y_curve = self.Wp_y_init[self.k]



        self.init_x_curve = self.Wp_x_init[self.k]
        self.init_y_curve = self.Wp_y_init[self.k]
        

        self.x_closest, self.y_closest = self.closest_point(eta,self.init_x_curve,self.init_y_curve)


        curve_slope_angle=math.atan2(self.init_y_curve[self.min_index+1]-self.init_y_curve[self.min_index],self.init_x_curve[self.min_index+1]-self.init_x_curve[self.min_index]) 

        #cross-track error
        y_e=-(x_v-self.x_closest )*math.sin(curve_slope_angle)+(y_v- self.y_closest)*math.cos(curve_slope_angle) 
        # print('y_e',y_e)
        self.var_lookhead_distance=(self.delta_max-self.delta_min)*math.exp(-self.delta_k*y_e**2)+self.delta_min
    
      ####################################################################

        wk = 0
        curve_len =0
        exact_index_location=0
        index_len = 0
        print('K value:',self.k)

        if self.k == 0:
            pass
     
        else:
            for m in range(self.k):
                index_len += len(self.Wp_x_init[m])
                print('IDEX LENGTH',index_len)
                
            exact_index_location =exact_index_location+index_len
            print('Exact id len',exact_index_location)            
        
        while  curve_len<=self.var_lookhead_distance:

            if wk == 0:

                curve_len+=np.sqrt((self.init_x_curve[self.min_index]-self.init_x_curve[self.min_index+1])**2+(self.init_y_curve[self.min_index]-self.init_y_curve[self.min_index+1])**2)

            if self.k==0:
            
                self.x_los = self.x_init[self.min_index+wk]
                self.y_los = self.x_init[self.min_index+wk]


               
            if self.x_los != self.x_init[-1] and self.y_los != self.y_init[-1]:

                if self.x_init[exact_index_location+self.min_index+wk] ==self.x_init[-1] and self.x_init[exact_index_location+self.min_index+wk==self.y_init[-1]]:
                    break

                curve_len+=np.sqrt((self.x_init[exact_index_location+self.min_index+wk]-self.x_init[exact_index_location+self.min_index+wk+1])**2+(self.y_init[exact_index_location+self.min_index+wk]-self.y_init[exact_index_location+self.min_index+wk+1])**2)
            
                wk+=1

            else:
                break

        if self.x_los == self.x_init[-1] and self.y_los == self.y_init[-1]:
            self.x_los = self.x_init[-1]
            self.y_los = self.y_init[-1]

        self.x_los = self.x_init[exact_index_location+self.min_index+wk]
        self.y_los = self.y_init[exact_index_location+self.min_index+wk]



       
        ########################## NORMAL LOS POINT  ##################################################
        # self.x_los =  self.x_closest+ self.var_lookhead_distance*math.cos(curve_slope_angle)
        # self.y_los =  self.y_closest + self.var_lookhead_distance*math.sin(curve_slope_angle)
        ##############################################################################################

        d = np.sqrt((self.x_closest-self.x_los)**2+(self.y_closest-self.y_los)**2)
        # print('actual lookhead',d)
        self.chi_r=math.atan(-y_e/abs(d)) # los angle 

        self.chi_d=curve_slope_angle+self.chi_r # ref açı
        error_angle=self.chi_d-eta[5] # açı hatası

        # referans hız değeri
        term1=abs(y_e)/self.y_max
        # print('term1',term1)
        term2=abs(error_angle)/self.chi_max
        
        U_desired = max(self.U_max*(1-term1-term2)+self.U_min,self.U_min)
          
        path_curv_radius=self.R_calculator.R_cal(eta)

        # U_desired = speed_generate(path_curv_radius,sapma) 

        # print('U_desired',U_desired)
        
        return error_angle, U_desired,y_e,self.chi_d

    def los_simulation(self,eta,Wpx,Wpy):
        plt.clf()

        plt.plot(Wpx,Wpy,'ro',label='Waypoints')
        
        # plt.plot(self.x_curve,self.y_curve,'b-')
        # plt.plot(self.x_int,self.y_int)
        plt.plot(self.x_init,self.y_init,'m--')
        
        plt.plot(eta[0],eta[1],'bo',label='vahicle position')

        plt.plot(self.x_closest,self.y_closest,'co')
        
        plt.plot([eta[0],self.x_closest],[eta[1],self.y_closest],'g--',label='cross track error ')

        plt.plot([self.x_closest,self.x_los],[self.y_closest,self.y_los],'m--',label='Varing Delta')

        plt.plot(self.x_los,self.y_los,'ko')

        plt.plot([eta[0],eta[0]+5*math.cos(eta[5])],[eta[1],eta[1]+5*math.sin(eta[5])])

        plt.show(block=False)

        plt.pause(0.02)









