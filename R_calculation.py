

import numpy as np
import math



class R_Calculator():

    def __init__(self):
        self.init_heading=0
        self.x_list =[]
        self.y_list =[]
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_heading = 0.0


    def R_cal(self,current_eta):

        current_x=current_eta[0]
        current_y=current_eta[1]
        self.x_list.append(current_x)
        self.y_list.append(current_y)
        self.current_heading=current_eta[5]*180/math.pi %360
        # print('current heading',self.current_heading)
        delta_heading = self.current_heading-self.prev_heading

        if delta_heading==0:
            Radius = np.inf
        else:
            dist = math.sqrt((self.prev_x-current_x)**2+(self.prev_y-current_y)**2)
            Radius = np.abs(360*dist/(2*np.pi*delta_heading))

        # Radius = np.sqrt(dist**2/(2*(1-np.cos(delta_heading))))
        # print('Radius',Radius)
        self.prev_x=current_x
        self.prev_y=current_y
        self.prev_heading = self.current_heading
        # prev_values =[self.prev_x,self.prev_y,prev_heading]
        
        return Radius
