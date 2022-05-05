
from random import randrange
import matplotlib.pyplot as plt
from scipy.interpolate import pchip_interpolate
import numpy as np
import math


x_observed = np.linspace(0.0, 1.0, 11)
y_observed = np.sin(x_observed)

#####

x = np.linspace(min(x_observed), max(x_observed), 100)
y = pchip_interpolate(x_observed, y_observed, x)
plt.plot(x_observed, y_observed, x, y, '-')
plt.plot(x,y,label='pchip')
plt.legend()


x_current = 0.99 # vehicle x-location
y_current = 0.345 # vehicle y_location

error_list = np.zeros_like(x)
min_dist = np.inf
min_index = 0
for i in range(len(x)):
    error_list[i] = math.sqrt((x[i] - x_current)**2 + (y[i] - y_current)**2)
    if error_list[i] < min_dist:
        min_dist = error_list[i]
        min_index = i

cross_track_error = error_list[min_index]
    
x_closest = x[min_index]
y_closest = y[min_index]

print("Cross track error: ", cross_track_error)
print("min_index: ", min_index)
print("Closest point: ", x_closest, y_closest)

# add a circle to current position
plt.plot(x_current, y_current, 'o')

# plot line between current position and the closest point
plt.plot([x_current, x_closest], [y_current, y_closest], 'b--')



spline_slope = (y[min_index]-y[min_index+1]) / (x[min_index]-x[min_index+1])
los_angle = math.atan(spline_slope)

print("Spline slope: ", spline_slope)
print("LOS angle: ", los_angle)

#delta = 0.15
### time-varying lookhead distance

delta_min=0.15  #meters
delta_max=0.3  #meters
delta_k=50 #design parameters

delta=(delta_max-delta_min)*math.exp(-delta_k*cross_track_error**2)+delta_min
print('varyin_delta:',delta)
los_point_x = x_closest + delta * math.cos(los_angle)
los_point_y = y_closest + delta * math.sin(los_angle)

print('x_los',los_point_x)
print('y_los',los_point_y)


# plot line of sight
plt.plot([x_closest, los_point_x], [y_closest, los_point_y], 'r--')
# plot line of sight point
plt.plot(los_point_x, los_point_y, 'ro')


psi = math.atan(math.radians(60))
print("Psi: ", psi)

sideslip_angle = 8.0 
print("Sideslip angle: ", sideslip_angle)

course_angle = psi + sideslip_angle
print("Course angle: ", course_angle)

angle_error = course_angle - los_angle
print("Angle error: ", angle_error)



plt.show(block=True)

