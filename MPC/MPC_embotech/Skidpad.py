import numpy as np
import matplotlib.pyplot as plt
import math 

def PointsInCircumOne(r,n=100,right=1):
    output_array = []
    if(right==1):
        for x in range (0,n):
            output_array.append([math.sin(-(2*np.pi/n*x) + (np.pi))*r+center,math.cos( -(2*np.pi/n*x)+ (np.pi))*r+radius])
    else: 
        for x in range (0,n):
            output_array.append([math.sin((2*np.pi/n*x))*r+center,math.cos( (2*np.pi/n*x))*r-radius])
    return output_array
ds_wanted = 0.1
d0 = 1.1
center = d0+15
radius = 9.125
final = 25-d0+center
# res1 = int((center-0.0)/ds_wanted)
# res2 = int((2*np.pi*9.125)/ds_wanted)
# res3=res2
# res4=int((final-center)/ds_wanted)
res1 = 10
res2 = 15
res3= res2
res4= 10
x1 = np.linspace(0.0,center,res1)
x4 = np.linspace(center,final,res4)
y1 = np.linspace(0.0,0.0,res1)
y4 = np.linspace(0.0,0.0,res4)
array_straight_one=[]
for i in range(np.shape(x1)[0]):
    array_straight_one.append([x1[i],y1[i]])
array_straight_two=[]
for i in range(np.shape(x4)[0]):
    array_straight_two.append([x4[i],y4[i]])
array_right = PointsInCircumOne(radius,n=res2,right=1)
array_left = PointsInCircumOne(radius,n=res3,right=0)
track_final = np.concatenate((array_straight_one,array_right,array_right,array_left,array_left,array_straight_two),axis=0)
print(track_final,np.shape(track_final))
np.savetxt("Data/skidpad_straight1.txt",array_straight_one)
np.savetxt("Data/skidpad_left.txt",array_left)
np.savetxt("Data/skidpad_right.txt",array_right)
np.savetxt("Data/skidpad_straight2.txt",array_straight_two)
np.savetxt("Data/skidpad_all.txt",track_final)


