import numpy as np
import matplotlib.pyplot as plt
import math
 
lol = 1.59 
WD_front = 0.467
l_f = lol*(1-WD_front)
l_r = lol*WD_front
ds_wanted = 0.1
wing = 0.874
start = -0.3-l_f-wing
final = start+2*75
length = final-start  
manual_stop = -start+75+1
percentage = (manual_stop)/(length)
print("stop at percentage: ",percentage)
res1= int(final/ds_wanted)
x1 = np.linspace(start,final,res1)
y1 = np.linspace(0.0,0.0,res1)
array_straight_one=[]
print(res1)
for i in range(np.shape(x1)[0]):
    array_straight_one.append([x1[i],y1[i]])
np.savetxt("Data/Acceleration.txt",array_straight_one)


