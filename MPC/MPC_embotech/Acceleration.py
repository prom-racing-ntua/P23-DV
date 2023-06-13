import numpy as np
import matplotlib.pyplot as plt
import math 

d0= 1.1
ds_wanted = 0.1
final = 2*75+(d0+0.3)
res1= int(final/ds_wanted)
x1 = np.linspace(0.0,final,res1)
y1 = np.linspace(0.0,0.0,res1)
array_straight_one=[]
print(res1)
for i in range(np.shape(x1)[0]):
    array_straight_one.append([x1[i],y1[i]])
np.savetxt("Data/Acceleration.txt",array_straight_one)


