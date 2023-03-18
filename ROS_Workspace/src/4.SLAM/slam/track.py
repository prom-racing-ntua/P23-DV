import numpy as np

yellow_cones_x = np.array([2, 2, 2, 2, 2, 2, -2, -4.5, -7, -9.5, -12, -14.5, -17, -19.5, -22, -22, -22, -22, -22, -22, -24.5, -27, -29.5, -32, -34.5, -37, -37, -37, -37, -37, -37, -34.5,-32, -29.5, -27, -24.5, -22, -19.5, -17, -14.5, -12, -9.5, -7, -4.5, -2, 2, 2, 2, 2])

yellow_cones_y = np.array([0, 2.5, 5, 7.5, 10, 12.5, 12.5, 12.5, 12.5, 12.5, 12.5, 12.5, 12.5, 12.5, 12.5, 10, 7.5, 5, 2.5, 0, 0, 0, 0, 0, 0, 0, -2.5, -5, -7.5, -10, -10, -10, -10, -10, -10, -10, -10, -10, -10, -10, -10, -10, -10, -10, -10, -10, -7.5, -5, -2.5])

blue_cones_x = np.array([-2, -2, -2, -2, -4.5, -7, -9.5, -12, -14.5, -17, -17, -17, -17, -17, -17, -19.5, -22, -24.5, -27, -29.5, -32, -14.5, -12, -9.5, -7, -4.5, -2, -2])

blue_cones_y = np.array([0, 2.5, 5, 7.5, 7.5, 7.5, 7.5, 7.5, 7.5, 7.5, 5, 2.5, 0, -2.5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -2.5])


yellow = np.zeros((yellow_cones_x.size,1))
yellow_cones = np.hstack((yellow, yellow_cones_y.reshape((-1,1)), yellow_cones_x.reshape(-1,1)))

blue = np.ones((blue_cones_x.size, 1))
blue_cones = np.hstack((blue, blue_cones_y.reshape((-1,1)), blue_cones_x.reshape((-1,1))))

track = np.vstack((yellow_cones, blue_cones))
print(track)

# np.savetxt("AutoX.txt" ,track)