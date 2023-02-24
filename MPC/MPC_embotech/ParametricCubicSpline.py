import numpy as np
from scipy.linalg import solve_banded
from scipy.sparse import diags
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

from TriangulationNew import Triangulation
from track_plotter import getTrackdrive, getSkidpad, getAccel, getAutoX


def getDiagonalForm(banded_matrix, upper_band=1, lower_band=1):
    rows, cols = banded_matrix.shape
    df = np.zeros((upper_band+lower_band+1, rows))
    dr, _ = df.shape
    for i in range(rows):
        for j in range(cols):
            i_new = upper_band + i - j
            if i_new >= dr or i_new < 0: continue
            df[upper_band+i-j,j] = banded_matrix[i,j]
    return df

def getSplineDerivatives(interpolation_points, boundary_condition = 'closed_loop'):
    '''
    '''
    num_points, _ = interpolation_points.shape

    # Create the tridiagonal matrix based on the boundary conditions (for now it is closed-loop)
    tri_matrix = diags((1,4,1), (0,1,2), shape=(num_points-2, num_points), dtype=np.float32).toarray()
    if boundary_condition == 'closed_loop':
        first_row = np.zeros((1,num_points), dtype=np.float32)
        first_row[0,0] = 1
        first_row[0,1] = 1

        last_row = np.zeros((1,num_points), dtype=np.float32)
        last_row[0,-1] = -2
        last_row[0,-2] = -2

        tri_matrix = np.vstack((first_row, tri_matrix, last_row))

        u = np.zeros((num_points,1), dtype=np.float32)
        u[0,0]  = 1
        u[-1,0] = 1

        v = np.zeros((num_points,1), dtype=np.float32)
        v[0,0]  = 1
        v[-1,0] = 2
        v[-2,0] = 1

    elif boundary_condition == 'natural_spline':
        first_row = np.zeros((1,num_points), dtype=np.float32)
        first_row[0,0] = 1

        last_row = np.zeros((1,num_points), dtype=np.float32)
        last_row[0,-1] = 1

        tri_matrix = np.vstack((first_row, tri_matrix, last_row))

    elif boundary_condition == 'anchored':
        first_row = np.zeros((1,num_points), dtype=np.float32)
        first_row[0,0] =  1
        first_row[0,1] = -1

        last_row = np.zeros((1,num_points), dtype=np.float32)
        last_row[0,-1] =  1
        last_row[0,-2] = -1

        tri_matrix = np.vstack((first_row, tri_matrix, last_row))

    diag = getDiagonalForm(tri_matrix)
        
    d = np.zeros((num_points,2), dtype=np.float32)
    for i in range(1, num_points-1):
        d[i,:] = interpolation_points[i-1,:] - 2*interpolation_points[i,:] + interpolation_points[i+1,:]
        d[0,:] = interpolation_points[1,:] - interpolation_points[0,:] - interpolation_points[-1,:] + interpolation_points[-2,:]
        d[-1,:] = 0

    if boundary_condition == 'closed_loop':
        y = solve_banded((1,1),diag,6*d)
        q = solve_banded((1,1),diag,u)
        second_derivative = y - np.divide((q @ v.T @ y), (1 + v.T @ q))
    else:
        second_derivative = solve_banded((1,1), diag, 6*d)

    return second_derivative


def getSpline(track_cones, axis=None, resolution=10000, boundary_condition='closed_loop'):
    global_index = np.linspace(0,1,resolution)
    inter = 1 / (track_cones.shape[0] - 1)
    spline = np.empty((1,2),dtype=np.float32)

    spline_derivatives = getSplineDerivatives(track_cones, boundary_condition)
    print("spline derivatives are: ",spline_derivatives,type(spline_derivatives),np.shape(spline_derivatives))
    
    for t in global_index:
        i = int(t // inter)
        u = t % inter / inter
        if u < 1e-10 and i != 0.0:
            u = 1.0
            i -= 1
        temp = track_cones[i,:] + ((track_cones[i+1,:] - track_cones[i,:]) - 1/6*spline_derivatives[i+1,:] - 1/3*spline_derivatives[i,:]) * u + 1/2*spline_derivatives[i,:] * u**2 + 1/6*(spline_derivatives[i+1,:]-spline_derivatives[i,:]) * u**3
        spline = np.vstack((spline, temp))

    if axis is not None:
        axis.plot(spline[1:,0], spline[1:,1], color='orange', label='Path Splines', linewidth=2.5)

    return spline[1:,:]


def main():
    # Plot shit
    fig,ax = plt.subplots()
    fig.set_size_inches(13,9)
    track = getTrackdrive(ax, return_points=True)
    tri = Triangulation(track)
    points = np.vstack((tri.midpoints, tri.midpoints[0,:]))
    # print("points from triangulation are", points, np.shape(points), type(points))
    # print()
    # print("transposed points from triangulation are", points.T, np.shape(points.T), type(points.T))
    # print()

    # points = np.array([[0,0], [3,1], [8,6], [2,10], [-5,7], [-4,0], [-2,-1], [0,0]])
    spline = getSpline(points, axis=ax, boundary_condition='closed_loop')
    # print("spline is", spline, np.shape(spline), type(spline))
    # print()
    # print("transposed spline is", spline.T, np.shape(spline.T), type(spline.T))
    # print()
    interpolation_line = Line2D(points[:,0], points[:,1], color='firebrick', linestyle='dashed', linewidth=1, marker='o', markeredgecolor='firebrick', markerfacecolor='orange')
    ax.add_line(interpolation_line)

    # points = np.array([[0,0], [3,1], [8,6], [2,10], [-5,7], [-8,2], [-2,-1], [0,0]])
    # spline = getSpline(points, axis=ax, boundary_condition='anchored')
    ax.autoscale()



if __name__=='__main__':
    main()
    plt.show()
