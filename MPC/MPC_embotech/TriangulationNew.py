import numpy as np
from scipy.spatial import Delaunay
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon



class TrianglePlotter:
    def __init__(self, axis, points, simplices):
        self.ax = axis
        self.canvas = axis.figure.canvas
        self.points = points
        self.simplices = simplices
        self.size, _ = self.simplices.shape
        self.index = 0

        self.drawn_triangle = Polygon(self.points[self.simplices[self.index,:],:2], animated=True, color='green', alpha=0.4)
        self.ax.add_patch(self.drawn_triangle)

        self.canvas.mpl_connect('draw_event', self.on_draw)
        self.canvas.mpl_connect('key_press_event', self.on_key_press)

    def on_draw(self, event):
        self.background = self.canvas.copy_from_bbox(self.ax.bbox)
        self.ax.draw_artist(self.drawn_triangle)
    
    def on_key_press(self, event):
        if not event.inaxes:
            return
        if event.key=='left':
            if self.index == self.size-1:
                self.index = 0
            else:
                self.index+=1
            self.drawn_triangle.xy = self.points[self.simplices[self.index,:],:2]
        elif event.key == 'right':
            if self.index == 0:
                self.index = self.size-1
            else:
                self.index-=1
            self.drawn_triangle.xy = self.points[self.simplices[self.index,:],:2]
        self.canvas.restore_region(self.background)
        self.ax.draw_artist(self.drawn_triangle)
        self.canvas.blit(self.ax.bbox)


class Triangulation():
    def __init__(self, track_map, starting_position=[0,0], axis=None, triangles_to_print='only_valid', plot_midpoints=False) -> None:
        self.track_map = track_map
        self.starting_position = starting_position
        self.axis = axis
        self.sorted_triangle_indices = []

        self.triangulation = Delaunay(self.track_map[:,:2], incremental=False)
        self.first_triangle = self.triangulation.find_simplex(self.starting_position)
        self.valid_triangles = self.validSimplices()

        if any((self.valid_triangles[:] == self.triangulation.simplices[self.first_triangle]).all(1)):
            self.sorted_triangle_indices.append(self.first_triangle)
            self.sortTriangles()
            self.midpoints = self.findMidpoints()
            self.unique()

            if self.axis is not None:
                if triangles_to_print == 'only_valid':
                    self.axis.triplot(self.track_map[:,0], self.track_map[:,1], self.valid_triangles)
                elif triangles_to_print == 'all':
                    self.axis.triplot(self.track_map[:,0], self.track_map[:,1], self.triangulation.simplices)
                if plot_midpoints:
                    self.axis.scatter(self.midpoints[:,0], self.midpoints[:,1], color='firebrick', marker='o')
        else:
            print('-- Error: Starting position is not in a valid triangle')
    
    def validSimplices(self) -> None:
        valid_simplices = np.empty(shape=[0,0], dtype=np.int32)
        for i in range(len(self.triangulation.simplices)):
            invalid_edge = 0
            edges = np.array([[self.triangulation.simplices[i][0], self.triangulation.simplices[i][1]],
                            [self.triangulation.simplices[i][2], self.triangulation.simplices[i][0]],
                            [self.triangulation.simplices[i][1], self.triangulation.simplices[i][2]]])
            for j in range(3):
                if (self.track_map[edges[j][0]][2] == self.track_map[edges[j][1]][2]):
                    invalid_edge += 1
            if invalid_edge < 3:
                valid_simplices = np.append(valid_simplices, [[self.triangulation.simplices[i]]]) 
        valid_simplices = valid_simplices.reshape((-1, 3))
        return valid_simplices

    def sortTriangles(self):
        completed = False
        next_triangle = self.first_triangle
        while not completed:
            invalid_neighbors = 0
            for neighbor in self.triangulation.neighbors[next_triangle]:
                if (neighbor != -1) and (any((self.valid_triangles[:] == self.triangulation.simplices[neighbor]).all(1))) \
                    and (neighbor not in self.sorted_triangle_indices):
                    self.sorted_triangle_indices.append(neighbor)
                    next_triangle = neighbor
                    break
                else:
                    invalid_neighbors += 1
            if invalid_neighbors == 3:
                completed = True
    
    def findMidpoints(self):
        midpoints = np.empty(shape=[0,0], dtype=np.int32)
        for index in self.sorted_triangle_indices:
            simplex = self.triangulation.simplices[index]
            edges = np.array([[simplex[0], simplex[1]],
                            [simplex[2], simplex[0]],
                            [simplex[1], simplex[2]]])
            for j in range(3):
                if (self.track_map[edges[j][0]][2] != self.track_map[edges[j][1]][2]):
                    x = float((self.track_map[edges[j][0]][0] + self.track_map[edges[j][1]][0]) / 2.0)
                    y = float((self.track_map[edges[j][0]][1] + self.track_map[edges[j][1]][1]) / 2.0)
                    midpoints = np.append(midpoints, [[x, y]])
        midpoints = midpoints.reshape((-1,2))
        return midpoints
    
    def unique(self) -> None:
        uni = np.array([self.midpoints[0,:]])
        for point in self.midpoints:
            if not any((uni[:] == point).all(1)):
                uni = np.vstack((uni, point))
        self.midpoints = uni.reshape(-1,2)
    
    def getSorted(self):
        return self.triangulation.simplices[self.sorted_triangle_indices]



if __name__=='__main__':
    from track_plotter import getTrackdrive, getSkidpad, getAutoX

    plot = True             # plot triangles and midpoints
    to_plot = 'only_valid'  # set to 'only_valid' to plot the valid triangles, sey to 'all' to plot all of them
    midpoints = True   # whether to plot midpoints or not
    a = None

    fig, ax = plt.subplots()
    fig.set_size_inches([9,9])
    track = getTrackdrive(ax, return_points=True)
    
    if plot:
        a = ax
    tri = Triangulation(track, axis=a, triangles_to_print=to_plot, plot_midpoints=midpoints)
    t = TrianglePlotter(ax, track, tri.getSorted())

    plt.show()