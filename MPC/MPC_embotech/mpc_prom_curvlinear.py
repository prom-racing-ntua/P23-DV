# See also FORCES_NLP
# (c) Embotech AG, Zurich, Switzerland, 2013-2022.

import sys
import numpy as np
import casadi
import forcespro
import forcespro.nlp
import matplotlib.pyplot as plt
import matplotlib.patches
from matplotlib.gridspec import GridSpec
from scipy.interpolate import CubicSpline
import casadi

#imports from path planning
import numpy as np
from scipy.linalg import solve_banded
from scipy.sparse import diags
from matplotlib.lines import Line2D
from Trackdrive import cones_blue, cones_orange_big, cones_yellow

#global params

def arc_length_parameterization(x, y):
    # Calculate the cumulative arc length
    dx = np.diff(x)
    dy = np.diff(y)
    ds = np.sqrt(dx**2 + dy**2)
    s = np.cumsum(ds)
    global s_max
    s_max = s[-1]
    print(s_max)
    # s /= s[-1]
    s = np.insert(s,0,0)
    print(s)
    return s

# set physical constants
l_f = 0.9141
l_r = 0.7359
CdA = 1.8 # for drag
ClA = 5.47 # for downforce
pair = 1.225
mi_y_max=1.0
u_upper=12.0
m = 190.0   # mass of the car
g = 9.81
Iz = 110.0
ds_wanted=0.1
eff=0.85
window=10

#for dynamic model
B=-8.266
C=1.456
C_tire=0.66
D= 1739.47
cs=-16419.31

#compute l_a
umin=4.0
umax=7.0
INDEX_MAX=310.0
DINDEX_MAX=310.0

#ellipse params
a=1.46
b=1.62

# Read the txt file data
def Dataloader(txt):
    with open(txt) as f:
        lines = f.readlines()
    # Convert the data to a numpy array
    data = []
    for line in lines:
        row = [float(x) for x in line.split()]
        data.append(row)
    data = np.transpose(np.array(data))
    return data


def continuous_dynamics(x, u, current_target):
    """Defines dynamics of the car, i.e. equality constraints.
    parameters:
    state x = [xPos,yPos,phi, vx, vy, r, F, delta, index, ni, mi]
    input u = [dF,ddelta]
    """
    temp1=casadi.fmax((x[3]-umin)/(umax-umin),0)
    l_a=casadi.fmin(temp1,1)

    #slip angles and Fys
    # saf=casadi.arctan((x[4]+l_f*x[5])/np.sqrt(x[3]**2+1)) - x[7]
    # sar=casadi.arctan((x[4]-l_r*x[5])/np.sqrt(x[3]**2+1))
    saf=casadi.arctan((x[4]+l_f*x[5])/(x[3]+1e-3)) - x[7]
    sar=casadi.arctan((x[4]-l_r*x[5])/(x[3]+1e-3))
    Ffy = C_tire*D*casadi.sin(C*casadi.arctan(B*saf))
    Fry = C_tire*D*casadi.sin(C*casadi.arctan(B*sar))
    Frz = (l_f/(l_f+l_r))*m*g + 0.25*pair*ClA*(x[3]**2)
    Ffz = (l_r/(l_r+l_f))*m*g + 0.25*pair*ClA*(x[3]**2)
    # Ffy = -saf*cs
    # Fry = -sar*cs

    #friction forces
    Fdrag = 0.5*CdA*pair*(x[3])**2 + 0.03*(Frz+Ffz)

    #blending with changing lambda
    beta = casadi.arctan(l_r/(l_f + l_r) * casadi.tan(x[7]))
    # xdot = (l_a)*(x[3]*casadi.cos(x[2]) - x[4]*casadi.sin(x[2])) + (1-l_a)*(x[3]*casadi.cos(x[2] + beta))
    # ydot = (l_a)*(x[3]*casadi.sin(x[2]) + x[4]*casadi.cos(x[2])) + (1-l_a)*(x[3]*casadi.sin(x[2] + beta))
    xdot = x[3]*casadi.cos(x[2]) - x[4]*casadi.sin(x[2])
    ydot = x[3]*casadi.sin(x[2]) + x[4]*casadi.cos(x[2])
    phidot = x[5]
    vxdot = (1-l_a)*((x[6] - Fdrag)/ m) + (l_a)*((x[6] - Fdrag + Ffy*casadi.sin(x[7]) + m*x[4]*x[5])/ m)
    vydot = (1-l_a)*((l_r/(l_r+l_f))*(vxdot*casadi.tan(x[7])+x[3]*(u[1]/(casadi.cos(x[7]))**2))) + (l_a)*(((-x[3]*x[5]) + (Fry + Ffy*casadi.cos(x[7])))/(1.0*m))
    rdot = (1-l_a)*((1/(l_r+l_f))*(vxdot*casadi.tan(x[7])+x[3]*(u[1]/(casadi.cos(x[7]))**2))) + (l_a)*((Ffy*l_f*casadi.cos(x[7]) - Fry*l_r)/(1.0*Iz))
    Fdot= u[0]
    deltadot = u[1]
    indexdot = (x[3]*casadi.cos(x[10]) - x[4]*casadi.sin(x[10]))/(1-current_target[2]*x[9])
    midot = x[3]*casadi.sin(x[10]) + x[4]*casadi.cos(x[10])
    ndot = x[5] - current_target[2]*indexdot
    
    return casadi.vertcat(xdot,ydot,phidot,vxdot,vydot,rdot,Fdot,deltadot,indexdot,midot,ndot)

err_array=[]
def obj(z,current_target):
    """Least square costs on deviating from the path and on the inputs F and phi
    z = [dF,ddelta,xPos,yPos,phi, vx, vy, r, F, delta,index, ni, mi] ->0 ws 12
    current_target = point on path that is to be headed for
    """
    saf=casadi.arctan((z[6]+l_f*z[7])/(z[5]+1e-3)) - z[9]
    sar=casadi.arctan((z[6]-l_r*z[7])/(z[5]+1e-3))
    dyn_sa  = casadi.arctan(z[7]/(z[6]+1e-3))
    beta = casadi.arctan(l_r/(l_f + l_r) * casadi.tan(z[9]))
    dsa =(dyn_sa-beta)
    e_c= casadi.sin(current_target[2])*(z[2]-current_target[0]) - casadi.cos(current_target[2])*((z[3]-current_target[1])) #katakorifi
    e_l= -casadi.cos(current_target[2])*(z[2]-current_target[0]) - casadi.sin(current_target[2])*((z[3]-current_target[1])) #orizontia
    Ffy = C_tire*D*casadi.sin(C*casadi.arctan(B*saf))
    Fry = C_tire*D*casadi.sin(C*casadi.arctan(B*sar))
    Frz = (l_f/(l_f+l_r))*m*g + 0.25*pair*ClA*(z[5]**2)
    Ffz = (l_r/(l_r+l_f))*m*g + 0.25*pair*ClA*(z[5]**2)
    print("Forces are: ",z[9]," ", Fry, " ",Frz)

    return (
        0e0*(z[2]-current_target[0])**2 # costs on deviating on the path in x-direction
            + 0e0*(z[2]-current_target[1])**2 # costs on deviating on the path in y-direction
            # + 1e3*(e_c)**2 # costs on deviating on the
            #                             #path in y-direction
            # + 1e3*(e_l)**2 # costs on deviating on the
            #                         #path in x-direction
            + 1e3*z[11]**2
            + 1e3*z[12]**2
            + 0e-3*z[0]**2 # penalty on input F,dF
            + 0e-3*z[8]**2
            + 0e-3*z[1]**2 #penalty on delta,ddelta
            + 0e-3*z[9]**2
            + 0e-3*(z[4]-current_target[2])**2 #dphi gap
            + 0e-3*(sar**2) #tyre grip
            + 0e-3*(dsa**2)
            + 0e-3*((z[8]/(a*Frz))**2 + (Fry/(b*Frz))**2)
            + 1e1*((1/(z[5]**2 +1e-3))) #vx and index
            - 1e1*(z[5])
            - 1e1*(z[10]/INDEX_MAX)
            )

def constr(z,current_target):
    """Least square costs on deviating from the path and on the inputs F and phi
    z = [dF,ddelta,xPos,yPos,phi, vx, vy, r, F, delta,index, ni, mi] ->0 ws 12
    current_target = point on path that is to be headed for
    """
    saf=casadi.arctan((z[6]+l_f*z[7])/(z[5]+1e-3)) - z[9]
    sar=casadi.arctan((z[6]-l_r*z[7])/(z[5]+1e-3))
    Ffy = C_tire*D*casadi.sin(C*casadi.arctan(B*saf))
    Fry = C_tire*D*casadi.sin(C*casadi.arctan(B*sar))
    Frz = (l_f/(l_f+l_r))*m*g + 0.25*pair*ClA*(z[6]**2)
    Ffz = (l_r/(l_r+l_f))*m*g + 0.25*pair*ClA*(z[6]**2)
    constr1 = (z[3]-current_target[0])**2 + (z[4]-current_target[1])**2 #inside track constraint
    constr2 = (z[9]/(a*Frz))**2 + (Fry/(b*Frz))**2 #tyre constraints
    constr3 = saf
    return (constr1,constr2,constr3) 

def find_closest_point(points, ref_point):
    """Find the index of the closest point in points from the current car position
    points = array of points on path
    ref_point = current car position
    """
    num_points = points.shape[1]
    diff = np.transpose(points) - ref_point
    diff = np.transpose(diff)
    squared_diff = np.power(diff,2)
    squared_dist = squared_diff[0,:] + squared_diff[1,:]
    return np.argmin(squared_dist)

def find_closest_point_vertical(data, ref_point):
    """Find the index of the closest point in points from the current car position
    points = array of points on path
    ref_point = current car position
    """
    num_points = data.shape[1]
    phi_array = data[2,:]
    diff_x = -np.transpose(data[0,:]) + ref_point[0]
    diff_x = np.transpose(diff_x)
    diff_y = -np.transpose(data[1,:]) + ref_point[1]
    diff_y = np.transpose(diff_y)
    part_x = diff_x*np.sin(data[2,:])
    part_y = diff_y*np.cos(data[2,:])
    dist_final = np.abs(part_x - part_y)
    # num_points = points.shape[1]
    # diff = np.transpose(points) - ref_point
    # diff = np.transpose(diff)
    # squared_diff = np.power(diff,2)
    # squared_dist = squared_diff[0,:] + squared_diff[1,:]
    return np.argmin(dist_final)

    
def extract_next_path_points(data_points, pos, N):
    """Extract the next N points on the path for the next N stages starting from 
    the current car position pos
    """
    path_points=data_points[:2,:] #keep only X,Y coords
    # idx = find_closest_point(path_points,pos) 
    idx = find_closest_point(path_points,pos)
    num_points = path_points.shape[1]
    num_ellipses = np.ceil((idx+N+1)/num_points)
    path_points = np.tile(path_points,(1,int(num_ellipses)))
    return data_points[:,idx+1:idx+N+1]

def generate_closest_s(reference_track, pos):
    """Extract the next N points on the path for the next N stages starting from 
    the current car position pos
    """
    path_points=reference_track[:2,:]
    # idx = find_closest_point(path_points,pos) 
    idx0 = find_closest_point(path_points,pos)
    print("eucledian closest point is: ",path_points[:,idx0])
    if(idx0<2*window):
        idx_ver = find_closest_point_vertical(reference_track[:,0:2*window],pos)
        idx = idx_ver 
    else:
        idx_ver = find_closest_point_vertical(reference_track[:,idx0-window:idx0+window],pos)
        idx = idx0 + (idx_ver-window)
    return (idx/(np.shape(reference_track)[1]))*INDEX_MAX, idx

def generate_pathplanner():
    """Generates and returns a FORCESPRO solver that calculates a path based on
    constraints and dynamics while minimizing an objective function
    """
    # Model Definition
    # ----------------
    
    # Problem dimensions
    model = forcespro.nlp.SymbolicModel()
    model.N = 40  # horizon length
    model.nvar = 13  # number of variables
    model.neq = 11  # number of equality constraints
    model.npar = 4 # number of runtime parameters
    num_ins=model.nvar-model.neq

    # Objective function
    model.objective = obj
    model.objectiveN = obj # increased costs for the last stage
    # The function must be able to handle symbolic evaluation,
    # by passing in CasADi symbols. This means certain numpy funcions are not
    # available.

    # We use an explicit RK4 integrator here to discretize continuous dynamics
    integrator_stepsize = 0.05
    model.eq = lambda z, current_target: forcespro.nlp.integrate(continuous_dynamics, z[num_ins:model.nvar], z[0:num_ins], current_target,
                                                integrator=forcespro.nlp.integrators.RK4,
                                                stepsize=integrator_stepsize)
    
    # Indices on LHS of dynamical constraint - for efficiency reasons, make
    # sure the matrix E has structure [0 I] where I is the identity matrix.
    model.E = np.concatenate([np.zeros((model.neq,num_ins)), np.eye(model.neq)], axis=1)

    # Inequality constraints
    # from brake -> Fbrake = -4120.0
    model.lb = np.array([-3560.7*eff,  -np.deg2rad(30),  -400.,   -400.,  -np.inf,  0.0, -15.0, -15.0, -3560.7*eff, -np.deg2rad(30), 0,-1.0,-np.deg2rad(10.0)])
    model.ub = np.array([+3560.7*eff,  np.deg2rad(+30),  400.,   400.,   +np.inf, 15.0, +15.0, 15.0, 3560.7*eff, np.deg2rad(30), INDEX_MAX*2,1.0,np.deg2rad(10.0)])

    model.nh = 3 #number of inequality constr
    model.ineq = constr
    model.hu = np.array([+np.inf,np.inf,np.inf])
    model.hl = np.array([-np.inf,-np.inf,-np.inf])
    #track - tyres - sa
    # model.hu = np.array([2.0,1.0,0.15])p.
    # model.hl = np.array([-np.inf,-np.inf,-0.15])

    # Initial condition on vehicle states x
    model.xinitidx = range(num_ins,model.nvar) # use this to specify on which variables initial conditions
    # are imposed

    # Solver generation
    # -----------------
    # Set solver options
    codeoptions = forcespro.CodeOptions('FORCESNLPsolver')
    codeoptions.maxit = 10000    # Maximum number of iterations
    codeoptions.printlevel = 2  # Use printlevel = 2 to print progress (but 
    #                             not for timings)
    codeoptions.optlevel = 0    # 0 no optimization, 1 optimize for size, 
    #                             2 optimize for speed, 3 optimize for size & speed
    codeoptions.cleanup = False
    codeoptions.timing = 1
    # codeoptions.parallel = 4
    codeoptions.nlp.hessian_approximation = 'bfgs'
    codeoptions.solvemethod = 'SQP_NLP' # choose the solver method Sequential #Quadratic Programming
    codeoptions.nlp.bfgs_init = 2.5*np.identity(model.nvar)
    codeoptions.sqp_nlp.maxqps = 1      # maximum number of quadratic problems to be solved
    codeoptions.sqp_nlp.reg_hessian = 5e-5 # increase this if exitflag=-8
    # change this to your server or leave uncommented for using the 
    # standard embotech server at https://forces.embotech.com 
    codeoptions.server = 'https://forces.embotech.com'
    
    # Creates code for symbolic model formulation given above, then contacts 
    # server to generate new solver
    solver = model.generate_solver(options=codeoptions)

    return model,solver


def updatePlots(x,u,pred_x,pred_u,model,k):
    """Deletes old data sets in the current plot and adds the new data sets 
    given by the arguments x, u and predicted_z to the plot.
    x: matrix consisting of a set of state column vectors
    u: matrix consisting of a set of input column vectors
    pred_x: predictions for the next N state vectors
    pred_u: predictions for the next N input vectors
    model: model struct required for the code generation of FORCESPRO
    k: simulation step
    """
    fig = plt.gcf()
    ax_list = fig.axes
    
    # Delete old data in plot
    ax_list[0].get_lines().pop(-1).remove() # remove old prediction of trajectory
    ax_list[0].get_lines().pop(-1).remove() # remove old trajectory
    
    ax_list[1].get_lines().pop(-1).remove() # remove old prediction of velocity
    ax_list[1].get_lines().pop(-1).remove() # remove old velocity
    ax_list[2].get_lines().pop(-1).remove() # remove old prediction of heading angle
    ax_list[2].get_lines().pop(-1).remove() # remove old heading angle
    ax_list[3].get_lines().pop(-1).remove() # remove old prediction of steering angle
    ax_list[3].get_lines().pop(-1).remove() # remove old steering angle
    ax_list[4].get_lines().pop(-1).remove() # remove old prediction of acceleration force
    ax_list[4].get_lines().pop(-1).remove() # remove old acceleration force
    ax_list[5].get_lines().pop(-1).remove() # remove old prediction of steering rate
    ax_list[5].get_lines().pop(-1).remove() # remove old steering rate
    
    # state x = [xPos,yPos,phi, vx, vy, r, F, delta, index, ni, mi]
    # input u = [dF,ddelta]

    # Update plot with current simulation data
    ax_list[0].plot(x[0,0:k+2],x[1,0:k+2], '-b')             # plot new trajectory
    ax_list[0].plot(pred_x[0,1:], pred_x[1,1:], 'g-')   # plot new prediction of trajectory
   
    
    ax_list[1].plot(x[3,0:k+2],'b-')                         # plot new velocity
    ax_list[1].plot(range(k+1,k+model.N), pred_x[3,1:],'g-') # plot new prediction of velocity
    
    ax_list[2].step(range(0, k+1), \
        u[0, 0:k+1],'b-')                        # plot new force rate
    ax_list[2].step(range(k, k+model.N), \
        pred_u[0,:],'g-')                        # plot new prediction of force rate
    
    ax_list[3].plot(np.rad2deg(x[7, 0:k+2]),'b-')            # plot new steering angle
    ax_list[3].plot(range(k+1,k+model.N), \
        np.rad2deg(pred_x[7,1:]),'g-')                       # plot new prediction of steering angle
    ax_list[4].step(x[6, 0:k+2],'b-')         # plot new acceleration force
    ax_list[4].step(range(k+1, k+model.N), pred_x[6,1:],'g-')   # plot new prediction of acceleration force

    
    ax_list[5].step(range(0, k+1), \
        np.rad2deg(u[1, 0:k+1]),'b-')                        # plot new steer.rate
    ax_list[5].step(range(k, k+model.N), np.rad2deg(pred_u[1,:]),'g-')           # plot new steer.rate pred               
    plt.pause(0.05)


def createPlot(x,u,start_pred,sim_length,model,path_points,xinit,cones_yellow,cones_blue,cones_orange_big):
    """Creates a plot and adds the initial data provided by the arguments"""
    cones_blue = np.flip(cones_blue[:,:2], 1)
    print("cones yellow bef are: ",cones_yellow[:,:2])
    cones_yellow = np.flip(cones_yellow[:,:2], 1)
    print("cones yellow after are:", cones_yellow)
    cones_orange_big = np.flip(cones_orange_big[:,:2], 1)
     # Create empty plot
    fig = plt.figure()
    plt.clf()
    gs = GridSpec(5,2,figure=fig)
    print("what is gs ",gs)
    # Plot trajectory
    ax_pos = fig.add_subplot(gs[:,0])
    l0, = ax_pos.plot(np.transpose(path_points[0,:]), np.transpose(path_points[1,:]), '.',color='red')
    l1, = ax_pos.plot(xinit[0], xinit[1], 'bx')
    plt.title('Position')
    #plt.axis('equal')
    plt.xlim([-100.,20])
    plt.ylim([-30, 50])
    plt.xlabel('x-coordinate')
    plt.ylabel('y-coordinate')
    plt.axis('equal')
    l2, = ax_pos.plot(cones_blue[:,0], cones_blue[:,1], 'bo')
    l3, = ax_pos.plot(cones_yellow[:,0], cones_yellow[:,1], 'yo')
    l4, = ax_pos.plot(cones_orange_big[:,0],cones_orange_big[:,1], color='None', marker='o', markerfacecolor='darkorange', markeredgecolor='None', markersize=7)
    l5, = ax_pos.plot(x[0,0],x[1,0],'b-')
    l6, = ax_pos.plot(start_pred[2,:], start_pred[3,:],'g-')
    ax_pos.legend([l0,l5,l6],['desired trajectory','car trajectory',\
        'predicted car traj.'],loc='lower right')
    
    # state x = [xPos,yPos,phi, vx, vy, r, F, delta, index,ni,mi]
    # input u = [dF,ddelta]

    # Plot velocity
    ax_vel = fig.add_subplot(5,2,2)
    plt.grid("both")
    plt.title('Velocity')
    plt.xlim([0., sim_length-1])
    plt.plot([0, sim_length-1], np.transpose([model.ub[5], model.ub[5]]), 'r:') #me z index
    plt.plot([0, sim_length-1], np.transpose([model.lb[5], model.lb[5]]), 'r:')
    ax_vel.plot(0.,x[3,0], '-b')
    ax_vel.plot(start_pred[5,:], 'g-') #me z index
    
    # Plot force rate
    ax_frate = fig.add_subplot(5,2,4)
    plt.grid("both")
    plt.title('Force Rate')
    plt.xlim([0., sim_length-1])
    plt.plot([0, sim_length-1], np.transpose([model.ub[0], model.ub[0]]), 'r:')
    plt.plot([0, sim_length-1], np.transpose([model.lb[0], model.lb[0]]), 'r:')
    ax_frate.step(0., np.rad2deg(u[0,0]), 'b-')
    ax_frate.step(range(model.N), start_pred[0,:],'g-')

    # Plot steering angle
    ax_delta = fig.add_subplot(5,2,6)
    plt.grid("both")
    plt.title('Steering angle')
    plt.xlim([0., sim_length-1])
    plt.plot([0, sim_length-1], np.rad2deg(np.transpose([model.ub[9], model.ub[9]])), 'r:')
    plt.plot([0, sim_length-1], np.rad2deg(np.transpose([model.lb[9], model.lb[9]])), 'r:')
    ax_delta.plot(np.rad2deg(x[7,0]),'b-')
    ax_delta.plot(np.rad2deg(start_pred[9,:]),'g-')

    # Plot acceleration force
    ax_F = fig.add_subplot(5,2,8)
    plt.grid("both")
    plt.title('Acceleration force')
    plt.xlim([0., sim_length-1])
    plt.plot([0, sim_length-1], np.transpose([model.ub[8], model.ub[8]]), 'r:')
    plt.plot([0, sim_length-1], np.transpose([model.lb[8], model.lb[8]]), 'r:')
    ax_F.step(0, x[6,0], 'b-')
    ax_F.step(range(model.N), start_pred[8,:],'g-')

    # Plot steering rate
    ax_phi = fig.add_subplot(5,2,10)
    plt.grid("both")
    plt.title('Steering rate')
    plt.xlim([0., sim_length-1])
    plt.plot([0, sim_length-1], np.rad2deg(np.transpose([model.ub[1], model.ub[1]])), 'r:')
    plt.plot([0, sim_length-1], np.rad2deg(np.transpose([model.lb[1], model.lb[1]])), 'r:')
    ax_phi.step(0., np.rad2deg(u[1,0]), 'b-')
    ax_phi.step(range(model.N),  np.rad2deg(start_pred[1,:]),'g-')

    plt.tight_layout()

    # Make plot fullscreen. Comment out if platform dependent errors occur.
    mng = plt.get_current_fig_manager()
    
def cubic_spline_generation(x, y):
    dx = np.diff(x)
    dy = np.diff(y)
    ds = np.sqrt(dx**2 + dy**2)
    s = np.cumsum(ds)
    num_points = int(s[-1]/ds_wanted)
    s = arc_length_parameterization(x, y)
    t = np.linspace(0,s_max,num_points)
    cs = CubicSpline(s, np.column_stack((x, y)), axis=0,bc_type='natural')
    interpolated_points = cs(t).T
    dy = cs.derivative(1)(t)[:,1]
    dx = cs.derivative(1)(t)[:,0]
    ddy = cs.derivative(2)(t)[:,1]
    ddx = cs.derivative(2)(t)[:,0]
    # Compute curvature, heading_angle and u_ref
    local_angle = np.arctan2(dy, dx)
    curvature = np.abs(dx * ddy - dy * ddx) / (dx**2 + dy**2)**(3/2)
    interpolated_points = np.vstack([interpolated_points,local_angle,curvature])
    return cs, interpolated_points

def cubic_spline_inference(cs,parameter):
    parameter_array=[]
    for i in range(np.shape(parameter)[0]):
        interpolated_points = cs(parameter[i])
        # Compute derivatives
        dx = cs.derivative(1)(parameter)[i,0]
        ddx = cs.derivative(2)(parameter)[i,0]
        dy = cs.derivative(1)(parameter)[i,1]
        ddy = cs.derivative(2)(parameter)[i,1]

        # Compute curvature, heading_angle and u_ref
        local_angle = np.arctan2(dy, dx)
        curvature = np.abs(dx * ddy - dy * ddx) / (dx**2 + dy**2)**(3/2)
        u_ref=(mi_y_max*g/curvature)
        if(u_ref>u_upper): u_ref=u_upper
        parameter_array.append([interpolated_points[0]])
        parameter_array.append([interpolated_points[1]])
        parameter_array.append([local_angle])
        parameter_array.append([curvature])
        # parameter_array.append([u_ref])
    # print(np.shape(parameter_array))
    return parameter_array

def main():
    #import data from path planning
    emergency_count=0
    fig,ax = plt.subplots()
    fig.set_size_inches(13,9)
    #from C++ path planning
    data_points=Dataloader("P23-DV/MPC/MPC_embotech/Data/als_data.txt")
    midpoints = Dataloader("P23-DV/MPC/MPC_embotech/Data/trackdrive_midpoints.txt")
    print(np.shape(1))
    cs,reference_track = cubic_spline_generation(midpoints[0,:],midpoints[1,:])
    print("reference track: ",reference_track, np.shape(reference_track))
    # generate code for estimator
    model, solver = generate_pathplanner()
    num_ins = model.nvar-model.neq
    
    # Simulation
    sim_length = 500 # simulate sim_length*0.05

    # Variables for storing simulation data
    x = np.zeros((model.neq,sim_length+1))  # states
    u = np.zeros((num_ins,sim_length))  # inputs

    # Set initial guess to start solver from
    x0i = np.zeros((model.nvar,1))
    x0 = np.transpose(np.tile(x0i, (1, model.N)))
    print("shape x0: ",np.shape(x0))
    # Set initial condition
    #  x    y     theta    vx   vy  r F delta index ni mi
    vx0 = 0.1
    Frx0 = 300
    xinit = np.transpose(np.array([reference_track[0][0], reference_track[1][0],reference_track[2][0], vx0, 0.0, 0.0, Frx0, 0.,0.,0.,0.]))
    print("xinit is: ",xinit)
    x[:,0] = xinit

    problem = {"x0": x0,
            "xinit": xinit}
    # Create 2D points on ellipse which the car is supposed to follow
    start_pred = np.reshape(problem["x0"],(model.nvar,model.N)) # first prdicition corresponds to initial guess

    # generate plot with initial values
    createPlot(x,u,start_pred,sim_length,model,reference_track[:2,:],xinit,cones_yellow, cones_blue, cones_orange_big)
    wheel_torques_txt=[]
    steering_txt=[]
    time_array=[]
    err_1=0.0
    err_2=0.0

    # Simulation
    for k in range(sim_length):
        print("Im at iteration: ",k+1)
        print("shape x0: ",np.shape(x0))
        # Set initial condition
        problem["xinit"] = x[:,k]
        parameters_array = []
        parameters_array_temp = []
        print("x_init is:",problem["xinit"],problem["xinit"][8])
        print("Im at: ", problem["xinit"][0], problem["xinit"][1])
        if(k==0):
            s_start=0.0
            next_data_points = reference_track[:,1:model.N+1]
            problem["all_parameters"] = np.reshape(np.transpose(next_data_points),(model.npar*model.N,1))
            print(np.shape(next_data_points),next_data_points[3][0])
        if(k>0):
            s_start = problem["xinit"][8]
            if(s_start>309.90): s_start=0.0
            splines_loc = cubic_spline_inference(cs,[s_start,s_start])
            print("splines_loc is:",splines_loc[0][0]," ", splines_loc[1][0], " ",s_start)
            s_closest, _ = generate_closest_s(reference_track, x[0:2,k])
            closest_loc = cubic_spline_inference(cs,[s_closest,s_closest])
            print("closest loc is:",closest_loc[0][0]," ", closest_loc[1][0], " ",s_closest)
            # print("where i am is:",where_i_am,np.shape(where_i_am))
            err_1 = np.abs(np.sin(closest_loc[2][0])*(problem["xinit"][0]-closest_loc[0][0]) - np.cos(closest_loc[2][0])*(problem["xinit"][1]-closest_loc[1][0])) #katakorifi
            err_2 = np.sqrt((problem["xinit"][0] - closest_loc[0][0])**2 + (problem["xinit"][1] - closest_loc[1][0])**2)
            ni = problem["xinit"][9]
            mi = problem["xinit"][10]
            err_array.append(err_1)
            print("errors from closest are are:", err_1," ",err_2," ",np.max(err_array)," ",np.mean(err_array))
            print("ni and mi is: ",ni, " ", np.rad2deg(mi))
            if(err_1 < 0.9):
                print("no emergency!")
                emergency_bool=0
                parameters_array = [s_closest]
                s_new=s_closest
                if(s_closest>309.990): s_closest = 0.0 #reset index for next lap
                for i in range (model.N-1):
                    s_solver=pred_x[8,i]
                    if(s_solver<s_closest):
                        s_new=s_new+0.2
                        s_solver=s_new  
                    parameters_array.append(s_solver)
                    next_data_points = cubic_spline_inference(cs,np.array(parameters_array))
                    next_data_points = cubic_spline_inference(cs,np.array(pred_x[8,:]))
                    problem["all_parameters"] = next_data_points
                print("parameters array with ds(used) is: ",parameters_array,np.shape(parameters_array))
            else:
                print("mpika emergency!!!")
                emergency_count+=1
                emergency_bool=1
                s_start, idx_start = generate_closest_s(reference_track, x[0:2,k])
                next_data_points = reference_track[:,idx_start:idx_start+model.N]
                problem["all_parameters"] = np.reshape(np.transpose(next_data_points),(model.npar*model.N,1))
            print("s_array is: ",pred_x[8,:])
        # print("final parameters are: ", problem["all_parameters"],np.shape(problem["all_parameters"]))
        # Time to solve the NLP!
        # print("Im trying to follow are at: ",data_points[0, int(problem["xinit"][8])], data_points[1, int(problem["xinit"][8])])
        print("First param for MPC is : ",problem["all_parameters"][0],problem["all_parameters"][1])
        print("Second param for MPC is : ",problem["all_parameters"][4],problem["all_parameters"][5])
        # print("dr between me and params is: ", np.sqrt((problem["xinit"][0]-problem["all_parameters"][0][0])**2 + (problem["xinit"][1]-problem["all_parameters"][1][0])**2))
        output, exitflag, info = solver.solve(problem)

        # Make sure the solver has exited properly.
        # print(exitflag)
        assert exitflag == 1, "bad exitflag"
        sys.stderr.write("FORCESPRO took {} iterations and {} seconds to solve the problem.\n"\
            .format(info.it, info.solvetime))
        time_array.append(info.solvetime)

        # Extract output
        temp = np.zeros((np.max(model.nvar), model.N))
        for i in range(0, model.N):
            temp[:, i] = output['x{0:02d}'.format(i+1)]
        pred_u = temp[0:num_ins, :]
        pred_x = temp[num_ins:model.nvar, :]
        # Apply optimized input u of first st
        # u[:,k] = pred_u[:,0]
        
        u[:,k] = pred_u[:,1]
        curv_array=[next_data_points[0][0],next_data_points[1][0],next_data_points[3][0]]
        x[:,k+1] = np.transpose(model.eq(np.concatenate((u[:,k],x[:,k])),np.array(curv_array)))
        print("u_bef is: ",u[:,k]," ",np.shape(u), " ",np.shape(u[:,k]))
        print("x_bef is: ",x[:,k]," ",np.shape(x), " ", np.shape(x[:,k]))
        print("publishing torque and steering cmd: ",(x[6,k+1])/(5*3.9/0.85)," ",np.rad2deg(x[7,k+1]))
        print("i have used emergency manouvre: ",emergency_count," times")
        print("velocities after integration are: ", x[3,k], " ", x[4,k], " ", x[5,k])
        wheel_torques_txt.append(x[:,k][6]*0.2)
        steering_txt.append(x[:,k][7])
        # plot results of current simulation step
        if(k%100==0):
            file = open("P23-DV/MPC/MPC_embotech/Data/steering.txt", "w+")
            file.write(str(steering_txt))
            file.close()
            file2 = open("P23-DV/MPC/MPC_embotech/Data/torques.txt", "w+")
            file2.write(str(wheel_torques_txt))
            file2.close()
        if(k%10==0):
            updatePlots(x,u,pred_x,pred_u,model,k)  
            if k == sim_length-1:
                fig=plt.gcf()
                ax_list = fig.axes
                ax_list[0].get_lines().pop(-1).remove()   # remove old prediction of trajectory
                ax_list[0].legend(['desired trajectory','init pos','car trajectory'], \
                    loc='lower right')
                plt.show()
            else:
                plt.draw()
        print()

    print("steering_txt is:", steering_txt)
    print()
    print("wheel_torques_txt is: ",wheel_torques_txt)
    print()
    print("error array is: ",err_array)
    print()
    print("time array is: ", time_array)
    print()
    print("error data are: ", np.max(err_array)," ",np.mean(err_array))
    print()
    print("time data are: ",np.max(time_array), " ",np.mean(time_array))

if __name__ == "__main__":
    main()