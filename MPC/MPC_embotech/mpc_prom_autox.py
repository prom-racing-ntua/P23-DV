# See also FORCES_NLP
# (c) Embotech AG, Zurich, Switzerland, 2013-2022.

import sys
import os
print(sys.version)
file_path = os.path.realpath(__file__)
print("file path is: ",file_path)
import numpy as np
from numpy import *
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
    print("s_max is: ",s_max)
    # s /= s[-1]
    s = np.insert(s,0,0)
    print(s)
    return s

# set physical constants
#up to date
lol = 1.59 
WD_front = 0.467
l_f = lol*(1-WD_front)
l_r = lol*WD_front
CdA = 2.0 # for drag (changed)
ClA = 7.0 # for downforce (changed)
pair = 1.225
u_upper=15.0
m = 190.0   # mass of the car
g = 9.81
Iz = 110.0
ds_wanted=0.1
eff=0.85
h_cog=0.27
window=10
ellipse_array=[]

#for dynamic model
# B=-7.0
# C=1.456
C_tire=0.66
# D= 3000.0
cs=-16419.31
dt_integration=0.025

#compute l_a
umin=3.0
umax=6.0
INDEX_MAX=309.1
DINDEX_MAX=309.1 #just for one lap (TBC)

#ellipse params
# a=1.46
# b=1.62

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


def continuous_dynamics(x, u):
    """Defines dynamics of the car, i.e. equality constraints.
    parameters:
    state x = [xPos,yPos,phi, vx, vy, r, F, delta, index]
    input u = [dF,ddelta, dindex]
    """
    temp1=casadi.fmax((x[3]-umin)/(umax-umin),0)
    l_a=casadi.fmin(temp1,1)

    # saf=casadi.arctan((x[4]+l_f*x[5])/np.sqrt(x[3]**2+1)) - x[7]
    # sar=casadi.arctan((x[4]-l_r*x[5])/np.sqrt(x[3]**2+1))
    saf,sar = getSasWithState(x) 
    Ffz,Frz = getFzWithState(x)
    Ffy = getFy(Ffz,saf)
    Fry = getFy(Frz,sar)

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
    dindexdot = u[2]
    return casadi.vertcat(xdot,ydot,phidot,vxdot,vydot,rdot,Fdot,deltadot,dindexdot)

err_array=[]
def getEllipseRatioWithState(x):
    saf,sar = getSasWithState(x) 
    Ffz,Frz = getFzWithState(x)
    a,b = getEllipseParams(Frz)
    Fry = getFy(Frz,sar)
    per_= (x[6]/(a*Frz))**2 + (Fry/(b*Frz))**2
    return per_

def getFy(Fz,sa):
    ex=1e-7
    P=[1.45673747e+00, -1.94554660e-04,  2.68063018e+00,  3.19197941e+04, 2.58020549e+00, -7.13319396e-04]
    C=P[0]
    D=P[1]*Fz*Fz+P[2]*Fz
    BCD = P[3]*casadi.sin(P[4]*casadi.arctan(P[5]*Fz))
    B=BCD/(C*D)
    F_temp = C_tire*D*casadi.sin(C*casadi.arctan(B*sa))
    return F_temp

def getSas(z):
    saf_temp=casadi.arctan((z[7]+l_f*z[8])/(z[6]+1e-3)) - z[10]
    sar_temp=casadi.arctan((z[7]-l_r*z[8])/(z[6]+1e-3))
    return saf_temp,sar_temp

def getFz(z):
    a_temp = (z[9] - 0.5*CdA*pair*(z[6])**2)/ m
    dw = (h_cog/lol)*(a_temp/g)*m*g
    Ffz_temp=(l_r/(l_f+l_r))*m*g + 0.25*pair*ClA*(z[6]**2) - dw
    Frz_temp=(l_f/(l_f+l_r))*m*g + 0.25*pair*ClA*(z[6]**2) + dw
    return Ffz_temp,Frz_temp

def getFzWithState(x):
    a_temp = (x[6] - 0.5*CdA*pair*(x[3])**2)/ m
    dw = (h_cog/lol)*(a_temp/g)*m*g
    Ffz_temp=(l_r/(l_f+l_r))*m*g + 0.25*pair*ClA*(x[3]**2) - dw
    Frz_temp=(l_f/(l_f+l_r))*m*g + 0.25*pair*ClA*(x[3]**2) + dw
    return Ffz_temp,Frz_temp


def getSasWithState(x):
    saf_temp=casadi.arctan((x[4]+l_f*x[5])/(x[3]+1e-3)) - x[7]
    sar_temp=casadi.arctan((x[4]-l_r*x[5])/(x[3]+1e-3))
    return saf_temp,sar_temp

def getEllipseParams(Fz):
    Fz0=1112.0554070627252
    dfz=(Fz-Fz0)/Fz0
    C_tire=0.66
    mx_max=C_tire*(2.21891927-1.36151651e-07*dfz)
    my_max=C_tire*(2.46810824-0.21654031*dfz)
    return mx_max,my_max

def obj(z,current_target):
    """Least square costs on deviating from the path and on the inputs F and phi
    z = [dF,ddelta,dindex,xPos,yPos,phi, vx, vy, r, F, delta,index]
    current_target = point on path that is to be headed for
    """
    saf,sar = getSas(z) 
    Ffz,Frz = getFz(z)
    a,b = getEllipseParams(Frz)
    Fry = getFy(Frz,sar)
    #extra terms for cost function
    dyn_sa  = casadi.arctan(z[7]/(z[6]+1e-3))
    beta = casadi.arctan(l_r/(l_f + l_r) * casadi.tan(z[10]))
    dsa =(dyn_sa-beta)
    e_c= casadi.sin(current_target[2])*(z[3]-current_target[0]) - casadi.cos(current_target[3])*((z[4]-current_target[1])) #katakorifi
    e_l= -casadi.cos(current_target[2])*(z[3]-current_target[0]) - casadi.sin(current_target[3])*((z[4]-current_target[1])) #orizontia
    return (
        5e2*(z[3]-current_target[0])**2 # costs on deviating on the path in x-direction
            + 5e2*(z[4]-current_target[1])**2 # costs on deviating on the path in y-direction
            # + 0e1*(e_c)**2 # costs on deviating on the
            #                             # path in y-direction
            # + 1e3*(e_l)**2 # costs on deviating on the
            #                     #path in x-direction
            # + 1e-3*(z[5]-current_target[2])**2 #dphi gap
            + 1e2*(z[6]-current_target[3])**2
            + 1e2*(z[0]/1000)**2 # penalty on input F,dF
            + 1e2*(z[9]/1000)**2
            + 5e2*z[1]**2 #penalty on delta,ddelta
            + 5e2*z[10]**2
            + 1e-3*(sar**2)
            # + 1e-3*(dsa**2)
            + 1e-2*((z[9]/(a*Frz))**2 + (Fry/(b*Frz))**2)
            # + 1e-1*((1/(z[6]**2 +1e-3))) #vx and index
            - 1e-3*(z[6])
            - 1e-3*(z[11]/INDEX_MAX)
            - 1e-3*(z[2]/INDEX_MAX))

def constr(z,current_target):
    """Least square costs on deviating from the path and on the inputs F and phi
    z = [dF,ddelta,dindex, xPos,yPos,phi, vx, vy, r, F, delta, index]
    current_target = point on path that is to be headed for
    """
    saf,sar = getSas(z) 
    Ffz,Frz = getFz(z)
    Fry = getFy(Frz,sar)
    a,b = getEllipseParams(Frz)
    constr1 = (z[2]-current_target[0])**2 + (z[3]-current_target[1])**2 #inside track constraint
    # constr1 = (z[6]-current_target[3])**2 #velocity constraint
    constr2 = (z[8]/(a*Frz))**2 + (Fry/(b*Frz))**2 #tyre constraints
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
    dist_final_no_abs = part_x - part_y
    print("vertical distance array is ", dist_final, " ",dist_final_no_abs)
    dist_final = dist_final_no_abs #remove absolute value
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

def generate_closest_s(reference_track, pos,emergency_bool):
    """Extract the next N points on the path for the next N stages starting from 
    the current car position pos
    """
    path_points=reference_track[:2,:]
    if(emergency_bool):
        idx = find_closest_point(path_points,pos) + int((1/ds_wanted))
    else:
        idx = find_closest_point(path_points,pos) 
    # idx0 = find_closest_point(path_points,pos)
    print("final(=euclidean) closest point is: ",path_points[:,idx]," ",idx)
    # if(idx0<2*window):
    #     idx_ver = find_closest_point_vertical(reference_track[:,0:2*window],pos)
    #     idx = idx_ver 
    # else:
    #     idx_ver = find_closest_point_vertical(reference_track[:,idx0-window:idx0+window],pos)
    #     idx = idx0 + (idx_ver-window)
    return (idx/(np.shape(reference_track)[1]))*INDEX_MAX, idx

def generate_pathplanner():
    """Generates and returns a FORCESPRO solver that calculates a path based on
    constraints and dynamics while minimizing an objective function
    """
    # Model Definition
    # ----------------
    
    # Problem dimensions
    model = forcespro.nlp.SymbolicModel()
    model.N = 30  # horizon length
    model.nvar = 12  # number of variables
    model.neq = 9  # number of equality constraints
    model.npar = 4 # number of runtime parameters
    num_ins=model.nvar-model.neq

    # Objective function
    model.objective = obj
    model.objectiveN = obj # increased costs for the last stage
    # The function must be able to handle symbolic evaluation,
    # by passing in CasADi symbols. This means certain numpy funcions are not
    # available.

    # We use an explicit RK4 integrator here to discretize continuous dynamics
    integrator_stepsize = dt_integration
    model.eq = lambda z: forcespro.nlp.integrate(continuous_dynamics, z[num_ins:model.nvar], z[0:num_ins],
                                                integrator=forcespro.nlp.integrators.RK4,
                                                stepsize=integrator_stepsize)
    
    # Indices on LHS of dynamical constraint - for efficiency reasons, make
    # sure the matrix E has structure [0 I] where I is the identity matrix.
    model.E = np.concatenate([np.zeros((model.neq,num_ins)), np.eye(model.neq)], axis=1)

    # Inequality constraints
    # from brake -> Fbrake = -4120.0
    model.lb = np.array([-3560.7*eff,  -np.deg2rad(30), 0.0, -400.,   -400.,  -np.inf,  -1e-6, -15.0, -15.0, -3560.7*eff, -np.deg2rad(30), 0])
    model.ub = np.array([+3560.7*eff,  np.deg2rad(+30), INDEX_MAX, 400.,   400.,   +np.inf, 15.0, +15.0, 15.0, 3560.7*eff, np.deg2rad(30), INDEX_MAX*100])

    model.nh = 3 #number of inequality constr
    model.ineq = constr
    model.hu = np.array([+np.inf,+1.0,+np.inf])
    model.hl = np.array([-np.inf,-np.inf,-np.inf])
    #track - tyres - sar
    # model.hu = np.array([2.0,1.0,0.15])
    # model.hl = np.array([-np.inf,-np.inf,-0.15])

    # Initial condition on vehicle states x
    model.xinitidx = range(num_ins,model.nvar) # use this to specify on which variables initial conditions
    # are imposed

    # Solver generation
    # -----------------
    # Set solver options
    codeoptions = forcespro.CodeOptions('FORCESNLPsolver')
    codeoptions.maxit = 500    # Maximum number of iterations
    codeoptions.printlevel = 2  # Use printlevel = 2 to print progress (but 
    #                             not for timings)
    codeoptions.optlevel = 0    # 0 no optimization, 1 optimize for size, 
    #                             2 optimize for speed, 3 optimize for size & speed
    codeoptions.cleanup = False
    codeoptions.timing = 1
    codeoptions.mip.mipgap = 0.5
    codeoptions.mip.explore = 'depthFirst'
    codeoptions.mip.branchon = 'mostAmbiguous'
    codeoptions.parallel = 4
    # codeoptions.server = 'https://forces-6-2-0.embotech.com'
    codeoptions.nlp.hessian_approximation = 'bfgs'
    codeoptions.solvemethod = 'SQP_NLP' # choose the solver method Sequential #Quadratic Programming
    # codeoptions.nlp.bfgs_init = 3.0*np.identity(model.nvar)
    codeoptions.sqp_nlp.maxqps = 1   # maximum number of quadratic problems to be solved
    codeoptions.sqp_nlp.reg_hessian = 1e-5 # increase this if exitflag=-8
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
    
    # state x = [xPos,yPos,phi, vx, vy, r, F, delta, index]
    # input u = [dF,ddelta, dindex]

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
    
    # state x = [xPos,yPos,phi, vx, vy, r, F, delta, index]
    # input u = [dF,ddelta, dindex]

    # Plot velocity
    ax_vel = fig.add_subplot(5,2,2)
    plt.grid("both")
    plt.title('Velocity')
    plt.xlim([0., sim_length-1])
    plt.plot([0, sim_length-1], np.transpose([model.ub[6], model.ub[6]]), 'r:') #me z index
    plt.plot([0, sim_length-1], np.transpose([model.lb[6], model.lb[6]]), 'r:')
    ax_vel.plot(0.,x[3,0], '-b')
    ax_vel.plot(start_pred[6,:], 'g-') #me z index
    
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
    plt.plot([0, sim_length-1], np.rad2deg(np.transpose([model.ub[10], model.ub[10]])), 'r:')
    plt.plot([0, sim_length-1], np.rad2deg(np.transpose([model.lb[10], model.lb[10]])), 'r:')
    ax_delta.plot(np.rad2deg(x[7,0]),'b-')
    ax_delta.plot(np.rad2deg(start_pred[10,:]),'g-')

    # Plot acceleration force
    ax_F = fig.add_subplot(5,2,8)
    plt.grid("both")
    plt.title('Acceleration force')
    plt.xlim([0., sim_length-1])
    plt.plot([0, sim_length-1], np.transpose([model.ub[9], model.ub[9]]), 'r:')
    plt.plot([0, sim_length-1], np.transpose([model.lb[9], model.lb[9]]), 'r:')
    ax_F.step(0, x[6,0], 'b-')
    ax_F.step(range(model.N), start_pred[9,:],'g-')

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
    print(curvature,np.shape(curvature))
    vel_prof_dummy = [1.0 for i in range (np.shape(curvature)[0])]
    interpolated_points = np.vstack([interpolated_points,local_angle,curvature])
    return cs, interpolated_points

def cubic_spline_inference(cs,parameter,x,finish_flag):
    x_temp=x.copy()
    parameter_array=[]
    _,Frz=getFzWithState(x)
    _,b=getEllipseParams(Frz)
    u_max=[]
    u_final=[]
    u_first = x_temp[3]
    ux_init=x_temp[3]
    u_first_array = []
    for i in range(np.shape(parameter)[0]):
        interpolated_points = cs(parameter[i])
        # Compute derivatives
        dx = cs.derivative(1)(parameter)[i,0]
        ddx = cs.derivative(2)(parameter)[i,0]
        dy = cs.derivative(1)(parameter)[i,1]
        ddy = cs.derivative(2)(parameter)[i,1]
        # Compute curvature, heading_angle and u_ref
        local_angle = np.arctan2(dy, dx)
        ##velocity profile started
        curvature = np.abs(dx * ddy - dy * ddx) / (dx**2 + dy**2)**(3/2)
        u_first=np.sqrt((b*g/curvature))
        if(u_first>u_upper):u_first=u_upper
        u_max.append(u_first)
        if(i==0): 
            # u_forward=x_temp[3] 
            u_forward = u_first
        elif(i>0 and i<=np.shape(parameter)[0]-1):
            x_temp[3]=u_output1
            Ffz,Frz = getFzWithState(x_temp)
            a,b = getEllipseParams(Frz)
            Fy_remain = m*(x_temp[3]**2)*curvature #Fry with kentromolos
            saf,sar = getSasWithState(x) 
            Fy_remain = getFy(Frz,sar) #actual Fry
            if((Frz)**2-(Fy_remain/b)**2<0): Fx_remain=0
            else: Fx_remain = a*np.sqrt(Frz**2-(Fy_remain/b)**2)
            ds_temp=parameter[i]-parameter[i-1]
            # ds_temp=0.1
            u_forward=np.sqrt(x_temp[3]**2+2*(Fx_remain/m)*np.abs(ds_temp))
        u_output1=np.min(np.array([u_forward,u_first]))
        # if(u_output>u_upper): u_output=u_upper 
        parameter_array.append([interpolated_points[0]])
        parameter_array.append([interpolated_points[1]])
        parameter_array.append([local_angle])
        parameter_array.append([u_output1])
        u_first_array.append(u_output1)
    x_temp[3]=u_first_array[np.shape(parameter)[0]-1]
    for j in range(np.shape(parameter)[0]):
        interpolated_points = cs(parameter[i])
        # Compute derivatives
        dx = cs.derivative(1)(parameter)[np.shape(parameter)[0]-1-j,0]
        ddx = cs.derivative(2)(parameter)[np.shape(parameter)[0]-1-j,0]
        dy = cs.derivative(1)(parameter)[np.shape(parameter)[0]-1-j,1]
        ddy = cs.derivative(2)(parameter)[np.shape(parameter)[0]-1-j,1]
        # Compute curvature, heading_angle and u_ref
        local_angle = np.arctan2(dy, dx)
        ##velocity profile started
        curvature = np.abs(dx * ddy - dy * ddx) / (dx**2 + dy**2)**(3/2)
        if(j==0):
            u_backward = u_first_array[np.shape(parameter)[0]-1]
        elif(j>0 and i<=np.shape(parameter)[0]-1):
            x_temp[3]=u_output2
            saf,sar = getSasWithState(x_temp) 
            Ffz,Frz = getFzWithState(x_temp)
            a,b = getEllipseParams(Frz)
            Fy_remain = m*(x_temp[3]**2)*curvature #Fry with kentromolos
            saf,sar = getSasWithState(x) 
            Fy_remain = getFy(Frz,sar) #actual Fry
            if(Frz**2-(Fy_remain/b)**2<0): Fx_remain = 0.0
            else: Fx_remain = a*np.sqrt(Frz**2-(Fy_remain/b)**2)
            ds_temp=parameter[np.shape(parameter)[0]-1-j]-parameter[np.shape(parameter)[0]-j]
            # ds_temp=0.1
            if(x_temp[3]**2-2*(Fx_remain/m)*np.abs(ds_temp)<0):u_backward=x_temp[3]
            else: u_backward=np.sqrt(np.abs(x_temp[3]**2-2*(Fx_remain/m)*np.abs(ds_temp)))
        u_output2=np.min(np.array([u_backward,u_first_array[np.shape(parameter)[0]-1-j]]))   
        u_final.append(u_output2) 
    u_final_final = []
    u_final = np.flip(u_final)
    for i in range(np.shape(parameter)[0]): 
        print("spline parameter and ux is: ",parameter[i]," ",ux_init)
        if(finish_flag==0): u_final_final.append(u_final[i]) #pernaw to max
        if(finish_flag==1 and ux_init>=3.0):
            print("started slowing down")
            u_final_final.append(3.0)
        if(finish_flag==1 and ux_init<3.0):
            print("started hitting brake")
            u_final_final.append(0.0)   
    print("ux_init is: ",ux_init)
    print("u_max is: ",u_max,np.shape(u_max))
    print("u_first is: ",u_first_array,np.shape(u_first_array))
    print("u_second is: ",np.flip(u_final),np.shape(u_final))
    print("u_final is: ",u_final_final,np.shape(u_final_final)) #komple mexri dw
    print("params array has shape",np.shape(parameter_array))
    if(np.shape(parameter)[0]==2):
        return parameter_array
    else:
        for k in range(4*np.shape(parameter)[0]):
            if(k%4==3):
                parameter_array[k] = [u_final_final[int((k-3)/4)]]
        print("parameter array after writing down: ",parameter_array,np.shape(parameter_array))
        return parameter_array


def main():
    #import data from path planning
    model, solver = generate_pathplanner()
    emergency_count=0
    fig,ax = plt.subplots()
    fig.set_size_inches(13,9)
    #from C++ path planning
    data_points = Dataloader("Data/als_data.txt")
    midpoints = Dataloader("Data/trackdrive_midpoints.txt")
    print(np.shape(1))
    cs,reference_track = cubic_spline_generation(midpoints[0,:],midpoints[1,:])
    print("reference track: ",reference_track, np.shape(reference_track))
    # for_mpc = cubic_spline_inference(cs,[0.0,0.0])
    # generate code for estimator
    num_ins = model.nvar-model.neq
    print("after solver generation")
    
    # Simulation
    sim_length = 10000 # simulate sim_length*0.05
    counter_all=0

    # Variables for storing simulation data
    x = np.zeros((model.neq,sim_length+1))  # states
    u = np.zeros((num_ins,sim_length))  # inputs

    # Set initial guess to start solver from
    x0i = np.zeros((model.nvar,1))
    x0 = np.transpose(np.tile(x0i, (1, model.N)))
    print("shape x0: ",np.shape(x0))
    # Set initial condition
    #  x    y     theta    vx   vy  r F delta
    vx0 = 0.1
    Frx0 = 300
    xinit = np.transpose(np.array([reference_track[0][0], reference_track[1][0],reference_track[2][0], vx0, 0.0, 0.0, Frx0, 0.,0.]))
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
    ellipse_counter=0
    finish_flag=0
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
        if(k>0):
            s_start = problem["xinit"][8]
            if(s_start>INDEX_MAX): s_start=INDEX_MAX
            critical_dist = np.sqrt((problem["xinit"][0])**2+problem["xinit"][1]**2)
            dist_thres=0.5
            #skidpad_logic_started
            if(np.abs(critical_dist) < dist_thres and change_flag==0):
                counter_all+=1
                change_flag=1
            if(np.abs(critical_dist)>dist_thres):
                change_flag=0
            print("counter, change flag, distance are: ", counter_all, " ", change_flag, " ", critical_dist)
            x_for_splines=problem["xinit"]
            splines_loc = cubic_spline_inference(cs,[s_start,s_start],x_for_splines,finish_flag)
            print("splines_loc is:",splines_loc[0][0]," ", splines_loc[1][0], " ",s_start)
            s_closest, _ = generate_closest_s(reference_track, x[0:num_ins-1,k],0)
            closest_loc = cubic_spline_inference(cs,[s_closest,s_closest],x_for_splines,finish_flag)
            print("closest loc is:",closest_loc[0][0]," ", closest_loc[1][0], " ",s_closest)
            # print("where i am is:",where_i_am,np.shape(where_i_am))
            err_1= np.abs(np.sin(closest_loc[2][0])*(problem["xinit"][0]-closest_loc[0][0]) - np.cos(closest_loc[2][0])*(problem["xinit"][1]-closest_loc[1][0])) #katakorifi
            err_2 = np.sqrt((problem["xinit"][0] - closest_loc[0][0])**2 + (problem["xinit"][1] - closest_loc[1][0])**2)
            err_array.append(err_1)
            print("errors from closest are are:", err_1," ",err_2," ",np.max(err_array)," ",np.mean(err_array))
            if(err_1 < 1.0):
                print("no emergency!")
                emergency_bool=0
                parameters_array = [s_closest]
                parameters_array_temp = [s_closest]
                s_new=s_closest
                print("finish flag is: ",finish_flag)
                if(counter_all==2): 
                    finish_flag=1
                    s_closest = INDEX_MAX + 7.5 #reset index for next lap
                for i in range (model.N-1):
                    ds_step=0.25 #constant with vel.profile 
                    s_new+=ds_step
                    if(s_new>INDEX_MAX+7.5):s_new=INDEX_MAX+7.5
                    s_solver=pred_x[8,i] 
                    if(finish_flag==0):parameters_array.append(s_new)
                    else:parameters_array.append(INDEX_MAX+7.5)
                    parameters_array_temp.append(s_solver)
                x_for_splines=problem["xinit"]
                next_data_points = cubic_spline_inference(cs,np.array(parameters_array),x_for_splines,finish_flag)
                problem["all_parameters"] = next_data_points
                print("parameters array with s(not_used) is: ",parameters_array_temp,np.shape(parameters_array_temp))
                print("parameters array with ds(used) is: ",parameters_array,np.shape(parameters_array))
            else:
                print("mpika emergency!!!")
                emergency_count+=1
                emergency_bool=1
                s_start, idx_start = generate_closest_s(reference_track, x[0:num_ins-1,k], emergency_bool)
                ds_emerg=0.25
                if(s_start > INDEX_MAX): s_start = INDEX_MAX
                parameters_array_emergency=[s_start]
                for i in range (model.N-1):
                    s_start+=ds_emerg
                    if(s_start>INDEX_MAX): s_start = INDEX_MAX
                    parameters_array_emergency.append(s_start)
                x_for_splines=problem["xinit"]
                next_data_points = reference_track[:,idx_start:idx_start+model.N]
                problem["all_parameters"] = np.reshape(np.transpose(next_data_points),(model.npar*model.N,1))
            print("ds array is: ",pred_u[2,:])
            print("s_array is: ",pred_x[8,:])
            print("min,max and mean ds values, ",np.min(pred_u[2,:model.N])," ",np.max(pred_u[2,:model.N])," ",np.mean(pred_u[2,:model.N]))
        # print("final parameters are: ", problem["all_parameters"],np.shape(problem["all_parameters"]))
        # Time to solve the NLP!
        # print("Im trying to follow are at: ",data_points[0, int(problem["xinit"][8])], data_points[1, int(problem["xinit"][8])])
        print("First param for MPC is : ",problem["all_parameters"][0],problem["all_parameters"][1])
        print("Second param for MPC is : ",problem["all_parameters"][4],problem["all_parameters"][5])
        # print("dr between me and params is: ", np.sqrt((problem["xinit"][0]-problem["all_parameters"][0][0])**2 + (problem["xinit"][1]-problem["all_parameters"][1][0])**2))
        output, exitflag, info = solver.solve(problem)
        print("info from kostas is: ",info)

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
        u[:,k] = pred_u[:,2]
        x[:,k+1] = np.transpose(model.eq(np.concatenate((u[:,k],x[:,k]))))
        ellipse_plot = getEllipseRatioWithState(x[:,k+1])
        ellipse_array.append(ellipse_plot)
        print("ellipse_plot is: ",ellipse_plot)
        if(ellipse_plot>1.0): 
            ellipse_counter+=1
            print("vgika apo ellipsi")
        print("exw vgei apo ellipsi: ",ellipse_counter," times")
        print("u_bef is: ",u[:,k]," ",np.shape(u), " ",np.shape(u[:,k]))
        print("x_bef is: ",x[:,k]," ",np.shape(x), " ", np.shape(x[:,k]))
        print("publishing torque and steering cmd: ",(x[6,k+1])/(5*3.9/0.85)," ",np.rad2deg(x[7,k+1]))
        print("i have used emergency manouvre: ",emergency_count," times")
        wheel_torques_txt.append(x[:,k][6]*0.2)
        steering_txt.append(x[:,k][7])
        # plot results of current simulation step
        if(k%100==0):
            file = open("Data/steering.txt", "w+")
            file.write(str(steering_txt))
            file.close()
            file2 = open("Data/torques.txt", "w+")
            file2.write(str(wheel_torques_txt))
            file2.close()
        if(k%20==0):
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
    
def main_solver(): model, solver = generate_pathplanner()

if __name__ == "__main__":
    cmd_arg = sys.argv[1]
    if(cmd_arg=="one_simulation"): 
        print("solver generator script called + simulation")
        main()
    if(cmd_arg=="no_simulation"): 
        print("solver generator script called + no simulation")
        main_solver()