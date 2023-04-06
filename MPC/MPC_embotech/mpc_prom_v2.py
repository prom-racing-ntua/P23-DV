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
import casadi

#imports from path planning
import numpy as np
from scipy.linalg import solve_banded
from scipy.sparse import diags
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from Trackdrive import cones_blue, cones_orange_big, cones_yellow

#global params

# set physical constants
l_f = 0.9141
l_r = 0.7359
CdA = 1.8 # for drag
ClA = 5.47    # for downforce
pair = 1.225
m = 190.0   # mass of the car
g = 9.81
Iz = 110.0

#for dynamic model
B=-8.266
C=1.456
C_tire=0.66
D= 1739.47
cs=-16419.31

#compute l_a
umin=4.0
umax=7.0

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


def continuous_dynamics(x, u):
    """Defines dynamics of the car, i.e. equality constraints.
    parameters:
    state x = [xPos,yPos,phi, vx, vy, r, F, delta, index]
    input u = [dF,ddelta, dindex]
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

    # dynamic dxs/dt
    # xdot=x[3]*casadi.cos(x[2]) - x[4]*casadi.sin(x[2])
    # ydot=x[3]*casadi.sin(x[2]) + x[4]*casadi.cos(x[2])
    # phidot= x[5] 
    # vxdot=(x[6] - Fdrag + Ffy*casadi.sin(x[7]) + m*x[4]*x[5])/ (0.5*m)
    # vydot= (-x[3]*x[5]) + (Fry + Ffy*casadi.cos(x[7]))/m           #FR,y +FF,y cos δ−mvxr
    # rdot=  (Ffy*l_f - Fry*l_r)/(0.5*Iz)           #FF,y lFcos δ−FR,y lR
    # Fdot= u[0]
    # deltadot = u[1]

    # kinematic dxs/dt
    # beta = casadi.arctan(l_r/(l_f + l_r) * casadi.tan(x[7]))
    # xdot = x[3]*casadi.cos(x[2] + beta) 
    # ydot = x[3]*casadi.sin(x[2] + beta) 
    # phidot= x[5] 
    # vxdot=(x[6] - Fdrag)/ m 
    # vydot= (l_r/(l_r+l_f))*(vxdot*casadi.tan(x[7])+x[3]*(u[1]/(casadi.cos(x[7]))**2))
    # rdot= (1/(l_r+l_f))*(vxdot*casadi.tan(x[7])+x[3]*(u[1]/(casadi.cos(x[7]))**2))
    # Fdot= u[0]
    # deltadot = u[1]

    #blending with changing lambda
    beta = casadi.arctan(l_r/(l_f + l_r) * casadi.tan(x[7]))
    xdot = (l_a)*(x[3]*casadi.cos(x[2]) - x[4]*casadi.sin(x[2])) + (1-l_a)*(x[3]*casadi.cos(x[2] + beta))
    ydot = (l_a)*(x[3]*casadi.sin(x[2]) + x[4]*casadi.cos(x[2])) + (1-l_a)*(x[3]*casadi.sin(x[2] + beta))
    phidot = x[5]
    vxdot = (1-l_a)*((x[6] - Fdrag)/ m) + (l_a)*((x[6] - Fdrag + Ffy*casadi.sin(x[7]) + m*x[4]*x[5])/ m)
    vydot = (1-l_a)*((l_r/(l_r+l_f))*(vxdot*casadi.tan(x[7])+x[3]*(u[1]/(casadi.cos(x[7]))**2))) + (l_a)*(((-x[3]*x[5]) + (Fry + Ffy*casadi.cos(x[7])))/(1.0*m))
    rdot = (1-l_a)*((1/(l_r+l_f))*(vxdot*casadi.tan(x[7])+x[3]*(u[1]/(casadi.cos(x[7]))**2))) + (l_a)*((Ffy*l_f*casadi.cos(x[7]) - Fry*l_r)/(1.0*Iz))
    Fdot= u[0]
    deltadot = u[1]
    dindexdot = u[2]

    return casadi.vertcat(xdot,ydot,phidot,vxdot,vydot,rdot,Fdot,deltadot,dindexdot)

INDEX_MAX=2000
DINDEX_MAX=40
err_array=[]
def obj(z,current_target):
    """Least square costs on deviating from the path and on the inputs F and phi
    z = [dF,ddelta,dindex,xPos,yPos,phi, vx, vy, r, F, delta,index]
    current_target = point on path that is to be headed for
    """
    dyn_sa  = casadi.arctan(z[7]/(z[6]+1e-3))
    beta = casadi.arctan(l_r/(l_f + l_r) * casadi.tan(z[10]))
    dsa =(dyn_sa-beta)
    e_c= casadi.sin(current_target[2])*(z[3]-current_target[0]) - casadi.cos(current_target[3])*((z[4]-current_target[1])) #katakorifi
    e_l= -casadi.cos(current_target[2])*(z[3]-current_target[0]) - casadi.sin(current_target[3])*((z[4]-current_target[1])) #orizontia
    
    return (
        2e3*(z[3]-current_target[0])**2 # costs on deviating on the
#                                              path in x-direction
            + 2e3*(z[4]-current_target[1])**2 # costs on deviating on the
#                                               path in y-direction
            +1e1*(e_c)**2 # costs on deviating on the
                                        #path in y-direction
            + 1e1*(e_l)**2 # costs on deviating on the
                                    #path in x-direction
            + 1e-5*z[0]**2 # penalty on input F, 
            + 1e-3*z[9]**2
            + 1e-5*z[1]**2 #penalty on delta,ddelta
            + 1e-3*z[10]**2
            + 1e-5*(z[5]-current_target[2])**2 #dphi gap
            + 1e-5*(dsa**2)
            + 2e1*((1/(z[6]**2 +1e-3)))
            - 2e1*(z[6])
            - 2e1*(z[11]/INDEX_MAX)
            - 2e1*(z[2]/DINDEX_MAX))
#     return (1e4*(e_c)**2 # costs on deviating on the
# #                                              path in x-direction
#             + 1e1*(e_l)**2 # costs on deviating on the
# #                                               path in y-direction
#             + 1e-2*z[0]**2 # penalty on input F, 
#             + 1e-2*z[8]**2
#             + 1e-2*z[1]**2 
#             + 1e-2*z[9]**2
#             + 1e2*(z[4]-current_target[2])**2 #dphi gap
#             + 1e-2*(dsa**2)
#             + 1e-1*((1/(z[5]**2 +1e-3)))
#             - 1e-1*(z[5]**2)) 


def constr(z,current_target):
    """Least square costs on deviating from the path and on the inputs F and phi
    z = [dF,ddelta,xPos,yPos,phi, vx, vy, r, F, delta, index]
    current_target = point on path that is to be headed for
    """
    saf=casadi.arctan((z[7]+l_f*z[8])/(z[6]+1e-3)) - z[10]
    sar=casadi.arctan((z[7]-l_r*z[8])/(z[6]+1e-3))
    Ffy = C_tire*D*casadi.sin(C*casadi.arctan(B*saf))
    Fry = C_tire*D*casadi.sin(C*casadi.arctan(B*sar))
    Frz = (l_f/(l_f+l_r))*m*g + 0.25*pair*ClA*(z[6]**2)
    Ffz = (l_r/(l_r+l_f))*m*g + 0.25*pair*ClA*(z[6]**2)
    constr1 = (z[3]-current_target[0])**2 + (z[4]-current_target[1])**2 #inside track constraint
    constr2 = (z[9]/(a*Frz))**2 + (z[9]/(b*Frz))**2 #tyre constraints
    constr3 = saf
    return (constr1,constr2,constr3) 

def calc_points_on_ellipse(num_points):
    """Desired trajectory on ellipoid represented by 2D points"""
    dT = 2 * np.pi / num_points
    t = np.arange(dT,(num_points+1)*dT,dT)
    path_points = np.array([0.5*np.cos(t),
                    2.0*np.sin(t)])
    return path_points

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

    
def extract_next_path_points(data_points, pos, N):
    """Extract the next N points on the path for the next N stages starting from 
    the current car position pos
    """
    path_points=data_points[:2,:] #keep only X,Y coords
    idx = find_closest_point(path_points,pos)
    num_points = path_points.shape[1]
    num_ellipses = np.ceil((idx+N+1)/num_points)
    path_points = np.tile(path_points,(1,int(num_ellipses)))
    return data_points[:,idx+1:idx+N+1]

def extract_next_path_points_new(data_points, pos, N,idx):
    """Extract the next N points on the path for the next N stages starting from 
    the current car position pos
    """
    path_points=data_points[:2,:] #keep only X,Y coords
    num_points = path_points.shape[1]
    num_ellipses = np.ceil((idx+N+1)/num_points)
    path_points = np.tile(path_points,(1,int(num_ellipses)))
    return data_points[:,idx+1:idx+N+1]


def generate_pathplanner():
    """Generates and returns a FORCESPRO solver that calculates a path based on
    constraints and dynamics while minimizing an objective function
    """
    # Model Definition
    # ----------------
    
    # Problem dimensions
    model = forcespro.nlp.SymbolicModel()
    model.N = 40  # horizon length
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
    integrator_stepsize = 0.05
    model.eq = lambda z: forcespro.nlp.integrate(continuous_dynamics, z[num_ins:model.nvar], z[0:num_ins],
                                                integrator=forcespro.nlp.integrators.RK4,
                                                stepsize=integrator_stepsize)

    # Indices on LHS of dynamical constraint - for efficiency reasons, make
    # sure the matrix E has structure [0 I] where I is the identity matrix.
    model.E = np.concatenate([np.zeros((model.neq,num_ins)), np.eye(model.neq)], axis=1)

    # Inequality constraints
    # from brake -> -Fbrake = -4120.0
    model.lb = np.array([-4120.0,  np.deg2rad(-90.), 1.0, -400.,   -400.,  -np.inf,  0.0, -20.0, -20.0, -4120., -0.5*np.pi, 0])
    model.ub = np.array([+3560.7,  np.deg2rad(+90.), model.N, 400.,   400.,   +np.inf,  20.0, +20.0, 20.0, 3560.7, 0.5*np.pi, INDEX_MAX])

    model.nh = 3 #number of inequality constr
    model.ineq = constr
    # model.hu = np.array([+35.0,1.0,0.3])
    # model.hl = np.array([-np.inf,-np.inf,-0.3])
    model.hu = np.array([+np.inf,+np.inf,np.inf])
    model.hl = np.array([-np.inf,-np.inf,-np.inf])

    # Initial condition on vehicle states x
    model.xinitidx = range(num_ins,model.nvar) # use this to specify on which variables initial conditions
    # are imposed

    # Solver generation
    # -----------------
    # Set solver options
    codeoptions = forcespro.CodeOptions('FORCESNLPsolver')
    codeoptions.maxit = 20000    # Maximum number of iterations
    codeoptions.printlevel = 2  # Use printlevel = 2 to print progress (but 
    #                             not for timings)
    codeoptions.optlevel = 0    # 0 no optimization, 1 optimize for size, 
    #                             2 optimize for speed, 3 optimize for size & speed
    codeoptions.cleanup = False
    codeoptions.timing = 1
    codeoptions.nlp.hessian_approximation = 'bfgs'
    codeoptions.solvemethod = 'SQP_NLP' # choose the solver method Sequential #Quadratic Programming
    codeoptions.nlp.bfgs_init = 2.5*np.identity(model.nvar)
    codeoptions.sqp_nlp.maxqps = 1      # maximum number of quadratic problems to be solved
    codeoptions.sqp_nlp.reg_hessian = 5e-5 # increase this if exitflag=-8
    # change this to your server or leave uncommented for using the 
    # standard embotech server at https://forces.embotech.com 
    # codeoptions.server = 'https://forces.embotech.com'
    
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
    
    ax_list[4].plot(np.rad2deg(x[7, 0:k+2]),'b-')            # plot new steering angle
    ax_list[4].plot(range(k+1,k+model.N), \
        np.rad2deg(pred_x[7,1:]),'g-')                       # plot new prediction of steering angle
    
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


def main():
    #import data from path planning
    fig,ax = plt.subplots()
    fig.set_size_inches(13,9)
    #from C++ path planning
    data_points=Dataloader("P23-DV/MPC/MPC_embotech/cubic_spline_data.txt")
    
    print("c++ data are: ",data_points, np.shape(data_points))
    print()
    # generate code for estimator
    model, solver = generate_pathplanner()
    num_ins = model.nvar-model.neq
    
    # Simulation
    sim_length = 1000 # simulate 8sec

    # Variables for storing simulation data
    x = np.zeros((model.neq,sim_length+1))  # states
    u = np.zeros((num_ins,sim_length))  # inputs

    # Set initial guess to start solver from
    x0i = np.zeros((model.nvar,1))
    x0 = np.transpose(np.tile(x0i, (1, model.N)))
    # Set initial condition
    #  x    y     theta    vx   vy  r F delta
    vx0 = 0.1
    Frx0 = 100
    xinit = np.transpose(np.array([data_points[0][0], data_points[1][0],data_points[2][0], vx0, 0.0, 0.0, Frx0, 0.,0.]))
    print("xinit is: ",xinit)
    x[:,0] = xinit

    problem = {"x0": x0,
            "xinit": xinit}
    # Create 2D points on ellipse which the car is supposed to follow
    start_pred = np.reshape(problem["x0"],(model.nvar,model.N)) # first prdicition corresponds to initial guess

    # generate plot with initial values
    createPlot(x,u,start_pred,sim_length,model,data_points[:2,:],xinit,cones_yellow, cones_blue, cones_orange_big)
    wheel_torques_txt=[]
    steering_txt=[]
    err_temp=0.0

    # Simulation
    for k in range(sim_length):
        print("Im at iteration: ",k+1)
        # Set initial condition
        problem["xinit"] = x[:,k]
        print("x_init is:",problem["xinit"],problem["xinit"][8])
        if(k>0):
            err_temp = np.sqrt((problem["xinit"][0] - problem["all_parameters"][0][0])**2 + (problem["xinit"][1] - problem["all_parameters"][1][0])**2)
            err_array.append(np.sqrt((problem["xinit"][0] - problem["all_parameters"][0][0])**2 + (problem["xinit"][1] - problem["all_parameters"][1][0])**2))
            print("error array is:",err_array," ",np.max(err_array)," ",np.mean(err_array))
        # Set runtime parameters (here, the next N points on the path)
        if(err_temp<=100):
            next_data_points = extract_next_path_points_new(data_points, x[0:num_ins-1,k], model.N, int(problem["xinit"][8]))
        else:
            next_data_points = extract_next_path_points(data_points, x[0:num_ins-1,k], model.N)
        # print("next N path points are: ",next_data_points,np.shape(next_data_points),type(next_data_points))
        problem["all_parameters"] = np.reshape(np.transpose(next_data_points), \
            (model.npar*model.N,1))
        # print("all_parameters are: ",problem["all_parameters"], np.shape(problem["all_parameters"]),type(problem["all_parameters"]))

        # Time to solve the NLP!
        output, exitflag, info = solver.solve(problem)

        # Make sure the solver has exited properly.
        # print(exitflag)
        assert exitflag == 1, "bad exitflag"
        sys.stderr.write("FORCESPRO took {} iterations and {} seconds to solve the problem.\n"\
            .format(info.it, info.solvetime))

        # Extract output
        temp = np.zeros((np.max(model.nvar), model.N))
        for i in range(0, model.N):
            temp[:, i] = output['x{0:02d}'.format(i+1)]
        pred_u = temp[0:num_ins, :]
        pred_x = temp[num_ins:model.nvar, :]

        # Apply optimized input u of first stage to system and save simulation data
        # u[:,k] = pred_u[:,0]
        u[:,k] = pred_u[:,1]
        x[:,k+1] = np.transpose(model.eq(np.concatenate((u[:,k],x[:,k]))))
        print("u_bef is: ",u[:,k]," ",np.shape(u), " ",np.shape(u[:,k]))
        print("x_bef is: ",x[:,k]," ",np.shape(x), " ", np.shape(x[:,k]))
        wheel_torques_txt.append(x[:,k][6]*0.2)
        steering_txt.append(x[:,k][7])
        # plot results of current simulation step
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
    print("error array is: ",err_array, " ", np.max(err_array)," ",np.mean(err_array))
    print()

if __name__ == "__main__":
    main()