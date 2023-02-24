# Example script for getting started with FORCESPRO NLP solver.

# This example solves an optimization problem for a car with the simple
# continuous-time, nonlinear dynamics (bicycle model):

#    dxPos/dt = v*cos(theta + beta)
#    dyPos/dt = v*sin(theta + beta)
#    dv/dt = F/m
#    dtheta/dt = v/l_r*sin(beta)
#    ddelta/dt = phi

#    with:
#    beta = arctan(l_r/(l_f + l_r)*tan(delta))

# where xPos,yPos are the position, v the velocity in heading angle theta 
# of the car, and delta is the steering angle relative to the heading 
# angle. The inputs are acceleration force F and steering rate phi. The 
# physical constants m, l_r and l_f denote the car's mass and the distance 
# from the car's center of gravity to the rear wheels and the front wheels.

# The car starts from standstill with a certain heading angle, and the
# optimization problem is to minimize the distance of the car's position 
# to a given set of points on a path with respect to time.

# Quadratic costs for the acceleration force and steering rate are added to
# the objective to avoid excessive maneouvers.

# There are bounds on all variables except theta.

# Variables are collected stage-wise into 

#     z = [F phi xPos yPos v theta delta].

# This example models the task as a MPC problem using the SQP method.

# See also FORCES_NLP

# (c) Embotech AG, Zurich, Switzerland, 2013-2022.


import sys
import numpy as np
import casadi
from casadi import *
import forcespro
import forcespro.nlp
import matplotlib.pyplot as plt
import matplotlib.patches
from matplotlib.gridspec import GridSpec

#imports from path planning
import numpy as np
from scipy.linalg import solve_banded
from scipy.sparse import diags
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from TriangulationNew import Triangulation
from track_plotter import getTrackdrive, getSkidpad, getAccel, getAutoX
from ParametricCubicSpline import getSpline, getSplineDerivatives


class ModelParams: #for P22
    #torque related params
    T_M_max=182.6
    Rdf=0.079
    Rdr=0.0735
    mi_disk=0.6
    A_cal_f=0.001963
    A_cal_r=0.000981
    PMC_max_f=7958000
    PMC_max_r=1932000
    gr=3.9
    #drag related params
    Cr0 = 0.03
    CdA = 1.8
    ClA = 5.47
    #general car params
    m = 190.0
    Iz  = 110.0
    lf = 0.9141
    lr = 0.7359
    Rf = 0.2
    Rr = 0.2
    #other
    g=9.81
    pair=1.225
    #for blending
    v_up=18.0
    v_low=3.0

class CostParams:
    scale = 2.0
    #for contouring cost
    wX = 100.0
    wY = 100.0
    wvel = 1.0
    #for input cost    
    wD = 1e-4
    wdelta=1e-3
    wvs=1e-5
    wdD = 1e-4
    wddelta = 50.0
    wdvs=1e-4
    #for slip angle
    wslip = 10.0

class StateToObject: #for xs
    def __init__(self,arr) -> None: 
        self.X=arr[0]
        self.Y=arr[1]
        self.phi=arr[2]
        self.vx=arr[3]
        self.vy=arr[4]
        self.r=arr[5]
        self.D=arr[6]
        self.delta=arr[7]
        self.s=arr[8]
        self.vs=arr[9]

class InputToObject: #for us
    def __init__(self,arr) -> None: 
        self.dD=arr[0]
        self.ddelta=arr[1]
        self.dvs=arr[2]

class ArrayToObject: #for zs
    def __init__(self,arr)-> None: 
        self.dD=arr[0]
        self.ddelta=arr[1]
        self.dvs=arr[2]
        self.X=arr[3]
        self.Y=arr[4]
        self.phi=arr[5]
        self.vx=arr[6]
        self.vy=arr[7]
        self.r=arr[8]
        self.D=arr[9]
        self.delta=arr[10]
        self.s=arr[11]
        self.vs=arr[12]

class SlipAngles:
    def __init__(self,arr)-> None:
        self.saf=arr[0]
        self.sar=arr[1]

class Torques:
    def __init__(self,arr)-> None:
        self.TG=arr[0]
        self.TBf=arr[1]
        self.TBr=arr[2]

class Forces:
    def __init__(self,arr) -> None:
        self.Ffx=arr[0]
        self.Ffy=arr[1]
        self.Ffz=arr[2]
        self.Frx=arr[3]
        self.Fry=arr[4]
        self.Frz=arr[5]
        self.Ffric=arr[6]

class ForceParamsY:
    def __init__(self,arr) -> None:
        self.B=arr[0]
        self.C=arr[1]
        self.C_tire=arr[2]
        self.D=arr[3]

class EllipseParams:
    def __init__(self,arr) -> None:
        self.a=arr[0]
        self.b=arr[1]

def getForceParamsY(Fz):
    C_tire=0.66
    P = [1.45673747e+00, -1.94554660e-04,  2.68063018e+00,  3.19197941e+04, 2.58020549e+00, -7.13319396e-04]
    ex=0.000001
    C=P[0]
    D=P[1]*np.power(Fz,2)+P[2]*Fz
    BCD=P[3]*np.sin(P[4]*np.arctan(P[5]*Fz))
    B=(BCD)/(C*D+ex)
    object_temp=ForceParamsY([-B,C,C_tire,D])
    return object_temp

def getEllipseParams(Fz):
    Fz0 = 1112.0554070627252
    dFz = (Fz-Fz0)/Fz0
    C_tire=0.66
    a = C_tire*(2.21891927-1.36151651e-07*dFz)
    b = C_tire*(2.46810824-0.21654031*dFz)
    object_temp=EllipseParams([a,b])
    return object_temp

def getTorques(x,param_=ModelParams()): #[tg,tbf,tbr]
    TG0=0.0
    TBf0=0.0
    TBr0=0.0
    TBr1=param_.A_cal_r*param_.PMC_max_r*param_.mi_disk*(-x.D)
    TBf1=param_.A_cal_f*param_.PMC_max_f*param_.mi_disk*(-x.D)
    TG1=param_.T_M_max*param_.gr*x.D
    TGout = casadi.if_else(x.D<0.0,TG0,TG1,True)
    TBfout = casadi.if_else(x.D<0.0,TBf1,TBf0,True)
    TBrout = casadi.if_else(x.D<0.0,TBr1,TBr0,True)
    # TGout=TG1
    # TBfout=0.0
    # TBrout=TBr1
    torques_ = Torques([TGout,TBfout,TBrout])
    print("torque is: ", TGout, np.shape(TGout),type(TGout))
    return torques_

def getSlipAngles(x,param_=ModelParams()):
    #anapoda prosima
    saf=+x.delta-casadi.arctan2(x.vy+x.r*param_.lf,x.vx)
    sar=-casadi.arctan2(x.vy-param_.lr*x.r,x.vx)
    sa=SlipAngles([saf,sar])
    return sa

def getForcesAll(x,param_=ModelParams()):
    Ffz=((param_.lr)/(param_.lf + param_.lr))*param_.m*param_.g + 0.25*param_.pair*param_.ClA*np.power(x.vx,2)
    Frz=((param_.lf)/(param_.lf + param_.lr))*param_.m*param_.g + 0.25*param_.pair*param_.ClA*np.power(x.vx,2)
    pyf=getForceParamsY(Ffz) #front & rear tyre parameters based on load & magic formula 
    pyr=getForceParamsY(Frz)
    T=getTorques(x)
    sa=getSlipAngles(x)
    #for front
    Ffx = (-T.TBf/param_.Rf)  - param_.Cr0*Ffz
    Ffy = pyf.C_tire*pyf.D*casadi.sin(pyf.C*casadi.arctan(pyf.B*sa.saf)) 
    #for rear
    Frx = (T.TG-T.TBr/param_.Rr) - param_.Cr0*Frz
    Fry = pyr.C_tire*pyr.D*casadi.sin(pyr.C*casadi.arctan(pyr.B*sa.sar)) 
    #for friction
    F_friction= -0.5*param_.CdA*param_.pair*np.power(x.vx,2)
    Forces_=Forces([Ffx,Ffy,Ffz,Frx,Fry,Frz,F_friction])
    return Forces_

def getLambda(x,param_=ModelParams()):
    temp1=(x.vx-param_.v_up)/(param_.v_up-param_.v_low)
    temp2=casadi.fmax(temp1,0.0)
    temp3=casadi.fmin(temp2,1.0)
    print("lambda is",temp3,type(temp3),np.shape(temp3))
    return temp3

def continuous_dynamics(x, u):
    #imported functions for car model
    print("types of inputs are",type(x),type(u))
    param_=ModelParams()
    xs=StateToObject(x)
    us=InputToObject(u)
    print("types of objects are",type(xs),type(us))
    forces_=getForcesAll(xs)
    lambda_ = getLambda(xs)    

    #dx/dt computation (use of blended model)
    dx_dt=xs.vx*casadi.cos(xs.phi) -xs.vy*casadi.sin(xs.phi)
    dy_dt=xs.vy*casadi.cos(xs.phi) + xs.vx*casadi.sin(xs.phi)
    dphi_dt= xs.r
    dvx_dt= (1.0/param_.m)* (lambda_*(forces_.Frx + forces_.Ffric - forces_.Ffy*casadi.sin(xs.delta) + forces_.Ffx*casadi.cos(xs.delta) + param_.m*xs.vy*xs.r) + (1-lambda_)*(forces_.Frx+ param_.m*xs.vy*xs.r))
    dvy_dt= (1.0/param_.m)*(lambda_*(forces_.Fry + forces_.Ffy*casadi.cos(xs.delta) + forces_.Ffx*casadi.sin(xs.delta) - param_.m*xs.vx*xs.r)) + (1-lambda_)*((param_.lf/(param_.lr+param_.lf))*(xs.delta*forces_.Frx + us.ddelta*xs.vx))
    dr_dt= (1.0/param_.Iz)*(lambda_*(forces_.Ffy*param_.lf*casadi.cos(xs.delta) - forces_.Fry*param_.lr + forces_.Ffx*param_.lf*casadi.sin(xs.delta))) + (1-lambda_)*((1.0/(param_.lr+param_.lf))*(xs.delta*forces_.Frx + us.ddelta*xs.vx))
    dD_dt= us.dD
    ddelta_dt= us.ddelta
    ds_dt = xs.vs
    dvs_dt= us.dvs
    return casadi.vertcat(dx_dt,dy_dt,dphi_dt,dvx_dt,dvy_dt,dr_dt,dD_dt,ddelta_dt,ds_dt,dvs_dt)

 #z = |dD    ddelta dvs | X       Y      phi          |  vx   vy    r |  D  delta| s vs 

def getReferenceCost(zs,current_target,param_=CostParams()): #to be extended/changes
    offset_X = np.power((zs.X-current_target[0]),2)
    offset_Y = np.power((zs.Y-current_target[1]),2)
    return param_.wX*offset_X +param_.wY*offset_Y

def getInputCost(zs,param_=CostParams()):
    cost_in= param_.wD*zs.D+param_.wdelta*zs.delta+param_.wvs*zs.vs
    cost_der=param_.wdD*zs.dD+param_.wddelta*zs.ddelta+param_.wdvs*zs.dvs
    return cost_in+cost_der
    
def getSlipCost(zs,params1=ModelParams(),params2=CostParams()):
    dynamic=casadi.arctan(zs.vx/zs.vy)
    kinematic=casadi.arctan(casadi.tan(zs.delta)*(params1.lr/(params1.lf+params1.lr)))
    return params2.wslip*np.power((dynamic-kinematic),2)

def getAccelCost(zs,param_=CostParams()):
    return -param_.wvel*np.power(zs.vs,2)

def obj(z,current_target):
    zs=ArrayToObject(z)
    J1=getReferenceCost(zs,current_target)
    J2=getInputCost(zs)
    J3=getSlipCost(zs)
    J4=getAccelCost(zs)
    Jol=(J1+J2+J3+J4)
    print("cost is",Jol,type(Jol))
    return Jol

def objN(z,current_target):
    zs=ArrayToObject(z)
    params_=CostParams()
    J1=getReferenceCost(zs,current_target)
    J2=getInputCost(zs)
    J3=getSlipCost(zs)
    J4=getAccelCost(zs)
    Jol=(J1+J2+J3+J4)
    print("scaled costs are:", params_.scale*Jol, type(params_.scale*Jol))
    return params_.scale*Jol

def ineq_constr(z):
    zs=ArrayToObject(z)
    forces_ = getForcesAll(zs) 
    sas_ = getSlipAngles(zs) 
    el_pars = getEllipseParams(forces_.Frz)
    #tire term
    constr1 = np.power((forces_.Frx/(el_pars.a*forces_.Frz)),2) + np.power((forces_.Fry/(el_pars.b*forces_.Frz)),2)
    #slip angle term
    constr2 = sas_.saf
    return casadi.vertcat(constr1,constr2)

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

def extract_next_path_points(path_points, pos, N):
    """Extract the next N points on the path for the next N stages starting from 
    the current car position pos
    """
    idx = find_closest_point(path_points,pos)
    num_points = path_points.shape[1]
    num_ellipses = np.ceil((idx+N+1)/num_points)
    path_points = np.tile(path_points,(1,int(num_ellipses)))
    return path_points[:,idx+1:idx+N+1]


def generate_pathplanner():
    """Generates and returns a FORCESPRO solver that calculates a path based on
    constraints and dynamics while minimizing an objective function
    """
    # Model Definition
    # ----------------
    # Problem dimensions
    model = forcespro.nlp.SymbolicModel() #pairnei to horizon length

    model.N = 25 # horizon length
    model.nvar = 13  # number of variables
    model.neq = 10  # number of equality constraints
    model.npar = 2 # number of runtime parameters #mallon ta N x,y points
    model.nh =  2 # number of inequality constraints

    # Objective function
    model.objective = obj
    model.objectiveN = objN # increased costs for the last stage
    # The function must be able to handle symbolic evaluation,
    # by passing in CasADi symbols. This means certain numpy funcions are not
    # available.

    # We use an explicit RK4 integrator here to discretize continuous dynamics
    # z[0:3] -> dD, ddelta, dvs
    # z[3:13] -> x, y, phi, vx, vy, r, D, delta, s, vs
    integrator_stepsize = 0.1
    model.eq = lambda z: forcespro.nlp.integrate(continuous_dynamics, z[3:13], z[0:3],
                                                integrator=forcespro.nlp.integrators.RK4,
                                                stepsize=integrator_stepsize)
    model.ineq = lambda z: ineq_constr(z)
    # Indices on LHS of dynamical constraint - for efficiency reasons, make
    # sure the matrix E has structure [0 I] where I is the identity matrix.
    model.E = np.concatenate([np.zeros((10,3)), np.eye(10)], axis=1) #state and input size respectively

    # Inequality constraints
    #  upper/lower variable bounds lb <= z <= ub                        
    #          z =         |dD    ddelta dvs | X       Y      phi          |  vx   vy    r |  D  delta| s vs 
    model.lb = np.array([-10., -10., -10., -100., -100., np.deg2rad(-90.), 0., -10., -5., -1, -0.5, -0. ,0. ])
    model.ub = np.array([10., 10., 10., 100., 100., np.deg2rad(+90.), 20., 10., 5., 1, 0.5, 2500., 20. ])
    # constraints       tire|slip angle 
    model.hl = np.array([0.0, -0.15]) 
    model.hu = np.array([1.0, 0.15])

    # Initial condition on vehicle states x
    model.xinitidx = range(3,13) # use this to specify on which variables initial conditions #xmm
    # are imposed

    # Solver generation
    # -----------------

    # Set solver options
    codeoptions = forcespro.CodeOptions('FORCESNLPsolver')
    codeoptions.maxit = 1000     # Maximum number of iterations
    codeoptions.printlevel = 2  # Use printlevel = 2 to print progress (but not for timings)
    codeoptions.optlevel = 0    # 0 no optimization, 1 optimize for size, 2 optimize for speed, 3 optimize for size & speed
    codeoptions.cleanup = False
    codeoptions.timing = 1
    codeoptions.nlp.hessian_approximation = 'bfgs'
    codeoptions.solvemethod = 'SQP_NLP' # choose the solver method Sequential Quadratic Programming
    codeoptions.nlp.bfgs_init = 2.5*np.identity(13)
    codeoptions.sqp_nlp.maxqps = 1      # maximum number of quadratic problems to be solved
    codeoptions.sqp_nlp.reg_hessian = 5e-9 # increase this if exitflag=-8
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
        #z = |dD    ddelta dvs | X       Y      phi          |  vx   vy    r |  D  delta| s vs 
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

    # Update plot with current simulation data
    ax_list[0].plot(x[0,0:k+2],x[1,0:k+2], '-b')             # plot new trajectory
    ax_list[0].plot(pred_x[0,1:], pred_x[1,1:], 'g-')        # plot new prediction of trajectory
    ax_list[1].plot(x[3,0:k+2],'b-')                         # plot new velocity
    ax_list[1].plot(range(k+1,k+model.N), pred_x[3,1:],'g-') # plot new prediction of velocity
    ax_list[2].plot(np.rad2deg(x[2, 0:k+2]),'b-')            # plot new heading angle
    ax_list[2].plot(range(k+1,k+model.N), \
        np.rad2deg(pred_x[2,1:]),'g-')                       # plot new prediciton of heading angle
    ax_list[3].plot(np.rad2deg(x[7, 0:k+2]),'b-')            # plot new steering angle
    ax_list[3].plot(range(k+1,k+model.N), \
        np.rad2deg(pred_x[7,1:]),'g-')                       # plot new prediction of steering angle
    ax_list[4].step(range(0, k+1), x[6, 0:k+1],'b-')         # plot new input
    ax_list[4].step(range(k, k+model.N), pred_x[6,:],'g-')        # plot new prediction of input
    ax_list[5].step(range(0, k+1), \
        np.rad2deg(u[1, 0:k+1]),'b-')                        # plot new steering rate
    ax_list[5].step(range(k, k+model.N), \
        np.rad2deg(pred_u[1,:]),'g-')                        # plot new prediction of steering rate
    plt.pause(0.05)


def createPlot(x,u,start_pred,sim_length,model,path_points,xinit):
    """Creates a plot and adds the initial data provided by the arguments"""

     # Create empty plot
    fig = plt.figure()
    plt.clf()
    gs = GridSpec(5,2,figure=fig)

    #z = |dD    ddelta dvs | X       Y      phi          |  vx   vy    r |  D  delta| s vs 
    # Plot trajectory
    ax_pos = fig.add_subplot(gs[:,0])
    l0, = ax_pos.plot(np.transpose(path_points[0,:]), np.transpose(path_points[1,:]), 'rx')
    l1, = ax_pos.plot(xinit[0], xinit[1], 'bx')
    plt.title('Position')
    #plt.axis('equal')
    plt.xlim([-100.,100])
    plt.ylim([-100, 100])
    plt.xlabel('x-coordinate')
    plt.ylabel('y-coordinate')
    l2, = ax_pos.plot(x[0,0],x[1,0],'b-')
    l3, = ax_pos.plot(start_pred[3,:], start_pred[4,:],'g-')
    ax_pos.legend([l0,l1,l2,l3],['desired trajectory','init pos','car trajectory',\
        'predicted car traj.'],loc='lower right')
    
    # Plot velocity
    ax_vel = fig.add_subplot(5,2,2)
    plt.grid("both")
    plt.title('Velocity')
    plt.xlim([0., sim_length-1])
    plt.ylim([-10, 10]) 
    plt.plot([0, sim_length-1], np.transpose([model.ub[4], model.ub[4]]), 'r:')
    plt.plot([0, sim_length-1], np.transpose([model.lb[4], model.lb[4]]), 'r:')
    ax_vel.plot(0.,x[3,0], '-b')
    ax_vel.plot(start_pred[6,:], 'g-')
    
    # Plot heading angle
    ax_theta = fig.add_subplot(5,2,4)
    plt.grid("both")
    plt.title('Heading angle')
    plt.ylim([-2, 2]) 
    plt.xlim([0., sim_length-1])
    plt.plot([0, sim_length-1], np.rad2deg(np.transpose([model.ub[5], model.ub[5]])), 'r:')
    plt.plot([0, sim_length-1], np.rad2deg(np.transpose([model.lb[5], model.lb[5]])), 'r:')
    ax_theta.plot(np.rad2deg(x[2,0]), 'b-')
    ax_theta.plot(np.rad2deg(start_pred[5,:]), 'g-')

    # Plot delta
    ax_delta = fig.add_subplot(5,2,6)
    plt.grid("both")
    plt.title('Steering angle')
    plt.xlim([0., sim_length-1])
    plt.ylim([-2, 2]) 
    plt.plot([0, sim_length-1], np.rad2deg(np.transpose([model.ub[6], model.ub[6]])), 'r:')
    plt.plot([0, sim_length-1], np.rad2deg(np.transpose([model.lb[6], model.lb[6]])), 'r:')
    ax_delta.plot(np.rad2deg(x[7,0]),'b-')
    ax_delta.plot(np.rad2deg(start_pred[10,:]),'g-')

    # Plot D 
    ax_F = fig.add_subplot(5,2,8)
    plt.grid("both")
    plt.title('Gas/Brake Input')
    plt.xlim([0., sim_length-1])
    plt.ylim([-2, 2]) 
    plt.plot([0, sim_length-1], np.transpose([model.ub[0], model.ub[0]]), 'r:')
    plt.plot([0, sim_length-1], np.transpose([model.lb[0], model.lb[0]]), 'r:')
    ax_F.step(0, x[6,0], 'b-')
    ax_F.step(range(model.N), start_pred[9,:],'g-')

    # Plot steering rate
    ax_phi = fig.add_subplot(5,2,10)
    plt.grid("both")
    plt.title('Steering rate')
    plt.xlim([0., sim_length-1])
    plt.ylim([-2, 2]) 
    plt.plot([0, sim_length-1], np.rad2deg(np.transpose([model.ub[1], model.ub[1]])), 'r:')
    plt.plot([0, sim_length-1], np.rad2deg(np.transpose([model.lb[1], model.lb[1]])), 'r:')
    ax_phi.step(0., np.rad2deg(u[1,0]), 'b-')
    ax_phi.step(range(model.N), start_pred[1,:],'g-')
    plt.tight_layout()

    # Make plot fullscreen. Comment out if platform dependent errors occur.
    mng = plt.get_current_fig_manager()


def main():
    #import data from path planning
    fig,ax = plt.subplots()
    fig.set_size_inches(13,9)
    track = getTrackdrive(ax, return_points=True)
    tri = Triangulation(track)
    points = np.vstack((tri.midpoints, tri.midpoints[0,:]))
    points_mpc = np.vstack((tri.midpoints, tri.midpoints[0,:])).T
    spline = getSpline(points, axis=ax, boundary_condition='closed_loop')
    spline_mpc = getSpline(points, axis=ax, boundary_condition='closed_loop').T
    # generate code for estimator
    model, solver = generate_pathplanner()
    sim_length = 800 # simulate 80sec

    # Variables for storing simulation data
    x = np.zeros((10,sim_length+1)) # states
    u = np.zeros((3,sim_length)) # inputs

    # Set initial guess to start solver from
    x0i = np.zeros((model.nvar,1))
    x0 = np.transpose(np.tile(x0i, (1, model.N)))
    # Set initial condition
    #                     inputs                 |  states
    #                     F          phi            x    y     v    theta         delta
    #z = |dD    ddelta dvs | X       Y      phi          |  vx   vy    r |  D  delta| s vs 
    phi0=casadi.arctan((points_mpc[1][1]-points_mpc[1][0])/(points_mpc[0][1]-points_mpc[0][0]) ) #initial heading of car
    print("phi0 is: ",phi0)
    xinit = np.transpose(np.array([points_mpc[0][0], points_mpc[1][0], phi0, 0.1, 0., 0., 0.1, 0. ,0.,0.1]))
    x[:,0] = xinit
    problem = {"x0": x0,
            "xinit": xinit}

    # Create 2D points on ellipse which the car is supposed to follow
    num_points = 80
    path_points = calc_points_on_ellipse(num_points)

    start_pred = np.reshape(problem["x0"],(13,model.N)) # first prdicition corresponds to initial guess

    # generate plot with initial values
    createPlot(x,u,start_pred,sim_length,model,points_mpc,xinit)
   
    # Simulation
    for k in range(sim_length):
        # Set initial condition
        problem["xinit"] = x[:,k]

        # Set runtime parameters (here, the next N points on the path)
        # print("initial path points from ellipse are: ",path_points,np.shape(path_points),type(path_points))
        # print("car's position is: ", x[0:2,k])
        next_path_points = extract_next_path_points(points_mpc, x[0:2,k], model.N)
        print("next N path points are: ",next_path_points,np.shape(next_path_points),type(next_path_points))
        problem["all_parameters"] = np.reshape(np.transpose(next_path_points), \
            (2*model.N,1))
        print("final next N Path points are:", problem["all_parameters"], np.shape(problem["all_parameters"]),type(problem["all_parameters"]))

        # Time to solve the NLP!
        output, exitflag, info = solver.solve(problem)
        print(output)

        # Make sure the solver has exited properly.
        assert exitflag == 1, "bad exitflag"
        sys.stderr.write("FORCESPRO took {} iterations and {} seconds to solve the problem.\n"\
            .format(info.it, info.solvetime))

        # Extract output
        temp = np.zeros((np.max(model.nvar), model.N))
        for i in range(0, model.N):
            temp[:, i] = output['x{0:02d}'.format(i+1)]
        pred_u = temp[0:3, :]
        pred_x = temp[3:13, :]
        print("predicted u is",pred_u)
        print("predicted x is",pred_x)

        # Apply optimized input u of first stage to system and save simulation data
        u[:,k] = pred_u[:,0]
        x[:,k+1] = np.transpose(model.eq(np.concatenate((u[:,k],x[:,k]))))

        # plot results of current simulation step
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


if __name__ == "__main__":
    main()