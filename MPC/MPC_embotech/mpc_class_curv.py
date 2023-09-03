import casadi
import forcespro
import forcespro.nlp
import numpy as np
from scipy.interpolate import CubicSpline
from vd_functions import *
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from Trackdrive import cones_blue, cones_orange_big, cones_yellow
import sys

#set physical constants
#up to date
lol = 1.59 
wol = 1.0 #to be obtained...  
WD_front = 0.467 #percentage
l_f = lol*(1-WD_front)
l_r = lol*WD_front
CdA = 2.0 # for drag (changed)
ClA = 7.0 # for downforce (changed)
pair = 1.225
m = 190.0   # mass of the car
g = 9.81
Iz = 110.0
h_cog=0.27
eff=0.85
mi_fric = 0.03
ellipse_array=[]
dt_integration=0.025

#motion planning-tracking params
u_upper=12.0
ds_wanted=0.1
INDEX_MAX = 310.0

#track planning related params
# wing = 0.874
# start = -0.3-l_f-wing
# final = start+2*75
# length = final-start  
# factor_bef = 0.52

class State:
    def __init__(self, X, U):
        self.F = U[0]
        self.delta = U[1]
        self.X = X[0]
        self.Y = X[1]
        self.phi = X[2]
        self.vx = X[3]
        self.vy = X[4]
        self.r = X[5]
    def get_validation(self):
        print("dF, X and Y are: ",self.dF, " ", self.X, " ",self.Y)

class State_curv:
    def __init__(self, X, U):
        self.dF = U[0]
        self.dDelta = U[1]
        self.s = X[0]
        self.n = X[1]
        self.mu = X[2]
        self.vx = X[3]
        self.vy = X[4]
        self.r = X[5]
        self.F = X[6]
        self.delta = X[7]
    def get_validation(self):
        print("dF, X and Y are: ",self.dF, " ", self.X, " ",self.Y)

def getFy(Fz,sa):
    C_tire = 0.66
    ex=1e-7
    P=[1.45673747e+00, -1.94554660e-04,  2.68063018e+00,  3.19197941e+04, 2.58020549e+00, -7.13319396e-04]
    C=P[0]
    D=P[1]*Fz*Fz+P[2]*Fz
    BCD = P[3]*casadi.sin(P[4]*casadi.arctan(P[5]*Fz))
    B=BCD/(C*D)
    F_temp = C_tire*D*casadi.sin(C*casadi.arctan(B*sa))
    return F_temp

def getFySim(Fz,sa):
    C_tire = 0.66
    ex=1e-7
    P=[1.45673747e+00, -1.94554660e-04,  2.68063018e+00,  3.19197941e+04, 2.58020549e+00, -7.13319396e-04]
    C=P[0]
    D=P[1]*Fz*Fz+P[2]*Fz
    BCD = P[3]*np.sin(P[4]*np.arctan(P[5]*Fz))
    B=BCD/(C*D)
    F_temp = C_tire*D*np.sin(C*np.arctan(B*sa))
    return F_temp
 
def getSas(model_class):
    saf_temp=casadi.arctan((model_class.vy+l_f*model_class.r)/(model_class.vx+1e-3)) - model_class.delta
    sar_temp=casadi.arctan((model_class.vy-l_r*model_class.r)/(model_class.vx+1e-3))
    return saf_temp,sar_temp

def getSasSim(model_class):
    saf_temp=np.arctan((model_class.vy+l_f*model_class.r)/(model_class.vx+1e-3)) - model_class.delta
    sar_temp=np.arctan((model_class.vy-l_r*model_class.r)/(model_class.vx+1e-3))
    return saf_temp,sar_temp

def getFz(model_class):
    a_temp = (model_class.F - 0.5*CdA*pair*(model_class.vx)**2)/ m
    dw = (h_cog/lol)*(a_temp/g)*m*g
    Ffz_temp=(l_r/(l_f+l_r))*m*g + 0.25*pair*ClA*(model_class.vx**2) - dw
    Frz_temp=(l_f/(l_f+l_r))*m*g + 0.25*pair*ClA*(model_class.vx**2) + dw
    return Ffz_temp,Frz_temp

def getEllipseRatio(model_class):
    saf,sar = getSas(model_class) 
    Ffz,Frz = getFz(model_class)
    a,b = getEllipseParams(Frz)
    Fry = getFy(Frz,sar)
    per_= (model_class.F/(a*Frz))**2 + (Fry/(b*Frz))**2
    return per_

def continuous_dynamics(x, u, current_target):
    """Defines curvature-based models of the car, i.e. equality constraints.
    parameters:
    state x = [s, mi, ni, vx, vy, r, F, delta]
    input u = [dF,ddelta]
    """
    state_ = State_curv(x,u)
    # saf=casadi.arctan((x[4]+l_f*x[5])/np.sqrt(x[3]**2+1)) - x[7]
    # sar=casadi.arctan((x[4]-l_r*x[5])/np.sqrt(x[3]**2+1))
    saf,sar = getSas(state_) 
    Ffz,Frz = getFz(state_)
    Ffy = getFy(Ffz,saf)
    Fry = getFy(Frz,sar)

    #friction forces all
    Fdrag = 0.5*CdA*pair*(state_.vx)**2 + 0.03*(Frz+Ffz)
    print("curvature i get is -> ",current_target[3])
    s_dot = (state_.vx*casadi.cos(state_.mu) - state_.vy*casadi.sin(state_.mu))/(1-state_.n*current_target[3])
    n_dot = state_.vx*casadi.sin(state_.mu) + state_.vy*casadi.cos(state_.mu)
    mu_dot = state_.r - current_target[3]*s_dot
    vx_dot = (1/m)*(state_.F - Fdrag - Ffy*casadi.sin(state_.delta) + m*state_.vy*state_.r)
    vy_dot = (1/m)*(Fry + Ffy*casadi.cos(state_.delta) - m*state_.vx*state_.r)
    r_dot = (1/Iz)*(Ffy*l_f*casadi.cos(state_.delta) - Fry*l_r)
    delta_dot = state_.dDelta
    force_dot = state_.dF  
    dx = casadi.vertcat(s_dot,n_dot,mu_dot,vx_dot,vy_dot,r_dot,force_dot,delta_dot)
    return dx

def constr(z,current_target):
    """Least square costs on deviating from the path and on the inputs F and phi
    z = [dF,ddelta,s,ni,mi,vx,vy,r,F,delta]
    current_target = point on path that is to be headed for (includes x,y,phi and curvature from splines)
    """
    state_ = State_curv(z[2:],z[0:2])
    saf,sar = getSas(state_) 
    Ffz,Frz = getFz(state_)
    Fry = getFy(Frz,sar)
    a,b = getEllipseParams(Frz)
    constr1 = state_.n - (lol/2)*casadi.sin(casadi.fabs(state_.mu)) + (wol/2)*casadi.cos(state_.mu) #positive dev
    constr2 = -state_.n + (lol/2)*casadi.sin(casadi.fabs(state_.mu)) + (wol/2)*casadi.cos(state_.mu) #negative dev
    constr3 = (state_.F/(a*Frz))**2 + (Fry/(b*Frz))**2 #tyre grip
    return (constr1,constr2,constr3) 

def obj(z,current_target):
    """Least square costs on deviating from the path and on the inputs F and phi
    z = [dF,ddelta,s,ni,mi,vx,vy,r,F,delta]
    current_target = point on path that is to be headed for
    """
    state_ = State_curv(z[2:],z[0:2]) 
    saf,sar = getSas(state_) 
    Ffz,Frz = getFz(state_)
    a,b = getEllipseParams(Frz)
    per_ = getEllipseRatio(state_)
    Fry = getFy(Frz,sar)
    beta = casadi.arctan(l_r*state_.delta/(l_f + l_r))
    dsa =(sar-beta)
    s_dot = (state_.vx*casadi.cos(state_.mu) - state_.vy*casadi.sin(state_.mu))/(1-state_.n*current_target[3])
    return (1e4*(state_.n)**2 #cost on local vertical distance deviation
            + 1e4*(state_.mu)**2 # cost on local curv. angle deviation
            # penalty on input F,dF
            + 1e1*(state_.dF/1000)**2 
            + 1e1*(state_.F/1000)**2
            #penalty on delta,ddelta
            + 1e2*(state_.delta)**2 
            + 1e2*(state_.dDelta)**2
            #slip angle and ellipse related costs
            + 1e-3*(sar**2)
            + 1e-3*(dsa)
            + 1e-2*per_
            #performance costs
            - 1e-3*(state_.vx) 
            - 1e-3*(dt_integration*s_dot))

def generate_pathplanner():
    """
    Generates and returns a FORCESPRO solver that calculates a path based on
    constraints and dynamics while minimizing an objective function
    """
    # Model Definition
    # ----------------
    
    # Problem dimensions
    model = forcespro.nlp.SymbolicModel()
    model.N = 30  # horizon length
    model.nvar = 10  # number of variables
    model.neq = 8  # number of equality constraints
    model.npar = 4 # number of runtime parameters (X,Y,phi,curv)
    num_ins=model.nvar-model.neq
    
    # Objective function
    model.objective = obj
    model.objectiveN = obj # increased costs for the last stage
    model.xinitidx = range(num_ins,model.nvar) # use this to specify on which variables initial conditions
    # We use an explicit RK4 integrator here to discretize continuous dynamics
    integrator_stepsize = dt_integration
    kappa_temp = 1e-5
    # model.eq = lambda z, kappa: forcespro.nlp.integrate(continuous_dynamics, z[num_ins:model.nvar], z[0:num_ins], kappa,
    #                                             integrator=forcespro.nlp.integrators.RK4,
    #                                             stepsize=integrator_stepsize)
    
    # Inequality constraints
    # from brake -> Fbrake = -4120.0
    #z = [dF,ddelta,s,ni,mi,vx,vy,r,F,delta]
    model.lb = np.array([-3000.0,-np.deg2rad(30),0.0,       -10.0,-np.inf,-1e-6,-15.0,-15.0,-3000.0, -np.deg2rad(30)])
    model.ub = np.array([3000.0,+np.deg2rad(30),INDEX_MAX*10,10.0,+np.inf,20.0, +15.0, 15.0, 3000.0, +np.deg2rad(30)]) 

    model.nh = 3 #number of inequality constr
    model.ineq = constr
    ######
    model.eq = lambda z, current_target: forcespro.nlp.integrate(continuous_dynamics, z[num_ins:model.nvar], z[0:num_ins], current_target,
                                                integrator=forcespro.nlp.integrators.RK4,
                                                stepsize=integrator_stepsize)
    # model.continuous_dynamics = continuous_dynamics
    model.E = np.concatenate([np.zeros((model.neq,num_ins)), np.eye(model.neq)], axis=1)
    # model.hu = np.array([+np.inf,+1.0,+0.3])
    # model.hl = np.array([-np.inf,-np.inf,-0.3])
    # positive_dev, negative_dev, tyres
    # model.hu = np.array([+1.0,1.0,+1.0]) 
    model.hu = np.array([+np.inf,np.inf,np.inf])
    model.hl = np.array([-np.inf,-np.inf,-np.inf])
    #track - tyres - sar
    # model.hu = np.array([2.0,1.0,0.15])
    # model.hl = np.array([-np.inf,-np.inf,-0.15])

    # Initial condition on vehicle states x
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
    codeoptions.mip.mipgap = 0.5
    # codeoptions.nlp.integrator.Ts = dt_integration
    codeoptions.mip.explore = 'bestFirst'
    codeoptions.nlp.integrator.type = 'ERK4'
    codeoptions.mip.branchon = 'mostAmbiguous'
    codeoptions.parallel = 4
    codeoptions.nlp.hessian_approximation = 'bfgs'
    codeoptions.nlp.integrator.nodes = 1
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


def f(x_sim, u_mpc):
    state_ = State(x_sim,u_mpc)
    saf,sar = getSas(state_) 
    Ffz,Frz = getFz(state_)
    a,b = getEllipseParams(Frz)
    Fry = getFy(Frz,sar)
    Ffy = getFy(Ffz,saf)
    
    #friction forces all
    Fdrag = 0.5*CdA*pair*(state_.vx)**2 + 0.03*(Frz+Ffz)
    
    x_dot = state_.vx*np.cos(state_.phi) - state_.vy*np.sin(state_.phi)
    y_dot = state_.vx*np.sin(state_.phi) + state_.vy*np.cos(state_.phi)
    phi_dot = state_.r
    vx_dot = (1/m)*(state_.F - Fdrag - Ffy*np.sin(state_.delta) + m*state_.vy*state_.r)
    vy_dot = (1/m)*(Fry + Ffy*np.cos(state_.delta) - m*state_.vx*state_.r)
    r_dot = (1/Iz)*(Ffy*l_f*np.cos(state_.delta) - Fry*l_r) 
    output_array = np.array([x_dot,y_dot,phi_dot,vx_dot,vy_dot,r_dot])
    return output_array

def runge_kutta_upd(x_sim, u_mpc, ts):
    k1 = f(x_sim, u_mpc)
    k2 = f(x_sim+k1*(ts/2), u_mpc)
    k3 = f(x_sim+k2*(ts/2), u_mpc)
    k4 = f(x_sim+k3*ts, u_mpc)
    dx_ol = (k1 + k2*2 + k3*2 + k4)*(ts/6)
    x_next = x_sim + dx_ol
    return x_next

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

#splines related functions
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

def cubic_spline_inference(cs,parameter,x,u):
    state_ = State(x,u)
    x_temp=x.copy()
    parameter_array=[]
    _,Frz=getFz(state_)
    _,b=getEllipseParams(Frz)
    u_max=[]
    u_final=[]
    u_first = state_.vx #for curvature profile
    ux_init=state_.vx
    print("u and x arrays inside v.p. are: ",x, " ",u)
    print("State inside v.p. is: ",state_)
    print("ux_init is: ",state_.vx)
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
        # curv_array.append(curvature)
        u_first=np.sqrt((b*g/curvature))
        if(u_first>u_upper):u_first=u_upper
        u_max.append(u_first)
        if(i==0): 
            u_forward = u_first
            # u_forward = state_.vx
        elif(i>0 and i<=np.shape(parameter)[0]-1):
            state_.vx=u_output1
            Ffz,Frz = getFz(state_)
            a,b = getEllipseParams(Frz)
            Fy_remain = m*(state_.vx**2)*curvature #Fry with kentromolos
            saf,sar = getSas(state_) 
            # Fy_remain = getFy(Frz,sar) #actual Fry
            mi_fric_act = mi_fric
            if((m*g*mi_fric_act)**2-(Fy_remain)**2<0): Fx_remain=0
            else: Fx_remain = np.sqrt((mi_fric_act*m*g)**2-(Fy_remain)**2) - 0.5*CdA*pair*(u_output1)**2 - 0.03*(Frz+Ffz)
            print("Forces on forward pass are: ",Fx_remain,Fy_remain,m*g*mi_fric_act)
            ds_temp=ds_wanted
            # ds_temp=0.1
            u_forward=np.sqrt(state_.vx**2+2*(Fx_remain/m)*np.abs(ds_temp))
            print("uforward is: ",u_forward," ",i)
        u_output1=np.min(np.array([u_forward,u_first]))
        # if(u_output>u_upper): u_output=u_upper 
        parameter_array.append([interpolated_points[0]])
        parameter_array.append([interpolated_points[1]])
        parameter_array.append([local_angle])
        parameter_array.append([u_output1])
        u_first_array.append(u_output1)
    state_.vx=u_first_array[np.shape(parameter)[0]-1]
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
            state_.vx=u_output2
            saf,sar = getSas(state_) 
            Ffz,Frz = getFz(state_)
            a,b = getEllipseParams(Frz)
            Fy_remain = m*(state_.vx**2)*curvature #Fry with kentromolos
            # Fy_remain = getFy(Frz,sar) #actual Fry
            mi_fric_act = mi_fric
            if((m*g*mi_fric_act)**2-(Fy_remain)**2<0): Fx_remain=0
            else: Fx_remain = np.sqrt((mi_fric_act*m*g)**2-(Fy_remain)**2) - 0.5*CdA*pair*(u_output2)**2 - 0.03*(Frz+Ffz)
            ds_temp=ds_wanted
            # ds_temp=0.1
            if(state_.vx**2-2*(Fx_remain/m)*np.abs(ds_temp)<0): u_backward=u_output2
            else: u_backward=np.sqrt(np.abs(state_.vx**2-2*(Fx_remain/m)*np.abs(ds_temp)))
        u_output2=np.min(np.array([u_backward,u_first_array[np.shape(parameter)[0]-1-j]]))   
        u_final.append(u_output2) 
    u_final_final=np.flip(u_final)
    # u_final_final = u_first_array    
    print("ux_init is: ",ux_init)
    print("u_max is: ",u_max,np.shape(u_max))
    print("u_first is: ",u_first_array,np.shape(u_first_array))
    print("u_second is: ",np.flip(u_final),np.shape(u_final))
    print("u_final is: ",u_final_final,np.shape(u_final_final)) #komple mexri dw
    if(np.shape(parameter)[0]==2):
        return parameter_array
    else:
        for k in range(4*np.shape(parameter)[0]):
            if(k%4==3):
                parameter_array[k] = [u_final_final[int((k-3)/4)]]
    return parameter_array

#s-profile related functions
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

def generate_closest_s(reference_track, pos,emergency_bool):
    """Extract the next N points on the path for the next N stages starting from 
    the current car position pos
    """
    path_points=reference_track[:2,:]
    print("position inside closest func is -> ",pos)
    print("path points inside cs are -> ",path_points)
    if(emergency_bool):
        idx = find_closest_point(path_points,pos) + int((1/ds_wanted))
    else:
        idx = find_closest_point(path_points,pos) 
    print("final(=euclidean) closest point is: ",path_points[:,idx]," ",idx)
    # if(idx0<2*window):
    #     idx_ver = find_closest_point_vertical(reference_track[:,0:2*window],pos)
    #     idx = idx_ver 
    # else:
    #     idx_ver = find_closest_point_vertical(reference_track[:,idx0-window:idx0+window],pos)
    #     idx = idx0 + (idx_ver-window)
    return (idx/(np.shape(reference_track)[1]))*INDEX_MAX, idx


#plot related functions
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
    
    # state x = [xPos,yPos,phi, vx, vy, r, F, delta]
    # input u = [F,delta]

    # Plot vx
    ax_vel = fig.add_subplot(5,2,2)
    plt.grid("both")
    plt.title('Vx')
    plt.xlim([0., sim_length-1])
    plt.plot([0, sim_length-1], np.transpose([model.ub[5], model.ub[5]]), 'r:') #me z index
    plt.plot([0, sim_length-1], np.transpose([model.lb[5], model.lb[5]]), 'r:')
    ax_vel.plot(x[3,0], '-b')
    ax_vel.plot(start_pred[5,:], 'g-') #me z index
    
    # Plot vy
    ax_vy = fig.add_subplot(5,2,4)
    plt.grid("both")
    plt.title('Vy')
    plt.xlim([0., sim_length-1])
    plt.plot([0, sim_length-1], np.transpose([model.ub[6], model.ub[6]]), 'r:')
    plt.plot([0, sim_length-1], np.transpose([model.lb[6], model.lb[6]]), 'r:')
    ax_vy.plot(x[4,0], 'b-')
    ax_vy.plot(start_pred[6,:],'g-')
    
    # Plot r
    ax_r = fig.add_subplot(5,2,6)
    plt.grid("both")
    plt.title('r')
    plt.xlim([0., sim_length-1])
    plt.plot([0, sim_length-1], np.transpose([model.ub[7], model.ub[7]]), 'r:')
    plt.plot([0, sim_length-1], np.transpose([model.lb[7], model.lb[7]]), 'r:')
    ax_r.plot(x[5,0], 'b-')
    ax_r.plot(start_pred[7,:],'g-')

    # Plot acceleration force
    ax_F = fig.add_subplot(5,2,8)
    plt.grid("both")
    plt.title('Acceleration force')
    plt.xlim([0., sim_length-1])
    plt.plot([0, sim_length-1], np.transpose([model.ub[8], model.ub[8]]), 'r:')
    plt.plot([0, sim_length-1], np.transpose([model.lb[8], model.lb[8]]), 'r:')
    ax_F.plot(u[0,0], 'b-')
    ax_F.plot(start_pred[0,:],'g-')
    
    # Plot steering angle
    ax_delta = fig.add_subplot(5,2,10)
    plt.grid("both")
    plt.title('Steering angle')
    plt.xlim([0., sim_length-1])
    plt.plot([0, sim_length-1], np.rad2deg(np.transpose([model.ub[9], model.ub[9]])), 'r:')
    plt.plot([0, sim_length-1], np.rad2deg(np.transpose([model.lb[9], model.lb[9]])), 'r:')
    ax_delta.plot(np.rad2deg(u[1,0]),'b-')
    ax_delta.plot(np.rad2deg(start_pred[1,:]),'g-')

    plt.tight_layout()

    # Make plot fullscreen. Comment out if platform dependent errors occur.
    mng = plt.get_current_fig_manager()
    
def updatePlots(x,u,pred_x,pred_u,model,k):
    """
    Deletes old data sets in the current plot and adds the new data sets 
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
    
    ax_list[1].get_lines().pop(-1).remove() # remove old vx_pred
    ax_list[1].get_lines().pop(-1).remove() # remove old vx
    ax_list[2].get_lines().pop(-1).remove() # remove old vy_pred
    ax_list[2].get_lines().pop(-1).remove() # remove old vy
    ax_list[3].get_lines().pop(-1).remove() # remove old r_pred
    ax_list[3].get_lines().pop(-1).remove() # remove old r
    ax_list[4].get_lines().pop(-1).remove() # remove old F_accel_pred
    ax_list[4].get_lines().pop(-1).remove() # remove old F_accel
    ax_list[5].get_lines().pop(-1).remove() # remove old delta_pred
    ax_list[5].get_lines().pop(-1).remove() # remove old delta
    
    # state x = [xPos,yPos,phi, vx, vy, r, F, delta]
    # input u = [dF,ddelta]

    # Update plot with current simulation data
    ax_list[0].plot(x[0,0:k+2],x[1,0:k+2], '-b')             # plot new trajectory
    ax_list[0].plot(pred_x[0,1:], pred_x[1,1:], 'g-')   # plot new prediction of trajectory
   
    ax_list[1].plot(x[3,0:k+2],'b-')                         # plot new vx
    ax_list[1].plot(range(k+1,k+model.N), pred_x[3,1:],'g-') # plot new prediction of vx
    
    ax_list[2].plot(x[4,0:k+2],'b-')                         # plot new vy
    ax_list[2].plot(range(k+1,k+model.N), pred_x[4,1:],'g-') # plot new prediction of vy
    
    ax_list[3].plot(x[5,0:k+2],'b-')                         # plot new r
    ax_list[3].plot(range(k+1,k+model.N), pred_x[5,1:],'g-') # plot new prediction of r
    
    ax_list[3].step(u[0,0:k+2],'b-')                         # plot new F
    ax_list[3].step(range(k+1,k+model.N), pred_u[0,1:],'g-') # plot new prediction of F
    
    ax_list[4].step(u[1,0:k+2],'b-')                         # plot new delta
    ax_list[4].step(range(k+1,k+model.N), pred_u[1,1:],'g-') # plot new prediction of delta             
    plt.pause(0.05)