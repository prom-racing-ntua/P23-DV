# See also FORCES_NLP
# (c) Embotech AG, Zurich, Switzerland, 2013-2022.
import os
import sys
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

# set physical constants
#up to date
lol = 1.59 
WD_front = 0.467
l_f = lol*(1-WD_front)
l_r = lol*WD_front
CdA = 2.0 # for drag (changed)
ClA = 7.0 # for downforce (changed)
pair = 1.225
u_upper=12.0
m = 190.0   # mass of the car
g = 9.81
Iz = 110.0
ds_wanted=0.1
eff=0.85
window=10

#for dynamic model
# B=-7.0
# C=1.456
C_tire=0.66
# D= 3000.0
cs=-16419.31
dt_integration=0.025

#compute l_a
umin=4.0
umax=7.0
INDEX_MAX=310.0
DINDEX_MAX=310.0 #just for one lap (TBC)

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
    Ffz_temp=(l_r/(l_f+l_r))*m*g + 0.25*pair*ClA*(z[6]**2)
    Frz_temp=(l_f/(l_f+l_r))*m*g + 0.25*pair*ClA*(z[6]**2)
    return Ffz_temp,Frz_temp

def getFzWithState(x):
    Ffz_temp=(l_r/(l_f+l_r))*m*g + 0.25*pair*ClA*(x[3]**2)
    Frz_temp=(l_f/(l_f+l_r))*m*g + 0.25*pair*ClA*(x[3]**2)
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
        3e3*(z[3]-current_target[0])**2 # costs on deviating on the path in x-direction
            + 3e3*(z[4]-current_target[1])**2 # costs on deviating on the path in y-direction
            + 0e1*(e_c)**2 # costs on deviating on the
                                        # path in y-direction
            # + 1e3*(e_l)**2 # costs on deviating on the
                                #path in x-direction0.025
            + 1e-1*(z[5]-current_target[2])**2 #dphi gap
            + 1e-1*(z[6]-current_target[3])**2
            + 1e-5*z[0]**2 # penalty on input F,dF
            + 1e-5*z[9]**2
            + 5e2*z[1]**2 #penalty on delta,ddelta
            + 5e2*z[10]**2
            + 1e-3*(sar**2)
            + 1e-3*(dsa**2)
            + 5e3*((z[9]/(a*Frz))**2 + (Fry/(b*Frz))**2)
            # + 1e-1*((1/(z[6]**2 +1e-3))) #vx and index
            - 1e-4*(z[6])
            - 1e-4*(z[11]/INDEX_MAX)
            - 1e-4*(z[2]/INDEX_MAX))

def constr(z,current_target):
    """Least square costs on deviating from the path and on the inputs F and phi
    z = [dF,ddelta,xPos,yPos,phi, vx, vy, r, F, delta, index]
    current_target = point on path that is to be headed for
    """
    saf,sar = getSas(z) 
    Ffz,Frz = getFz(z)
    Fry = getFy(Frz,sar)
    a,b = getEllipseParams(Frz)
    constr1 = (z[2]-current_target[0])**2 + (z[3]-current_target[1])**2 #inside track constraint
    constr2 = (z[8]/(a*Frz))**2 + (Fry/(b*Frz))**2 #tyre constraints
    constr3 = saf
    return (constr1,constr2,constr3) 

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
    integrator_stepsize = dt_integration
    model.eq = lambda z: forcespro.nlp.integrate(continuous_dynamics, z[num_ins:model.nvar], z[0:num_ins],
                                                integrator=forcespro.nlp.integrators.RK4,
                                                stepsize=integrator_stepsize)
    
    # Indices on LHS of dynamical constraint - for efficiency reasons, make
    # sure the matrix E has structure [0 I] where I is the identity matrix.
    model.E = np.concatenate([np.zeros((model.neq,num_ins)), np.eye(model.neq)], axis=1)

    # Inequality constraints
    # from brake -> Fbrake = -4120.0
    model.lb = np.array([-3560.7*eff,  -np.deg2rad(30), 0.0, -400.,   -400.,  -np.inf,  0.0, -15.0, -15.0, -3560.7*eff, -np.deg2rad(30), 0])
    model.ub = np.array([+3560.7*eff,  np.deg2rad(+30), INDEX_MAX, 400.,   400.,   +np.inf, 15.0, +15.0, 15.0, 3560.7*eff, np.deg2rad(30), INDEX_MAX*10])

    model.nh = 3 #number of inequality constr
    model.ineq = constr
    model.hu = np.array([+np.inf,+1.0,+0.3])
    model.hl = np.array([-np.inf,-np.inf,-0.3])
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
    codeoptions.maxit = 10000    # Maximum number of iterations
    codeoptions.printlevel = 2  # Use printlevel = 2 to print progress (but 
    #                             not for timings)
    codeoptions.optlevel = 0    # 0 no optimization, 1 optimize for size, 
    #                             2 optimize for speed, 3 optimize for size & speed
    codeoptions.cleanup = False
    codeoptions.timing = 1
    codeoptions.mip.inttol = 0.1
    codeoptions.parallel = 4
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

def main():
    model, solver = generate_pathplanner()

if __name__ == "__main__":
    main()