#A script to produce the solver
import casadi
import numpy as np
import forcespro
import forcespro.nlp

#All the patameters i need
Fz0 = 1112.0554070627252
C_tire = 0.66
h_cog = 0
lol = 1.59
WD_front = 0.467
lr = lol*(1-WD_front)
lf = lol*WD_front
pair = 1.225
ClA = 7.0
m = 190.0
g = 9.81
Iz = 110.0
dt_integration = 0.025
CdA = 2.0 
eff=0.85
umin = 3.0
umax = 7.0
mi_fric = 0.03
mission = str(input("Enter your mission (skidpad, autocross, acceleration): ")) 

def getSasWithState(x):
    saf_temp=casadi.arctan((x[4]+lf*x[5])/(x[3]+1e-3)) - x[7]
    sar_temp=casadi.arctan((x[4]-lr*x[5])/(x[3]+1e-3))
    return saf_temp,sar_temp

def getFzWithState(x):
    a_temp = (x[6] - 0.5*CdA*pair*(x[3])**2)/m
    dw = (h_cog/lol)*(a_temp/g)*m*g
    Ffz_temp=(lr/(lf+lr))*m*g + 0.25*pair*ClA*(x[3]**2) - dw
    Frz_temp=(lf/(lf+lr))*m*g + 0.25*pair*ClA*(x[3]**2) + dw
    return Ffz_temp,Frz_temp

def getEllipseParams(Fz):
    Fz0=1112.0554070627252
    dfz=(Fz-Fz0)/Fz0
    C_tire=0.66
    mx_max=C_tire*(2.21891927-1.36151651e-07*dfz)
    my_max=C_tire*(2.46810824-0.21654031*dfz)
    return mx_max,my_max

def getFy(Fz,sa):
    P=[1.45673747e+00, -1.94554660e-04,  2.68063018e+00,  3.19197941e+04, 2.58020549e+00, -7.13319396e-04]
    C=P[0]
    D=P[1]*Fz*Fz+P[2]*Fz
    BCD = P[3]*casadi.sin(P[4]*casadi.arctan(P[5]*Fz))
    B=BCD/(C*D)
    F_temp = C_tire*D*casadi.sin(C*casadi.arctan(B*sa))
    return F_temp

def getFz(z):
    a_temp = (z[8] - 0.5*CdA*pair*(z[5])**2)/m
    dw = (h_cog/lol)*(a_temp/g)*m*g
    Ffz_temp=(lr/(lf+lr))*m*g + 0.25*pair*ClA*(z[5]**2) - dw
    Frz_temp=(lf/(lf+lr))*m*g + 0.25*pair*ClA*(z[5]**2) + dw

    return Ffz_temp,Frz_temp

def getSas(z):
    saf_temp=casadi.arctan((z[6]+lf*z[7])/(z[5]+1e-3)) - z[9]
    sar_temp=casadi.arctan((z[6]-lr*z[7])/(z[5]+1e-3))
    return saf_temp,sar_temp

def obj(z,current_target):
    """Least square costs on deviating from the path and on the inputs F and phi
    z = [dF,ddelta,xPos,yPos,phi, vx, vy, r, F, delta]
    current_target = point on path that is to be headed for
    """
    _,sar = getSas(z) 
    _,Frz = getFz(z)
    a,b = getEllipseParams(Frz)
    Fry = getFy(Frz,sar)
    if(mission == 'skidpad'):
        return (
                3e3*(z[2]-current_target[0])**2 
                + 3e3*(z[3]-current_target[1])**2 
                + 1e-4*(z[4]-current_target[2])**2 #dphi gap
                + 3e0*(z[5]-current_target[3])**2
                + 1e-4*(z[0]/1000)**2 # penalty on input F,dF
                + 1e-4*(z[8]/1000)**2
                + 1e2*z[1]**2 #penalty on delta,ddelta
                + 1e4*z[9]**2
                + 1e-3*(sar**2)
                + 1e-2*((z[8]/(a*Frz))**2 + (Fry/(b*Frz))**2)
            )
    elif(mission == 'autocross'):
        return (
                3e3*(z[2]-current_target[0])**2 
                + 3e3*(z[3]-current_target[1])**2 
                + 1e-4*(z[4]-current_target[2])**2 
                + 3e2*(z[5]-current_target[3])**2
                + 1e2*(z[0]/1000)**2 # penalty on input F,dF
                + 1e2*(z[8]/1000)**2
                + 1e2*z[1]**2 #penalty on delta,ddelta
                + 1e4*z[9]**2
                + 1e-3*(sar**2)
                + 1e-2*((z[8]/(a*Frz))**2 + (Fry/(b*Frz))**2)
                )
    elif(mission == 'acceleration'):
        return (
                3e3*(z[2]-current_target[0])**2 
                + 3e3*(z[3]-current_target[1])**2 
                + 1e-4*(z[4]-current_target[2])**2 
                + 1e4*(z[5]-current_target[3])**2
                + 1e5*(z[0]/1000)**2 # penalty on input F,dF
                + 1e3*(z[8]/1000)**2
                + 1e2*z[1]**2 #penalty on delta,ddelta
                + 1e1*z[9]**2
                + 1e-3*(sar**2)
                + 1e-2*((z[8]/(a*Frz))**2 + (Fry/(b*Frz))**2)
                )
    return

def continuous_dynamics(x, u):
    """Defines dynamics of the car, i.e. equality constraints.
    parameters:
    state x = [xPos,yPos,phi, vx, vy, r, F, delta]
    input u = [dF,ddelta]
    augmented z = [x u]
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
    Fdrag = 0.5*CdA*pair*(x[3])**2 + mi_fric*(Frz+Ffz)

    #blending with changing lambda
    beta = casadi.arctan(lr/(lf + lr) * casadi.tan(x[7]))
    # xdot = (l_a)*(x[3]*casadi.cos(x[2]) - x[4]*casadi.sin(x[2])) + (1-l_a)*(x[3]*casadi.cos(x[2] + beta))
    # ydot = (l_a)*(x[3]*casadi.sin(x[2]) + x[4]*casadi.cos(x[2])) + (1-l_a)*(x[3]*casadi.sin(x[2] + beta))
    xdot = x[3]*casadi.cos(x[2]) - x[4]*casadi.sin(x[2])
    ydot = x[3]*casadi.sin(x[2]) + x[4]*casadi.cos(x[2])
    phidot = x[5]
    vxdot = (1-l_a)*((x[6] - Fdrag)/ m) + (l_a)*((x[6] - Fdrag + Ffy*casadi.sin(x[7]) + m*x[4]*x[5])/ m)
    vydot = (1-l_a)*((lr/(lr+lf))*(vxdot*casadi.tan(x[7])+x[3]*(u[1]/(casadi.cos(x[7]))**2))) + (l_a)*(((-x[3]*x[5]) + (Fry + Ffy*casadi.cos(x[7])))/(1.0*m))
    rdot = (1-l_a)*((1/(lr+lf))*(vxdot*casadi.tan(x[7])+x[3]*(u[1]/(casadi.cos(x[7]))**2))) + (l_a)*((Ffy*lf*casadi.cos(x[7]) - Fry*lr)/(1.0*Iz))
    Fdot= u[0]
    deltadot = u[1]
    return casadi.vertcat(xdot,ydot,phidot,vxdot,vydot,rdot,Fdot,deltadot)

def generate_pathplanner():
    """Generates and returns a FORCESPRO solver that calculates a path based on
    constraints and dynamics while minimizing an objective function
    """
    # Model Definition
    # ----------------
    
    # Problem dimensions
    model = forcespro.nlp.SymbolicModel()
    model.N = 30  # horizon length
    model.nvar = 10  # number of variables
    model.neq = 8  # number of equality constraints
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
    model.lb = np.array([-3560.7*eff,  -np.deg2rad(30), -400.,   -400.,  -np.inf,  -1e-6, -15.0, -15.0, -3560.7*eff, -np.deg2rad(30)])
    model.ub = np.array([+3560.7*eff,  np.deg2rad(+30), 400.,   400.,   +np.inf, 15.0, +15.0, 15.0, 3560.7*eff, np.deg2rad(30)])

    #model.nh = 3 #number of inequality constr
    #model.ineq = lambda z : 0
    #model.hu = np.array([+np.inf,+1.0,+np.inf])
    #model.hl = np.array([-np.inf,-np.inf,-np.inf])
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
    codeoptions.printlevel = 0  # Use printlevel = 2 to print progress (but 
    #                             not for timings)
    codeoptions.optlevel = 0    # 0 no optimization, 1 optimize for size, 
    #                             2 optimize for speed, 3 optimize for size & speed
    codeoptions.cleanup = False
    codeoptions.timing = 1
    codeoptions.mip.mipgap = 0.5
    codeoptions.mip.explore = 'bestFirst'
    codeoptions.mip.branchon = 'mostAmbiguous'
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

generate_pathplanner()