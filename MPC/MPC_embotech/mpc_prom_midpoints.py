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
from TriangulationNew import Triangulation
from track_plotter import getTrackdrive, getSkidpad, getAccel, getAutoX
from ParametricCubicSpline import getSpline, getSplineDerivatives
from Trackdrive import cones_blue, cones_orange_big, cones_yellow




# class toSimulator:
#     def __init__(self,) -> None:
        

def continuous_dynamics(x, u):
    """Defines dynamics of the car, i.e. equality constraints.
    parameters:
    state x = [xPos,yPos,v,theta,delta]
    input u = [F,phi]
    """
    # set physical constants
    l_f = 0.9141
    l_r = 0.7359
    CdA = 1.8
    pair = 1.225
    m = 190.0   # mass of the car

    # set parameters
    beta = casadi.arctan(l_r/(l_f + l_r) * casadi.tan(x[4]))

    #friction forces
    Fdrag = 0.5*CdA*pair*(x[2])**2

    # calculate dx/dt
    print("F on x is",u[0] - Fdrag)
    return casadi.vertcat(  x[2] * casadi.cos(x[3] + beta),  # dxPos/dt = v*cos(theta+beta)
                            x[2] * casadi.sin(x[3] + beta),  # dyPos/dt = v*sin(theta+beta)
                            (u[0] - Fdrag)/ m,                # dv/dt = F/m
                            x[2]/l_r * casadi.sin(beta),     # dtheta/dt = v/l_r*sin(beta) = w
                            u[1])                           # ddelta/dt = phi

def obj(z,current_target):
    """Least square costs on deviating from the path and on the inputs F and phi
    z = [F,phi,xPos,yPos,v,theta,delta]
    current_target = point on path that is to be headed for
    """
    return (9500.0*(z[2]-current_target[0])**2 # costs on deviating on the
#                                              path in x-direction
            + 9500.0*(z[3]-current_target[1])**2 # costs on deviating on the
#                                               path in y-direction
            + 0.01*z[0]**2 # penalty on input F
            + 0.01*z[1]**2 
            + 0.01*z[6]**2
            + 0.001*((1/(z[4]**2 +1e-6)))) 

def objN(z,current_target):
    """Increased least square costs for last stage on deviating from the path and 
    on the inputs F and phi
    z = [F,phi,xPos,yPos,v,theta,delta]
    current_target = point on path that is to be headed for
    """
    return (10000.0*(z[2]-current_target[0])**2 # costs on deviating on the
#                                              path in x-direction
            + 7500.0*(z[3]-current_target[1])**2 # costs on deviating on the
#                                               path in y-direction
            + 0.1*z[0]**2 # penalty on input F
            + 0.1*z[1]**2 
            + 0.01*((1/(z[4]**2 +1e-6)))) 

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
    print(np.shape(path_points))
    num_ellipses = np.ceil((idx+N+1)/num_points)
    print(num_ellipses)
    path_points = np.tile(path_points,(1,int(num_ellipses)))
    return path_points[:,idx+1:idx+N+1]

def ineq_constr(z, current_target):
    return casadi.vertcat((z[2]-current_target[0])**2  + (z[3]-current_target[1])**2)

def generate_pathplanner():
    """Generates and returns a FORCESPRO solver that calculates a path based on
    constraints and dynamics while minimizing an objective function
    """
    # Model Definition
    # ----------------

    # Problem dimensions
    model = forcespro.nlp.SymbolicModel()
    model.N = 20  # horizon length
    model.nvar = 7  # number of variables
    model.neq = 5  # number of equality constraints
    model.npar = 2 # number of runtime parameters
    num_ins=model.nvar-model.neq

    # Objective function
    model.objective = obj
    model.objectiveN = objN # increased costs for the last stage
    # The function must be able to handle symbolic evaluation,
    # by passing in CasADi symbols. This means certain numpy funcions are not
    # available.

    # We use an explicit RK4 integrator here to discretize continuous dynamics
    integrator_stepsize = 0.1
    model.eq = lambda z: forcespro.nlp.integrate(continuous_dynamics, z[num_ins:model.nvar], z[0:num_ins],
                                                integrator=forcespro.nlp.integrators.RK4,
                                                stepsize=integrator_stepsize)

    # Indices on LHS of dynamical constraint - for efficiency reasons, make
    # sure the matrix E has structure [0 I] where I is the identity matrix.
    model.E = np.concatenate([np.zeros((model.neq,num_ins)), np.eye(model.neq)], axis=1)

    # Inequality constraints
    #  upper/lower variable bounds lb <= z <= ub
    #                     inputs                 |  states
    #                     F          phi            x    y     v    theta         delta
    model.lb = np.array([-4120.0,  np.deg2rad(-60.),  -200.,   -200.,  0.,   -np.inf, -0.48*np.pi])
    model.ub = np.array([+3560.7,  np.deg2rad(+60.),   200.,   200.,   20.,   np.inf,  0.48*np.pi])

    model.nh = 1 #number of inequality constr
    model.ineq = ineq_constr
    # model.hu = np.array([100.0])
    # model.hl = np.array([0.0])

    # Initial condition on vehicle states x
    model.xinitidx = range(num_ins,model.nvar) # use this to specify on which variables initial conditions
    # are imposed

    # Solver generation
    # -----------------

    # Set solver options
    codeoptions = forcespro.CodeOptions('FORCESNLPsolver')
    codeoptions.maxit = 200     # Maximum number of iterations
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
    codeoptions.sqp_nlp.reg_hessian = 5e-7 # increase this if exitflag=-8
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

    # Update plot with current simulation data
    ax_list[0].plot(x[0,0:k+2],x[1,0:k+2], '-b')             # plot new trajectory
    ax_list[0].plot(pred_x[0,1:], pred_x[1,1:], 'g-')        # plot new prediction of trajectory
    ax_list[1].plot(x[2,0:k+2],'b-')                         # plot new velocity
    ax_list[1].plot(range(k+1,k+model.N), pred_x[2,1:],'g-') # plot new prediction of velocity
    ax_list[2].plot(np.rad2deg(x[3, 0:k+2]),'b-')            # plot new heading angle
    ax_list[2].plot(range(k+1,k+model.N), \
        np.rad2deg(pred_x[3,1:]),'g-')                       # plot new prediciton of heading angle
    ax_list[3].plot(np.rad2deg(x[4, 0:k+2]),'b-')            # plot new steering angle
    ax_list[3].plot(range(k+1,k+model.N), \
        np.rad2deg(pred_x[4,1:]),'g-')                       # plot new prediction of steering angle
    ax_list[4].step(range(0, k+1), u[0, 0:k+1],'b-')         # plot new acceleration force
    ax_list[4].step(range(k, k+model.N), pred_u[0,:],'g-')   # plot new prediction of acceleration force
    ax_list[5].step(range(0, k+1), \
        np.rad2deg(u[1, 0:k+1]),'b-')                        # plot new steering rate
    ax_list[5].step(range(k, k+model.N), \
        np.rad2deg(pred_u[1,:]),'g-')                        # plot new prediction of steering rate

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
    l0, = ax_pos.plot(np.transpose(path_points[0,:]), np.transpose(path_points[1,:]), 'rx')
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

    # Plot velocity
    ax_vel = fig.add_subplot(5,2,2)
    plt.grid("both")
    plt.title('Velocity')
    plt.xlim([0., sim_length-1])
    plt.plot([0, sim_length-1], np.transpose([model.ub[4], model.ub[4]]), 'r:')
    plt.plot([0, sim_length-1], np.transpose([model.lb[4], model.lb[4]]), 'r:')
    ax_vel.plot(0.,x[2,0], '-b')
    ax_vel.plot(start_pred[4,:], 'g-')
    
    # Plot heading angle
    ax_theta = fig.add_subplot(5,2,4)
    plt.grid("both")
    plt.title('Heading angle')
    plt.ylim([0., 900.]) 
    plt.xlim([0., sim_length-1])
    plt.plot([0, sim_length-1], np.rad2deg(np.transpose([model.ub[5], model.ub[5]])), 'r:')
    plt.plot([0, sim_length-1], np.rad2deg(np.transpose([model.lb[5], model.lb[5]])), 'r:')
    ax_theta.plot(np.rad2deg(x[3,0]), 'b-')
    ax_theta.plot(np.rad2deg(start_pred[5,:]), 'g-')

    # Plot steering angle
    ax_delta = fig.add_subplot(5,2,6)
    plt.grid("both")
    plt.title('Steering angle')
    plt.xlim([0., sim_length-1])
    plt.plot([0, sim_length-1], np.rad2deg(np.transpose([model.ub[6], model.ub[6]])), 'r:')
    plt.plot([0, sim_length-1], np.rad2deg(np.transpose([model.lb[6], model.lb[6]])), 'r:')
    ax_delta.plot(np.rad2deg(x[4,0]),'b-')
    ax_delta.plot(np.rad2deg(start_pred[6,:]),'g-')

    # Plot acceleration force
    ax_F = fig.add_subplot(5,2,8)
    plt.grid("both")
    plt.title('Acceleration force')
    plt.xlim([0., sim_length-1])
    plt.plot([0, sim_length-1], np.transpose([model.ub[0], model.ub[0]]), 'r:')
    plt.plot([0, sim_length-1], np.transpose([model.lb[0], model.lb[0]]), 'r:')
    ax_F.step(0, u[0,0], 'b-')
    ax_F.step(range(model.N), start_pred[0,:],'g-')

    # Plot steering rate
    ax_phi = fig.add_subplot(5,2,10)
    plt.grid("both")
    plt.title('Steering rate')
    plt.xlim([0., sim_length-1])
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
    track_new = np.concatenate([cones_blue,cones_yellow,cones_orange_big])
    tri = Triangulation(track)
    points = np.vstack((tri.midpoints, tri.midpoints[0,:]))
    points_mpc = np.vstack((tri.midpoints, tri.midpoints[0,:])).T
    spline = getSpline(points, axis=ax, boundary_condition='closed_loop')
    spline_mpc = getSpline(points, axis=ax, boundary_condition='closed_loop').T
    # print("points are: ",points,np.shape(points))
    # print("points mpc is: ",points_mpc,np.shape(points_mpc))
    path_points=points_mpc
    # generate code for estimator
    model, solver = generate_pathplanner()
    num_ins = model.nvar-model.neq

    #for txt save:
    force_txt=[]
    steering_txt=[]
    st_rate_txt=[]

    # Simulation
    # ----------
    sim_length = 50 # simulate 8sec
    # model.N = 10  # horizon length
    # model.nvar = 7  # number of variables
    # model.neq = 5  # number of equality constraints
    # model.npar = 2 # number of runtime parameters
    # num_ins=model.nvar-model.neq

    # Variables for storing simulation data
    x = np.zeros((model.neq,sim_length+1)) # states
    u = np.zeros((num_ins,sim_length)) # inputs

    # Set initial guess to start solver from
    x0i = np.zeros((model.nvar,1))
    x0 = np.transpose(np.tile(x0i, (1, model.N)))
    phi0 = casadi.arctan((path_points[1][1]-path_points[1][0])/(path_points[0][1]-path_points[0][0]))
    print("dy is",path_points[1][1]," ",path_points[1][0])
    print("dx is",path_points[0][1]," ",path_points[0][0])
    print("path points are: ",path_points)
    print("phi0 is",phi0)
    # Set initial condition
    #  x    y     v    theta         delta
    xinit = np.transpose(np.array([path_points[0][0], path_points[1][0], 0.01, -phi0, 0.]))
    x[:,0] = xinit

    problem = {"x0": x0,
            "xinit": xinit}

    # Create 2D points on ellipse which the car is supposed to follow
    # num_points = 80
    # path_points = calc_points_on_ellipse(num_points)
    print("path points are:",path_points,np.shape(path_points),type(path_points))

    start_pred = np.reshape(problem["x0"],(model.nvar,model.N)) # first prdicition corresponds to initial guess

    # generate plot with initial values
    createPlot(x,u,start_pred,sim_length,model,path_points,xinit,cones_yellow, cones_blue, cones_orange_big)
    
    # Simulation
    for k in range(sim_length):
        print("Im at iteration: ",k+1)
        # Set initial condition
        problem["xinit"] = x[:,k]
        print("x_init is:",problem["xinit"])
        # Set runtime parameters (here, the next N points on the path)
        next_path_points = extract_next_path_points(path_points, x[0:num_ins,k], model.N)
        print("next N path points are: ",next_path_points,np.shape(next_path_points),type(next_path_points))
        problem["all_parameters"] = np.reshape(np.transpose(next_path_points), \
            (2*model.N,1))
        print("all_parameters are: ",problem["all_parameters"], np.shape(problem["all_parameters"]),type(problem["all_parameters"]))

        # Time to solve the NLP!
        output, exitflag, info = solver.solve(problem)

        # Make sure the solver has exited properly.
        print(exitflag)
        # assert exitflag == 1, "bad exitflag"
        sys.stderr.write("FORCESPRO took {} iterations and {} seconds to solve the problem.\n"\
            .format(info.it, info.solvetime))

        # Extract output
        temp = np.zeros((np.max(model.nvar), model.N))
        for i in range(0, model.N):
            temp[:, i] = output['x{0:02d}'.format(i+1)]
        pred_u = temp[0:num_ins, :]
        pred_x = temp[num_ins:model.nvar, :]

        # Apply optimized input u of first stage to system and save simulation data
        u[:,k] = pred_u[:,0]
        print("u_bef is: ",u[:,k]," ",np.shape(u), " ",np.shape(u[:,k]))
        print("x_bef is: ",x[:,k]," ",np.shape(x), " ", np.shape(x[:,k]))
        force_txt.append(u[:,k][0])
        steering_txt.append(x[:,k][4])
        st_rate_txt.append(u[:,k][1])
        x[:,k+1] = np.transpose(model.eq(np.concatenate((u[:,k],x[:,k]))))
        # print("u is: ",u[:,k]," ",np.shape(u), " ",np.shape(u[:,k]))
        # print("x is: ",x[:,k+1]," ",np.shape(x), " ", np.shape(x[:,k+1]))
        # print("u_ol is: ",u ," ",np.shape(u), " ",np.shape(u))
        # print("x_ol is: ",x ," ",np.shape(x), " ", np.shape(x))
        # force_txt.append()
        # steering_txt.append()

        # plot results of current simulation step
        if(k>0):
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
    print("list force is: ", force_txt )
    print("list steering is: ", steering_txt)
    print("list steering rate is: ", st_rate_txt)
    open

if __name__ == "__main__":
    main()