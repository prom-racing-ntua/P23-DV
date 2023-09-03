from mpc_class_curv import *

def main():
    model,solver = generate_pathplanner()
    emergency_count=0
    fig,ax = plt.subplots()
    fig.set_size_inches(13,9)
    #from C++ path planning
    data_points = Dataloader("Data/als_data.txt")
    midpoints = Dataloader("Data/trackdrive_midpoints.txt")
    cs,reference_track = cubic_spline_generation(midpoints[0,:],midpoints[1,:])
    print("reference track: ",reference_track, np.shape(reference_track))
    # for_mpc = cubic_spline_inference(cs,[0.0,0.0])
    # generate code for estimator
    num_ins = model.nvar-model.neq
    print("after solver generation")
    # Simulation
    sim_length = 10000 # simulate sim_length*0.05
    vx0 = 0.1
    Frx0 = 300
    # Variables for storing simulation data
    x = np.zeros((model.neq,sim_length+1))  # states
    x_model = np.zeros((model.neq-2,sim_length+1))
    u = np.zeros((num_ins,sim_length))  # inputs
    u_model = np.zeros((num_ins,sim_length))
    # Set initial guess to start solver from
    x0i = np.zeros((model.nvar,1))
    x0 = np.transpose(np.tile(x0i, (1, model.N)))
    x0_model = np.transpose(np.tile(np.zeros((model.nvar-2,1)), (1, model.N)))
    # z = [dF,ddelta,s,ni,mi,vx,vy,r,F,delta] # 8D-state // 2D-input
    # z_sim = [X,Y,phi,vx,vy,r,F,delta] # 6D-state // 2D-input
    # define initial conditions of x_init - x0
    xinit = np.transpose(np.array([0.0, 0.0, 0.0, vx0, 0.0, 0.0, Frx0, 0.0]))
    xinit_model = np.transpose(np.array([reference_track[0][0], reference_track[1][0],reference_track[2][0], vx0, 0.0, 0.0])) 
    problem = {"x0": x0, "xinit": xinit}
    start_pred = np.reshape(x0_model,(model.nvar-2,model.N)) # first predicition corresponds to initial guess
    x[:,0] = xinit
    x_model[:,0] = xinit_model
    createPlot(x_model,u,start_pred,sim_length,model,reference_track[:2,:],xinit,cones_yellow, cones_blue, cones_orange_big)
    #define arrays for error correction and logging
    time_array=[]
    err_array=[]
    err_1=0.0
    err_2=0.0
    for k in range(sim_length):
        print("Im at iteration: ",k+1)
        problem["xinit"] = x[:,k] #model update
        print("Im at: ", x_model[:,k][0], " " ,x_model[:,k][1])
        if(k==0):
            s_start=0.0
            next_data_points = reference_track[:,1:model.N+1]
            problem["all_parameters"] = np.reshape(np.transpose(next_data_points),(model.npar*model.N,1))
            x_model[:,k+1] = x_model[:,k]
            u_model[:,k+1] = np.array([Frx0,0.0])
        if(k>0):
            s_start = problem["xinit"][0]  
            if(s_start>INDEX_MAX): s_start=0.0 #reset lap
            x_for_splines = x_model[:,k]
            u_for_splines = u_model[:,k]
            splines_loc = cubic_spline_inference(cs,[s_start,s_start],x_for_splines,u_for_splines)
            print("splines_loc is:",splines_loc[0][0]," ", splines_loc[1][0], " ",s_start)
            s_closest, _ = generate_closest_s(reference_track, x_model[0:num_ins,k],0)
            closest_loc = cubic_spline_inference(cs,[s_closest,s_closest],x_for_splines,u_for_splines)
            print("closest loc is:",closest_loc[0][0]," ", closest_loc[1][0], " ",s_closest)
            # print("where i am is:",where_i_am,np.shape(where_i_am))
            err_1 = np.abs(np.sin(closest_loc[2][0])*(x_model[:,k][0]-closest_loc[0][0]) - np.cos(closest_loc[2][0])*(x_model[:,k][1]-closest_loc[1][0])) #katakorifi
            err_2 = np.sqrt((x_model[:,k][0] - closest_loc[0][0])**2 + (x_model[:,k][1] - closest_loc[1][0])**2)
            err_array.append(err_1)
            print("errors from closest are are:", err_1," ",err_2," ",np.max(err_array)," ",np.mean(err_array))
            print("error from state is: ",x[:,k][1])
            if(err_2 < 1.0): print("no emergency!")
            else: print("emergency!!!!")
            emergency_bool=0
            parameters_array = [s_closest]
            s_new=s_closest
            if(s_closest>INDEX_MAX): s_closest = 0.0 #reset index for next lap
            for i in range (model.N-1):
                ds_step=ds_wanted
                s_new+=ds_step
                parameters_array.append(s_new)
            x_for_splines=x[:,k]
            next_data_points = cubic_spline_inference(cs,np.array(parameters_array),x_for_splines,u_for_splines)
            print("data points are...",next_data_points)
            problem["all_parameters"] = next_data_points
            print("parameters array with ds(used) is: ",parameters_array,np.shape(parameters_array))
            output, exitflag, info = solver.solve(problem)
            # Make sure the solver has exited properly.
            assert exitflag == 1, "bad exitflag"
            sys.stderr.write("FORCESPRO took {} iterations and {} seconds to solve the problem.\n"\
                .format(info.it, info.solvetime))
            time_array.append(info.solvetime)
            # Extract outputs from solver
            temp = np.zeros((np.max(model.nvar), model.N))
            for i in range(0, model.N):
                temp[:, i] = output['x{0:02d}'.format(i+1)]
            pred_u = temp[0:num_ins, :]
            pred_x = temp[num_ins:model.nvar, :]
            pred_x_model = temp[num_ins:model.nvar-2,:] #just to initialize the array for update loop
            pred_u_model = temp[6:8,:]
            #apply second solution for new MPC state
            u[:,k] = pred_u[:,1]
            x[:,k+1] = np.transpose(model.eq(np.concatenate((u[:,k],x[:,k]))))
            #apply second solution for new car state
            u_mpc = np.array([x[:,k+1][-2],x[:,k+1][-1]]) #solution after MPC integration
            u_mpc_2 = np.array([x[:,k][-2],x[:,k][-1]]) #solution that MPC gives
            for i in range(int(model.N)): 
                if(i==0): 
                    x_model[:,k+1] = runge_kutta_upd(u_mpc,x_model[:,k],dt_integration) #first model step
                    u_model[:,k] = u_mpc
                    pred_u_model[:,i] = u_mpc
                    pred_x_model[:,i] = x_model[:,k+1]
                else:
                    pred_u_model[:,i] = pred_x[6:8,i] #next MPC solution
                    pred_x_model[:,i] = runge_kutta_upd(pred_u_model[:,i],pred_x_model[:,i-1],dt_integration) #next model step
            #plot MPC all
            if(k%20==0):
                updatePlots(x_model,u_model,pred_x_model,pred_u_model,model,k)  
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
                    
def main_solver(): model, solver = generate_pathplanner()

if __name__ == "__main__":
    cmd_arg = sys.argv[1]
    if(cmd_arg=="one_simulation"): 
        print("solver generator script called + simulation")
        main()
    if(cmd_arg=="no_simulation"): 
        print("solver generator script called + no simulation")
        main_solver()