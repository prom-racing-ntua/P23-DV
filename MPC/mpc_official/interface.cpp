#include "interface.h"
using namespace mpc_cpp;

int main() {
    readData();
    for (int i = 0; i < SIM; i++) {
        if(i==0){ //various needed initalizations before first run
            Initialize_all();
        }
        std::cout << " " << std::endl;
        std::cout << "I'm at iteration " << i+1 << std::endl;
        node_out node_out;
        node_out.motor_torque = (X[6]*Rw/gr);
        node_out.brake_torque = 0.0;
        node_out.steering_angle = X[7]/sr;
        std::cout << "node output is: " << node_out.motor_torque << " " << node_out.brake_torque << " " << node_out.steering_angle << std::endl; 
        for (int j = 0; j < X_SIZE; ++j) params.xinit[j] = X[j]; //update init state
        writeParams();
        exitflag = FORCESNLPsolver_solve(&params, &output, &info, mem, NULL, extfunc_eval);
        if (exitflag != 1) {
            printf("\n\nFORCESNLPsolver did not return optimal solution at step %d. Exiting.\n", i + 1);
            return_val = 1;
            break;
        }
        //dF ddelta dindex
        for(int k = 0; k<3; k++){
            U[k]=output.x01[k];
        }
        std::cout << "U array is: " << U[0] << " " << U[1] << " " << U[2] << std::endl;
        Integrator();
        std::cout << "X,Y,velocities are: " << X[0] << " " << X[1] << " " << X[3] << std::endl;
        std::cout << "inputs final are: " << X[6] << " " << X[7] << " " << X[8] << std::endl;
}
}


