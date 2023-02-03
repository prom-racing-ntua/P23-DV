// Copyright 2019 Alexander Liniger

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

#include "Tests/spline_test.h"
#include "Tests/model_integrator_test.h"
#include "Tests/constratins_test.h"
#include "Tests/cost_test.h"

#include "MPC/mpc.h"
#include "Model/integrator.h"
#include "Model/model.h"
#include "Params/track.h"
#include "Plotting/plotting.h"

#include <nlohmann/json.hpp>
using json = nlohmann::json;

int main() {
    using namespace mpcc; 
    std::ifstream iConfig("Params/config.json");
    json jsonConfig;
    iConfig >> jsonConfig;

    PathToJson json_paths {jsonConfig["model_path"],
                           jsonConfig["cost_path"],
                           jsonConfig["bounds_path"],
                           jsonConfig["track_path"],
                           jsonConfig["normalization_path"]};

    //std::cout << testSpline() << std::endl;
    //std::cout << testArcLengthSpline(json_paths) << std::endl;

    //std::cout << testIntegrator(json_paths) << std::endl;
    //std::cout << testLinModel(json_paths) << std::endl;

    // std::cout << testAlphaConstraint(json_paths) << std::endl;
    // std::cout << testTireForceConstraint(json_paths) << std::endl;
    //std::cout << testTrackConstraint(json_paths) << std::endl;

    // std::cout << testCost(json_paths) << std::endl;
    Integrator integrator = Integrator(jsonConfig["Ts"],json_paths);
    Model model = Model(jsonConfig["Ts"],json_paths);
    Plotting plotter = Plotting(jsonConfig["Ts"],json_paths);
    std::cout<<"integrator & plotter done"<<std::endl;

    Track track = Track(json_paths.track_path);
    std::cout<<"track done"<<std::endl;
    TrackPos track_xy = track.getTrack();
    std::cout<<"track pos done"<<std::endl;


    std::list<MPCReturn> log;
    std::cout<<"MPCReturn log list done"<<std::endl;
    MPC mpc(jsonConfig["n_sqp"],jsonConfig["n_reset"],jsonConfig["sqp_mixing"],jsonConfig["Ts"],json_paths);
    std::cout<<"mpc done"<<std::endl;
    mpc.setTrack(track_xy.X,track_xy.Y);
    std::cout<<"mpc.setTrack done"<<std::endl;
    const double phi_0 = std::atan2(track_xy.Y(1) - track_xy.Y(0),track_xy.X(1) - track_xy.X(0));
    State x0 = {track_xy.X(0),track_xy.Y(0),phi_0,jsonConfig["v0"],0,0,0,0,0,0.0,0,jsonConfig["v0"]};
    std::cout<<"phi_0 & x0 done"<<std::endl;
    std::cout<<"Number of simulation steps is: "<< jsonConfig["n_sim"] << std::endl;
    for(int i=0;i<jsonConfig["n_sim"];i++)
    {   
        std::cout<<"I'm at iteration of simulation " << i <<std::endl;
        MPCReturn mpc_sol = mpc.runMPC(x0);
        std::cout << "MPC run successfully"<<std::endl;
        // for (int j = 0; j < mpc_sol.mpc_horizon.max_size(); ++j){
        //     //  std::cout << i << std::endl;
        //     std::cout << mpc_sol.mpc_horizon[j].xk.vx << " " << mpc_sol.mpc_horizon[j].xk.vy << std::endl;
        // }
        // Use the MPC prediction as sim step
        x0 = mpc_sol.mpc_horizon[1].xk;
        // double frx_now=model.getForceRear(x0).F_x;
        // double fry_now=model.getForceRear(x0).F_y;
        // std::cout << "forces before int: " <<  " " << frx_now << " " << fry_now << std::endl;
        // std::cout << "rwr from horizon is: " << x0.rwr << std::endl;
        // Use ODE integrator
        x0 = integrator.simTimeStep(x0,mpc_sol.u0,jsonConfig["Ts"]);
        // double frx_now2=model.getForceRear(x0).F_x;
        // double fry_now2=model.getForceRear(x0).F_y;
        // std::cout << "forces after int: " <<  " " << frx_now2 << " " << fry_now2 << std::endl;
        log.push_back(mpc_sol);
    } 
    std::cout << "is all good?" << std::endl;
    plotter.plotRun(log,track_xy);
     std::cout << "is all good2?" << std::endl;
    plotter.plotSim(log,track_xy);
    std::cout << "is all good3?" << std::endl;

    double mean_time = 0.0;
    double max_time = 0.0;
    for(MPCReturn log_i : log)
    {
        mean_time += log_i.time_total;
        if(log_i.time_total > max_time)
            max_time = log_i.time_total;
    }
    std::cout << "mean nmpc time " << mean_time/double(jsonConfig["n_sim"]) << std::endl;
    std::cout << "max nmpc time " << max_time << std::endl;
    return 0;
}


