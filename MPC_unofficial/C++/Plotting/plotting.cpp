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

#include "plotting.h"
namespace mpcc
{

    Plotting::Plotting(double Ts, PathToJson path)
        : model_(Model(Ts, path)),
          param_(Param(path.param_path))
    {
    }
    void Plotting::plotRun(const std::list<MPCReturn> &log, const TrackPos &track_xy) const
    {

        std::vector<double> plot_xc(track_xy.X.data(), track_xy.X.data() + track_xy.X.size());
        std::vector<double> plot_yc(track_xy.Y.data(), track_xy.Y.data() + track_xy.Y.size());

        std::vector<double> plot_xi(track_xy.X_inner.data(), track_xy.X_inner.data() + track_xy.X_inner.size());
        std::vector<double> plot_yi(track_xy.Y_inner.data(), track_xy.Y_inner.data() + track_xy.Y_inner.size());
        std::vector<double> plot_xo(track_xy.X_outer.data(), track_xy.X_outer.data() + track_xy.X_outer.size());
        std::vector<double> plot_yo(track_xy.Y_outer.data(), track_xy.Y_outer.data() + track_xy.Y_outer.size());

        std::vector<double> plot_x;
        std::vector<double> plot_y;
        std::vector<double> plot_phi;
        std::vector<double> plot_vx;
        std::vector<double> plot_vy;
        std::vector<double> plot_r;
        std::vector<double> plot_s;
        std::vector<double> plot_d;
        std::vector<double> plot_delta;
        std::vector<double> plot_vs;
        std::vector<double> plot_rwf;
        std::vector<double> plot_rwr;
        std::vector<double> plot_ax;

        std::vector<double> plot_dd;
        std::vector<double> plot_ddelta;
        std::vector<double> plot_dvs;

        std::vector<double> plot_alpha_f;
        std::vector<double> plot_alpha_r;
        std::vector<double> plot_slip_f;
        std::vector<double> plot_slip_r;
        std::vector<double> plot_F_rx0;
        std::vector<double> plot_F_ry0;
        std::vector<double> plot_F_fx0;
        std::vector<double> plot_F_fy0;
        
        std::vector<double> plot_TG;
        std::vector<double> plot_TBf;
        std::vector<double> plot_TBr;
        

        for (MPCReturn log_i : log)
        {
            plot_x.push_back(log_i.mpc_horizon[0].xk.X);
            plot_y.push_back(log_i.mpc_horizon[0].xk.Y);
            plot_phi.push_back(log_i.mpc_horizon[0].xk.phi);
            plot_vx.push_back(log_i.mpc_horizon[0].xk.vx);
            plot_vy.push_back(log_i.mpc_horizon[0].xk.vy);
            plot_r.push_back(log_i.mpc_horizon[0].xk.r);
            plot_s.push_back(log_i.mpc_horizon[0].xk.s);
            plot_d.push_back(log_i.mpc_horizon[0].xk.D);
            plot_delta.push_back(log_i.mpc_horizon[0].xk.delta);
            plot_vs.push_back(log_i.mpc_horizon[0].xk.vs);
            plot_rwf.push_back(log_i.mpc_horizon[0].xk.rwf);
            plot_rwr.push_back(log_i.mpc_horizon[0].xk.rwr);

            plot_dd.push_back(log_i.mpc_horizon[0].uk.dD);
            plot_ddelta.push_back(log_i.mpc_horizon[0].uk.dDelta);
            plot_dvs.push_back(log_i.mpc_horizon[0].uk.dVs);

            double alpha_f = model_.getSlipAngles(log_i.mpc_horizon[0].xk).sa_f;
            double alpha_r = model_.getSlipAngles(log_i.mpc_horizon[0].xk).sa_r;
            double slip_f = model_.getSlipRatios(log_i.mpc_horizon[0].xk).sr_f;
            double slip_r = model_.getSlipRatios(log_i.mpc_horizon[0].xk).sr_r;
            double a_x = model_.getF(log_i.mpc_horizon[0].xk,log_i.mpc_horizon[0].uk)(3);
            // std::cout << "horizon 0 is:" << log_i.mpc_horizon[0].xk << std::endl;
            // std::cout << "horizon 1 is:" << log_i.mpc_horizon[1].xk << std::endl;
            TireForces F_r0 = model_.getForceRear(log_i.mpc_horizon[0].xk);
            TireForces F_f0 = model_.getForceFront(log_i.mpc_horizon[0].xk);
            plot_alpha_f.push_back(alpha_f);
            plot_alpha_r.push_back(alpha_r);
            plot_slip_f.push_back(slip_f);
            plot_slip_r.push_back(slip_r);
            plot_F_rx0.push_back(F_r0.F_x);
            plot_F_ry0.push_back(F_r0.F_y);
            plot_F_fx0.push_back(F_f0.F_x);
            plot_F_fy0.push_back(F_f0.F_y);
            plot_ax.push_back(a_x);

            Torques T = model_.getTorques(log_i.mpc_horizon[0].xk);
            plot_TG.push_back(T.T_G);
            plot_TBf.push_back(T.T_Bf);
            plot_TBr.push_back(T.T_Br);
        }

        std::vector<double> plot_eps_x;
        std::vector<double> plot_eps_y;
        for (double t = 0; t < 2 * M_PI; t += 0.1)
        {
            plot_eps_x.push_back(cos(t) * 1300 * 0.95);
            plot_eps_y.push_back(sin(t) * 1300 * 1. / 1.25 * 0.95);
        }

        plt::figure(1);
        plt::plot(plot_xc, plot_yc, "r--");
        plt::plot(plot_xi, plot_yi, "k-");
        plt::plot(plot_xo, plot_yo, "k-");
        plt::plot(plot_x, plot_y, "b-");
        plt::axis("equal");
        plt::xlabel("X [m]");
        plt::ylabel("Y [m]");
        plt::figure(2);
        plt::subplot(3, 2, 1);
        plt::plot(plot_x);
        plt::ylabel("X [m]");
        plt::subplot(3, 2, 2);
        plt::plot(plot_y);
        plt::ylabel("Y [m]");
        plt::subplot(3, 2, 3);
        plt::plot(plot_phi);
        plt::ylabel("phi [rad]");
        plt::subplot(3, 2, 4);
        plt::plot(plot_vx);
        plt::ylabel("v_x [m/s]");
        plt::subplot(3, 2, 5);
        plt::plot(plot_vy);
        plt::ylabel("v_y [m/s]");
        plt::subplot(3, 2, 6);
        plt::plot(plot_r);
        plt::ylabel("r [rad/s]");

        plt::figure(3);
        plt::subplot(3, 1, 1);
        plt::plot(plot_d);
        plt::ylabel("D [-]");
        plt::subplot(3, 1, 2);
        plt::plot(plot_delta);
        plt::ylabel("delta [rad]");
        plt::subplot(3, 1, 3);
        plt::plot(plot_vs);
        plt::ylabel("v_s [m/s]");

        plt::figure(4);
        plt::subplot(3, 1, 1);
        plt::plot(plot_dd);
        plt::ylabel("dot{D} [-]");
        plt::subplot(3, 1, 2);
        plt::plot(plot_ddelta);
        plt::ylabel("dot{delta} [rad/s]");
        plt::subplot(3, 1, 3);
        plt::plot(plot_dvs);
        plt::ylabel("dot{v_s} [m/s^2]");

        plt::figure(5);
        plt::plot(plot_s);
        plt::ylabel("s [m]");

        plt::figure(6);
        plt::subplot(1, 6, 1);
        plt::plot(plot_alpha_f);
        plt::ylabel("alpha_f [rad]");
        plt::subplot(1, 6, 2);
        plt::plot(plot_alpha_r);
        plt::ylabel("alpha_r [rad]");
        plt::subplot(1, 6, 3);
        plt::plot(plot_slip_f);
        plt::ylabel("slip_f [rad]");
        plt::subplot(1, 6, 4);
        plt::plot(plot_slip_r);
        plt::ylabel("slip_r [rad]");
        plt::subplot(1, 6, 5);
        plt::plot(plot_F_ry0, plot_F_rx0);
        plt::xlabel("Fry");
        plt::ylabel("Frx");
        // plt::plot(plot_F_ry1, plot_F_rx1);
        // plt::plot(plot_eps_x, plot_eps_y);
        plt::subplot(1, 6, 6);
        plt::plot(plot_ax);
        plt::ylabel("ax");
        plt::axis("equal");
        plt::xlabel("F_y [N]");
        plt::ylabel("F_x [N]");

        plt::figure(7);
        plt::subplot(1, 4, 1);
        plt::plot(plot_F_fx0);
        plt::ylabel("Ffx");
        plt::subplot(1, 4, 2);
        plt::plot(plot_F_fy0);
        plt::ylabel("Ffy");
        plt::subplot(1, 4, 3);
        plt::plot(plot_F_rx0);
        plt::ylabel("Frx");
        plt::subplot(1, 4, 4);
        plt::plot(plot_F_ry0);
        plt::ylabel("Fry");

        plt::figure(8);
        plt::subplot(1, 2, 1);
        plt::plot(plot_rwf);
        plt::ylabel("rwf");
        plt::subplot(1, 2, 2);
        plt::plot(plot_rwr);
        plt::ylabel("rwr");

        plt::figure(9);
        plt::subplot(1, 3, 1);
        plt::plot(plot_TG);
        plt::ylabel("TG");
        plt::subplot(1, 3, 2);
        plt::plot(plot_TBf);
        plt::ylabel("TBf");
        plt::subplot(1, 3, 3);
        plt::plot(plot_TBr);
        plt::ylabel("TBr");
        plt::show();
    }
    void Plotting::plotSim(const std::list<MPCReturn> &log, const TrackPos &track_xy) const
    {
        std::vector<double> plot_xc(track_xy.X.data(), track_xy.X.data() + track_xy.X.size());
        std::vector<double> plot_yc(track_xy.Y.data(), track_xy.Y.data() + track_xy.Y.size());

        std::vector<double> plot_xi(track_xy.X_inner.data(), track_xy.X_inner.data() + track_xy.X_inner.size());
        std::vector<double> plot_yi(track_xy.Y_inner.data(), track_xy.Y_inner.data() + track_xy.Y_inner.size());
        std::vector<double> plot_xo(track_xy.X_outer.data(), track_xy.X_outer.data() + track_xy.X_outer.size());
        std::vector<double> plot_yo(track_xy.Y_outer.data(), track_xy.Y_outer.data() + track_xy.Y_outer.size());

        std::vector<double> plot_x;
        std::vector<double> plot_y;

        for (MPCReturn log_i : log)
        {
            plot_x.resize(0);
            plot_y.resize(0);
            // std::cout << "mpc horizon size is: " << log_i.mpc_horizon.size() << std::endl;
            for (int j = 0; j < log_i.mpc_horizon.size(); j++)
            {
                plot_x.push_back(log_i.mpc_horizon[j].xk.X);
                plot_y.push_back(log_i.mpc_horizon[j].xk.Y);
            }
            plt::clf();
            plt::plot(plot_xc, plot_yc, "r--");
            plt::plot(plot_xi, plot_yi, "k-");
            plt::plot(plot_xo, plot_yo, "k-");
            plotBox(log_i.mpc_horizon[0].xk);
            plt::plot(plot_x, plot_y, "b-");
            plt::axis("equal");
            // plt::xlim(-2,2);
            // plt::ylim(-2,2);
            plt::pause(0.01);
        }
    }

    void Plotting::plotBox(const State &x0) const
    {
        std::vector<double> corner_x;
        std::vector<double> corner_y;
        double body_xl = std::cos(x0.phi) * param_.car_l;
        double body_xw = std::sin(x0.phi) * param_.car_w;
        double body_yl = std::sin(x0.phi) * param_.car_l;
        double body_yw = -std::cos(x0.phi) * param_.car_w;

        corner_x.push_back(x0.X + body_xl + body_xw);
        corner_x.push_back(x0.X + body_xl - body_xw);
        corner_x.push_back(x0.X - body_xl - body_xw);
        corner_x.push_back(x0.X - body_xl + body_xw);
        corner_x.push_back(x0.X + body_xl + body_xw);

        corner_y.push_back(x0.Y + body_yl + body_yw);
        corner_y.push_back(x0.Y + body_yl - body_yw);
        corner_y.push_back(x0.Y - body_yl - body_yw);
        corner_y.push_back(x0.Y - body_yl + body_yw);
        corner_y.push_back(x0.Y + body_yl + body_yw);

        plt::plot(corner_x, corner_y, "k-");
    }
}