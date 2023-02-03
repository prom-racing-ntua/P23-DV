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

#ifndef MPCC_MODEL_H
#define MPCC_MODEL_H

#include "config.h"
#include "types.h"
#include "Params/params.h"

namespace mpcc{
//Return
struct LinModelMatrix {
    A_MPC A;
    B_MPC B;
    g_MPC g;
};

struct TireForces {
    const double F_x;
    const double F_y;
    const double F_z;
};

struct Torques {
    const double T_G;
    const double T_Bf;
    const double T_Br; 
};

struct NormalForces {
    const double F_N_front;
    const double F_N_rear;
};

struct TireForcesDerivatives{
    const double dF_x_vx;
    const double dF_x_vy;
    const double dF_x_r;
    const double dF_x_rwf;
    const double dF_x_rwr;
    const double dF_x_delta;

    const double dF_y_vx;
    const double dF_y_vy;
    const double dF_y_r;
    const double dF_y_rwf;
    const double dF_y_rwr;
    const double dF_y_delta;
    // const double dF_y_ax;
    // const double dF_x_ax;
    const double dF_z_vx;

};

struct TorquesDerivatives {
    const double dTG_D;
    const double dTBf_D;
    const double dTBr_D;
};

struct FrictionForceDerivatives {
    const double dF_f_vx;
    const double dF_f_vy;
    const double dF_f_r;
    const double dF_f_D;
    const double dF_f_delta;
};


struct KappaDerivatives {
    const double dk_vx;
    // const double dk_ax;
};

struct ForceParameters {
    const double B;
    const double C;
    const double C_tire;
    const double D;
};

struct SlipAngles{
    const double sa_f;
    const double sa_r;
};

struct SlipRatios{
    const double sr_f;
    const double sr_r;
};

struct SlipAngleDerivatives {
    const double dsaf_vx;
    const double dsaf_vy;
    const double dsaf_r;
    const double dsaf_delta;

    const double dsar_vx;
    const double dsar_vy;
    const double dsar_r;
};

struct SlipRatioDerivatives {
    const double dsrf_vx;
    const double dsrf_vy;
    const double dsrf_r;
    const double dsrf_rwf;
    const double dsrf_delta;

    const double dsrr_vx;
    const double dsrr_rwr;
};


class Model {
public:
    double getSlipAngleFront(const State &x) const;
    double getSlipAngleRear(const State &x) const;
    double getKappa(double Fz) const;
    double getKappa2(double Fz) const;
    double getLengthFront(const State &x) const;
    double getLengthRear(const State &x) const;
    //double getAcceleration(const State &x) const;

    SlipAngles getSlipAngles(const State &x) const;
    SlipRatios getSlipRatios(const State &x) const;
    SlipAngleDerivatives getSlipAngleDerivatives(const State &x) const;
    SlipRatioDerivatives getSlipRatioDerivatives(const State &x) const;

    TireForces getForceFront(const State &x) const;
    TireForces getForceRear(const State &x) const;
    double getForceFriction(const State &x) const;
    ForceParameters getForceParamsX(const double Fz) const;
    ForceParameters getForceParamsY(const double Fz) const;
    NormalForces getForceNormal(const State &x) const;
    Torques getTorques(const State &x) const;

    TireForcesDerivatives getForceFrontDerivatives(const State &x) const;
    TireForcesDerivatives getForceRearDerivatives(const State &x) const;
    FrictionForceDerivatives getForceFrictionDerivatives(const State &x) const;
    TorquesDerivatives getTorquesDerivatives(const State &x) const;
    KappaDerivatives getKappaDerivatives(const double Fz, const double dFz_vx, const double dF_ax) const;

    StateVector getF(const State &x,const Input &u) const;

    LinModelMatrix getLinModel(const State &x, const Input &u,const State &x_next) const;

    Model();
    Model(double Ts,const PathToJson &path);
private:
    LinModelMatrix getModelJacobian(const State &x, const Input &u) const;
    LinModelMatrix discretizeModel(const LinModelMatrix &lin_model_c,const State &x, const Input &u,const State &x_next) const;

    Param param_;
    const double Ts_;
};
}
#endif //MPCC_MODEL_H
