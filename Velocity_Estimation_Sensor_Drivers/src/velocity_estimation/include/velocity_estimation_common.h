#ifndef VELOCITY_ESTIMATION_COMMON_H
#define VELOCITY_ESTIMATION_COMMON_H

#include <unordered_map>

namespace ns_vel_est
{

/*
 * Enumeration of the state variables. This way it is easy to add or remove variables
 * without having to change every reference to that variable in the filter equations.
 */
enum StateVectorEnum {
    StateVx,
    StateVy,
    StateVyaw,
    StateAx,
    StateAy,
    StateAyaw,
    StateSize,
};

enum ObservationVectorEnum {
    ObservationVx,
    ObservationVy,
    ObservationVyaw,
    ObservationAx,
    ObservationAy,
    ObservationVhall_fr,
    ObservationVhall_fl,
    ObservationVhall_rr,
    ObservationVhall_rl,
    ObservationSize,
};

enum SensorEnum {
    VelocitySensor,
    Gyroscope,
    Accelerometer,
    FrHall,
    FlHall,
    RrHall,
    RlHall,
    SensorSize,
};

} // namespace ns_vel_est

#endif // VELOCITY_ESTIMATION_COMMON_H