#ifndef VELOCITY_ESTIMATION_H
#define VELOCITY_ESTIMATION_H

#include <memory>
#include <array>
#include <eigen3/Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include "velocity_estimation_common.h"

namespace ns_vel_est
{
// Forward declaration of the VelocityEstimationHandler class so it can be used for the Handle type
class VelocityEstimationHandler;

using StateVector = Eigen::Matrix<double, StateSize, 1>;
using StateMatrix = Eigen::Matrix<double, StateSize, StateSize>;

using ObservationVector = Eigen::Matrix<double, ObservationSize, 1>;
using ObservationMatrix = Eigen::Matrix<double, ObservationSize, StateSize>;
using MeasurementNoiseMatrix = Eigen::Matrix<double, ObservationSize, ObservationSize>;

using Handle = VelocityEstimationHandler*;

class VelocityEstimator {
private:
    Handle node_handler_;       // pointer to handler class
    double delta_time_;         // time between iterations in seconds

    StateVector state_;
    StateMatrix state_covariance_;
    ObservationVector observations_;

    ObservationVector measurements_;
    std::array<bool, ObservationSize> update_vector_;

    // Jacobian Matrices
    StateMatrix transfer_function_jacobian_;
    ObservationMatrix measurement_function_jacobian_;

    // Noise Matrices
    StateMatrix process_noise_cov_;
    MeasurementNoiseMatrix measurement_noise_cov_;

    // Identity Matrix  used for covariance update
    StateMatrix identity_;

    // Sensor parameters
    double vn_200_rx_;
    double vn_200_ry_;
    double vn_300_rx_;
    double vn_300_ry_;

    // Projects the state vector forward
    void getNextState();

    // Gets the expected values of the observations based on the state prediction
    void predictObservations();

    /* --- Executes the predict step of the algorithm ---
     * Basically projects the state and state error forward by
     * one time step using the EKF algorithm.
     */
    bool predict();

    bool update(const std::vector<size_t>& indices);

public:
    explicit VelocityEstimator(Handle nh);
    // ~VelocityEstimator();

    // Initializes the Kalman filter matrices and other variables
    void init();

    void runAlgorithm();

    // Setters
    void setDeltaTime(const double dt) { delta_time_ = dt; }
    void setMeasurements(const ObservationVector& meas) { measurements_ = meas; }
    void setUpdateVector(const std::array<bool, ObservationSize>& update) { update_vector_ = update; }

    // Getters
    StateVector getState() { return state_; }
    StateMatrix getStateCovariance() { return state_covariance_; }
};
} //namespace ns_vel_est

#endif // VELOCITY_ESTIMATION_H