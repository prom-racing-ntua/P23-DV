#ifndef VELOCITY_ESTIMATION_H
#define VELOCITY_ESTIMATION_H

#include <memory>
#include <array>
#include <vector>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include "velocity_estimation_common.h"

namespace ns_vel_est
{
// Forward declaration of the VelocityEstimationHandler class so it can be used for the Handle type
class VelocityEstimationHandler;

using StateVector = Eigen::Matrix<double, StateSize, 1>;
using ObservationVector = Eigen::Matrix<double, ObservationSize, 1>;
using InputVector = Eigen::Matrix<double, InputSize, 1>;

using StateMatrix = Eigen::Matrix<double, StateSize, StateSize>;
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
    std::array<bool, SensorSize> updated_sensors_;
    InputVector input_;

    // Innovation and stuff for update
    ObservationVector innovation_;
    Eigen::Matrix<double, StateSize, ObservationSize> pht_;
    MeasurementNoiseMatrix innovation_cov_inverse_;

    // Jacobian Matrices
    StateMatrix transfer_function_jacobian_;
    ObservationMatrix measurement_function_jacobian_;

    // Noise Matrices
    StateMatrix process_noise_cov_;
    MeasurementNoiseMatrix measurement_noise_cov_;

    // Identity Matrix  used for covariance update
    StateMatrix identity_;

    // Map of Sensors to Observations
    std::unordered_map<int, std::vector<int>> sensor_map;

    // Sensor parameters
    double vn_200_rx_;
    double vn_200_ry_;
    double vn_300_rx_;
    double vn_300_ry_;

    // Car characteristics
    double wheel_radius_;
    double front_axle_;
    double rear_axle_;

    // Pass all current measurements through the mahalanobis threshold in order to reject outliers. Returns
    // a vector with the observation indices that passed the threshold.
    std::vector<int> mahalanobisThreshold();

    // Projects the state vector forward
    void getNextState();

    // Gets the expected values of the observations based on the state prediction
    void predictObservations();

public:
    explicit VelocityEstimator(Handle nh);
    // ~VelocityEstimator();

    // Initializes the Kalman filter matrices and other variables
    void init();

    /* --- Executes the predict step of the algorithm ---
     * Basically projects the state and state error forward by
     * one time step using the EKF algorithm.
     */
    bool predict();

    /* --- Executes the update step of the algorithm ---
     * The expected measurements are calculated from the prediction of the current state
     * and the error and its covariance are found. The outlier measurements are rejected
     * and the rest are used to update the predicted state. For the state covariance update
     * the Joseph form is used to ensure the stability of the solution.
     */
    bool update();

    // Setters
    void setDeltaTime(const double dt) { delta_time_ = dt; }
    void setMeasurements(const ObservationVector& meas) { measurements_ = meas; }
    void setUpdateVector(const std::array<bool, SensorSize>& update) { updated_sensors_ = update; }
    void setInputVector(const InputVector& input) { input_ = input; }

    // Getters
    StateVector getState() { return state_; }
    StateMatrix getStateCovariance() { return state_covariance_; }
};
} //namespace ns_vel_est

#endif // VELOCITY_ESTIMATION_H