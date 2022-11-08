#include "velocity_estimation.h"
#include "velocity_estimation_handler.h"

// TODO: wrap state angles (if you add angles in the state vector)
// TODO: mahalanobis threshold

namespace ns_vel_est
{
// Gets the handle pointer, the filter matrices are initialized in the init() method
VelocityEstimator::VelocityEstimator(Handle nh) : node_handler_(nh) {}

void VelocityEstimator::init() {
    RCLCPP_INFO(node_handler_->get_logger(), "Initializing Velocity Estimator");

    // Get the state initialization from the .yaml file
    auto state_temp = node_handler_->get_parameter("initial_state_vector").as_double_array();
    state_ = Eigen::Map< StateVector >(state_temp.data());
    RCLCPP_INFO_STREAM(node_handler_->get_logger(), "Initialized state vector with values:" << '\n' << state_.transpose());

    // Get the state covariance initialization from the.yaml file
    auto state_cov_temp = node_handler_->get_parameter("initial_state_covariance").as_double_array();
    state_covariance_ = Eigen::Map< StateMatrix >(state_cov_temp.data());
    RCLCPP_INFO_STREAM(node_handler_->get_logger(), "Initialized state covariance with values:" << '\n' << state_covariance_);

    // Get the process noise covariance from the.yaml file
    auto process_noise_temp = node_handler_->get_parameter("process_noise_covariance").as_double_array();
    process_noise_cov_ = Eigen::Map< StateMatrix >(process_noise_temp.data());
    RCLCPP_INFO_STREAM(node_handler_->get_logger(), "Initialized process noise covariance with values:" << '\n' << process_noise_cov_);

    // Get the measurement noise covariance from the.yaml file
    auto measure_noise_temp = node_handler_->get_parameter("measurement_noise_covariance").as_double_array();
    measurement_noise_cov_ = Eigen::Map< MeasurementNoiseMatrix >(measure_noise_temp.data());
    RCLCPP_INFO_STREAM(node_handler_->get_logger(), "Initialized state covariance with values:" << '\n' << measurement_noise_cov_);

    // Get sensor data
    vn_200_rx_ = node_handler_->get_parameter("vectornav-vn-200.x").as_double();
    vn_200_ry_ = node_handler_->get_parameter("vectornav-vn-200.y").as_double();
    vn_300_rx_ = node_handler_->get_parameter("vectornav-vn-300.x").as_double();
    vn_300_ry_ = node_handler_->get_parameter("vectornav-vn-300.y").as_double();

    // Initialize the transfer and measurement function jacobian matrices
    transfer_function_jacobian_ <<
        1, 0, 0, delta_time_, 0, 0,
        0, 1, 0, 0, delta_time_, 0,
        0, 0, 1, 0, 0, delta_time_,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1;
    // TODO: add hall sensor constants
    measurement_function_jacobian_ <<
        1, 0, -vn_300_ry_, 0, 0, 0,
        0, 1, vn_300_rx_, 0, 0, 0,
        0, 0, 1, 0, 0, 0,
        0, 0, 0, 1, 0, -vn_200_ry_,
        0, 0, 0, 0, 1, vn_200_rx_,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0;

    identity_.setIdentity();
}

// The state equations of the filter can be modified through this function
void VelocityEstimator::getNextState() {
    state_(StateVx) = state_(StateVx) + (state_(StateAx) - state_(StateVyaw) * state_(StateVy)) * delta_time_;

    state_(StateVy) = state_(StateVy) + (state_(StateAy) + state_(StateVyaw) * state_(StateVx)) * delta_time_;

    state_(StateVyaw) = state_(StateVyaw) + state_(StateAyaw) * delta_time_;

    state_(StateAx) = state_(StateAx);

    state_(StateAy) = state_(StateAy);

    state_(StateAyaw) = state_(StateAyaw);
}

// The measurement equations of the filter can be modified through this function
void VelocityEstimator::predictObservations() {
    observations_(ObservationVx) = state_(StateVx) - state_(StateVyaw) * vn_300_ry_;

    observations_(ObservationVy) = state_(StateVy) + state_(StateVyaw) * vn_300_rx_;

    observations_(ObservationVyaw) = state_(StateVyaw);

    observations_(ObservationAx) = state_(StateAx) - state_(StateVyaw) * state_(StateVyaw) * vn_200_rx_ - state_(StateAyaw) * vn_200_ry_;

    observations_(ObservationAy) = state_(StateAy) - state_(StateVyaw) * state_(StateVyaw) * vn_200_ry_ + state_(StateAyaw) * vn_200_rx_;

    // TODO: add the hall sensor equations here
}

bool VelocityEstimator::predict() {
    // Set the transfer function jacobian variables according to the current state
    transfer_function_jacobian_(StateVx, StateVy) = -state_(StateVyaw) * delta_time_;
    transfer_function_jacobian_(StateVx, StateVyaw) = -state_(StateVy) * delta_time_;
    transfer_function_jacobian_(StateVy, StateVx) = state_(StateVyaw) * delta_time_;
    transfer_function_jacobian_(StateVy, StateVyaw) = state_(StateVx) * delta_time_;

    // Project state forward (we don't get a new variable, the state_ member holds the predicted state as well)
    getNextState();

    // Possibility of aliasing issues with the next line
    state_covariance_ = transfer_function_jacobian_ * state_covariance_ * transfer_function_jacobian_.transpose() + process_noise_cov_;

    return true;
}

bool VelocityEstimator::update(const std::vector<size_t>& indices) {
    // Set the measurement function jacobian variables according to the current state
    measurement_function_jacobian_(ObservationAx, StateVyaw) = -2 * vn_200_rx_ * state_(StateVyaw);
    measurement_function_jacobian_(ObservationAy, StateVyaw) = -2 * vn_200_ry_ * state_(StateVyaw);
    // TODO: add hall sensor variables

    // Get predicted observations
    predictObservations();

    // Set subset matrices to execute the algorithm
    size_t update_size{ indices.size() };

    Eigen::VectorXd measurement_subset(update_size);
    Eigen::VectorXd observation_subset(update_size);
    Eigen::MatrixXd measurement_cov_subset(update_size, update_size);
    Eigen::MatrixXd measurement_jacobian_subset(update_size, static_cast<int>(StateSize));

    Eigen::MatrixXd kalman_gain(static_cast<int>(StateSize), update_size);
    Eigen::VectorXd innovation(update_size);

    for (size_t i{ 0 }; i < update_size; ++i)
    {
        measurement_subset(i) = measurements_(indices[i]);
        observation_subset(i) = observations_(indices[i]);
        measurement_jacobian_subset.row(i) = measurement_function_jacobian_.row(indices[i]);

        // If we do dynamic measurement covariance, we should check for negative and very small values
        for (size_t j{ 0 }; j < update_size; ++j)
        {
            measurement_cov_subset(i, j) = measurement_noise_cov_(indices[i], indices[j]);
        }
    }

    // Compute innovation
    innovation = measurement_subset - observation_subset;

    // Compute the Kalman Gain
    // pht has size (StateSize, update_size)
    Eigen::MatrixXd pht = state_covariance_ * measurement_jacobian_subset.transpose();
    // hphr_inverse has size (update_size, update_size)
    Eigen::MatrixXd hphr_inverse = (measurement_jacobian_subset * pht + measurement_cov_subset).inverse();
    kalman_gain = pht * hphr_inverse;

    // Mahalanobis threshold
    // TODO: (...)

    // Compute state estimate and state covariance

    // we set no aliasing because the matrices multiplied are different and 
    // element wise multiplication can be done directly into the state vector
    state_.noalias() += kalman_gain * innovation;

    // consider changing the covariance update formula to the Joseph form (I - KH)P(I - KH)' + KRK'
    state_covariance_ = (identity_ - kalman_gain * measurement_jacobian_subset) * state_covariance_;

    return true;
}

/* Runs both steps of the EKF algorithm (predict and update) and takes care of vector subsets based on the update vector
 * input. The arguments are passed in by value although they are computationally expensive to copy. This is done in case
 * we use multithreading to run this node, where passing by reference would possibly be a problem. Research is actually
 * needed to optimize this part of the algorithm in either case.
 */
void VelocityEstimator::runAlgorithm() {
    // Create an array with the indices of the measurements to be updated
    // !size_t (unsigned long) might be too big, maybe we can use just uint
    std::vector<size_t> update_indices;
    for (size_t i{ 0 }; i < ObservationSize; ++i)
    {
        // Maybe check for nan or inf values in the measurements?  
        if (update_vector_[i])
        {
            update_indices.push_back(i);
        }
    }
    // Run predict step
    predict();
    // Pass the indices in the update function so subsets can be created
    update(update_indices);
    node_handler_->publishResults();
}
} // namespace ns_vel_est