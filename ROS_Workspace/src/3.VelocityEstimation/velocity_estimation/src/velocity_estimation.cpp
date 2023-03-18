#include "velocity_estimation.h"
#include "velocity_estimation_handler.h"

// NOTE: wrap state angles in (-pi, pi) range. (only if we add angles in the state vector)
// NOTE: If we do dynamic measurement covariance, we should check for negative and very small values.

// IDEA: sort of a measurement object for each sensor containing measurement values, thresholds and other info

namespace ns_vel_est
{
// Gets the handle pointer, the filter matrices are initialized in the init() method
VelocityEstimator::VelocityEstimator(Handle nh): node_handler_(nh) {}

void VelocityEstimator::init() {
    // RCLCPP_INFO(node_handler_->get_logger(), "Initializing Velocity Estimator");

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
    RCLCPP_INFO_STREAM(node_handler_->get_logger(), "Initialized measurement noise covariance with values:" << '\n' << measurement_noise_cov_);

    // Get sensor data
    vn_200_rx_ = node_handler_->get_parameter("vectornav-vn-200.x").as_double();
    vn_200_ry_ = node_handler_->get_parameter("vectornav-vn-200.y").as_double();
    vn_300_rx_ = node_handler_->get_parameter("vectornav-vn-300.x").as_double();
    vn_300_ry_ = node_handler_->get_parameter("vectornav-vn-300.y").as_double();

    // Get car characteristics
    wheel_radius_ = node_handler_->get_parameter("wheel_radius").as_double();
    front_axle_ = node_handler_->get_parameter("front_axle").as_double();
    rear_axle_ = node_handler_->get_parameter("rear_axle").as_double();

    // A hash map of all the sensors used for the Kalman Filter with their respective measurements as defined in
    // the velocity_estimation_common.h file. New measurements and sensors must be added here in order to pass
    // through the mahalanobis threshold for outlier detection.
    sensor_map.clear();
    sensor_map.insert(std::make_pair(VelocitySensor, std::vector<int>{ObservationVx, ObservationVy}));
    sensor_map.insert(std::make_pair(Gyroscope, std::vector<int>{ObservationVyaw}));
    sensor_map.insert(std::make_pair(Accelerometer, std::vector<int>{ObservationAx, ObservationAy}));
    sensor_map.insert(std::make_pair(FrontWheelEncoders, std::vector<int>{ObservationVhall_front}));
    sensor_map.insert(std::make_pair(RearWheelEncoders, std::vector<int>{ObservationVhall_rear}));

    // Initialize the transfer and measurement function jacobian matrices
    transfer_function_jacobian_ <<
        1.0, 0.0, 0.0, delta_time_, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0, delta_time_, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0, delta_time_,
        0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

    measurement_function_jacobian_ <<
        1.0, 0.0, -vn_300_ry_, 0.0, 0.0, 0.0,
        0.0, 1.0, vn_300_rx_, 0.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.0, 0.0, -vn_200_ry_,
        0.0, 0.0, 0.0, 0.0, 1.0, vn_200_rx_,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        9.5493 / wheel_radius_, 0.0, 0.0, 0.0, 0.0, 0.0;

    identity_.setIdentity();
}

std::vector<int> VelocityEstimator::mahalanobisThreshold() {
    std::vector<int> passed_idx{};
    // TODO: Check again if threshold works properly (for now it will be disabled)
    // Iterate through all sensors in the sensor map. Note that all the keys of the map are actually ints
    for (int i{ 0 }; i < SensorSize; ++i)
    {
        // Check if sensor has output a measurement and is to be tested for outliers
        if (updated_sensors_[i])
        {
            // Get the observations that correspond to current sensor set their number and the matrices needed for 
            // the execution of the algorithm.
            std::vector<int> idx{ sensor_map[i] };
            size_t idx_size{ idx.size() };
            Eigen::VectorXd r(idx_size);
            Eigen::MatrixXd S_inv(idx_size, idx_size);

            // Sample the innovation and covariance matrices created in the update function to get the corresponding
            // values of the current sensor.
            for (size_t i{ 0 }; i < idx_size; ++i)
            {
                r(i) = innovation_(idx[i]);
                for (size_t j{ 0 }; j < idx_size; ++j)
                {
                    S_inv(i, j) = innovation_cov_inverse_(idx[i], idx[j]);
                }
            }
            // Compute the mahalanobis distance of the measurements from the distribution of the prediction
            double mahalanobis_dist{ r.transpose() * S_inv * r };

            // TODO: ros parameter method to get each of the threshold values
            double thresh{ 100 };

            // Compare distance to threshold. If distance is smaller the measurement is accepted and the observation indices
            // are appended to the output vector.
            // if (mahalanobis_dist < thresh)
            {
                std::move(idx.begin(), idx.end(), std::back_inserter(passed_idx));
            }
        }
    }
    return passed_idx;
}

// The state equations of the filter can be modified through this function
void VelocityEstimator::getNextState() {
    state_(StateVx) = state_(StateVx) + (state_(StateAx) + state_(StateVyaw) * state_(StateVy)) * delta_time_;

    state_(StateVy) = state_(StateVy) + (state_(StateAy) - state_(StateVyaw) * state_(StateVx)) * delta_time_;

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

    // 9.5493 is to convert rad/s to rpm because the measurement from the wheel encoders is in rpm while in the state vector is in rad/s
    observations_(ObservationVhall_front) = (state_(StateVx) * std::cos(input_(InputSteering)) + (state_(StateVy) - \
        front_axle_ * state_(StateVyaw)) * std::sin(input_(InputSteering))) / wheel_radius_ * 9.5493;

    observations_(ObservationVhall_rear) = state_(StateVx) / wheel_radius_ * 9.5493;
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

bool VelocityEstimator::update() {
    // Set the measurement function jacobian variables according to the current state
    measurement_function_jacobian_(ObservationAx, StateVyaw) = -2 * vn_200_rx_ * state_(StateVyaw);
    measurement_function_jacobian_(ObservationAy, StateVyaw) = -2 * vn_200_ry_ * state_(StateVyaw);
    measurement_function_jacobian_(ObservationVhall_front, StateVx) = std::cos(input_(InputSteering)) * 9.5493 / wheel_radius_;
    measurement_function_jacobian_(ObservationVhall_front, StateVy) = std::sin(input_(InputSteering)) * 9.5493 / wheel_radius_;
    measurement_function_jacobian_(ObservationVhall_front, StateVyaw) = -front_axle_ * std::sin(input_(InputSteering)) * 9.5493 / wheel_radius_;

    // Get predicted observations
    predictObservations();

    // In order to pass through the mahalanobis threshold the measurement residual or innovation from the predicted
    // measurements is needed along with its covariance. We calculate the whole vector and matrix for now even with
    // measurements we didn't get and later we'll subset said objects with only the ones that passed the threshold.
    innovation_ = measurements_ - observations_;
    //RCLCPP_INFO_STREAM(node_handler_->get_logger(), "Innovation Vector:" << "\n" << innovation_);
    pht_ = state_covariance_ * measurement_function_jacobian_.transpose();
    innovation_cov_inverse_ = (measurement_function_jacobian_ * pht_ + measurement_noise_cov_).inverse();

    // Pass measurements through the mahalanobis threshold, only the indices in the returned vector are going to be
    // used for the update of the estimation.
    std::vector<int> passed_indices{ mahalanobisThreshold() };

    // See if no measurement passed the threshold and skip the update step
    if (passed_indices.empty())
    {
        RCLCPP_WARN(node_handler_->get_logger(), "\nNo valid measurement for the update of the estimate. Dead-reckoning...");
        return false;
    }

    // Set subset matrices to execute the update algorithm
    size_t update_size{ passed_indices.size() };

    Eigen::VectorXd innovation_subset(update_size);
    Eigen::MatrixXd pht_subset(static_cast<int>(StateSize), update_size);
    Eigen::MatrixXd innovation_cov_inv_subset(update_size, update_size);
    Eigen::MatrixXd measurement_jacobian_subset(update_size, static_cast<int>(StateSize));
    Eigen::MatrixXd measurement_cov_subset(update_size, update_size);
    Eigen::MatrixXd kalman_gain(static_cast<int>(StateSize), update_size);

    // Fill them with the correct values
    for (size_t i{ 0 }; i < update_size; ++i)
    {
        innovation_subset(i) = innovation_(passed_indices[i]);
        pht_subset.col(i) = pht_.col(passed_indices[i]);
        measurement_jacobian_subset.row(i) = measurement_function_jacobian_.row(passed_indices[i]);
        for (size_t j{ 0 }; j < update_size; ++j)
        {
            innovation_cov_inv_subset(i, j) = innovation_cov_inverse_(passed_indices[i], passed_indices[j]);
            measurement_cov_subset(i, j) = measurement_noise_cov_(passed_indices[i], passed_indices[j]);
        }
    }
    // Compute the Kalman Gain
    // We set no aliasing because the matrices multiplied are different and element wise multiplication can be done 
    // directly into the kalman gain.
    kalman_gain.noalias() = pht_subset * innovation_cov_inv_subset;

    // Compute state estimate and state covariance.
    state_.noalias() += kalman_gain * innovation_subset;

    StateMatrix A{ identity_ - kalman_gain * measurement_jacobian_subset };
    state_covariance_ = A * state_covariance_ * A.transpose() + kalman_gain * measurement_cov_subset * kalman_gain.transpose();

    return true;
}
} // namespace ns_vel_est