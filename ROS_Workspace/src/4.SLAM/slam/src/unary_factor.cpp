#include "slam.h"


namespace ns_slam
{
UnaryFactor::UnaryFactor(gtsam::Key j, double range, double theta, double x, double y, const gtsam::SharedNoiseModel& model)
	: gtsam::NoiseModelFactor1<gtsam::Pose2>(model, j), measured_range(range), measured_angle(theta), actual_cone_x(x), actual_cone_y(y) {}

/* The method evaluateError() is required by gtsam to implement a custom factor.
 * It calculates the error of the Factor based on the cone measurement.
 * If the optional Matrix reference argument is specified, the method computes both
 * the error and its derivative at the current values of the car_pose variables.
 */
gtsam::Vector UnaryFactor::evaluateError(const gtsam::Pose2& car_pose, gtsam::OptionalMatrixType H) const {
	// Calculate the error of the cone measurement [range error, angle error]
	gtsam::Vector2 local_cone_position{ measured_range* std::cos(measured_angle), measured_range* std::sin(measured_angle) };
	gtsam::Vector2 global_cone_position{ actual_cone_x, actual_cone_y };
	gtsam::Vector2 global_car_postion{ car_pose.x(), car_pose.y() };
	gtsam::Matrix2 global2local{ car_pose.rotation().transpose() };

	if (H)
	{
		gtsam::Matrix23 error_jacobian{};
		double ct{ std::cos(car_pose.theta()) };
		double st{ std::sin(car_pose.theta()) };

		// Calculated the jacobian of the error vector above with respect to the car x, y and θ.
		error_jacobian <<
			-1, 0, -st * (actual_cone_x - car_pose.x()) + ct * (actual_cone_y - car_pose.y()),
			0, -1, -ct * (actual_cone_x - car_pose.x()) - st * (actual_cone_y - car_pose.y());
		*H = error_jacobian;
	}
	return global2local * (global_cone_position - global_car_postion) - local_cone_position;
}

gtsam::NonlinearFactor::shared_ptr UnaryFactor::clone() const {
	return std::static_pointer_cast<gtsam::NonlinearFactor>(
		gtsam::NonlinearFactor::shared_ptr(new UnaryFactor(*this)));
}
}