#include "test_slam.h"


namespace ns_slam
{
UnaryFactor::UnaryFactor(gtsam::Key j, double m_x, double m_y, double x, double y, const gtsam::SharedNoiseModel& model)
	: gtsam::NoiseModelFactor1<gtsam::Pose2>(model, j), measured_x(m_x), measured_y(m_y), actual_cone_x(x), actual_cone_y(y) {}

/* The method evaluateError() is required by gtsam to implement a custom factor.
 * It calculates the error of the Factor based on the cone measurement.
 * If the optional Matrix reference argument is specified, the method computes both
 * the error and its derivative at the current values of the car_pose variables.
 */
gtsam::Vector UnaryFactor::evaluateError(const gtsam::Pose2& car_pose, gtsam::OptionalMatrixType H) const {
	// Calculate the error of the cone measurement [range error, angle error]
	gtsam::Rot2 R{ car_pose.rotation() };
    gtsam::Vector2 out{};

	if (H)
	{
		gtsam::Matrix23 error_jacobian{};
		error_jacobian <<
            R.c(), -R.s(), 0.0,
            R.s(), R.c(), 0.0;
			
		*H = error_jacobian;
	}
    out << car_pose.x() - measured_x, car_pose.y() - measured_y;

	return out;
}

gtsam::NonlinearFactor::shared_ptr UnaryFactor::clone() const {
	return std::static_pointer_cast<gtsam::NonlinearFactor>(
		gtsam::NonlinearFactor::shared_ptr(new UnaryFactor(*this)));
}
}