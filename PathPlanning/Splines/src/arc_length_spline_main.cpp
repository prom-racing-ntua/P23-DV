#include "arc_length_spline.h"
#include "read_track.h"

int main() {
    path_planning::PointsArray midpoints{ readTrack("../test_tracks/trackdrive_midpoints.txt") };
    midpoints.conservativeResize(midpoints.rows() + 1, midpoints.cols());
    midpoints.row(midpoints.rows() - 1) = midpoints.row(0);

    // path_planning::PointsArray midpoints{ 8, 2 };
    // midpoints <<
    //     0, 0,
    //     3, 1,
    //     8, 6,
    //     2, 10,
    //     -5, 7,
    //     -4, 0,
    //     -2, -1,
    //     0, 0;

    path_planning::ArcLengthSpline spline{midpoints, path_planning::BoundaryCondition::ClosedLoop};

    // std::cout << "Second Derivatives:\n" << spline.getSecondDerivativeAtTargets() << "\n\n";

    std::cout << "At s = 0.67" << '\n';
    std::cout << "Spline coordinates: " << spline.getPoint(0.67) << '\n';
    std::cout << "Spline curvature: " << spline.getCurvature(0.67) << '\n';
    std::cout << "Spline tangent: " << spline.getTangent(0.67) << "\n\n";

    std::cout << "Start: x' = " << spline.getFirstDerivative(0) << " , x'' = " << spline.getSecondDerivative(0) << '\n';
    std::cout << "End: x' = " << spline.getFirstDerivative(1) << " , x'' = " << spline.getSecondDerivative(1) << "\n\n";

    std::ofstream spline_curve("arc_length_spline_points.txt");
    spline_curve << spline.getSplineCurve(2000);
    spline_curve.close();

    std::ofstream spline_data("../../../MPC/MPC_embotech/Data/als_data.txt"); // !! renamed the file for arc length splines !!
    spline_data << spline.getSplineData(1500);
    spline_data.close();

    return 0;
}