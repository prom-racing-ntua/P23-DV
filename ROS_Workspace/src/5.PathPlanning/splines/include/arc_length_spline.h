#ifndef ARC_LENGTH_SPLINE_H
#define ARC_LENGTH_SPLINE_H

#include <vector>
#include "cubic_spline.h"

namespace path_planning
{
class ArcLengthSpline : public CubicSpline {
private:
    // The linear approximation of the total length of the spline passing from the set target points
    double approximate_curve_length_;
    std::vector<double> target_lengths_;
    std::vector<double> target_indices_;

    virtual LocalSplineIndex getLocalIndex(double parameter) override {
        if (parameter < 0.0 or parameter > 1.0)
        {
            throw std::runtime_error("CubicSpline -> Parameter value is out of range");
        }
        LocalSplineIndex local_index{};

        for (int i{ 1 }; i < spline_size_; i++)
        {
            if (parameter <= target_indices_[i])
            {
                local_index.spline_segment = i - 1;
                local_index.local_parameter = (parameter - target_indices_[i - 1]) / (target_indices_[i] - target_indices_[i - 1]);
                break;
            }
        }

        if (local_index.local_parameter < 1e-10 and local_index.spline_segment != 0)
        {
            local_index.local_parameter = 1.0;
            local_index.spline_segment--;
        }
        return local_index;
    }

public:
    // Constructor
    ArcLengthSpline(const PointsArray& targets, const BoundaryCondition condition) \
        : CubicSpline{ targets, condition } {
        target_lengths_.push_back(0.0);

        for (int i{ 1 }; i < spline_size_; i++)
        {
            target_lengths_.push_back(target_lengths_[i - 1] + \
                std::sqrt(std::pow(target_points_(i, 0) - target_points_(i - 1, 0), 2) + std::pow(target_points_(i, 1) - target_points_(i - 1, 1), 2)));
        }

        approximate_curve_length_ = target_lengths_[spline_size_ - 1];
        for (double target_length : target_lengths_)
        {
            target_indices_.push_back(target_length / approximate_curve_length_);
        }
    }

    // Deconstructor
    ~ArcLengthSpline() {}

    // Returns the approximation of the total spline length
    double getApproximateLength() { return approximate_curve_length_; }
};
} // namespace path_planning
#endif