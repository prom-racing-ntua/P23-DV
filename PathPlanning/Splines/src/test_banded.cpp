#include "solve_tridiagonal.h"

int main() {
    int size{ 5 };
    path_planning::TriDiagonal mat{size, size};
    mat.diagonal(0).setConstant(4);
    mat.diagonal(1).setConstant(1);
    mat.diagonal(-1).setConstant(1);

    Eigen::VectorXd val{size};
    val.setConstant(1.0);

    std::cout << mat << "\n\n";
    std::cout << val << "\n\n";

    Eigen::MatrixXd res{path_planning::solveTridiagonal(mat, val)};
    std::cout << res << "\n\n";

    return 0;
}