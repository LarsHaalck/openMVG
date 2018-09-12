#include "openMVG/multiview/solver_essential_three_point_fixed_rot_trans.hpp"
#include "openMVG/multiview/solver_fundamental_kernel.hpp"
#include "openMVG/multiview/solver_essential_five_point.hpp"


#include <iostream>

namespace openMVG {

Mat ThreePointsNullspaceBasis(const Mat3X &x1, const Mat3X &x2)
{
    Mat epipolar_constraint = Eigen::Matrix<double, 15, 4>::Constant(0.0);
    for(int i = 0; i < x1.cols() && i < 15; i++)
    {
        auto pt0 = x1.col(i);
        auto pt1 = x2.col(i);

        epipolar_constraint(i, 0) = pt0(0) * pt1(2);
        epipolar_constraint(i, 1) = pt0(1) * pt1(2);
        epipolar_constraint(i, 2) = pt0(2) * pt1(0);
        epipolar_constraint(i, 3) = pt0(2) * pt1(1);
    }
    Eigen::SelfAdjointEigenSolver<Mat> solver
    (epipolar_constraint.transpose() * epipolar_constraint);
    Mat spanningVecsShort = solver.eigenvectors().leftCols<4>();

    Mat spanningVecs = Mat::Zero(9, 4);
    spanningVecs.row(2) = spanningVecsShort.row(0);
    spanningVecs.row(5) = spanningVecsShort.row(1);
    spanningVecs.row(6) = spanningVecsShort.row(2);
    spanningVecs.row(7) = spanningVecsShort.row(3);
    return spanningVecs;
}

Mat ThreePointsPolynomialConstraints(const Mat &E_basis)
{
    // Build the polynomial form of E (equation (8) in Stewenius et al. [1])
    Vec E[3][3];
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            E[i][j] = Vec::Zero(20);
            E[i][j](coef_x) = E_basis(3 * i + j, 0);
            E[i][j](coef_y) = E_basis(3 * i + j, 1);
            E[i][j](coef_z) = E_basis(3 * i + j, 2);
            E[i][j](coef_1) = E_basis(3 * i + j, 3);
        }
    }

    // The constraint matrix.
    Mat M(10, 20);
    int mrow = 0;

    // Determinant constraint det(E) = 0; equation (19) of Nister [2].
    M.row(mrow++) = o2(o1(E[0][1], E[1][2]) - o1(E[0][2], E[1][1]), E[2][0]) +
                  o2(o1(E[0][2], E[1][0]) - o1(E[0][0], E[1][2]), E[2][1]) +
                  o2(o1(E[0][0], E[1][1]) - o1(E[0][1], E[1][0]), E[2][2]);

    // Cubic singular values constraint.
    // Equation (20).
    Vec EET[3][3];
    for (int i = 0; i < 3; ++i)
    {    // Since EET is symmetric, we only compute
        for (int j = 0; j < 3; ++j)
        {  // its upper triangular part.
            if (i <= j)
            {
                EET[i][j] = o1(E[i][0], E[j][0])
                          + o1(E[i][1], E[j][1])
                          + o1(E[i][2], E[j][2]);
            }
            else
                EET[i][j] = EET[j][i];
        }
    }

    // Equation (21).
    Vec (&L)[3][3] = EET;
    const Vec trace  = 0.5 * (EET[0][0] + EET[1][1] + EET[2][2]);
    for (int i = 0; i < 3; ++i)
        L[i][i] -= trace;

    // Equation (23).
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            Vec LEij = o2(L[i][0], E[0][j])
                   + o2(L[i][1], E[1][j])
                   + o2(L[i][2], E[2][j]);
            M.row(mrow++) = LEij;
        }
    }

    return M;
}

void ThreePointFixedRotTransRelativePose(const Mat3X &x1,
                            const Mat3X &x2,
                            std::vector<Mat3> *Es)
{
  // Step 1: Nullspace Extraction.
  const Eigen::Matrix<double, 9, 4> E_basis = ThreePointsNullspaceBasis(x1, x2);

  // Step 2: Constraint Expansion.
  const Eigen::Matrix<double, 10, 20> E_constraints = ThreePointsPolynomialConstraints(E_basis);

  // Step 3: Gauss-Jordan Elimination (done thanks to a LU decomposition).
  using Mat10 = Eigen::Matrix<double, 10, 10>;
  Eigen::FullPivLU<Mat10> c_lu(E_constraints.block<10, 10>(0, 0));
  const Mat10 M = c_lu.solve(E_constraints.block<10, 10>(0, 10));

  // For next steps we follow the matlab code given in Stewenius et al [1].

  // Build action matrix.

  const Mat10 & B = M.topRightCorner<10,10>();
  Mat10 At = Mat10::Zero(10,10);
  At.block<3, 10>(0, 0) = B.block<3, 10>(0, 0);
  At.row(3) = B.row(4);
  At.row(4) = B.row(5);
  At.row(5) = B.row(7);
  At(6,0) = At(7,1) = At(8,3) = At(9,6) = -1;

  Eigen::EigenSolver<Mat10> eigensolver(At);
  const auto& eigenvectors = eigensolver.eigenvectors();
  const auto& eigenvalues = eigensolver.eigenvalues();

  // Build essential matrices for the real solutions.
  Es->reserve(10);
  for (int s = 0; s < 10; ++s) {
    // Only consider real solutions.
    if (eigenvalues(s).imag() != 0) {
      continue;
    }
    Mat3 E;
    Eigen::Map<Vec9 >(E.data()) =
        E_basis * eigenvectors.col(s).tail<4>().real();
    Es->emplace_back(E.transpose());
  }

  double error = 0.0;
  for (int s = 0; s < Es->size(); ++s) {
    double localError = 0.0;
    for(int i = 0; i < x1.cols(); i++)
    {
      localError += x1.col(i).transpose() * Es->at(i) * x2.col(i);
    }
    localError /= x1.cols();
    std::cout << "Local mean error is: " << localError << std::endl;
    error += localError;
  }
  error /= Es->size();

  std::cout << "Global mean error is: " << error << "\n" << std::endl;
}

}
