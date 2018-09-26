#include "openMVG/multiview/solver_essential_fixed_rotation.hpp"
#include "openMVG/multiview/solver_fundamental_kernel.hpp"
#include "openMVG/numeric/eigen_alias_definition.hpp"
#include "openMVG/cameras/Camera_Pinhole.hpp"

#include "ceres/ceres.h"


#include <iostream>

namespace openMVG {

/*RadialEpipolarCostFunctor::RadialEpipolarCostFunctor(const double* const x,
    const double* const y, int n, const double* K, const double* Kinv)
    : m_x(Eigen::Map<const Mat2X>(x, 2, n))
    , m_y(Eigen::Map<const Mat2X>(y, 2, n))
    , m_K(Eigen::Map<const Mat3>(K))
    , m_Kinv(Eigen::Map<const Mat3>(Kinv))

{
    m_p << m_K(0, 2), m_K(1, 2);
}

template <typename T>
Eigen::Matrix<T, 2, 1> RadialEpipolarCostFunctor::undist(Vec2 p, const T* const k) const
{
    double r2 = p(0) * p(0) + p(1) * p(1);
    double r4 = r2 * r2;
    double r6 = r4 * r2;
    T r_coeff = ( 1. + k[0] * r2 + k[1] * r4 + k[2] * r6 );
    return ( p * r_coeff );
}*/

/*template <typename T>
bool RadialEpipolarCostFunctor::operator()(
    const T* const k,
    T* residual) const
{
    typedef Eigen::Matrix<T, 2, Eigen::Dynamic> Mat2XT;
    typedef Eigen::Matrix<T, 3, Eigen::Dynamic> Mat3XT;
    typedef Eigen::Matrix<T, 2, 1> Vec2T;
    typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> MatT;*/

    /***********************************************************************************
    * 1.) undistort x1, x2 with given intrinsics
    ************************************************************************************/
    /*double f, finv, px, py;
    f = m_K(0, 0);
    finv = m_Kinv(0, 0);
    px = m_K(0, 2);
    py = m_K(1, 2);

    Mat2XT undistX(2, m_x.cols());
    Mat2XT undistY(2, m_y.cols());
    for (int i = 0; i < m_x.cols(); i++)
    {
        Vec2 currUndistX = finv * (m_x.col(i) - m_p);
        Vec2 currUndistY = finv * (m_y.col(i) - m_p);
        Vec2T currUndistXT = undist(currUndistX, k);
        Vec2T currUndistYT = undist(currUndistY, k);
        currUndistXT = f * currUndistXT + m_p;
        currUndistYT = f * currUndistYT + m_p;

        undistX.col(i) = currUndistXT;
        undistY.col(i) = currUndistYT;
    }*/

    /************************************************************************************
    * 2.) compute bearings from x1, x2 (Kinv * x)
    ************************************************************************************/
    /*Mat3XT bearingX = m_Kinv * undistX.colwise().homogeneous();
    Mat3XT bearingY = m_Kinv * undistY.colwise().homogeneous();*/

    /************************************************************************************
    * 3.) compute E by SVD
    ************************************************************************************/
    /*MatT epiConstraint = MatT::Constant(m_x.cols(), 9, static_cast<T>(0.0));
    fundamental::kernel::EncodeEpipolarEquation(bearingX, bearingY, &epiConstraint);
    // cut of last column because we solve for essential matrix with last element = 0
    MatT epiConstraintCut = epiConstraint.block(0, 0, m_x.cols(), 8);
    //epiConstraint.conservativeResize(m_x.cols(), 8);

    Eigen::JacobiSVD<MatT> USV(epiConstraintCut, Eigen::ComputeFullU|Eigen::ComputeFullV);
    // intial estimate is last column of V (not transposed)
    Eigen::Matrix<T, Eigen::Dynamic, 1> initialE = USV.matrixV().rightCols(1);
    //Eigen::Matrix<T, Eigen::Dynamic, 1> initialE = Eigen::Matrix<T, Eigen::Dynamic, 1>::Zero(8);

    Eigen::Matrix<T, 3, 3> essentialEstimate = Eigen::Matrix<T, 3, 3>::Zero();
    essentialEstimate(0, 0) = initialE(0);
    essentialEstimate(0, 1) = initialE(1);
    essentialEstimate(0, 2) = initialE(2);
    essentialEstimate(1, 0) = initialE(3);
    essentialEstimate(1, 1) = initialE(4);
    essentialEstimate(1, 2) = initialE(5);
    essentialEstimate(2, 0) = initialE(6);
    essentialEstimate(2, 1) = initialE(7);

    // force rk(E) = 2 by setting the last singular value to zero
    Eigen::JacobiSVD<Eigen::Matrix<T, 3, 3>> USV2(essentialEstimate,
        Eigen::ComputeFullU|Eigen::ComputeFullV);

    Eigen::Matrix<T, 3, 1> singularValues = USV2.singularValues();
    singularValues(2) = static_cast<T>(0.0);
    essentialEstimate = USV2.matrixU() * singularValues.asDiagonal() * USV2.matrixV().transpose();

    // evaluate error
    Eigen::Matrix<T, Eigen::Dynamic, 1> evals = (bearingX.transpose() * essentialEstimate * bearingY).diagonal();
    for(int i = 0; i < 10; i++)
        residual[i] = evals[i];*/

    /*return true;
}*/

void FixedRotationRelativePose(const Mat3X &x1, const Mat3X &x2, std::vector<Mat3> *E)
{
    // invert radial distortion
    // extract ku, kv
    /*ceres::Problem problem;
    auto intrinsicParams = intrinsics->getParams();

    // copy dist coeffs (offset should be +3 to skip principal and focal)
    std::vector<double> distCoeffs(std::begin(intrinsicParams) + 3,
        std::end(intrinsicParams));
    invertIntrinsics(distCoeffs);
    double* distPtr = &distCoeffs[0];
    problem.AddParameterBlock(distPtr, 3);


    // add one cost function for all observations

    //intrinsicParams[0,1,2] equals focal, px, py
    ceres::CostFunction* costFunction =
        new ceres::AutoDiffCostFunction<RadialEpipolarCostFunctor, 10, 3>(
            new RadialEpipolarCostFunctor(x1.data(), x2.data(), x1.cols(),
                dynamic_cast<const cameras::Pinhole_Intrinsic*>(intrinsics)->K().data(),
                dynamic_cast<const cameras::Pinhole_Intrinsic*>(intrinsics)->Kinv().data()));
    problem.AddResidualBlock(costFunction, nullptr, distPtr);

    std::cout << "Initial Settings: " << std::endl;
    std::cout << "Dist coeffs: " << std::endl;
    for(const auto& elem : distCoeffs)
    {
        std::cout << elem << ", ";
    }
    std::cout << std::endl;

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR; // because few parameters
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";

    std::cout << "After solving Settings: " << std::endl;
    std::cout << "Dist coeffs: " << std::endl;
    for(const auto& elem : distCoeffs)
    {
        std::cout << elem << ", ";
    }
    std::cout << std::endl;

    std::cout << "Inverted: " << std::endl;
    invertIntrinsics(distCoeffs);
    for(const auto& elem : distCoeffs)
    {
        std::cout << elem << ", ";
    }
    std::cout << std::endl;*/

    Mat epiConstraint = Mat::Constant(x1.cols(), 9, 0.0);
    fundamental::kernel::EncodeEpipolarEquation(x1, x2, &epiConstraint);
    // cut of last column because we solve for essential matrix with last element = 0
    epiConstraint.conservativeResize(x1.cols(), 8);

    Eigen::JacobiSVD<Mat> USV(epiConstraint, Eigen::ComputeFullU|Eigen::ComputeFullV);
    // intial estimate is last column of V (not transposed)
    Vec initialE = USV.matrixV().rightCols<1>();

    Mat3 essentialEstimate = Mat3::Zero();
    essentialEstimate(0, 0) = initialE(0);
    essentialEstimate(0, 1) = initialE(1);
    essentialEstimate(0, 2) = initialE(2);
    essentialEstimate(1, 0) = initialE(3);
    essentialEstimate(1, 1) = initialE(4);
    essentialEstimate(1, 2) = initialE(5);
    essentialEstimate(2, 0) = initialE(6);
    essentialEstimate(2, 1) = initialE(7);

    // force rk(E) = 2 by setting the last singular value to zero
    Eigen::JacobiSVD<Mat3> USV2(essentialEstimate, Eigen::ComputeFullU|Eigen::ComputeFullV);
    Vec3 singularValues = USV2.singularValues();
    singularValues(2) = 0.0;
    essentialEstimate = USV2.matrixU() * singularValues.asDiagonal() * USV2.matrixV().transpose();

    E->resize(1);
    E->push_back(essentialEstimate);
    return;
}



/*
*/

void invertIntrinsics(std::vector<double>& params, int of)
{
    double inv0 = -params[of];
    double inv1 = 3 * (params[of] * params[of]) - params[1 + of];
    double inv2 = 8 * params[of] * params[1 + of]
        - 12 * (params[of] * params[of] * params[of]) - params[2 + of];

    params[of] = inv0;
    params[1 + of] = inv1;
    params[2 + of] = inv2;
    return;
}
}
