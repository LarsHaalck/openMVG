#include "openMVG/multiview/solver_essential_fixed_rotation.hpp"
#include "openMVG/multiview/solver_fundamental_kernel.hpp"
#include "openMVG/numeric/eigen_alias_definition.hpp"

#include "ceres/ceres.h"


#include <iostream>

namespace openMVG {

RadialEpipolarCostFunctor::RadialEpipolarCostFunctor(const double* const x,
    const double* const y, double px, double py)
    : m_vk0()
    , m_vk1()
    , m_vk2()
    , m_vk3()
{
    double xc0 = x[0] - px; double xc1 = x[1] - py;
    double xr2 = (xc0 * xc0) + (xc1 * xc1);
    double xr4 = xr2 * xr2;
    double xr6 = xr4 * xr2;
    double yc0 = y[0] - px; double yc1 = y[1] - py;
    double yr2 = (yc0 * yc0) + (yc1 * yc1);
    double yr4 = yr2 * yc0;
    double yr6 = yr4 * yr2;


    m_vk0 << xc0, xc1, yc0, yc1;
    m_vk1 = m_vk0;
    m_vk1.segment<2>(0) *= xr2;
    m_vk1.segment<2>(2) *= yr2;

    m_vk2 = m_vk0;
    m_vk2.segment<2>(0) *= xr4;
    m_vk2.segment<2>(2) *= yr4;

    m_vk3 = m_vk0;
    m_vk3.segment<2>(0) *= xr6;
    m_vk3.segment<2>(2) *= yr6;
}

template <typename T>
bool RadialEpipolarCostFunctor::operator()(
    const T* const k,
    const T* const e,
    T* residual) const
{
    T k0 = k[0];
    T k02 = k0 * k0;
    T k03 = k02 * k0;
    T k05 = k03 * k02;
    T k07 = k05 * k02;

    Eigen::Matrix<T, 4, 1> XYHat =
        k0 * m_vk0
        + (k03 * k[1]) * m_vk1
        + (k05 * k[2]) * m_vk2
        + (k07 * k[3]) * m_vk3;;

    // e is in row-major
    T res0 = XYHat[0] * (e[0] * XYHat[2] + e[1] * XYHat[3] + e[2]);
    T res1 = XYHat[1] * (e[3] * XYHat[2] + e[4] * XYHat[3] + e[5]);
    T res2 = (e[6] * XYHat[2] + e[7] * XYHat[3]);

    residual[0] = res0 + res1 + res2;
    return true;
}

void FixedRotationRelativePose(const Mat2X &x1, const Mat2X &x2, const Mat3& K,
    const cameras::IntrinsicBase* intrinsics, std::vector<Mat3> *E)
{
    // TODO: is normalization (1/sqrt(2) mean distance to origin) needed here?
    /***********************************************************************************
    * 1.) undistort x1, x2 with given intrinsics
    ************************************************************************************/
    auto undistX1 = Mat2X(2, x1.cols());
    auto undistX2 = Mat2X(2, x2.cols());
    for (int i = 0; i < x1.cols(); i++)
    {
        undistX1.col(i) = intrinsics->get_ud_pixel(x1.col(i));
        undistX2.col(i) = intrinsics->get_ud_pixel(x2.col(i));
    }

    /************************************************************************************
    * 2.) compute bearings from x1, x2 (Kinv * x)
    ************************************************************************************/
    auto bearingX1 = (*intrinsics)(undistX1);
    auto bearingX2 = (*intrinsics)(undistX2);

    /***********************************************************************************
    * 3.) esimate initial E using SVD on epicolar constraint matrix
    ************************************************************************************/
    Mat epiConstraint = Mat::Constant(x1.cols(), 9, 0.0);
    fundamental::kernel::EncodeEpipolarEquation(bearingX1, bearingX2, &epiConstraint);
    // cut of last column because we solve for essential matrix with last element = 0
    epiConstraint.conservativeResize(x1.cols(), 8);

    Eigen::JacobiSVD<Mat> USV(epiConstraint, Eigen::ComputeFullU|Eigen::ComputeFullV);
    // intial estimate is last column of V (not transposed)
    Vec initialE = USV.matrixV().rightCols<1>();

    /***********************************************************************************
    * 4.) ceres problem with given intrinsics as initial esimate and SVD solution for E
    ***********************************************************************************/
    // invert radial distortion
    // extract ku, kv
    ceres::Problem problem;
    std::shared_ptr<cameras::IntrinsicBase> intrinsicsInverted(intrinsics->clone());
    invertIntrinsics(intrinsicsInverted);
    auto intrinsicParams = intrinsicsInverted->getParams();

    // copy dist coeffs (offset should be +3 to skip principal and focal)
    // but artifical k0 parameter is inverse of focal we reserve one extra slot
    // and overwrite it in the next line
    std::vector<double> distCoeffs(std::begin(intrinsicParams) + 2,
        std::end(intrinsicParams));
    distCoeffs[0] = 1 / intrinsicParams[0];
    double* distPtr = &distCoeffs[0];
    problem.AddParameterBlock(distPtr, 4);
    problem.SetParameterLowerBound(distPtr, 0, 1e-8); // equals |focal| < 1e8

    // constrain k1, k2, k3 to be in [-1, 1]
    problem.SetParameterLowerBound(distPtr, 1, -1);
    problem.SetParameterLowerBound(distPtr, 2, -1);
    problem.SetParameterLowerBound(distPtr, 3, -1);

    problem.SetParameterUpperBound(distPtr, 1, 1);
    problem.SetParameterUpperBound(distPtr, 2, 1);
    problem.SetParameterUpperBound(distPtr, 3, 1);

    std::vector<double> essentialCoeffs(initialE.data(),
        initialE.data() + initialE.rows() * initialE.cols());
    double* essentialPtr = &essentialCoeffs[0];
    problem.AddParameterBlock(essentialPtr, 8);
    problem.SetParameterLowerBound(essentialPtr, 2, 0.1);

    // add cost function for every observation
    for (int i = 0; i < x1.cols(); i++)
    {
        Vec2 obsX = x1.col(i);
        Vec2 obsY = x2.col(i);

        ceres::CostFunction* costFunction =
            new ceres::AutoDiffCostFunction<RadialEpipolarCostFunctor, 1, 4, 8>(
                new RadialEpipolarCostFunctor(obsX.data(), obsY.data(),
                    intrinsicParams[1], intrinsicParams[2]));
        problem.AddResidualBlock(costFunction, nullptr, distPtr,
            essentialPtr);
    }

    std::cout << "Initial Settings: " << std::endl;
    std::cout << "Dist coeffs: " << std::endl;
    for(const auto& elem : distCoeffs)
    {
        std::cout << elem << ", ";
    }
    std::cout << std::endl;
    std::cout << "Essential coeffs: " << std::endl;
    for(const auto& elem : essentialCoeffs)
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
    std::cout << "Essential coeffs: " << std::endl;
    for(const auto& elem : essentialCoeffs)
    {
        std::cout << elem << ", ";
    }
    std::cout << std::endl;



    // to avoid trivial solution solve for tx > 1 and then for ty > 1 and let RANSAC
    // decide which results fits the matches better


    // 5.) use truncated SVD to force rank = 2 for both essential estimates

    // 6.) return intrinsics


    char bla;
    std::cin >> bla;
}

void invertIntrinsics(std::shared_ptr<cameras::IntrinsicBase> intrinsics)
{
    auto params = intrinsics->getParams();
    double inv0 = -params[3];
    double inv1 = 3 * (params[3] * params[3]) - params[4];
    double inv2 = 8 * params[3] * params[4]
        - 12 * (params[3] * params[3] * params[3]) - params[5];

    params[3] = inv0;
    params[4] = inv1;
    params[5] = inv2;
    intrinsics->updateFromParams(params);
    return;
}
}
