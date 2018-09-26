#ifndef OPENMVG_MULTIVIEW_SOLVER_ESSENTIAL_FIXED_ROTATION_HPP
#define OPENMVG_MULTIVIEW_SOLVER_ESSENTIAL_FIXED_ROTATION_HPP

#include <memory>
#include <vector>

#include "openMVG/numeric/eigen_alias_definition.hpp"
#include "openMVG/cameras/Camera_Intrinsics.hpp"

namespace openMVG {

/*class RadialEpipolarCostFunctor
{
public:
    // fixes alignment issues with the new operator
    // see http://eigen.tuxfamily.org/dox-devel/group__TopicStructHavingEigenMembers.html
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit RadialEpipolarCostFunctor(const double* const x, const double* const y,
        int n, const double* K, const double* Kinv);

    template <typename T>
    bool operator()(const T* const radialParams, T* residual) const;

    static int num_residuals() { return 1; }
private:
    template <typename T>
    Eigen::Matrix<T, 2, 1> undist(const Vec2 p, const T* const k) const;

    Mat2X m_x;
    Mat2X m_y;
    Mat3 m_K;
    Mat3 m_Kinv;
    Vec2 m_p;
};
*/
void FixedRotationRelativePose(const Mat3X &x1, const Mat3X &x2, std::vector<Mat3> *E);


//void invertIntrinsics(std::vector<double>& params, int offset = 0);

}

#endif // OPENMVG_MULTIVIEW_SOLVER_ESSENTIAL_FIXED_ROTATION_HPP
