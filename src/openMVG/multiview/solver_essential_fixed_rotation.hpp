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
    explicit RadialEpipolarCostFunctor(const double* const x, const double* const y,
        double f, double px, double py);

    template <typename T>
    bool operator()(const T* const radialParams, const T* const essentialParams,
        T* residual) const;

    static int num_residuals() { return 1; }
private:
    double m_f;
    Vec4 m_vk0;
    Vec4 m_vk1;
    Vec4 m_vk2;
    Vec4 m_vk3;
public:
    // fixes alignment issues with the new operator
    // see http://eigen.tuxfamily.org/dox-devel/group__TopicStructHavingEigenMembers.html
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};*/

void FixedRotationRelativePose(const Mat3X &x1, const Mat3X &x2, std::vector<Mat3> *E);


//void invertIntrinsics(std::vector<double>& params, int offset = 0);

}

#endif // OPENMVG_MULTIVIEW_SOLVER_ESSENTIAL_FIXED_ROTATION_HPP
