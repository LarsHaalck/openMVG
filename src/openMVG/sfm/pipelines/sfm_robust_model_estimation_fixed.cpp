// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "openMVG/sfm/pipelines/sfm_robust_model_estimation.hpp"

#include <array>
#include <iostream>

#include "openMVG/cameras/Camera_Intrinsics.hpp"
#include "openMVG/cameras/Camera_Pinhole.hpp"
#include "openMVG/multiview/motion_from_essential.hpp"
#include "openMVG/multiview/solver_essential_kernel.hpp"
#include "openMVG/multiview/solver_fundamental_kernel.hpp"
#include "openMVG/numeric/numeric.h"
#include "openMVG/robust_estimation/robust_estimator_ACRansac.hpp"
#include "openMVG/robust_estimation/robust_estimator_ACRansacKernelAdaptator.hpp"
#include "openMVG/robust_estimation/robust_estimator_ACRansacKernelAdapterEssentialFixed.hpp"

using namespace openMVG::cameras;
using namespace openMVG::geometry;

namespace openMVG {
namespace sfm {

bool robustRelativePoseFixed
(
    const IntrinsicBase * intrinsics1,
    const IntrinsicBase * intrinsics2,
    const Mat & x1,
    const Mat & x2,
    RelativePose_Info & relativePose_info,
    const std::pair<size_t, size_t> & size_ima1,
    const std::pair<size_t, size_t> & size_ima2,
    const size_t max_iteration_count
)
{
    // x1, x2 are in homogenous coordinates and distorted
    if (!intrinsics1 || !intrinsics2)
        return false;

    const Mat3X
      bearing1 = (*intrinsics1)(x1),
      bearing2 = (*intrinsics2)(x2);

    if (isPinhole(intrinsics1->getType())
        && isPinhole(intrinsics2->getType()))
    {

        // Define the AContrario adaptor to use the 5 point essential matrix solver.
        using KernelType = robust::ACKernelAdaptorEssential<
            openMVG::essential::kernel::FixedRotationSolver,
            openMVG::fundamental::kernel::EpipolarDistanceError,
            Mat3>;

        //KernelType kernel(x1, size_ima1.first, size_ima1.second, x2, intrinsics1);
        //KernelType kernel(x1, size_ima1.first, size_ima1.second, x2);

        KernelType kernel(x1, bearing1, size_ima1.first, size_ima1.second,
                          x2, bearing2, size_ima2.first, size_ima2.second,
                          dynamic_cast<const cameras::Pinhole_Intrinsic*>(intrinsics1)->K(),
                          dynamic_cast<const cameras::Pinhole_Intrinsic*>(intrinsics2)->K());

        // Robustly estimation of the Model and its precision
        const auto ac_ransac_output = robust::ACRANSAC(
        kernel, relativePose_info.vec_inliers,
        max_iteration_count, &relativePose_info.essential_matrix,
        50*relativePose_info.initial_residual_tolerance, false);

        relativePose_info.found_residual_precision = ac_ransac_output.first;

        if (relativePose_info.vec_inliers.size() <
        2.5 * KernelType::Solver::MINIMUM_SAMPLES )
        {
        std::cout << "not enough samples" << std::endl;
        return false; // no sufficient coverage (the model does not support enough samples)
        }
    }
  else
  {
    //TODO: if fixRotTrans print warning because only supported for pinhole cams
  }

  // Compute the bearing vectors but with refined intrinsics (focal + distortion coeffs)
  //const Mat3X
    //bearing1 = (*intrinsics1)(x1),
    //bearing2 = (*intrinsics2)(x2);
  // estimation of the relative poses based on the cheirality test
  Pose3 relative_pose;
  if (!RelativePoseFromEssential(
    bearing1,
    bearing2,
    relativePose_info.essential_matrix,
    relativePose_info.vec_inliers, &relative_pose))
  {
    std::cout << "rel pose from essential failed" << std::endl;
    return false;
  }
  relativePose_info.relativePose = relative_pose;
  return true;
}

} // namespace sfm
} // namespace openMVG
