#ifndef OPENMVG_MULTIVIEW_SOLVER_ESSENTIAL_THREE_POINT_FIXED_ROT_TRANS_HPP
#define OPENMVG_MULTIVIEW_SOLVER_ESSENTIAL_THREE_POINT_FIXED_ROT_TRANS_HPP

#include <vector>

#include "openMVG/numeric/eigen_alias_definition.hpp"

namespace openMVG {
/**
 * @brief Computes the relative pose of two calibrated cameras from 3 correspondences.
 *
 * \param x1 Corresponding bearing vectors in the first image. One per column.
 * \param x2 Corresponding bearing vectors in the second image. One per column.
 * \param E  A list of at most 10 candidate essential matrix solutions.
 */
void ThreePointFixedRotTransRelativePose( const Mat3X &x1, const Mat3X &x2,
                             std::vector<Mat3> *E );

/**
* @brief Compute the nullspace of the linear constraints given by the matches.
* @param x1 Corresponding bearing vectors in first camera
* @param x2 Corresponding bearing vectors in second camera
* @return Nullspace that maps x1 points to x2 points
*/
Mat ThreePointsNullspaceBasis( const Mat3X &x1, const Mat3X &x2 );


/**
* Builds the polynomial constraint matrix M.
* @param E_basis Basis essential matrix
* @return polynomial constraint associated to the essential matrix
*/
Mat ThreePointsPolynomialConstraints( const Mat &E_basis );
}

#endif // OPENMVG_MULTIVIEW_SOLVER_ESSENTIAL_THREE_POINT_FIXED_ROT_TRANS_HPP
