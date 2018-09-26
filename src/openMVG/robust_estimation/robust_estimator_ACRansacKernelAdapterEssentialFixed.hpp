#ifndef OPENMVG_ROBUST_ESTIMATOR_ACRANSAC_KERNEL_ADAPTATOR_ESSENTIAL_FIXED_HPP
#define OPENMVG_ROBUST_ESTIMATOR_ACRANSAC_KERNEL_ADAPTATOR_ESSENTIAL_FIXED_HPP

#include <vector>

#include "openMVG/multiview/conditioning.hpp"
#include "openMVG/multiview/essential.hpp"
#include "openMVG/numeric/extract_columns.hpp"
#include "openMVG/robust_estimation/robust_estimator_ACRansacKernelAdaptator.hpp"

#include "openMVG/cameras/Camera_Intrinsics.hpp"

namespace openMVG {
namespace robust {

// THIS ADAPTER WILL ONLY WORK FOR CAMERAS WITH SHARED INTRINISCS
// E.G. VIDEO SEQUENCES

struct EssentialFixedModel
{
    Mat3 essentialMatrix;
    std::shared_ptr<cameras::IntrinsicBase> intrinsics;
};

/// Essential matrix Kernel adaptor for the A contrario model estimator
template <typename SolverArg,
  typename ErrorArg,
  typename ModelArg = Mat3>
class ACKernelAdaptorEssentialFixed
{
public:
  using Solver = SolverArg;
  using Model = ModelArg;
  using ErrorT = ErrorArg;

  ACKernelAdaptorEssentialFixed
  (
    const Mat2X& x1, int w1, int h1,
    const Mat2X& x2,
    const cameras::IntrinsicBase* intrinsics1
  ):x1_(x1),
    x2_(x2),
    N1_(Mat3::Identity()),
    logalpha0_(0.0),
    K1_(dynamic_cast<const cameras::Pinhole_Intrinsic*>(intrinsics1)->K()),
    intrinsics1_(intrinsics1)
  {
    assert(2 == x1_.rows());
    assert(x1_.rows() == x2_.rows());
    assert(x1_.cols() == x2_.cols());

    logalpha0_ = ACParametrizationHelper<AContrarioParametrizationType::POINT_TO_LINE>::LogAlpha0(w1, h1, 0.5);
  }

  enum { MINIMUM_SAMPLES = Solver::MINIMUM_SAMPLES };
  enum { MAX_MODELS = Solver::MAX_MODELS };

  void Fit
  (
    const std::vector<uint32_t> &samples,
    std::vector<Model> *models
  ) const
  {
    const auto x1 = ExtractColumns(x1_, samples);
    const auto x2 = ExtractColumns(x2_, samples);
    Solver::Solve(x1, x2, models, this->intrinsics1_);
    //Solver::Solve(x1, x2, this->K1_, models);
  }

  double Error
  (
    uint32_t sample,
    const Model &model
  ) const
  {
    Mat3 F;
    // K_1 or models K? NOT K1
    FundamentalFromEssential(model, K1_, K1_, &F);

    // undistort first
    // apply Kinv (or operator() of estimated model)
    return ErrorT::Error(F, this->x1_.col(sample), this->x2_.col(sample));
  }

  void Errors
  (
    const Model & model,
    std::vector<double> & vec_errors
  ) const
  {
    Mat3 F;
    FundamentalFromEssential(model, K1_, K1_, &F);
    vec_errors.resize(x1_.cols());
    // as above
    for (uint32_t sample = 0; sample < x1_.cols(); ++sample)
      vec_errors[sample] = ErrorT::Error(F, this->x1_.col(sample), this->x2_.col(sample));
  }

  size_t NumSamples() const { return x1_.cols(); }
  void Unnormalize(Model * model) const {}
  double logalpha0() const {return logalpha0_;}
  double multError() const {return ACParametrizationHelper<AContrarioParametrizationType::POINT_TO_LINE>::MultError();}
  Mat3 normalizer1() const {return N1_;}
  Mat3 normalizer2() const {return N1_;}
  double unormalizeError(double val) const { return val; }

private:
  Mat2X x1_, x2_;         // image points
  Mat3 N1_;               // Matrix used to normalize data
  double logalpha0_;      // Alpha0 is used to make the error adaptive to the image size
  Mat3 K1_;              // Intrinsic camera parameter
  const cameras::IntrinsicBase* intrinsics1_; // radial distortion parameters
};

}
}

#endif // OPENMVG_ROBUST_ESTIMATOR_ACRANSAC_KERNEL_ADAPTATOR_ESSENTIAL_FIXED_HPP
