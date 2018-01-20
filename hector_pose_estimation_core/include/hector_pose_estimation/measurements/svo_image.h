//=================================================================================================
// Copyright (c) 2011, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef HECTOR_POSE_ESTIMATION_SVO_IMAGE_H
#define HECTOR_POSE_ESTIMATION_SVO_IMAGE_H

#include <hector_pose_estimation/measurement.h>
#include <hector_pose_estimation/global_reference.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace hector_pose_estimation {

class SvoImageModel : public MeasurementModel_<SvoImageModel,2> {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	SvoImageModel();
  virtual ~SvoImageModel();

  virtual SystemStatus getStatusFlags() { return  STATE_POSITION_XY  ; }

  virtual void getMeasurementNoise(NoiseVariance& R, const State&, bool init);
  virtual void getExpectedValue(MeasurementVector& y_pred, const State& state);
  virtual void getStateJacobian(MeasurementMatrix& C, const State& state, bool init);

protected:
  double position_stddev_;
  double velocity_stddev_;
  State::RotationMatrix R;
};

struct SvoImageUpdate : public MeasurementUpdate {
  double x;
  double y;
//  double vx;
//  double vy;
};

//namespace traits {
//  template <> struct Update<ImageModel> { typedef ImageUpdate type; };
//}

extern template class Measurement_<SvoImageModel>;

class SvoImage : public Measurement_<SvoImageModel>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Measurement_<SvoImageModel>::Model;
  using Measurement_<SvoImageModel>::Update;
  enum {MeasurementDimension=Measurement_<SvoImageModel>::MeasurementDimension};
  using Measurement_<SvoImageModel>::MeasurementVector;
  using Measurement_<SvoImageModel>::NoiseVariance;

  SvoImage(const std::string& name = "svo_image");
  virtual ~SvoImage();

  virtual void onReset();

  virtual MeasurementVector const& getVector(const Update &update, const State&);
  virtual bool prepareUpdate(State &state,const Update &update);

private:
  MeasurementVector y_;
  NoiseVariance R_;
};

} // namespace hector_pose_estimation

#endif // HECTOR_POSE_ESTIMATION_SVO_IMAGE_H
