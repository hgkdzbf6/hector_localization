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

#include <hector_pose_estimation/measurements/image.h>
#include <hector_pose_estimation/global_reference.h>
#include <hector_pose_estimation/filter/set_filter.h>


namespace hector_pose_estimation {

template class Measurement_<ImageModel>;

ImageModel::ImageModel()
{
  position_stddev_ = 1.0;
  velocity_stddev_ = 1.0;

  parameters().add("image_position_stddev", position_stddev_);
  parameters().add("image_velocity_stddev", velocity_stddev_);
}

ImageModel::~ImageModel() {}

void ImageModel::getMeasurementNoise(NoiseVariance& R, const State&, bool init)
{
  if (init) {
    R(0,0) = R(1,1) = pow(position_stddev_, 2);
    R(2,2) = R(3,3) = pow(velocity_stddev_, 2);
  }
}


void ImageModel::getExpectedValue(MeasurementVector& y_pred, const State& state)
{
	double yaw = state.getYaw();
	double sinyaw,cosyaw;
	sinyaw=sin(yaw);
	cosyaw=cos(yaw);
	y_pred(0) = state.getPosition().x()*cosyaw+state.getPosition().y()*sinyaw;
	y_pred(1) = state.getPosition().y()*cosyaw-state.getPosition().x()*sinyaw;
	y_pred(2) = state.getVelocity().x()*cosyaw+state.getVelocity().y()*sinyaw;
	y_pred(3) = state.getVelocity().y()*cosyaw-state.getVelocity().x()*sinyaw;
}

void ImageModel::getStateJacobian(MeasurementMatrix& C, const State& state, bool init)
{
	if (!init) return; // C is time-constant

	double yaw = state.getYaw();
	double sinyaw,cosyaw;
	sinyaw=sin(yaw);
	cosyaw=cos(yaw);

	if (state.position()) {
		state.position()->cols(C)(0,X) = cosyaw;
		state.position()->cols(C)(0,Y) = sinyaw;
		state.position()->cols(C)(1,X) = -sinyaw;
		state.position()->cols(C)(1,Y) = cosyaw;
	}

	if (state.velocity()){
		state.velocity()->cols(C)(2,X)=cosyaw;
		state.velocity()->cols(C)(2,Y)=sinyaw;
		state.velocity()->cols(C)(3,X)=-sinyaw;
		state.velocity()->cols(C)(3,Y)=cosyaw;
	}
}

Image::Image(const std::string &name)
  : Measurement_<ImageModel>(name)
  , y_(4)
{
}

Image::~Image()
{}

void Image::onReset() {

}

const ImageModel::MeasurementVector& Image::getVector(const Image::Update &update, const State& state) {
//  if (!reference_) {
//    y_.setConstant(0.0/0.0);
//    return y_;
//  }
//
//  reference_->fromWGS84(update.latitude, update.longitude, y_(0), y_(1));
//  reference_->fromNorthEast(update.velocity_north, update.velocity_east, y_(2), y_(3));
	y_=Measurement_<ImageModel>::getVector(update,state);
  return y_;
}

bool Image::prepareUpdate(State &state, const Update &update) {
//  // reset reference position if GPS has not been updated for a while
//  if (timedout()) reference_.reset();
//
//  // find new reference position
//  if (reference_ != GlobalReference::Instance()) {
//    reference_ = GlobalReference::Instance();
//    if (!auto_reference_ && !reference_->hasPosition()) {
//      ROS_ERROR("Cannot use GPS measurements if no reference latitude/longitude is set and %s/auto_reference parameter is false.", name_.c_str());
//      return false;
//    }
//    if (auto_reference_) reference_->setCurrentPosition(state, update.latitude, update.longitude);
//  }

  return true;
}

} // namespace hector_pose_estimation
