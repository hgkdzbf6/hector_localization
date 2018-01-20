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

#include <hector_pose_estimation/system.h>
#include <hector_pose_estimation/filter/ekf.h>

namespace hector_pose_estimation {

System::System(const std::string &name)
  : name_(name)
  , status_flags_(0)
{
}

System::~System() {
}

void System::getPrior(State &state) const
{
  getModel()->getPrior(state);
}

bool System::init(PoseEstimation& estimator, State& state)
{
  if (!getModel() || !getModel()->init(estimator, *this, state)) return false;
  return true;
}

void System::cleanup()
{
  if (getModel()) getModel()->cleanup();
}

void System::reset(State& state)
{
  if (getModel()) getModel()->reset(state);
  status_flags_ = 0;
}

bool System::active(const State& state) {
  bool active = (!getModel() || getModel()->active(state));
  if (!active) status_flags_ = 0;
  return active;
}

bool System::update(double dt) {
  if (!filter() || !active(filter()->state())) return false;

  if (getModel()) status_flags_ = getModel()->getStatusFlags(filter()->state());
  // ROS_INFO("system name: %s . status_flag: %d",getName().c_str(),status_flags_);
  if (!this->updateImpl(dt)) return false;
  filter()->state().updated();

  updated();
  return true;
}

void System::updated() {
}

bool System::limitState(State &state) {
  return getModel()->limitState(state);
}

} // namespace hector_pose_estimation
