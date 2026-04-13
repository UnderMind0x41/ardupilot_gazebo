/*
 * Copyright (C) 2024 ArduPilot
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
*/

#ifndef GIMBALSTABILIZERPLUGIN_HH_
#define GIMBALSTABILIZERPLUGIN_HH_

#include <memory>
#include <gz/sim/System.hh>

namespace gz {
namespace sim {
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems {

/// \brief Gimbal stabiliser plugin.
///
/// Reads the drone's world-frame attitude each simulation step and publishes
/// compensating roll/pitch angles to keep the camera pointing at a fixed
/// world direction (by default straight down / nadir).
///
/// This plugin publishes to the same topics as the JointPositionController
/// plugins in the iris_with_gimbal model, so no additional subscribers are
/// needed.
///
/// ## SDF Parameters
///
///   `<roll_topic>`   Topic for roll   compensation (default: /gimbal/cmd_roll)
///   `<pitch_topic>`  Topic for pitch  compensation (default: /gimbal/cmd_pitch)
///   `<pitch_offset>` Camera pitch angle when drone is level, in radians.
///                    Default: -1.5708 (-90 deg, camera points straight down).
///
class GimbalStabilizerPlugin :
    public System,
    public ISystemConfigure,
    public ISystemPreUpdate
{
public:
  virtual ~GimbalStabilizerPlugin();
  GimbalStabilizerPlugin();

  void Configure(const Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &_ecm,
                 EventManager &) final;

  void PreUpdate(const UpdateInfo &_info,
                 EntityComponentManager &_ecm) final;

private:
  class Impl;
  std::unique_ptr<Impl> impl;
};

}  // namespace systems
}
}  // namespace sim
}  // namespace gz

#endif  // GIMBALSTABILIZERPLUGIN_HH_
