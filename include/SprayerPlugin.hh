/*
 * Copyright (C) 2024 Open Source Robotics Foundation
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

#ifndef SPRAYERPLUGIN_HH_
#define SPRAYERPLUGIN_HH_

#include <memory>

#include <gz/sim/System.hh>

namespace gz {
namespace sim {
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems {

/// \brief Agri-drone sprayer plugin with dual spray nozzles, purple particle
/// cloud visualisation, and ground treatment marking.
///
/// ## System Parameters:
///
///   `<link_name>` Name of the base link used for pose tracking.
///   Default: "iris_with_standoffs::base_link"
///
///   `<emitter_left_name>` Name of the left particle emitter entity.
///   Default: "spray_left"
///
///   `<emitter_right_name>` Name of the right particle emitter entity.
///   Default: "spray_right"
///
///   `<spray_topic>` Topic to receive spray on/off commands (gz::msgs::Boolean).
///   Default: "/model/<model_name>/sprayer/cmd"
///
///   `<spray_height_max>` Maximum AGL height (m) at which spraying marks the
///   ground. Default: 15.0
///
///   `<mark_interval>` Minimum travel distance (m) between ground marks.
///   Default: 2.0
///
///   `<mark_radius>` Radius (m) of each circular ground treatment mark.
///   Default: 1.5
///
class SprayerPlugin :
    public System,
    public ISystemConfigure,
    public ISystemPreUpdate
{
  /// \brief Destructor
  public: virtual ~SprayerPlugin();

  /// \brief Constructor
  public: SprayerPlugin();

  // Documentation inherited
  public: void Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm,
                         EventManager &_eventMgr) final;

  // Documentation inherited
  public: void PreUpdate(const UpdateInfo &_info,
                         EntityComponentManager &_ecm) final;

  /// \internal
  /// \brief Private implementation
  private: class Impl;
  private: std::unique_ptr<Impl> impl;
};

}  // namespace systems
}
}  // namespace sim
}  // namespace gz

#endif  // SPRAYERPLUGIN_HH_
