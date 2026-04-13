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

/// \brief Gimbal stabilisation plugin.
///
/// Reads the drone's world-frame attitude from Gazebo every simulation step
/// and publishes compensating joint-position commands so that the gimbal
/// camera maintains a fixed world-frame direction regardless of drone motion.
///
/// Coordinate-frame analysis for the iris_with_gimbal model
/// -------------------------------------------------------
/// The gimbal_small_3d model is included with pose:
///   <pose degrees="true">0 -0.01 -0.124923 90 0 90</pose>
/// which gives the rotation matrix
///   R = Rz(90°) * Rx(90°) = [[0,0,1],[1,0,0],[0,1,0]]
///
/// The joint axes (in the gimbal model) and their equivalents in drone frame:
///   pitch_joint  axis (1 0 0) -> drone Y  (nose-up/down axis)
///   roll_joint   axis (0 0 1) -> drone X  (bank axis)
///   yaw_joint    axis (0 1 0) -> drone Z  (heading axis)
///
/// Camera looking direction as a function of pitch_joint angle theta
/// (with roll_joint and yaw_joint at 0):
///   looking = [-cos(theta), 0, sin(theta)] in drone frame
///
/// For nadir (looking straight down, drone -Z):
///   -cos(theta) = 0, sin(theta) = -1  ->  theta = -pi/2
///
/// Stabilisation:
///   cmd_pitch = pitchOffset - drone_pitch   (keeps world pitch = pitchOffset)
///   cmd_roll  = -drone_roll                 (keeps world roll  = 0)
///
/// IMPORTANT: We read WorldPose from iris_with_standoffs::base_link (the drone
/// body), NOT from gimbal::base_link. The gimbal attachment has a 90° roll
/// offset in its mounting pose, so reading from it would give Roll()=90° when
/// the drone is level, producing a wrong -90° roll command on the first step
/// and crashing the physics engine.

#include "GimbalStabilizerPlugin.hh"

#include <gz/common/Profiler.hh>
#include <gz/math/Quaternion.hh>
#include <gz/msgs/double.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>

#include <string>

namespace gz {
namespace sim {
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems {

class GimbalStabilizerPlugin::Impl
{
public:
  /// \brief The iris_with_gimbal model entity.
  Model parentModel{kNullEntity};

  /// \brief iris_with_standoffs::base_link entity (the drone body).
  /// Reading WorldPose from this gives clean roll/pitch = 0 when level.
  Entity irisBaseLinkEntity{kNullEntity};

  /// \brief Transport node.
  transport::Node node;

  /// \brief Publishers for roll and pitch joint commands.
  transport::Node::Publisher rollPub;
  transport::Node::Publisher pitchPub;

  /// \brief Topics.
  std::string rollTopic{"/gimbal/cmd_roll"};
  std::string pitchTopic{"/gimbal/cmd_pitch"};

  /// \brief Camera pitch angle (rad) when drone is level.
  /// -pi/2 = camera points straight down (nadir).
  double pitchOffset{-1.5708};

  /// \brief True once the drone base link has been found and WorldPose enabled.
  bool initialized{false};

  /// \brief True if plugin is ready.
  bool valid{false};
};

//////////////////////////////////////////////////
GimbalStabilizerPlugin::~GimbalStabilizerPlugin() = default;
GimbalStabilizerPlugin::GimbalStabilizerPlugin()
    : impl(std::make_unique<Impl>()) {}

//////////////////////////////////////////////////
void GimbalStabilizerPlugin::Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &)
{
  impl->parentModel = Model(_entity);
  if (!impl->parentModel.Valid(_ecm))
  {
    gzerr << "[GimbalStabilizerPlugin] Must be attached to a model.\n";
    return;
  }

  if (_sdf->HasElement("roll_topic"))
    impl->rollTopic = _sdf->Get<std::string>("roll_topic");
  if (_sdf->HasElement("pitch_topic"))
    impl->pitchTopic = _sdf->Get<std::string>("pitch_topic");
  if (_sdf->HasElement("pitch_offset"))
    impl->pitchOffset = _sdf->Get<double>("pitch_offset");

  impl->rollPub  = impl->node.Advertise<msgs::Double>(impl->rollTopic);
  impl->pitchPub = impl->node.Advertise<msgs::Double>(impl->pitchTopic);

  gzmsg << "[GimbalStabilizerPlugin] Stabilising gimbal.\n"
        << "  roll  topic : " << impl->rollTopic  << "\n"
        << "  pitch topic : " << impl->pitchTopic << "\n"
        << "  pitch offset: " << impl->pitchOffset << " rad\n";

  impl->valid = true;
}

//////////////////////////////////////////////////
void GimbalStabilizerPlugin::PreUpdate(
    const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("GimbalStabilizerPlugin::PreUpdate");
  if (!impl->valid || _info.paused) return;

  // ── One-time initialisation ────────────────────────────────────────────────
  if (!impl->initialized)
  {
    // Find iris_with_standoffs sub-model (the drone body, included inside
    // iris_with_gimbal). We read WorldPose from its base_link to get clean
    // roll/pitch angles that are zero when the drone is level.
    Entity irisModel = _ecm.EntityByComponents(
        components::Model(),
        components::ParentEntity(impl->parentModel.Entity()),
        components::Name("iris_with_standoffs"));

    if (irisModel == kNullEntity)
    {
      // Not spawned yet; try again next step.
      return;
    }

    impl->irisBaseLinkEntity = _ecm.EntityByComponents(
        components::Link(),
        components::ParentEntity(irisModel),
        components::Name("base_link"));

    if (impl->irisBaseLinkEntity == kNullEntity)
    {
      gzerr << "[GimbalStabilizerPlugin] iris_with_standoffs::base_link not found.\n";
      impl->valid = false;
      return;
    }

    // Enable WorldPose so Gazebo fills it each step.
    if (!_ecm.Component<components::WorldPose>(impl->irisBaseLinkEntity))
    {
      _ecm.CreateComponent(impl->irisBaseLinkEntity, components::WorldPose());
    }

    impl->initialized = true;
    gzmsg << "[GimbalStabilizerPlugin] Found iris_with_standoffs::base_link, stabilisation active.\n";
    return;  // WorldPose populated from next step onward
  }

  // ── Stabilisation ──────────────────────────────────────────────────────────
  auto *poseComp = _ecm.Component<components::WorldPose>(impl->irisBaseLinkEntity);
  if (!poseComp) return;

  const math::Quaterniond &q = poseComp->Data().Rot();

  // Body-frame roll and pitch of the drone (zero when drone is level).
  const double dronePitch = q.Pitch();
  const double droneRoll  = q.Roll();

  // Commanded joint angles:
  //   pitch_joint rotates around drone-Y (pitch axis) -> cancel drone pitch
  //   roll_joint  rotates around drone-X (roll axis)  -> cancel drone roll
  const double cmdPitch = impl->pitchOffset - dronePitch;
  const double cmdRoll  = -droneRoll;

  msgs::Double rollMsg, pitchMsg;
  rollMsg.set_data(cmdRoll);
  pitchMsg.set_data(cmdPitch);

  impl->rollPub.Publish(rollMsg);
  impl->pitchPub.Publish(pitchMsg);
}

}  // namespace systems
}
}  // namespace sim
}  // namespace gz

GZ_ADD_PLUGIN(
    gz::sim::systems::GimbalStabilizerPlugin,
    gz::sim::System,
    gz::sim::systems::GimbalStabilizerPlugin::ISystemConfigure,
    gz::sim::systems::GimbalStabilizerPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    gz::sim::systems::GimbalStabilizerPlugin,
    "GimbalStabilizerPlugin")
