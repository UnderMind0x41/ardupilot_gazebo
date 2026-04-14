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

#include "SprayerPlugin.hh"

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/particle_emitter.pb.h>

#include <atomic>
#include <cmath>
#include <functional>
#include <string>

#include <gz/plugin/Register.hh>
#include <gz/common/Profiler.hh>
#include <gz/math/Pose3.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/transport/Node.hh>

namespace gz {
namespace sim {
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems {

//////////////////////////////////////////////////
class SprayerPlugin::Impl
{
  // ── Callbacks ──────────────────────────────────────────────────────────── //

  /// \brief Toggle spray state from external command topic.
  public: void OnSprayCmd(const msgs::Boolean &_msg);

  // ── Helpers ────────────────────────────────────────────────────────────── //

  /// \brief Publish a ParticleEmitter message to enable/disable one emitter.
  public: void SetEmitting(transport::Node::Publisher &pub, bool emitting);

  /// \brief Spawn a flat purple disc at (x, y) to mark treated ground.
  public: void SpawnGroundMark(double x, double y);

  // ── Persistent state ───────────────────────────────────────────────────── //

  public: Model model{kNullEntity};
  public: World world{kNullEntity};
  public: std::string worldName;

  // ── Emitter topics (set by ParticleEmitter system, addressed by path)  ── //
  //  Format: /model/<name>/link/<link>/particle_emitter/<emitter>/cmd
  public: transport::Node::Publisher pubLeft;
  public: transport::Node::Publisher pubRight;

  // ── Config ─────────────────────────────────────────────────────────────── //

  public: std::string sprayTopic;
  public: double sprayHeightMax{15.0};
  public: double markInterval{2.0};
  public: double markRadius{1.5};

  // ── Left/right link and emitter names (for topic construction) ─────────── //

  public: std::string leftLinkName{"sprayer_left"};
  public: std::string rightLinkName{"sprayer_right"};
  public: std::string leftEmitterName{"spray_left"};
  public: std::string rightEmitterName{"spray_right"};

  // ── Runtime flags ──────────────────────────────────────────────────────── //

  public: std::atomic<bool> spraying{false};
  public: std::atomic<bool> emitterStateChanged{false};
  public: bool validConfig{false};

  // ── Ground-mark tracking ───────────────────────────────────────────────── //

  public: double lastMarkX{1.0e9};
  public: double lastMarkY{1.0e9};
  public: int markId{0};

  // ── Transport ──────────────────────────────────────────────────────────── //

  public: transport::Node node;
};

//////////////////////////////////////////////////
void SprayerPlugin::Impl::OnSprayCmd(const msgs::Boolean &_msg)
{
  this->spraying = _msg.data();
  this->emitterStateChanged = true;
  gzmsg << "SprayerPlugin: spray " << (_msg.data() ? "ON" : "OFF") << "\n";
}

//////////////////////////////////////////////////
void SprayerPlugin::Impl::SetEmitting(transport::Node::Publisher &pub,
                                      bool emitting)
{
  msgs::ParticleEmitter msg;
  msg.mutable_emitting()->set_data(emitting);
  pub.Publish(msg);
}

//////////////////////////////////////////////////
void SprayerPlugin::Impl::SpawnGroundMark(double x, double y)
{
  std::string sdf =
      "<?xml version=\"1.0\" ?>"
      "<sdf version=\"1.6\">"
      "<model name=\"spray_mark_" + std::to_string(this->markId++) + "\">"
      "  <static>true</static>"
      "  <pose>" + std::to_string(x) + " " + std::to_string(y) + " 0.005 0 0 0</pose>"
      "  <link name=\"mark_link\">"
      "    <visual name=\"mark_visual\">"
      "      <cast_shadows>false</cast_shadows>"
      "      <geometry>"
      "        <cylinder>"
      "          <radius>" + std::to_string(this->markRadius) + "</radius>"
      "          <length>0.01</length>"
      "        </cylinder>"
      "      </geometry>"
      "      <material>"
      "        <ambient>0.45 0.0 0.65 0.55</ambient>"
      "        <diffuse>0.45 0.0 0.65 0.55</diffuse>"
      "        <specular>0.1 0.0 0.15 0.3</specular>"
      "        <emissive>0.05 0.0 0.1 0.2</emissive>"
      "      </material>"
      "      <transparency>0.45</transparency>"
      "    </visual>"
      "  </link>"
      "</model>"
      "</sdf>";

  msgs::EntityFactory req;
  req.set_sdf(sdf);

  std::function<void(const msgs::Boolean &, bool)> cb =
      [](const msgs::Boolean &, bool) {};
  this->node.Request("/world/" + this->worldName + "/create", req, cb);
}

//////////////////////////////////////////////////
//////////////////////////////////////////////////
SprayerPlugin::~SprayerPlugin() = default;

//////////////////////////////////////////////////
SprayerPlugin::SprayerPlugin()
    : impl(std::make_unique<SprayerPlugin::Impl>())
{
}

//////////////////////////////////////////////////
void SprayerPlugin::Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &)
{
  // ── Model & world ──────────────────────────────────────────────────────── //

  this->impl->model = Model(_entity);
  if (!this->impl->model.Valid(_ecm))
  {
    gzerr << "SprayerPlugin must be attached to a model.\n";
    return;
  }
  const std::string modelName = this->impl->model.Name(_ecm);

  this->impl->world = World(_ecm.EntityByComponents(components::World()));
  if (!this->impl->world.Valid(_ecm))
  {
    gzerr << "SprayerPlugin: world entity not found.\n";
    return;
  }
  if (this->impl->world.Name(_ecm).has_value())
    this->impl->worldName = this->impl->world.Name(_ecm).value();

  // ── SDF parameters ─────────────────────────────────────────────────────── //

  if (_sdf->HasElement("left_link_name"))
    this->impl->leftLinkName = _sdf->Get<std::string>("left_link_name");
  if (_sdf->HasElement("right_link_name"))
    this->impl->rightLinkName = _sdf->Get<std::string>("right_link_name");
  if (_sdf->HasElement("left_emitter_name"))
    this->impl->leftEmitterName = _sdf->Get<std::string>("left_emitter_name");
  if (_sdf->HasElement("right_emitter_name"))
    this->impl->rightEmitterName = _sdf->Get<std::string>("right_emitter_name");

  this->impl->sprayTopic = "/model/" + modelName + "/sprayer/cmd";
  if (_sdf->HasElement("spray_topic"))
    this->impl->sprayTopic = _sdf->Get<std::string>("spray_topic");

  if (_sdf->HasElement("spray_height_max"))
    this->impl->sprayHeightMax = _sdf->Get<double>("spray_height_max");
  if (_sdf->HasElement("mark_interval"))
    this->impl->markInterval = _sdf->Get<double>("mark_interval");
  if (_sdf->HasElement("mark_radius"))
    this->impl->markRadius = _sdf->Get<double>("mark_radius");

  // ── Construct particle-emitter control topics ──────────────────────────── //
  //
  // gz-sim-particle-emitter-system automatically subscribes every
  // <particle_emitter> entity to:
  //   /model/<model>/link/<link>/particle_emitter/<emitter>/cmd
  // Publishing gz.msgs.ParticleEmitter messages to those topics is the
  // documented way to control emitters at runtime.
  //
  const std::string base = "/model/" + modelName;
  const std::string topicLeft  = base + "/link/" + this->impl->leftLinkName  +
                                  "/particle_emitter/" + this->impl->leftEmitterName  + "/cmd";
  const std::string topicRight = base + "/link/" + this->impl->rightLinkName +
                                  "/particle_emitter/" + this->impl->rightEmitterName + "/cmd";

  this->impl->pubLeft  =
      this->impl->node.Advertise<msgs::ParticleEmitter>(topicLeft);
  this->impl->pubRight =
      this->impl->node.Advertise<msgs::ParticleEmitter>(topicRight);

  // ── Subscribe to spray on/off command ─────────────────────────────────── //

  this->impl->node.Subscribe(
      this->impl->sprayTopic,
      &SprayerPlugin::Impl::OnSprayCmd, this->impl.get());

  gzmsg << "SprayerPlugin [" << modelName << "]:\n"
        << "  spray topic   : " << this->impl->sprayTopic << "\n"
        << "  emitter left  : " << topicLeft  << "\n"
        << "  emitter right : " << topicRight << "\n"
        << "  mark interval : " << this->impl->markInterval << " m\n"
        << "  mark radius   : " << this->impl->markRadius   << " m\n";

  this->impl->validConfig = true;
}

//////////////////////////////////////////////////
void SprayerPlugin::PreUpdate(const UpdateInfo &_info,
                              EntityComponentManager &_ecm)
{
  GZ_PROFILE("SprayerPlugin::PreUpdate");

  if (!this->impl->validConfig || _info.paused) return;

  // ── Toggle particle emitters when spray command received ──────────────── //

  if (this->impl->emitterStateChanged.exchange(false))
  {
    const bool spraying = this->impl->spraying.load();
    this->impl->SetEmitting(this->impl->pubLeft,  spraying);
    this->impl->SetEmitting(this->impl->pubRight, spraying);
  }

  // ── Drop ground marks while actively spraying ─────────────────────────── //

  if (!this->impl->spraying.load()) return;

  // Use the model entity's world pose to track drone position.
  // worldPose() from gz/sim/Util.hh traverses the entity hierarchy and
  // works correctly for a model that is a direct child of the world.
  const auto pose = worldPose(this->impl->model.Entity(), _ecm);
  const double x = pose.Pos().X();
  const double y = pose.Pos().Y();
  const double z = pose.Pos().Z();

  // Only mark when below max ceiling and actually in flight.
  if (z > this->impl->sprayHeightMax || z < 0.3) return;

  const double dx   = x - this->impl->lastMarkX;
  const double dy   = y - this->impl->lastMarkY;
  const double dist = std::sqrt(dx * dx + dy * dy);

  if (dist >= this->impl->markInterval)
  {
    this->impl->SpawnGroundMark(x, y);
    this->impl->lastMarkX = x;
    this->impl->lastMarkY = y;
  }
}

}  // namespace systems
}
}  // namespace sim
}  // namespace gz

GZ_ADD_PLUGIN(
    gz::sim::systems::SprayerPlugin,
    gz::sim::System,
    gz::sim::systems::SprayerPlugin::ISystemConfigure,
    gz::sim::systems::SprayerPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    gz::sim::systems::SprayerPlugin,
    "SprayerPlugin")
