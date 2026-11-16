#include <atomic>
#include <chrono>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/vector3d.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>

namespace cuadc::sim
{
class PayloadReleaseSystem final:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPostUpdate,
  public gz::sim::ISystemReset
{
  public: void Configure(
      const gz::sim::Entity &,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &) override
  {
    this->threshold = _sdf->Get<double>("threshold", 0.75).first;
    this->delaySeconds = _sdf->Get<double>("delay_s", 0.0).first;
    if (this->delaySeconds < 0.0)
      this->delaySeconds = 0.0;
    const auto input =
        _sdf->Get<std::string>("input_topic", "/cuadc/payload/release").first;
    const auto detach =
        _sdf->Get<std::string>("detach_topic", "/cuadc/payload/detach").first;
    const auto impact =
        _sdf->Get<std::string>("impact_topic", "/cuadc/payload/impact").first;
    this->detachPub = this->node.Advertise<gz::msgs::Empty>(detach);
    this->statePub =
        this->node.Advertise<gz::msgs::Boolean>("/cuadc/payload/release_latched");
    this->velocityPub =
        this->node.Advertise<gz::msgs::Vector3d>("/cuadc/payload/velocity");
    this->impactPub =
        this->node.Advertise<gz::msgs::StringMsg>(impact);
    this->node.Subscribe(input, &PayloadReleaseSystem::OnCommand, this);
    this->payloadLink = _ecm.EntityByComponents(
        gz::sim::components::Link(), gz::sim::components::Name("bottle"));
    if (this->payloadLink != gz::sim::kNullEntity)
      gz::sim::Link(this->payloadLink).EnableVelocityChecks(_ecm, true);
    gzmsg << "CUADC payload release latch waiting on [" << input << "]\n";
  }

  public: void PostUpdate(
      const gz::sim::UpdateInfo &_info,
      const gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused)
      return;
    if (this->releaseRequested.load() && !this->released.load())
    {
      if (!this->commandTimeSet)
      {
        this->commandTime = _info.simTime;
        this->commandTimeSet = true;
        gzmsg << "CUADC payload command accepted; applying "
              << this->delaySeconds << " s PTK/latch delay\n";
      }
      const double elapsed =
          std::chrono::duration<double>(_info.simTime - this->commandTime).count();
      if (elapsed >= this->delaySeconds && !this->released.exchange(true))
      {
        this->releaseTime = _info.simTime;
        if (this->payloadLink != gz::sim::kNullEntity)
        {
          const auto pose = _ecm.Component<gz::sim::components::WorldPose>(
              this->payloadLink);
          const auto velocity =
              _ecm.Component<gz::sim::components::WorldLinearVelocity>(
                  this->payloadLink);
          if (pose != nullptr && velocity != nullptr)
          {
            this->releasePosition = pose->Data().Pos();
            this->releaseVelocity = velocity->Data();
            this->releaseStateValid = true;
          }
        }
        gz::msgs::Empty empty;
        this->detachPub.Publish(empty);
        gz::msgs::Boolean state;
        state.set_data(true);
        this->statePub.Publish(state);
        gzmsg << "CUADC payload release latched after actuator delay; "
                 "repeat commands are ignored\n";
      }
    }
    if (_info.simTime - this->lastStatePublish >=
        std::chrono::milliseconds(100))
    {
      this->lastStatePublish = _info.simTime;
      gz::msgs::Boolean state;
      state.set_data(this->released.load());
      this->statePub.Publish(state);
    }
    if (this->payloadLink == gz::sim::kNullEntity)
      return;
    const auto velocityComponent =
        _ecm.Component<gz::sim::components::WorldLinearVelocity>(
            this->payloadLink);
    const bool hasVelocity = velocityComponent != nullptr;
    const auto currentVelocity = hasVelocity
        ? velocityComponent->Data()
        : gz::math::Vector3d::Zero;
    if (this->released.load() && !this->impactRecorded.load())
    {
      // ContactSensorData becomes visible after the collision solver has
      // already changed the link velocity. The preceding physics sample
      // is therefore the closest deterministic pre-impact velocity.
      const auto preImpactVelocity = this->previousVelocityValid
          ? this->previousVelocity
          : currentVelocity;
      this->RecordFirstImpact(_info, _ecm, preImpactVelocity);
    }
    if (hasVelocity)
    {
      this->previousVelocity = currentVelocity;
      this->previousVelocityValid = true;
    }
    if (_info.simTime - this->lastPublish < std::chrono::milliseconds(10))
      return;
    this->lastPublish = _info.simTime;
    if (!hasVelocity)
      return;
    gz::msgs::Vector3d message;
    message.set_x(currentVelocity.X());
    message.set_y(currentVelocity.Y());
    message.set_z(currentVelocity.Z());
    this->velocityPub.Publish(message);
  }

  public: void Reset(
      const gz::sim::UpdateInfo &,
      gz::sim::EntityComponentManager &) override
  {
    this->releaseRequested = false;
    this->released = false;
    this->impactRecorded = false;
    this->previousVelocity = gz::math::Vector3d::Zero;
    this->previousVelocityValid = false;
    this->commandTime = std::chrono::steady_clock::duration::zero();
    this->releaseTime = std::chrono::steady_clock::duration::zero();
    this->commandTimeSet = false;
    this->releaseStateValid = false;
    this->releasePosition = gz::math::Vector3d::Zero;
    this->releaseVelocity = gz::math::Vector3d::Zero;
    this->lastPublish = std::chrono::steady_clock::duration::zero();
    this->lastStatePublish = std::chrono::steady_clock::duration::zero();
  }

  private: void OnCommand(const gz::msgs::Double &_msg)
  {
    if (_msg.data() < this->threshold || this->releaseRequested.exchange(true))
      return;
  }

  private: void RecordFirstImpact(
      const gz::sim::UpdateInfo &_info,
      const gz::sim::EntityComponentManager &_ecm,
      const gz::math::Vector3d &_preImpactVelocity)
  {
    const gz::msgs::Contact *firstContact = nullptr;
    _ecm.Each<gz::sim::components::ContactSensorData>(
        [&](const gz::sim::Entity &,
            const gz::sim::components::ContactSensorData *_contacts)
        {
          for (int index = 0; index < _contacts->Data().contact_size(); ++index)
          {
            const auto &candidate = _contacts->Data().contact(index);
            if (candidate.collision1().name().find("bottle_collision") !=
                    std::string::npos ||
                candidate.collision2().name().find("bottle_collision") !=
                    std::string::npos)
            {
              firstContact = &candidate;
              return false;
            }
          }
          return true;
        });
    if (!firstContact || this->impactRecorded.exchange(true))
      return;

    gz::math::Vector3d position = gz::math::Vector3d::Zero;
    if (firstContact->position_size() > 0)
    {
      const auto &contactPosition = firstContact->position(0);
      position.Set(
          contactPosition.x(), contactPosition.y(), contactPosition.z());
    }
    std::ostringstream json;
    json << std::setprecision(12)
         << "{\"sim_time_s\":"
         << std::chrono::duration<double>(_info.simTime).count()
         << ",\"command_sim_time_s\":"
         << std::chrono::duration<double>(this->commandTime).count()
         << ",\"release_sim_time_s\":"
         << std::chrono::duration<double>(this->releaseTime).count()
         << ",\"actuator_delay_s\":"
         << std::chrono::duration<double>(
                this->releaseTime - this->commandTime).count()
         << ",\"release_state_valid\":"
         << (this->releaseStateValid ? "true" : "false")
         << ",\"release_position\":{\"x\":" << this->releasePosition.X()
         << ",\"y\":" << this->releasePosition.Y()
         << ",\"z\":" << this->releasePosition.Z()
         << "},\"release_velocity_mps\":{\"x\":"
         << this->releaseVelocity.X()
         << ",\"y\":" << this->releaseVelocity.Y()
         << ",\"z\":" << this->releaseVelocity.Z() << "}"
         << ",\"collision1\":\"" << firstContact->collision1().name()
         << "\",\"collision2\":\"" << firstContact->collision2().name()
         << "\",\"position\":{\"x\":" << position.X()
         << ",\"y\":" << position.Y()
         << ",\"z\":" << position.Z()
         << "},\"velocity_mps\":{\"x\":" << _preImpactVelocity.X()
         << ",\"y\":" << _preImpactVelocity.Y()
         << ",\"z\":" << _preImpactVelocity.Z()
         << "},\"speed_mps\":" << _preImpactVelocity.Length()
         << ",\"velocity_sample\":\"previous_physics_step\"}";
    gz::msgs::StringMsg message;
    message.set_data(json.str());
    this->impactPub.Publish(message);
    gzmsg << "CUADC first payload impact atomically recorded\n";
  }

  private: gz::transport::Node node;
  private: gz::transport::Node::Publisher detachPub;
  private: gz::transport::Node::Publisher statePub;
  private: gz::transport::Node::Publisher velocityPub;
  private: gz::transport::Node::Publisher impactPub;
  private: gz::sim::Entity payloadLink{gz::sim::kNullEntity};
  private: std::atomic<bool> releaseRequested{false};
  private: std::atomic<bool> released{false};
  private: std::atomic<bool> impactRecorded{false};
  private: gz::math::Vector3d previousVelocity{
      gz::math::Vector3d::Zero};
  private: bool previousVelocityValid{false};
  private: double threshold{0.75};
  private: double delaySeconds{0.0};
  private: std::chrono::steady_clock::duration commandTime{
      std::chrono::steady_clock::duration::zero()};
  private: std::chrono::steady_clock::duration releaseTime{
      std::chrono::steady_clock::duration::zero()};
  private: bool commandTimeSet{false};
  private: bool releaseStateValid{false};
  private: gz::math::Vector3d releasePosition{gz::math::Vector3d::Zero};
  private: gz::math::Vector3d releaseVelocity{gz::math::Vector3d::Zero};
  private: std::chrono::steady_clock::duration lastPublish{
      std::chrono::steady_clock::duration::zero()};
  private: std::chrono::steady_clock::duration lastStatePublish{
      std::chrono::steady_clock::duration::zero()};
};
}  // namespace cuadc::sim

GZ_ADD_PLUGIN(
    cuadc::sim::PayloadReleaseSystem,
    gz::sim::System,
    cuadc::sim::PayloadReleaseSystem::ISystemConfigure,
    cuadc::sim::PayloadReleaseSystem::ISystemPostUpdate,
    cuadc::sim::PayloadReleaseSystem::ISystemReset)
GZ_ADD_PLUGIN_ALIAS(
    cuadc::sim::PayloadReleaseSystem, "cuadc::sim::PayloadReleaseSystem")
