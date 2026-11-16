#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>

namespace cuadc::sim
{
/// Publish the actual aerodynamic control positions used by Gazebo. This is
/// intentionally diagnostic-only: it never writes a pose, velocity or force.
class AirframeDiagnosticsSystem final:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPostUpdate
{
  public: void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &) override
  {
    gz::sim::Model model(_entity);
    for (const auto *name : {
             "aileron_left_joint", "aileron_right_joint",
             "elevator_joint", "rudder_joint"})
    {
      const auto entity = model.JointByName(_ecm, name);
      if (entity == gz::sim::kNullEntity)
      {
        gzerr << "CUADC diagnostics cannot resolve joint [" << name << "]\n";
        continue;
      }
      this->joints.emplace_back(name, entity);
    }
    const auto topic =
        _sdf->Get<std::string>(
            "topic", "/cuadc/airframe/control_state").first;
    this->publisher = this->node.Advertise<gz::msgs::StringMsg>(topic);
  }

  public: void PostUpdate(
      const gz::sim::UpdateInfo &_info,
      const gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused)
      return;

    bool healthy{true};
    _ecm.Each<
        gz::sim::components::Link,
        gz::sim::components::Name>(
        [&](const gz::sim::Entity &_entity,
            const gz::sim::components::Link *,
            const gz::sim::components::Name *_name)
        {
          const auto pose = gz::sim::worldPose(_entity, _ecm);
          const auto *linear =
              _ecm.Component<gz::sim::components::LinearVelocity>(_entity);
          const auto *angular =
              _ecm.Component<gz::sim::components::AngularVelocity>(_entity);
          const auto finiteVector = [](const gz::math::Vector3d &_value)
          {
            return std::isfinite(_value.X()) &&
                   std::isfinite(_value.Y()) &&
                   std::isfinite(_value.Z());
          };
          const bool valid =
              finiteVector(pose.Pos()) &&
              std::isfinite(pose.Rot().W()) &&
              std::isfinite(pose.Rot().X()) &&
              std::isfinite(pose.Rot().Y()) &&
              std::isfinite(pose.Rot().Z()) &&
              pose.Pos().Length() < 1.0e6 &&
              (!linear ||
               (finiteVector(linear->Data()) &&
                linear->Data().Length() < 1.0e4)) &&
              (!angular ||
               (finiteVector(angular->Data()) &&
                angular->Data().Length() < 1.0e5));
          if (valid)
            return true;

          healthy = false;
          if (!this->healthFailureLogged)
          {
            gzerr << "CUADC physics health failure at link ["
                  << _name->Data() << "] entity=" << _entity
                  << " pose=" << pose;
            if (linear)
              gzerr << " linear_velocity=" << linear->Data();
            if (angular)
              gzerr << " angular_velocity=" << angular->Data();
            gzerr << "\n";
            this->healthFailureLogged = true;
          }
          return false;
        });
    if (healthy)
      this->healthFailureLogged = false;

    if (++this->divider < 20)
      return;
    this->divider = 0;
    std::ostringstream json;
    json << '{';
    bool first = true;
    for (const auto &[name, entity] : this->joints)
    {
      if (!first)
        json << ',';
      first = false;
      json << '"' << name << "\":{";
      this->WriteFirst(
          json, "position_rad",
          _ecm.Component<gz::sim::components::JointPosition>(entity));
      json << ',';
      this->WriteFirst(
          json, "velocity_rps",
          _ecm.Component<gz::sim::components::JointVelocity>(entity));
      json << ',';
      this->WriteFirst(
          json, "force_cmd",
          _ecm.Component<gz::sim::components::JointForceCmd>(entity));
      json << '}';
    }
    json << '}';
    gz::msgs::StringMsg message;
    message.set_data(json.str());
    this->publisher.Publish(message);
  }

  private: template<typename Component>
  void WriteFirst(
      std::ostringstream &_json,
      const char *_key,
      const Component *_component) const
  {
    _json << '"' << _key << "\":";
    if (!_component || _component->Data().empty() ||
        !std::isfinite(_component->Data()[0]))
    {
      _json << "null";
      return;
    }
    _json << _component->Data()[0];
  }

  private: gz::transport::Node node;
  private: gz::transport::Node::Publisher publisher;
  private: std::vector<std::pair<std::string, gz::sim::Entity>> joints;
  private: unsigned int divider{0};
  private: bool healthFailureLogged{false};
};
}  // namespace cuadc::sim

GZ_ADD_PLUGIN(
    cuadc::sim::AirframeDiagnosticsSystem,
    gz::sim::System,
    cuadc::sim::AirframeDiagnosticsSystem::ISystemConfigure,
    cuadc::sim::AirframeDiagnosticsSystem::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(
    cuadc::sim::AirframeDiagnosticsSystem,
    "cuadc::sim::AirframeDiagnosticsSystem")
