#include <memory>
#include <string>

#include <gz/msgs/vector3d.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/CustomSensor.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Wind.hh>
#include <gz/transport/Node.hh>

namespace cuadc::sim
{
class AnemometerSystem final:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPostUpdate
{
  public: void Configure(
      const gz::sim::Entity &,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &,
      gz::sim::EventManager &) override
  {
    this->sensorName =
        _sdf->Get<std::string>("sensor_name", "anemometer").first;
  }

  public: void PostUpdate(
      const gz::sim::UpdateInfo &_info,
      const gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused)
      return;
    if (this->sensor == gz::sim::kNullEntity)
    {
      _ecm.Each<gz::sim::components::CustomSensor,
                gz::sim::components::Name,
                gz::sim::components::ParentEntity>(
          [&](const gz::sim::Entity &_entity,
              const gz::sim::components::CustomSensor *,
              const gz::sim::components::Name *_name,
              const gz::sim::components::ParentEntity *_parent) -> bool
          {
            if (_name->Data() != this->sensorName)
              return true;
            this->sensor = _entity;
            this->link = _parent->Data();
            const auto topic = gz::sim::scopedName(_entity, _ecm) + "/anemometer";
            this->publisher = this->node.Advertise<gz::msgs::Vector3d>(topic);
            gzmsg << "CUADC anemometer publishing apparent wind on ["
                  << topic << "]\n";
            return false;
          });
      if (this->sensor == gz::sim::kNullEntity)
        return;
    }

    gz::math::Vector3d windWorld = gz::math::Vector3d::Zero;
    const auto windEntity =
        _ecm.EntityByComponents(gz::sim::components::Wind());
    if (windEntity != gz::sim::kNullEntity)
    {
      if (const auto worldWind =
          _ecm.Component<gz::sim::components::WorldLinearVelocity>(windEntity))
        windWorld = worldWind->Data();
      else if (const auto wind =
          _ecm.Component<gz::sim::components::LinearVelocity>(windEntity))
        windWorld = wind->Data();
    }

    const auto velocity =
        _ecm.Component<gz::sim::components::WorldLinearVelocity>(this->link);
    const auto pose =
        _ecm.Component<gz::sim::components::WorldPose>(this->sensor);
    if (!velocity || !pose)
      return;
    const auto apparentWorld = windWorld - velocity->Data();
    const auto apparentSensor =
        pose->Data().Rot().Inverse().RotateVector(apparentWorld);
    gz::msgs::Vector3d message;
    message.set_x(apparentSensor.X());
    message.set_y(apparentSensor.Y());
    message.set_z(apparentSensor.Z());
    this->publisher.Publish(message);
  }

  private: gz::transport::Node node;
  private: gz::transport::Node::Publisher publisher;
  private: std::string sensorName;
  private: gz::sim::Entity sensor{gz::sim::kNullEntity};
  private: gz::sim::Entity link{gz::sim::kNullEntity};
};
}  // namespace cuadc::sim

GZ_ADD_PLUGIN(
    cuadc::sim::AnemometerSystem,
    gz::sim::System,
    cuadc::sim::AnemometerSystem::ISystemConfigure,
    cuadc::sim::AnemometerSystem::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(
    cuadc::sim::AnemometerSystem, "cuadc::sim::AnemometerSystem")
