#include <chrono>
#include <memory>
#include <string>

#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace cuadc::sim
{
class ScenarioStateSystem final:
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
    const auto topic =
        _sdf->Get<std::string>("topic", "/cuadc/scenario/state").first;
    this->state = _sdf->Get<std::string>("state", "{}").first;
    this->publisher = this->node.Advertise<gz::msgs::StringMsg>(topic);
    gzmsg << "CUADC scenario truth state available on [" << topic << "]\n";
  }

  public: void PostUpdate(
      const gz::sim::UpdateInfo &_info,
      const gz::sim::EntityComponentManager &) override
  {
    if (_info.paused || _info.simTime < this->nextPublish)
      return;
    gz::msgs::StringMsg message;
    message.set_data(this->state);
    this->publisher.Publish(message);
    this->nextPublish = _info.simTime + std::chrono::seconds(1);
  }

  private: gz::transport::Node node;
  private: gz::transport::Node::Publisher publisher;
  private: std::string state;
  private: std::chrono::steady_clock::duration nextPublish{
      std::chrono::steady_clock::duration::zero()};
};
}  // namespace cuadc::sim

GZ_ADD_PLUGIN(
    cuadc::sim::ScenarioStateSystem,
    gz::sim::System,
    cuadc::sim::ScenarioStateSystem::ISystemConfigure,
    cuadc::sim::ScenarioStateSystem::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(
    cuadc::sim::ScenarioStateSystem, "cuadc::sim::ScenarioStateSystem")
