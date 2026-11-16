#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include <gz/math/Vector3.hh>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/vector3d.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/transport/Node.hh>

namespace cuadc::sim
{
/// Model two counter-rotating propellers from ArduPilot's physical throttle
/// command. Shaft speed has first-order motor dynamics, drives both visible
/// propeller joints, and generates force / reaction torque in Gazebo.
class TwinMotorSystem final:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
  public: void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &) override
  {
    this->model = gz::sim::Model(_entity);
    const auto bodyName =
        _sdf->Get<std::string>("body_link", "fuselage").first;
    const auto leftName =
        _sdf->Get<std::string>("left_joint", "prop_left_joint").first;
    const auto rightName =
        _sdf->Get<std::string>("right_joint", "prop_right_joint").first;
    this->body = this->model.LinkByName(_ecm, bodyName);
    this->leftJoint = this->model.JointByName(_ecm, leftName);
    this->rightJoint = this->model.JointByName(_ecm, rightName);
    this->leftOffset =
        _sdf->Get<gz::math::Vector3d>(
            "left_offset", gz::math::Vector3d(0.24, 0.78, 0.02)).first;
    this->rightOffset =
        _sdf->Get<gz::math::Vector3d>(
            "right_offset", gz::math::Vector3d(0.24, -0.78, 0.02)).first;
    this->thrustCoefficient =
        std::max(0.0, _sdf->Get<double>("thrust_coefficient", 1.71616e-5).first);
    this->torqueCoefficient =
        std::max(0.0, _sdf->Get<double>("torque_coefficient", 5.0e-7).first);
    this->maxThrust =
        std::max(0.0, _sdf->Get<double>("max_thrust_per_motor", 17.1616).first);
    this->maxShaftSpeed =
        std::max(1.0, _sdf->Get<double>("max_shaft_speed", 1000.0).first);
    this->motorTimeConstant =
        std::max(0.001, _sdf->Get<double>("motor_time_constant", 0.18).first);
    this->visualSpeedScale =
        std::clamp(_sdf->Get<double>("visual_speed_scale", 0.06).first,
                   0.0, 1.0);
    this->groundBrakeForce =
        std::max(0.0, _sdf->Get<double>("ground_brake_force", 10.0).first);
    this->groundBrakeHeight =
        std::max(0.0, _sdf->Get<double>("ground_brake_height", 0.65).first);
    this->groundBrakeThrottle =
        std::clamp(_sdf->Get<double>("ground_brake_throttle", 0.20).first,
                   0.0, 1.0);

    if (this->body == gz::sim::kNullEntity ||
        this->leftJoint == gz::sim::kNullEntity ||
        this->rightJoint == gz::sim::kNullEntity)
    {
      gzerr << "CUADC twin-motor system cannot resolve body or propeller "
            << "joints\n";
      this->valid = false;
      return;
    }

    this->throttleTopic =
        _sdf->Get<std::string>("throttle_topic", "/cuadc/motors/throttle").first;
    _ecm.CreateComponent(
        this->leftJoint, gz::sim::components::JointVelocityCmd({0.0}));
    _ecm.CreateComponent(
        this->rightJoint, gz::sim::components::JointVelocityCmd({0.0}));
    gz::sim::Link(this->body).EnableVelocityChecks(_ecm, true);
    this->node.Subscribe(
        this->throttleTopic, &TwinMotorSystem::OnThrottle, this);
    this->node.Subscribe(
        "/cuadc/hand_launch/status",
        &TwinMotorSystem::OnLaunchStatus,
        this);
    this->diagnosticPublisher =
        this->node.Advertise<gz::msgs::Vector3d>("/cuadc/motors/state");
    this->touchdownPublisher =
        this->node.Advertise<gz::msgs::StringMsg>("/cuadc/airframe/touchdown");
    gzmsg << "CUADC twin-motor physics enabled: Ct="
          << this->thrustCoefficient << ", Cq=" << this->torqueCoefficient
          << ", max=" << this->maxThrust << " N per motor, tau="
          << this->motorTimeConstant << " s\n";
  }

  public: void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused || !this->valid)
      return;

    const double dt =
        std::chrono::duration<double>(_info.dt).count();
    if (!std::isfinite(dt) || dt <= 0.0)
      return;
    const double targetSpeed =
        this->maxShaftSpeed * std::clamp(this->throttle.load(), 0.0, 1.0);
    const double response =
        1.0 - std::exp(-dt / std::max(0.001, this->motorTimeConstant));
    this->motorSpeed += (targetSpeed - this->motorSpeed) * response;
    _ecm.SetComponentData<gz::sim::components::JointVelocityCmd>(
        this->leftJoint, {this->motorSpeed * this->visualSpeedScale});
    _ecm.SetComponentData<gz::sim::components::JointVelocityCmd>(
        this->rightJoint, {-this->motorSpeed * this->visualSpeedScale});

    if (!this->launchConfirmed.load())
      return;

    const auto pose = gz::sim::worldPose(this->body, _ecm);
    const double speedSquared = this->motorSpeed * this->motorSpeed;
    const double perMotorThrust =
        std::min(this->maxThrust, this->thrustCoefficient * speedSquared);
    // Equal throttle and symmetric installation make both offset moments
    // cancel exactly. Applying their resultant at the center of mass avoids
    // injecting a redundant constraint wrench into the detachable airframe.
    const auto totalForceWorld = pose.Rot().RotateVector(
        gz::math::Vector3d(2.0 * perMotorThrust, 0, 0));
    auto bodyLink = gz::sim::Link(this->body);
    bodyLink.AddWorldForce(_ecm, totalForceWorld);

    // The fixed-wing model has no wheel/brake joints and Gazebo's contact
    // friction alone lets a belly-landed aircraft coast unrealistically far.
    // Apply bounded grass/skid drag only after launch, near the ground, and at
    // idle throttle.  At flight height or powered hand launch this is exactly
    // zero, so it cannot pull the aircraft onto or away from the route.
    const gz::msgs::Contact *airframeContact = nullptr;
    _ecm.Each<gz::sim::components::ContactSensorData>(
        [&](const gz::sim::Entity &,
            const gz::sim::components::ContactSensorData *_contacts)
        {
          for (int index = 0; index < _contacts->Data().contact_size(); ++index)
          {
            const auto &candidate = _contacts->Data().contact(index);
            const std::string collision1 = candidate.collision1().name();
            const std::string collision2 = candidate.collision2().name();
            const bool airframe =
                collision1.find("cuadc_plane::fuselage::") != std::string::npos ||
                collision2.find("cuadc_plane::fuselage::") != std::string::npos;
            const bool ground =
                collision1.find("ground_collision") != std::string::npos ||
                collision2.find("ground_collision") != std::string::npos;
            if (airframe && ground)
            {
              airframeContact = &candidate;
              return false;
            }
          }
          return true;
        });
    const bool touchingGround = airframeContact != nullptr;
    const auto velocity = bodyLink.WorldLinearVelocity(_ecm);
    const double throttle = this->throttle.load();
    if (touchingGround && throttle <= this->groundBrakeThrottle &&
        velocity.has_value())
    {
      const gz::math::Vector3d horizontal(
          velocity->X(), velocity->Y(), 0.0);
      const double horizontalSpeed = horizontal.Length();
      if (horizontalSpeed > 0.05)
      {
        // Fade below 2 m/s to avoid numerical chatter at the final stop.
        const double magnitude = this->groundBrakeForce *
            std::min(1.0, horizontalSpeed / 2.0);
        bodyLink.AddWorldForce(
            _ecm, -horizontal.Normalized() * magnitude);
      }
    }
    if (touchingGround && !this->touchdownRecorded && velocity.has_value())
    {
      this->touchdownRecorded = true;
      const auto &contact = *airframeContact;
      gz::math::Vector3d position = pose.Pos();
      if (contact.position_size() > 0)
      {
        const auto &point = contact.position(0);
        position.Set(point.x(), point.y(), point.z());
      }
      std::ostringstream json;
      json << std::setprecision(12)
           << "{\"sim_time_s\":"
           << std::chrono::duration<double>(_info.simTime).count()
           << ",\"collision1\":\"" << contact.collision1().name()
           << "\",\"collision2\":\"" << contact.collision2().name()
           << "\",\"position\":{\"x\":" << position.X()
           << ",\"y\":" << position.Y() << ",\"z\":" << position.Z()
           << "},\"groundspeed_mps\":"
           << std::hypot(velocity->X(), velocity->Y()) << "}";
      gz::msgs::StringMsg message;
      message.set_data(json.str());
      this->touchdownPublisher.Publish(message);
      gzmsg << "CUADC first physical airframe contact recorded\n";
    }
    const double leftSpeed = this->motorSpeed;
    const double rightSpeed = -this->motorSpeed;
    if (++this->diagnosticDivider >= 50)
    {
      this->diagnosticDivider = 0;
      gz::msgs::Vector3d state;
      state.set_x(leftSpeed);
      state.set_y(rightSpeed);
      state.set_z(
          this->thrustCoefficient *
          (leftSpeed * leftSpeed + rightSpeed * rightSpeed));
      this->diagnosticPublisher.Publish(state);
    }
  }

  private: void OnThrottle(const gz::msgs::Double &_message)
  {
    if (std::isfinite(_message.data()))
      this->throttle = std::clamp(_message.data(), 0.0, 1.0);
  }

  private: void OnLaunchStatus(const gz::msgs::StringMsg &_message)
  {
    if (_message.data() == "launched")
      this->launchConfirmed = true;
  }

  private: gz::sim::Model model{gz::sim::kNullEntity};
  private: gz::sim::Entity body{gz::sim::kNullEntity};
  private: gz::sim::Entity leftJoint{gz::sim::kNullEntity};
  private: gz::sim::Entity rightJoint{gz::sim::kNullEntity};
  private: gz::math::Vector3d leftOffset;
  private: gz::math::Vector3d rightOffset;
  private: gz::transport::Node node;
  private: gz::transport::Node::Publisher diagnosticPublisher;
  private: gz::transport::Node::Publisher touchdownPublisher;
  private: std::string throttleTopic;
  private: double thrustCoefficient{1.71616e-5};
  private: double torqueCoefficient{5.0e-7};
  private: double maxThrust{17.1616};
  private: double maxShaftSpeed{1000.0};
  private: double motorTimeConstant{0.18};
  private: double visualSpeedScale{0.06};
  private: double groundBrakeForce{10.0};
  private: double groundBrakeHeight{0.65};
  private: double groundBrakeThrottle{0.20};
  private: std::atomic<double> throttle{0.0};
  private: std::atomic<bool> launchConfirmed{false};
  private: double motorSpeed{0.0};
  private: unsigned int diagnosticDivider{0};
  private: bool touchdownRecorded{false};
  private: bool valid{true};
};
}  // namespace cuadc::sim

GZ_ADD_PLUGIN(
    cuadc::sim::TwinMotorSystem,
    gz::sim::System,
    cuadc::sim::TwinMotorSystem::ISystemConfigure,
    cuadc::sim::TwinMotorSystem::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(
    cuadc::sim::TwinMotorSystem, "cuadc::sim::TwinMotorSystem")
