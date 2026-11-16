#include <atomic>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/vector3d.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Actor.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Actor.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/PoseCmd.hh>
#include <gz/transport/Node.hh>

namespace cuadc::sim
{
class HandLaunchSystem final:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemReset
{
  public: void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &) override
  {
    this->model = gz::sim::Model(_entity);
    const auto linkName = _sdf->Get<std::string>("link_name", "fuselage").first;
    this->link = this->model.LinkByName(_ecm, linkName);
    this->topic = _sdf->Get<std::string>("topic", "/cuadc/hand_launch").first;
    this->detachTopic = _sdf
        ->Get<std::string>("detach_topic", "/cuadc/hand_launch/detach").first;
    this->heldTopic = _sdf
        ->Get<std::string>("held_topic", "/cuadc/hand_launch/held").first;
    this->launchMass =
        std::max(0.1, _sdf->Get<double>("launch_mass", 2.888).first);
    // 场景里没有操作员模型时留空即可；空名表示不做人物动画。
    this->actorName = _sdf->Get<std::string>("actor_name", "").first;
    this->idleAnimation =
        _sdf->Get<std::string>("idle_animation", "stand").first;
    this->launchAnimation =
        _sdf->Get<std::string>("launch_animation", "gesture").first;
    this->launchAnimationDuration = std::max(
        0.1, _sdf->Get<double>("launch_animation_duration", 1.5).first);
    this->impulseDuration =
        std::max(0.05, _sdf->Get<double>("impulse_duration", 0.20).first);
    if (this->link == gz::sim::kNullEntity)
    {
      gzerr << "CUADC hand launcher cannot find link [" << linkName << "]\n";
      return;
    }
    this->node.Subscribe(this->topic, &HandLaunchSystem::OnCommand, this);
    this->node.Subscribe(
        this->heldTopic, &HandLaunchSystem::OnJointState, this);
    this->detachPub = this->node.Advertise<gz::msgs::Empty>(this->detachTopic);
    this->statusPub =
        this->node.Advertise<gz::msgs::StringMsg>("/cuadc/hand_launch/status");
    gzmsg << "CUADC hand launcher waiting on [" << this->topic << "]\n";
  }

  public: void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (this->actorGestureActive)
    {
      const double animationTime = std::chrono::duration<double>(
          _info.simTime - this->actorGestureStart).count();
      if (animationTime >= this->launchAnimationDuration)
      {
        this->actorGestureActive = false;
        // The actor animation advances in Gazebo's rendering system. Change
        // animation components only at state transitions; rewriting them on
        // every physics tick races the renderer and can invalidate its
        // current keyframe while it is being evaluated.
        this->SetActorAnimation(_ecm, this->idleAnimation, 0.0);
      }
    }

    if (this->cleanup.exchange(false))
    {
      _ecm.RemoveComponent<gz::sim::components::WorldPoseCmd>(
          this->model.Entity());
    }

    bool requestDetach = false;
    bool stagePose = false;
    bool applyImpulse = false;
    bool finishLaunch = false;
    bool startActorGesture = false;
    gz::msgs::Vector3d command;
    {
      std::lock_guard<std::mutex> guard(this->mutex);
      if (!this->pending || this->launched)
        return;
      if (!this->detachRequested)
      {
        this->detachRequested = true;
        requestDetach = true;
      }
      else if (!this->jointDetached.load())
      {
        // Transport state notifications are best-effort. The detachable
        // joint is processed in the same physics loop, so a bounded
        // 250 ms fallback still guarantees it has been removed before launch.
        if (++this->detachWaitCycles < 250)
          return;
      }
      if (!this->poseStaged)
      {
        command = this->command;
        this->poseStaged = true;
        stagePose = true;
      }
      else if (!this->impulseActive)
      {
        command = this->command;
        this->impulseActive = true;
        this->impulseEnd = _info.simTime +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(this->impulseDuration));
        applyImpulse = true;
      }
      else if (_info.simTime < this->impulseEnd)
      {
        command = this->command;
        applyImpulse = true;
      }
      else
      {
        command = this->command;
        this->pending = false;
        this->launched = true;
        this->impulseActive = false;
        finishLaunch = true;
      }
    }

    if (requestDetach)
    {
      gz::msgs::Empty detach;
      this->detachPub.Publish(detach);
      gzmsg << "CUADC hand launcher detached its physical holding joint\n";
      return;
    }

    const double speed = std::max(0.0, command.x());
    // The command uses the aircraft flight-path convention (positive is
    // nose-up / upward).  Gazebo's right-handed +Y rotation sends +X down,
    // so the staged body pose uses the opposite sign while the impulse keeps
    // the requested positive vertical component.
    const double flightPitch = command.y() * M_PI / 180.0;
    const double bodyPitch = -flightPitch;
    const double impulseYaw = command.z() * M_PI / 180.0;
    const auto current = gz::sim::worldPose(this->model.Entity(), _ecm);
    // Group A/B heading is a physical initial condition in the generated
    // world. Preserve it here and only stage the requested launch pitch.
    // This also lets ArduPlane observe and settle on the correct heading while
    // the detachable restraint is still attached.
    const double physicalYaw = current.Rot().Euler().Z();
    const gz::math::Quaterniond rotation(0.0, bodyPitch, physicalYaw);
    const gz::math::Pose3d pose(current.Pos(), rotation);

    if (stagePose)
    {
      // Pose and velocity commands must occupy separate physics iterations:
      // applying both together lets the pose reset clear the launch velocity.
      _ecm.SetComponentData<gz::sim::components::WorldPoseCmd>(
          this->model.Entity(), pose);
      return;
    }

    if (applyImpulse)
    {
      _ecm.RemoveComponent<gz::sim::components::WorldPoseCmd>(
          this->model.Entity());
      // A finite impulse lets the detachable payload and all ribbon links
      // accelerate through their physical constraints with the airframe.
      const double forceMagnitude =
          this->launchMass * speed / this->impulseDuration;
      const double verticalForce = this->launchMass * (
          speed * std::sin(flightPitch) / this->impulseDuration + 9.80665);
      // Apply the requested flight-path angle explicitly in world ENU.
      // This keeps the hand's impulse direction exact while the active
      // controller starts responding to the staged body attitude.
      const auto forceWorld = gz::math::Vector3d(
          forceMagnitude * std::cos(flightPitch) * std::cos(impulseYaw),
          forceMagnitude * std::cos(flightPitch) * std::sin(impulseYaw),
          verticalForce);
      gz::sim::Link(this->link).AddWorldForce(_ecm, forceWorld);
      if (!this->actorGestureStarted)
      {
        this->actorGestureStarted = true;
        this->actorGestureStart = _info.simTime;
        this->actorGestureActive = true;
        startActorGesture = true;
      }
      if (startActorGesture)
        this->SetActorAnimation(_ecm, this->launchAnimation, 0.0);
      return;
    }

    if (!finishLaunch)
      return;
    this->cleanup = true;
    gz::msgs::StringMsg state;
    state.set_data("launched");
    this->statusPub.Publish(state);
    gzmsg << "CUADC physical hand launch: " << speed << " m/s, pitch "
          << command.y() << " deg, yaw " << command.z() << " deg\n";
  }

  public: void Reset(
      const gz::sim::UpdateInfo &,
      gz::sim::EntityComponentManager &) override
  {
    std::lock_guard<std::mutex> guard(this->mutex);
    this->launched = false;
    this->pending = false;
    this->detachRequested = false;
    this->jointDetached = false;
    this->detachWaitCycles = 0;
    this->poseStaged = false;
    this->impulseActive = false;
    this->impulseEnd = std::chrono::steady_clock::duration::zero();
    this->cleanup = false;
    this->actorEntity = gz::sim::kNullEntity;
    this->actorGestureActive = false;
    this->actorGestureStarted = false;
    this->actorGestureStart = std::chrono::steady_clock::duration::zero();
  }

  private: void OnCommand(const gz::msgs::Vector3d &_msg)
  {
    std::lock_guard<std::mutex> guard(this->mutex);
    if (this->launched || this->pending)
      return;
    this->command = _msg;
    this->pending = true;
  }

  private: void OnJointState(const gz::msgs::StringMsg &_msg)
  {
    if (_msg.data() == "detached")
      this->jointDetached = true;
  }

  private: bool SetActorAnimation(
      gz::sim::EntityComponentManager &_ecm,
      const std::string &_animation,
      double _timeSeconds)
  {
    if (this->actorName.empty())
      return false;   // 本场景未配置操作员模型，跳过动画即可。

    if (this->actorEntity == gz::sim::kNullEntity)
    {
      this->actorEntity = _ecm.EntityByComponents(
          gz::sim::components::Name(this->actorName));
    }
    if (this->actorEntity == gz::sim::kNullEntity)
    {
      gzerr << "CUADC hand launcher cannot find actor ["
            << this->actorName << "] for animation [" << _animation << "]\n";
      return false;
    }

    gz::sim::Actor actor(this->actorEntity);
    if (!actor.Valid(_ecm))
    {
      this->actorEntity = gz::sim::kNullEntity;
      return false;
    }

    // Gazebo's BVH loader registers the animation under its resolved file
    // path, while COLLADA animations keep the SDF alias.  Passing the BVH
    // alias (for example, "gesture") creates an animation component that is
    // absent from the actor skeleton.  Ogre2 then reports a missing skeleton
    // animation and can dereference a null keyframe during sensor rendering.
    // Resolve the alias from the actor's own SDF before changing the
    // component, so both formats use the name that the renderer loaded.
    std::string animationName = _animation;
    const auto *actorComponent =
        _ecm.Component<gz::sim::components::Actor>(this->actorEntity);
    if (actorComponent)
    {
      const sdf::Actor &actorData = actorComponent->Data();
      for (uint64_t index = 0; index < actorData.AnimationCount(); ++index)
      {
        const auto *animation = actorData.AnimationByIndex(index);
        if (!animation || animation->Name() != _animation)
          continue;

        const std::string filename = animation->Filename();
        const auto extensionStart = filename.find_last_of('.');
        std::string extension = extensionStart == std::string::npos
            ? std::string{}
            : filename.substr(extensionStart + 1);
        std::transform(extension.begin(), extension.end(), extension.begin(),
            [](unsigned char character)
            { return static_cast<char>(std::tolower(character)); });
        if (extension == "bvh")
        {
          animationName = gz::sim::asFullPath(
              filename, animation->FilePath());
        }
        break;
      }
    }

    // Garden 7.9 creates AnimationUpdateData without SDF waypoints when an
    // animation is commanded through ECM. Supply the stationary trajectory
    // too, otherwise RenderUtil dereferences a null Waypoints() on launch.
    actor.SetTrajectoryPose(_ecm, gz::math::Pose3d(0, 0, 1.0, 0, 0, 0));
    actor.SetAnimationName(_ecm, animationName);
    actor.SetAnimationTime(
        _ecm,
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(std::max(0.0, _timeSeconds))));
    gzmsg << "CUADC hand-launch actor [" << this->actorName
          << "] switched to animation [" << animationName << "]\n";
    return true;
  }

  private: gz::sim::Model model{gz::sim::kNullEntity};
  private: gz::sim::Entity link{gz::sim::kNullEntity};
  private: gz::transport::Node node;
  private: gz::transport::Node::Publisher detachPub;
  private: gz::transport::Node::Publisher statusPub;
  private: std::string topic;
  private: std::string detachTopic;
  private: std::string heldTopic;
  private: std::string actorName{};
  private: std::string idleAnimation{"stand"};
  private: std::string launchAnimation{"gesture"};
  private: std::mutex mutex;
  private: gz::msgs::Vector3d command;
  private: bool pending{false};
  private: bool launched{false};
  private: bool detachRequested{false};
  private: std::atomic<bool> jointDetached{false};
  private: unsigned int detachWaitCycles{0};
  private: bool poseStaged{false};
  private: bool impulseActive{false};
  private: gz::sim::Entity actorEntity{gz::sim::kNullEntity};
  private: bool actorGestureActive{false};
  private: bool actorGestureStarted{false};
  private: double launchAnimationDuration{1.5};
  private: std::chrono::steady_clock::duration actorGestureStart{
      std::chrono::steady_clock::duration::zero()};
  private: std::chrono::steady_clock::duration impulseEnd{
      std::chrono::steady_clock::duration::zero()};
  private: double launchMass{2.888};
  private: double impulseDuration{0.20};
  private: std::atomic<bool> cleanup{false};
};
}  // namespace cuadc::sim

GZ_ADD_PLUGIN(
    cuadc::sim::HandLaunchSystem,
    gz::sim::System,
    cuadc::sim::HandLaunchSystem::ISystemConfigure,
    cuadc::sim::HandLaunchSystem::ISystemPreUpdate,
    cuadc::sim::HandLaunchSystem::ISystemReset)
GZ_ADD_PLUGIN_ALIAS(cuadc::sim::HandLaunchSystem, "cuadc::sim::HandLaunchSystem")
