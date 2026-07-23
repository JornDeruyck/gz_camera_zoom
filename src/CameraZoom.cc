// Copyright 2026 Jorn Deruyck
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "gz_camera_zoom/CameraZoom.hh"

#include <algorithm>
#include <chrono>
#include <mutex>
#include <string>

#include <gz/msgs/camera_info.pb.h>
#include <gz/msgs/double.pb.h>

#include <gz/common/Console.hh>
#include <gz/math/Angle.hh>
#include <gz/math/Helpers.hh>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/transport/TopicUtils.hh>

#include <gz/rendering/Camera.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Utils.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Camera.hh>
#include <gz/sim/components/ThermalCamera.hh>
#include <gz/sim/rendering/Events.hh>

#include <sdf/Camera.hh>
#include <sdf/Sensor.hh>

using namespace gz_camera_zoom;

namespace gz_camera_zoom
{
/// \brief Private data for CameraZoom.
///
/// Threading model: `OnCmd` runs on a gz-transport worker thread and only
/// ever touches `cmdRate`/`lastCmdTime` behind `cmdMutex`. `OnPreRender`
/// runs on the Gazebo Sensors system's dedicated rendering thread (it is
/// bound to the gz::sim::events::PreRender event, which is emitted from
/// SensorsPrivate::RunOnce on that thread) and is the only place that ever
/// touches the gz::rendering::Camera pointer. This mirrors the pattern used
/// by gz-sim's own LensFlare and CameraVideoRecorder systems: rendering
/// objects are never touched from any thread other than the render thread.
class CameraZoomPrivate
{
  /// \brief Gz-transport node.
  public: gz::transport::Node node;

  /// \brief Camera entity this plugin is attached to.
  public: gz::sim::Entity entity{gz::sim::kNullEntity};

  /// \brief Event manager, used to (dis)connect from render events.
  public: gz::sim::EventManager *eventMgr{nullptr};

  /// \brief Connection to the pre-render event.
  public: gz::common::ConnectionPtr preRenderConn;

  /// \brief Scoped name of the camera, used to look it up in the scene.
  public: std::string cameraName;

  /// \brief Rendering scene (render thread only).
  public: gz::rendering::ScenePtr scene;

  /// \brief Rendering camera (render thread only).
  public: gz::rendering::CameraPtr camera;

  /// \brief Minimum horizontal FOV (radians), i.e. maximum zoom-in.
  public: double minHfov{0.05};

  /// \brief Maximum horizontal FOV (radians), i.e. no zoom / zoomed out.
  public: double maxHfov{1.05};

  /// \brief Zoom rate in rad/s of HFOV change at command magnitude 1.0.
  public: double zoomRate{0.4};

  /// \brief Current HFOV (render thread only).
  public: double currentHfov{1.05};

  /// \brief Last simulation time reported via PostUpdate. Written from the
  /// simulation thread, read from the render thread. Same benign-race
  /// pattern gz-sim's own CameraVideoRecorder system relies on for simTime.
  public: std::chrono::steady_clock::duration simTime{0};

  /// \brief Last simulation time a zoom step was integrated against
  /// (render thread only).
  public: std::chrono::steady_clock::duration lastAppliedSimTime{0};

  /// \brief Mutex protecting cmdRate / lastCmdTime / haveCmd.
  public: std::mutex cmdMutex;

  /// \brief Latest commanded zoom rate in [-1, 1]. Positive zooms in.
  public: double cmdRate{0.0};

  /// \brief Wall time the last command was received.
  public: std::chrono::steady_clock::time_point lastCmdTime;

  /// \brief True once at least one command has been received.
  public: bool haveCmd{false};

  /// \brief Watchdog: treat the commanded rate as zero if no new command
  /// arrives within this long, so a dead publisher can't leave the zoom
  /// motor running forever.
  public: std::chrono::steady_clock::duration cmdTimeout{
              std::chrono::milliseconds(500)};

  /// \brief Publisher for the live, zoom-corrected camera info.
  public: gz::transport::Node::Publisher infoPub;

  /// \brief Publisher for the current zoom level (maxHfov / currentHfov).
  public: gz::transport::Node::Publisher levelPub;

  /// \brief Sensor's own configured topic (from SDF), empty if it uses the
  /// auto-generated default.
  public: std::string sensorTopic;

  /// \brief Raw <topic> plugin param, empty if not overridden.
  public: std::string cmdTopicParam;

  /// \brief Raw <camera_info_topic> plugin param, empty if not overridden.
  public: std::string infoTopicParam;

  /// \brief Raw <level_topic> plugin param, empty if not overridden.
  public: std::string levelTopicParam;

  /// \brief True once topics are resolved, advertised, and the render
  /// callback is connected. Deferred to the first PostUpdate (rather than
  /// done in Configure) because the camera's ancestor Model/Link entities
  /// are not guaranteed to have their Name/ParentEntity components in the
  /// ECM yet at Configure time, which would make scopedName() resolve to
  /// an empty/wrong string. gz-sim's own CameraVideoRecorder system uses
  /// the same deferred-to-PostUpdate pattern for this reason.
  public: bool configured{false};

  /// \brief True if configuration was invalid; stop retrying.
  public: bool invalid{false};

  /// \brief Callback for the zoom command topic (transport thread).
  public: void OnCmd(const gz::msgs::Double &_msg);

  /// \brief Callback bound to gz::sim::events::PreRender (render thread).
  public: void OnPreRender();
};
}  // namespace gz_camera_zoom

//////////////////////////////////////////////////
void CameraZoomPrivate::OnCmd(const gz::msgs::Double &_msg)
{
  std::lock_guard<std::mutex> lock(this->cmdMutex);
  this->cmdRate = gz::math::clamp(_msg.data(), -1.0, 1.0);
  this->lastCmdTime = std::chrono::steady_clock::now();
  this->haveCmd = true;
}

//////////////////////////////////////////////////
void CameraZoomPrivate::OnPreRender()
{
  if (!this->scene)
  {
    this->scene = gz::rendering::sceneFromFirstRenderEngine();
  }

  if (!this->scene || !this->scene->IsInitialized() ||
      this->scene->SensorCount() == 0)
  {
    return;
  }

  if (!this->camera)
  {
    auto sensor = this->scene->SensorByName(this->cameraName);
    if (!sensor)
      return;

    this->camera =
        std::dynamic_pointer_cast<gz::rendering::Camera>(sensor);
    if (!this->camera)
    {
      gzerr << "[CameraZoom] sensor [" << this->cameraName
            << "] is not a camera." << std::endl;
    }
    // Either way, wait for the next tick before doing anything else so we
    // don't act on a half-initialized camera.
    return;
  }

  auto simTimeNow = this->simTime;
  if (this->lastAppliedSimTime > simTimeNow)
  {
    // simulation was reset; resync rather than integrating a negative dt
    this->lastAppliedSimTime = simTimeNow;
  }
  double dt = std::chrono::duration<double>(
      simTimeNow - this->lastAppliedSimTime).count();
  this->lastAppliedSimTime = simTimeNow;

  double rate = 0.0;
  {
    std::lock_guard<std::mutex> lock(this->cmdMutex);
    bool timedOut = !this->haveCmd ||
        (std::chrono::steady_clock::now() - this->lastCmdTime) >
        this->cmdTimeout;
    if (!timedOut)
      rate = this->cmdRate;
  }

  if (rate != 0.0 && dt > 0.0)
  {
    // Positive rate narrows the FOV (zoom in); negative widens it (zoom
    // out). This is optical zoom: it changes the camera's projection, not
    // a crop+upscale of the rendered image.
    this->currentHfov -= rate * this->zoomRate * dt;
    this->currentHfov =
        gz::math::clamp(this->currentHfov, this->minHfov, this->maxHfov);
    this->camera->SetHFOV(gz::math::Angle(this->currentHfov));
  }

  // Publish a live camera_info and zoom level every tick (not just while
  // actively zooming) so a subscriber that starts late still gets correct,
  // up-to-date intrinsics for the current zoom state.
  unsigned int width = this->camera->ImageWidth();
  unsigned int height = this->camera->ImageHeight();
  if (width == 0 || height == 0)
    return;

  auto intrinsics = gz::rendering::projectionToCameraIntrinsic(
      this->camera->ProjectionMatrix(), width, height);

  gz::msgs::CameraInfo infoMsg;
  *infoMsg.mutable_header()->mutable_stamp() =
      gz::msgs::Convert(simTimeNow);
  auto *frameData = infoMsg.mutable_header()->add_data();
  frameData->set_key("frame_id");
  frameData->add_value(this->cameraName + "/zoom_optical_frame");

  infoMsg.set_width(width);
  infoMsg.set_height(height);

  auto *distortion = infoMsg.mutable_distortion();
  distortion->set_model(gz::msgs::CameraInfo::Distortion::PLUMB_BOB);
  for (int i = 0; i < 5; ++i)
    distortion->add_k(0.0);

  auto *k = infoMsg.mutable_intrinsics();
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      k->add_k(intrinsics(r, c));

  auto *p = infoMsg.mutable_projection();
  p->add_p(intrinsics(0, 0)); p->add_p(0.0); p->add_p(intrinsics(0, 2));
  p->add_p(0.0);
  p->add_p(0.0); p->add_p(intrinsics(1, 1)); p->add_p(intrinsics(1, 2));
  p->add_p(0.0);
  p->add_p(0.0); p->add_p(0.0); p->add_p(1.0); p->add_p(0.0);

  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      infoMsg.add_rectification_matrix(r == c ? 1.0 : 0.0);

  this->infoPub.Publish(infoMsg);

  gz::msgs::Double levelMsg;
  levelMsg.set_data(this->maxHfov / this->currentHfov);
  this->levelPub.Publish(levelMsg);
}

//////////////////////////////////////////////////
CameraZoom::CameraZoom()
  : dataPtr(std::make_unique<CameraZoomPrivate>())
{
}

//////////////////////////////////////////////////
CameraZoom::~CameraZoom() = default;

//////////////////////////////////////////////////
void CameraZoom::Configure(const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm, gz::sim::EventManager &_eventMgr)
{
  // Regular color cameras register components::Camera; thermal cameras
  // register the separate components::ThermalCamera instead (both just wrap
  // an sdf::Sensor). Either way, the live gz::rendering object underneath is
  // a gz::rendering::Camera (ThermalCamera publicly inherits it), so the
  // same HFOV/projection logic in OnPreRender applies unchanged.
  sdf::Sensor sensorSdf;
  if (auto cameraComp = _ecm.Component<gz::sim::components::Camera>(_entity))
  {
    sensorSdf = cameraComp->Data();
  }
  else if (auto thermalComp =
      _ecm.Component<gz::sim::components::ThermalCamera>(_entity))
  {
    sensorSdf = thermalComp->Data();
  }
  else
  {
    gzerr << "[CameraZoom] must be attached inside a "
          << "<sensor type=\"camera\"> or <sensor type=\"thermal\"> "
          << "element. Ignoring." << std::endl;
    this->dataPtr->invalid = true;
    return;
  }

  this->dataPtr->entity = _entity;
  this->dataPtr->eventMgr = &_eventMgr;

  const sdf::Camera *camSdf = sensorSdf.CameraSensor();

  this->dataPtr->sensorTopic = sensorSdf.Topic();

  double defaultMaxHfov = camSdf ? camSdf->HorizontalFov().Radian()
                                  : GZ_PI_2;

  this->dataPtr->minHfov = _sdf->Get<double>("min_hfov", 0.05).first;
  this->dataPtr->maxHfov =
      _sdf->Get<double>("max_hfov", defaultMaxHfov).first;
  this->dataPtr->zoomRate = _sdf->Get<double>("zoom_rate", 0.4).first;
  // 2s default rather than something sub-second: this is a dead-publisher
  // safety net, not a control-loop rate requirement. A command is meant to
  // be held with occasional, possibly slow, repeats (e.g. a plain
  // `ros2 topic pub` with no -r defaults to 1 Hz) — a short timeout makes
  // the commanded rate flicker between the value and zero at the publish
  // period, which looks like visible stutter even though the per-tick FOV
  // integration itself is already smooth every render frame.
  double cmdTimeoutSec = _sdf->Get<double>("cmd_timeout", 2.0).first;
  this->dataPtr->cmdTimeout =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(cmdTimeoutSec));

  if (this->dataPtr->minHfov <= 0.0 ||
      this->dataPtr->minHfov >= this->dataPtr->maxHfov)
  {
    gzerr << "[CameraZoom] invalid min_hfov/max_hfov ("
          << this->dataPtr->minHfov << " / " << this->dataPtr->maxHfov
          << "). Disabling." << std::endl;
    this->dataPtr->invalid = true;
    return;
  }

  this->dataPtr->currentHfov = camSdf ? camSdf->HorizontalFov().Radian()
                                       : this->dataPtr->maxHfov;
  this->dataPtr->lastAppliedSimTime = std::chrono::steady_clock::duration{0};

  this->dataPtr->cmdTopicParam = _sdf->Get<std::string>("topic", "").first;
  this->dataPtr->infoTopicParam =
      _sdf->Get<std::string>("camera_info_topic", "").first;
  this->dataPtr->levelTopicParam =
      _sdf->Get<std::string>("level_topic", "").first;
}

//////////////////////////////////////////////////
void CameraZoom::PostUpdate(const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
  this->dataPtr->simTime = _info.simTime;

  if (this->dataPtr->configured || this->dataPtr->invalid)
    return;

  // Deferred from Configure(): see the `configured` member comment. Retry
  // every tick until the entity's ancestor chain is fully populated in the
  // ECM and scopedName() resolves to a real name.
  auto cameraName = gz::sim::removeParentScope(
      gz::sim::scopedName(this->dataPtr->entity, _ecm, "::", false), "::");
  if (cameraName.empty())
    return;
  this->dataPtr->cameraName = cameraName;

  std::string sensorTopic = this->dataPtr->sensorTopic;
  if (sensorTopic.empty())
  {
    auto scoped = gz::sim::scopedName(this->dataPtr->entity, _ecm);
    if (scoped.empty())
      return;
    sensorTopic = gz::transport::TopicUtils::AsValidTopic(scoped + "/image");
  }

  std::string cmdTopic = gz::transport::TopicUtils::AsValidTopic(
      !this->dataPtr->cmdTopicParam.empty() ? this->dataPtr->cmdTopicParam
                                             : sensorTopic + "/zoom/cmd");
  std::string infoTopic = gz::transport::TopicUtils::AsValidTopic(
      !this->dataPtr->infoTopicParam.empty() ? this->dataPtr->infoTopicParam
                                              : sensorTopic +
                                                "/zoom/camera_info");
  std::string levelTopic = gz::transport::TopicUtils::AsValidTopic(
      !this->dataPtr->levelTopicParam.empty() ? this->dataPtr->levelTopicParam
                                               : sensorTopic + "/zoom/level");

  if (cmdTopic.empty() || infoTopic.empty() || levelTopic.empty())
  {
    gzerr << "[CameraZoom] failed to build valid topics for ["
          << this->dataPtr->cameraName << "]." << std::endl;
    this->dataPtr->invalid = true;
    return;
  }

  if (!this->dataPtr->node.Subscribe(
      cmdTopic, &CameraZoomPrivate::OnCmd, this->dataPtr.get()))
  {
    gzerr << "[CameraZoom] failed to subscribe to [" << cmdTopic << "]."
          << std::endl;
    this->dataPtr->invalid = true;
    return;
  }

  this->dataPtr->infoPub =
      this->dataPtr->node.Advertise<gz::msgs::CameraInfo>(infoTopic);
  this->dataPtr->levelPub =
      this->dataPtr->node.Advertise<gz::msgs::Double>(levelTopic);

  gzmsg << "[CameraZoom] camera [" << this->dataPtr->cameraName
        << "]: cmd topic [" << cmdTopic << "], camera_info topic ["
        << infoTopic << "], level topic [" << levelTopic
        << "], hfov range [" << this->dataPtr->minHfov << ", "
        << this->dataPtr->maxHfov << "] rad, rate "
        << this->dataPtr->zoomRate << " rad/s." << std::endl;

  this->dataPtr->preRenderConn =
      this->dataPtr->eventMgr->Connect<gz::sim::events::PreRender>(
      std::bind(&CameraZoomPrivate::OnPreRender, this->dataPtr.get()));

  this->dataPtr->configured = true;
}

GZ_ADD_PLUGIN(CameraZoom, gz::sim::System,
  CameraZoom::ISystemConfigure,
  CameraZoom::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(CameraZoom, "gz_camera_zoom::CameraZoom")
