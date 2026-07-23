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

#ifndef GZ_CAMERA_ZOOM__CAMERA_ZOOM_HH_
#define GZ_CAMERA_ZOOM__CAMERA_ZOOM_HH_

#include <memory>

#include <gz/sim/System.hh>

namespace gz_camera_zoom
{
class CameraZoomPrivate;

/// \brief Gazebo Sim system plugin that drives continuous optical zoom
/// (horizontal field-of-view) on the camera sensor it is attached to.
///
/// Attach inside the <sensor type="camera"> element:
///
/// <plugin filename="CameraZoom" name="gz_camera_zoom::CameraZoom">
///   <topic>zoom/cmd</topic>
///   <min_hfov>0.05</min_hfov>
///   <max_hfov>1.05</max_hfov>
///   <zoom_rate>0.4</zoom_rate>
///   <cmd_timeout>0.5</cmd_timeout>
/// </plugin>
///
/// Publish a gz.msgs.Double on the command topic: positive values zoom in
/// (narrow the field of view), negative values zoom out, magnitude scales
/// the rate up to zoom_rate rad/s. Publishing 0.0 stops the zoom; the
/// cmd_timeout watchdog also stops it if no command arrives in time.
class CameraZoom :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPostUpdate
{
  public: CameraZoom();

  public: ~CameraZoom() override;

  public: void Configure(
              const gz::sim::Entity &_entity,
              const std::shared_ptr<const sdf::Element> &_sdf,
              gz::sim::EntityComponentManager &_ecm,
              gz::sim::EventManager &_eventMgr) override;

  public: void PostUpdate(
              const gz::sim::UpdateInfo &_info,
              const gz::sim::EntityComponentManager &_ecm) override;

  private: std::unique_ptr<CameraZoomPrivate> dataPtr;
};
}  // namespace gz_camera_zoom

#endif  // GZ_CAMERA_ZOOM__CAMERA_ZOOM_HH_
