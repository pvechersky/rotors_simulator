#ifndef RENDERING_TORQUE_H
#define RENDERING_TORQUE_H

#include <math.h>
#include <mutex>

#include "gazebo/common/MeshManager.hh"
#include <gazebo/gazebo.hh>
#include "gazebo/rendering/Material.hh"
#include "gazebo/rendering/MovableText.hh"
#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/SelectionObj.hh"

namespace gazebo {

class GAZEBO_VISIBLE RenderingTorque : public rendering::Visual {
 public:
  RenderingTorque(const std::string &_name, rendering::VisualPtr _parent_vis);
  virtual ~RenderingTorque();

  void Load();

  void SetTorque(const math::Vector3 &torque_vector);

  void UpdateTorqueVisual();

  void Resize();

  common::Mesh *CreateArc(const std::string &name,
                 float inner_radius,
                 float outer_radius,
                 float height,
                 int rings_input,
                 int segments,
                 double arc);

 private:
  math::Quaternion QuaternionFromVector(const math::Vector3 &vec);

  std::string selected_material_;
  std::string unselected_material_;

  math::Vector3 torque_vector_;

  std::vector<rendering::VisualPtr> torque_visuals_;

  rendering::VisualPtr parent_visual_;

  std::mutex mutex_;

  bool initialized_size_;

  double scale_x_;
  double scale_y_;
  double scale_z_;

  double position_scale_x_;
  double position_scale_y_;
  double position_scale_z_;
};

typedef boost::shared_ptr<RenderingTorque> RenderingTorquePtr;
}

#endif // RENDERING_TORQUE_H
