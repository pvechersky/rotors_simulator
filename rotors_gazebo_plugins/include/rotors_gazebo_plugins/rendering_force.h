#ifndef RENDERING_FORCE_H
#define RENDERING_FORCE_H

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

class GAZEBO_VISIBLE RenderingForce : public rendering::Visual {
 public:
  RenderingForce(const std::string &_name, rendering::VisualPtr _parent_vis);
  virtual ~RenderingForce();

  void Load();

  void SetForce(const math::Vector3 &force_vector);

  void UpdateForcesVisual();

  void Resize();

 private:
  math::Quaternion QuaternionFromVector(const math::Vector3 &vec);

  std::string selected_material_;
  std::string unselected_material_;

  math::Vector3 force_vector_;

  std::vector<rendering::VisualPtr> force_visuals_;

  rendering::VisualPtr parent_visual_;

  std::mutex mutex_;

  bool initialized_size_;

  double scale_x_;
  double scale_y_;

  double position_scale_x_;
  double position_scale_y_;
  double position_scale_z_;
};

typedef boost::shared_ptr<RenderingForce> RenderingForcePtr;
}

#endif // RENDERING_FORCE_H
