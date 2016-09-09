#include "rotors_gazebo_plugins/rendering_torque.h"

namespace gazebo {

RenderingTorque::RenderingTorque(const std::string &_name, rendering::VisualPtr _parent_vis)
    : rendering::Visual(_name, _parent_vis),
      initialized_size_(false) {
  parent_visual_ = _parent_vis;
}

RenderingTorque::~RenderingTorque() {

}

void RenderingTorque::Load()
{
  Visual::Load();

  if (!this->GetScene()) {
    gzerr << "Visual has no scene, not loading.\n";
    return;
  }

  selected_material_ = "Gazebo/Orange";

  this->InsertMesh("axis_shaft");
  this->InsertMesh("axis_head");

  // X Force visual
  /*rendering::VisualPtr force_visual_x;
  force_visual_x.reset(new rendering::Visual(
      this->GetName() + "_FORCE_VISUAL_X_", shared_from_this(), false));
  force_visual_x->Load();

  // Force shaft
  rendering::VisualPtr force_shaft_visual_x(new rendering::Visual(
      this->GetName() + "_FORCE_SHAFT_X_", force_visual_x, false));
  force_shaft_visual_x->Load();

  force_shaft_visual_x->AttachMesh("axis_shaft");
  Ogre::MovableObject *shaft_obj_x =
      force_shaft_visual_x->GetSceneNode()->getAttachedObject(0);
  shaft_obj_x->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);
  shaft_obj_x->getUserObjectBindings().setUserAny(
        Ogre::Any(std::string(force_visual_x->GetName())));
  force_shaft_visual_x->SetPosition(math::Vector3(0, 0, 0.1));

  // Force head
  rendering::VisualPtr force_head_visual_x(new rendering::Visual(
      this->GetName() + "_FORCE_HEAD_X_", force_visual_x, false));
  force_head_visual_x->Load();

  force_head_visual_x->AttachMesh("axis_head");
  Ogre::MovableObject *head_obj_x =
      force_head_visual_x->GetSceneNode()->getAttachedObject(0);
  head_obj_x->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);
  head_obj_x->getUserObjectBindings().setUserAny(
        Ogre::Any(std::string(force_visual_x->GetName())));
  force_head_visual_x->SetPosition(math::Vector3(0, 0, 0.24));

  force_visuals_.push_back(force_visual_x);

  // Y Force visual
  rendering::VisualPtr force_visual_y;
  force_visual_y.reset(new rendering::Visual(
      this->GetName() + "_FORCE_VISUAL_Y_", shared_from_this(), false));
  force_visual_y->Load();

  // Force shaft
  rendering::VisualPtr force_shaft_visual_y(new rendering::Visual(
      this->GetName() + "_FORCE_SHAFT_Y_", force_visual_y, false));
  force_shaft_visual_y->Load();

  force_shaft_visual_y->AttachMesh("axis_shaft");
  Ogre::MovableObject *shaft_obj_y =
      force_shaft_visual_y->GetSceneNode()->getAttachedObject(0);
  shaft_obj_y->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);
  shaft_obj_y->getUserObjectBindings().setUserAny(
        Ogre::Any(std::string(force_visual_y->GetName())));
  force_shaft_visual_y->SetPosition(math::Vector3(0, 0, 0.1));

  // Force head
  rendering::VisualPtr force_head_visual_y(new rendering::Visual(
      this->GetName() + "_FORCE_HEAD_Y_", force_visual_y, false));
  force_head_visual_y->Load();

  force_head_visual_y->AttachMesh("axis_head");
  Ogre::MovableObject *head_obj_y =
      force_head_visual_y->GetSceneNode()->getAttachedObject(0);
  head_obj_y->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);
  head_obj_y->getUserObjectBindings().setUserAny(
        Ogre::Any(std::string(force_visual_y->GetName())));
  force_head_visual_y->SetPosition(math::Vector3(0, 0, 0.24));

  force_visuals_.push_back(force_visual_y);

  // Z Force visual
  rendering::VisualPtr force_visual_z;
  force_visual_z.reset(new rendering::Visual(
      this->GetName() + "_FORCE_VISUAL_Z_", shared_from_this(), false));
  force_visual_z->Load();

  // Force shaft
  rendering::VisualPtr force_shaft_visual_z(new rendering::Visual(
      this->GetName() + "_FORCE_SHAFT_Z_", force_visual_z, false));
  force_shaft_visual_z->Load();

  force_shaft_visual_z->AttachMesh("axis_shaft");
  Ogre::MovableObject *shaft_obj_z =
      force_shaft_visual_z->GetSceneNode()->getAttachedObject(0);
  shaft_obj_z->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);
  shaft_obj_z->getUserObjectBindings().setUserAny(
        Ogre::Any(std::string(force_visual_z->GetName())));
  force_shaft_visual_z->SetPosition(math::Vector3(0, 0, 0.1));

  // Force head
  rendering::VisualPtr force_head_visual_z(new rendering::Visual(
      this->GetName() + "_FORCE_HEAD_Z_", force_visual_z, false));
  force_head_visual_z->Load();

  force_head_visual_z->AttachMesh("axis_head");
  Ogre::MovableObject *head_obj_z =
      force_head_visual_z->GetSceneNode()->getAttachedObject(0);
  head_obj_z->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);
  head_obj_z->getUserObjectBindings().setUserAny(
        Ogre::Any(std::string(force_visual_z->GetName())));
  force_head_visual_z->SetPosition(math::Vector3(0, 0, 0.24));

  force_visuals_.push_back(force_visual_z);*/

  for (int i = 0; i < torque_visuals_.size(); i++) {
    torque_visuals_[i]->SetMaterial(selected_material_);
    torque_visuals_[i]->GetSceneNode()->setInheritScale(false);
  }

  torque_vector_ = math::Vector3::Zero;

  this->SetVisibilityFlags(GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE);
  this->Resize();
  this->UpdateTorqueVisual();
}

void RenderingTorque::SetTorque(const math::Vector3 &torque_vector) {
  torque_vector_ = torque_vector;

  this->Resize();

  this->UpdateTorqueVisual();
}

void RenderingTorque::UpdateTorqueVisual() {
  if (torque_visuals_.empty()) {
    gzwarn << "No torque visuals.\n";
    return;
  }

  /*for (int i = 0; i < force_visuals_.size(); i++) {
    math::Vector3 force_norm;
    double position_scale;

    if (i == 0) {
      force_norm = math::Vector3::UnitX;
      position_scale = position_scale_x_;
      if (force_vector_.x < 0)
        force_norm *= -1.0;
    }
    else if (i == 1) {
      force_norm = math::Vector3::UnitY;
      position_scale = position_scale_y_;
      if (force_vector_.y < 0)
        force_norm *= -1.0;
    }
    else {
      force_norm = math::Vector3::UnitZ;
      position_scale = position_scale_z_;
      if (force_vector_.z < 0)
        force_norm *= -1.0;
    }

    rendering::VisualPtr visual = force_visuals_[i];

    // Set rotation in the vector direction
    math::Quaternion quat = this->QuaternionFromVector(force_norm.GetAbs());
    visual->SetRotation(quat * math::Quaternion(math::Vector3(0, M_PI/2.0, 0)));

    //math::Vector3 position = force_norm * 0.14 * visual->GetScale().z;
    math::Vector3 position = force_norm * position_scale;

    visual->SetMaterial(selected_material_);
    visual->SetPosition(position);
  }*/
}

void RenderingTorque::Resize() {
  if (!this || torque_visuals_.empty()) {
    gzwarn << "RenderingTorque is incomplete.\n";
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  double link_size = std::max(0.1, parent_visual_->GetBoundingBox().GetSize().GetLength());

  if (!initialized_size_) {
    scale_x_ = 2 * link_size;
    scale_y_ = 2 * link_size;

    position_scale_x_ = parent_visual_->GetBoundingBox().GetSize().x * 0.5;
    position_scale_y_ = parent_visual_->GetBoundingBox().GetSize().y * 0.5;
    position_scale_z_ = parent_visual_->GetBoundingBox().GetSize().z * 0.5;

    initialized_size_ = true;
  }

  /*for (int i = 0; i < torque_visuals_.size(); i++) {
    double scale_z;

    if (i == 0)
      scale_z = force_vector_.x;
    else if (i == 1)
      scale_z = force_vector_.y;
    else
      scale_z = force_vector_.z;

    force_visuals_[i]->SetScale(math::Vector3(scale_x_, scale_y_, 2 * scale_z));
  }*/
}

math::Quaternion RenderingTorque::QuaternionFromVector(const math::Vector3 &vec) {
  double roll = 0;
  double pitch = -atan2(vec.z, sqrt(pow(vec.x, 2) + pow(vec.y, 2)));
  double yaw = atan2(vec.y, vec.x);

  return math::Quaternion(roll, pitch, yaw);
}

}
