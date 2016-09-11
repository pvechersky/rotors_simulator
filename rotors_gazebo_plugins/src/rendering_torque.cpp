#include "rotors_gazebo_plugins/rendering_torque.h"

namespace gazebo {

RenderingTorque::RenderingTorque(const std::string &_name, rendering::VisualPtr _parent_vis)
    : rendering::Visual(_name, _parent_vis),
      initialized_size_(false),
      initialized_meshes_(false) {
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

  angle_x_ = 0;
  angle_y_ = 0;
  angle_z_ = 0;

  torque_vector_ = math::Vector3::Zero;

  rendering::VisualPtr visual_x;
  rendering::VisualPtr visual_y;
  rendering::VisualPtr visual_z;
  torque_visuals_.push_back(visual_x);
  torque_visuals_.push_back(visual_y);
  torque_visuals_.push_back(visual_z);

  CreateMeshes();

  this->SetVisibilityFlags(GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE);
  this->Resize();
  this->UpdateTorqueVisual();
}

void RenderingTorque::CreateMeshes() {
  double angle_x = torque_vector_.x * 2.0 * M_PI;
  double angle_y = torque_vector_.y * 2.0 * M_PI;
  double angle_z = torque_vector_.z * 2.0 * M_PI;

  const std::string current_time = common::Time::GetWallTimeAsISOString();

  if ((fabs(angle_x - angle_x_) > 0.03) || !initialized_meshes_) {
    angle_x_ = angle_x;

    if (initialized_meshes_)
      torque_visuals_[0]->SetVisible(false);

    // X Torque visual
    torque_visuals_[0].reset(new rendering::Visual(
        this->GetName() + "_TORQUE_VISUAL_X_", shared_from_this(), false));
    torque_visuals_[0]->Load();

    // X Torque arc
    std::string mesh_name = "x_torque_arc_" + current_time;
    common::Mesh *arc_mesh_x = CreateArc(mesh_name, 0.1, 0.13, 0.03, 1, 24, angle_x);
    this->InsertMesh(arc_mesh_x);

    rendering::VisualPtr torque_arc_visual_x(new rendering::Visual(
        this->GetName() + "_TORQUE_ARC_X_", torque_visuals_[0], false));
    torque_arc_visual_x->Load();

    torque_arc_visual_x->AttachMesh(mesh_name);
    Ogre::MovableObject *arc_obj_x =
        torque_arc_visual_x->GetSceneNode()->getAttachedObject(0);
    arc_obj_x->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);
    arc_obj_x->getUserObjectBindings().setUserAny(
        Ogre::Any(std::string(torque_visuals_[0]->GetName())));

    // X Torque arrow
    rendering::VisualPtr torque_arrow_visual_x(new rendering::Visual(
        this->GetName() + "_TORQUE_HEAD_X_", torque_visuals_[0], false));
    torque_arrow_visual_x->Load();

    torque_arrow_visual_x->AttachMesh("axis_head");
    Ogre::MovableObject *torque_head_obj_x =
        torque_arrow_visual_x->GetSceneNode()->getAttachedObject(0);
    torque_head_obj_x->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);
    torque_head_obj_x->getUserObjectBindings().setUserAny(
        Ogre::Any(std::string(torque_visuals_[0]->GetName())));

    torque_arrow_visual_x->SetScale(math::Vector3(2, 2, 1));
    torque_arrow_visual_x->SetPosition(math::Vector3(-0.04, 0.125, 0));
    math::Quaternion quat_x(0, -M_PI/2.0, 0);
    torque_arrow_visual_x->SetRotation(quat_x);

    torque_visuals_[0]->SetMaterial(selected_material_);
    torque_visuals_[0]->GetSceneNode()->setInheritScale(false);
  }

  if ((fabs(angle_y - angle_y_) > 0.03) || !initialized_meshes_) {
    angle_y_ = angle_y;

    if (initialized_meshes_)
      torque_visuals_[1]->SetVisible(false);

    // Y Torque visual
    torque_visuals_[1].reset(new rendering::Visual(
        this->GetName() + "_TORQUE_VISUAL_Y_", shared_from_this(), false));
    torque_visuals_[1]->Load();

    // Y Torque arc
    std::string mesh_name = "y_torque_arc_" + current_time;
    common::Mesh *arc_mesh_y = CreateArc(mesh_name, 0.1, 0.13, 0.03, 1, 24, angle_y);
    this->InsertMesh(arc_mesh_y);

    rendering::VisualPtr torque_arc_visual_y(new rendering::Visual(
        this->GetName() + "_TORQUE_ARC_Y_", torque_visuals_[1], false));
    torque_arc_visual_y->Load();

    torque_arc_visual_y->AttachMesh(mesh_name);
    Ogre::MovableObject *arc_obj_y =
        torque_arc_visual_y->GetSceneNode()->getAttachedObject(0);
    arc_obj_y->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);
    arc_obj_y->getUserObjectBindings().setUserAny(
        Ogre::Any(std::string(torque_visuals_[1]->GetName())));

    // Y Torque arrow
    rendering::VisualPtr torque_arrow_visual_y(new rendering::Visual(
        this->GetName() + "_TORQUE_HEAD_Y_", torque_visuals_[1], false));
    torque_arrow_visual_y->Load();

    torque_arrow_visual_y->AttachMesh("axis_head");
    Ogre::MovableObject *torque_head_obj_y =
        torque_arrow_visual_y->GetSceneNode()->getAttachedObject(0);
    torque_head_obj_y->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);
    torque_head_obj_y->getUserObjectBindings().setUserAny(
        Ogre::Any(std::string(torque_visuals_[1]->GetName())));

    torque_arrow_visual_y->SetScale(math::Vector3(2, 2, 1));
    torque_arrow_visual_y->SetPosition(math::Vector3(-0.04, 0.125, 0));
    math::Quaternion quat_y(0, -M_PI/2.0, 0);
    torque_arrow_visual_y->SetRotation(quat_y);

    torque_visuals_[1]->SetMaterial(selected_material_);
    torque_visuals_[1]->GetSceneNode()->setInheritScale(false);
  }

  if ((fabs(angle_z - angle_z_) > 0.03) || !initialized_meshes_) {
    angle_z_ = angle_z;

    if (initialized_meshes_)
      torque_visuals_[2]->SetVisible(false);

    // Y Torque visual
    torque_visuals_[2].reset(new rendering::Visual(
        this->GetName() + "_TORQUE_VISUAL_Z_", shared_from_this(), false));
    torque_visuals_[2]->Load();

    // Z Torque arc
    std::string mesh_name = "z_torque_arc_" + current_time;
    common::Mesh *arc_mesh_z = CreateArc(mesh_name, 0.1, 0.13, 0.03, 1, 24, angle_z);
    this->InsertMesh(arc_mesh_z);

    rendering::VisualPtr torque_arc_visual_z(new rendering::Visual(
        this->GetName() + "_TORQUE_ARC_Z_", torque_visuals_[2], false));
    torque_arc_visual_z->Load();

    torque_arc_visual_z->AttachMesh(mesh_name);
    Ogre::MovableObject *arc_obj_z =
        torque_arc_visual_z->GetSceneNode()->getAttachedObject(0);
    arc_obj_z->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);
    arc_obj_z->getUserObjectBindings().setUserAny(
        Ogre::Any(std::string(torque_visuals_[2]->GetName())));

    // Z Torque arrow
    rendering::VisualPtr torque_arrow_visual_z(new rendering::Visual(
        this->GetName() + "_TORQUE_HEAD_Z_", torque_visuals_[2], false));
    torque_arrow_visual_z->Load();

    torque_arrow_visual_z->AttachMesh("axis_head");
    Ogre::MovableObject *torque_head_obj_z =
        torque_arrow_visual_z->GetSceneNode()->getAttachedObject(0);
    torque_head_obj_z->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);
    torque_head_obj_z->getUserObjectBindings().setUserAny(
        Ogre::Any(std::string(torque_visuals_[2]->GetName())));

    torque_arrow_visual_z->SetScale(math::Vector3(2, 2, 1));
    torque_arrow_visual_z->SetPosition(math::Vector3(-0.04, 0.125, 0));
    math::Quaternion quat_z(0, M_PI/2.0, 0);
    torque_arrow_visual_z->SetRotation(quat_z);

    torque_visuals_[2]->SetMaterial(selected_material_);
    torque_visuals_[2]->GetSceneNode()->setInheritScale(false);
  }

  initialized_meshes_ = true;
}

void RenderingTorque::SetTorque(const math::Vector3 &torque_vector) {
  torque_vector_ = torque_vector;

  this->CreateMeshes();

  this->Resize();

  this->UpdateTorqueVisual();
}

void RenderingTorque::UpdateTorqueVisual() {
  if (torque_visuals_.empty()) {
    gzwarn << "No torque visuals.\n";
    return;
  }

  for (int i = 0; i < torque_visuals_.size(); i++) {
    rendering::VisualPtr visual = torque_visuals_[i];

    math::Vector3 torque_norm;
    double position_scale;
    double torque_component = 0;

    if (i == 0) {
      torque_norm = math::Vector3::UnitX;
      position_scale = position_scale_x_;
      torque_component = torque_vector_.x;
    }
    else if (i == 1) {
      torque_norm = math::Vector3::UnitY;
      position_scale = position_scale_y_;
      torque_component = torque_vector_.y;
    }
    else {
      torque_norm = math::Vector3::UnitZ;
      position_scale = position_scale_z_;
      torque_component = torque_vector_.z;
    }

    if (torque_component < 0)
      torque_norm *= -1.0;

    if (fabs(torque_component) < 0.05) {
      visual->SetVisible(false);
      continue;
    }
    else
      visual->SetVisible(true);

    math::Quaternion quat = this->QuaternionFromVector(torque_norm);
    visual->SetRotation(quat * math::Quaternion(math::Vector3(0, M_PI/2.0, 0)));

    math::Vector3 position = torque_norm.GetAbs() * position_scale;
    visual->SetPosition(position);
  }
}

void RenderingTorque::Resize() {
  if (!this || torque_visuals_.empty()) {
    gzwarn << "RenderingTorque is incomplete.\n";
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  double link_size = std::max(0.1, parent_visual_->GetBoundingBox().GetSize().GetLength());

  if (!initialized_size_) {
    scale_x_ = link_size;
    scale_y_ = link_size;
    scale_z_ = link_size;

    position_scale_x_ = parent_visual_->GetBoundingBox().GetSize().x;
    position_scale_y_ = parent_visual_->GetBoundingBox().GetSize().y;
    position_scale_z_ = parent_visual_->GetBoundingBox().GetSize().z;

    initialized_size_ = true;
  }

  for (int i = 0; i < torque_visuals_.size(); i++) {
    torque_visuals_[i]->SetScale(math::Vector3(scale_x_, scale_y_, scale_z_));
  }
}

math::Quaternion RenderingTorque::QuaternionFromVector(const math::Vector3 &vec) {
  double roll = 0;
  double pitch = -atan2(vec.z, sqrt(pow(vec.x, 2) + pow(vec.y, 2)));
  double yaw = atan2(vec.y, vec.x);

  return math::Quaternion(roll, pitch, yaw);
}

common::Mesh* RenderingTorque::CreateArc(const std::string &name, float inner_radius,
    float outer_radius, float height, int rings_input, int segments, double arc)
{
  math::Vector3 vert;
  math::Vector3 norm;
  unsigned int vertice_index = 0;
  int ring;
  int seg;

  // Needs at lest 1 ring, and 3 segments
  int rings = std::max(rings_input, 1);
  int segs = std::max(segments, 3);

  float delta_seg_angle = (arc / segs);

  float radius = 0;
  radius = outer_radius;

  if (common::MeshManager::Instance()->HasMesh(name))
    return NULL;

  common::Mesh *mesh = new common::Mesh();
  mesh->SetName(name);
  common::MeshManager::Instance()->AddMesh(mesh);
  common::SubMesh *sub_mesh = new common::SubMesh();
  mesh->AddSubMesh(sub_mesh);

  // Generate the group of rings for the outsides of the cylinder
  for (ring = 0; ring <= rings; ++ring)
  {
    vert.z = ring * height/rings - height/2.0;

    // Generate the group of segments for the current ring
    for (seg = 0; seg <= segs; ++seg)
    {
      vert.y = radius * cosf(seg * delta_seg_angle);
      vert.x = radius * sinf(seg * delta_seg_angle);

      // TODO: Don't think these normals are correct.
      norm = vert;
      norm.Normalize();

      // Add one vertex to the strip which makes up the arc
      sub_mesh->AddVertex(vert);
      sub_mesh->AddNormal(norm);
      sub_mesh->AddTexCoord(
          static_cast<float>(seg) / static_cast<float>(segs),
          static_cast<float>(ring) / static_cast<float>(rings));

      // outer triangles connecting ring [ring] to ring [ring + 1]
      if (ring != rings)
      {
        if (seg != 0)
        {
          sub_mesh->AddIndex(vertice_index + segs + 1);
          sub_mesh->AddIndex(vertice_index);
          sub_mesh->AddIndex(vertice_index + segs);
        }
        if (seg != segs)
        {
          sub_mesh->AddIndex(vertice_index + segs + 1);
          sub_mesh->AddIndex(vertice_index + 1);
          sub_mesh->AddIndex(vertice_index);
        }
      }
      // ring [rings] is the edge of the top cap
      else if (seg != segs)
      {
        // These indices form the top cap
        sub_mesh->AddIndex(vertice_index);
        sub_mesh->AddIndex(vertice_index + segs + 1);
        sub_mesh->AddIndex(vertice_index + 1);

        sub_mesh->AddIndex(vertice_index + 1);
        sub_mesh->AddIndex(vertice_index + segs + 1);
        sub_mesh->AddIndex(vertice_index + segs + 2);
      }

      // ring [0] is the edge of the bottom cap
      if (ring == 0 && seg < segs)
      {
        // These indices form the bottom cap
        sub_mesh->AddIndex(vertice_index + 1);
        sub_mesh->AddIndex(vertice_index + (segs + 1) * (((rings + 1) * 2) - 1));
        sub_mesh->AddIndex(vertice_index);

        sub_mesh->AddIndex(vertice_index + (segs + 1) * (((rings + 1) * 2) - 1) + 1);
        sub_mesh->AddIndex(vertice_index + (segs + 1) * (((rings + 1) * 2) - 1));
        sub_mesh->AddIndex(vertice_index + 1);
      }

      vertice_index++;
    }
  }

  // Generate the group of rings for the inside of the cylinder
  radius = inner_radius;
  for (ring = 0; ring <= rings; ++ring)
  {
    vert.z = (height / 2.0) - (ring * height/rings);

    // Generate the group of segments for the current ring
    for (seg = 0; seg <= segs; ++seg)
    {
      vert.y = radius * cosf(seg * delta_seg_angle);
      vert.x = radius * sinf(seg * delta_seg_angle);

      // TODO: Don't think these normals are correct.
      norm = vert;
      norm.Normalize();

      // Add one vertex to the strip which makes up the arc
      sub_mesh->AddVertex(vert);
      sub_mesh->AddNormal(norm);
      sub_mesh->AddTexCoord(
          static_cast<float>(seg) / static_cast<float>(segs),
          static_cast<float>(ring) / static_cast<float>(rings));

      // inner triangles connecting ring [ring] to ring [ring + 1]
      if (ring != rings)
      {
        // each vertex has six indices (2 triangles)
        if (seg != 0)
        {
          sub_mesh->AddIndex(vertice_index + segs + 1);
          sub_mesh->AddIndex(vertice_index);
          sub_mesh->AddIndex(vertice_index + segs);
        }
        if (seg != segs)
        {
          sub_mesh->AddIndex(vertice_index + segs + 1);
          sub_mesh->AddIndex(vertice_index + 1);
          sub_mesh->AddIndex(vertice_index);
        }
      }
      vertice_index++;
    }
  }

  mesh->RecalculateNormals();

  return mesh;
}

}
