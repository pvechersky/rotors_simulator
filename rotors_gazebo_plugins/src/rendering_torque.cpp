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

  // X Torque visual
  rendering::VisualPtr torque_visual_x;
  torque_visual_x.reset(new rendering::Visual(
      this->GetName() + "_TORQUE_VISUAL_X_", shared_from_this(), false));
  torque_visual_x->Load();

  // X Torque arc
  common::Mesh *arc_mesh_x = CreateArc("x_torque_arc", 0.1, 0.13, 0.03, 1, 24, 1.0 * M_PI);
  this->InsertMesh(arc_mesh_x);

  rendering::VisualPtr torque_arc_visual_x(new rendering::Visual(
      this->GetName() + "_TORQUE_ARC_X_", torque_visual_x, false));
  torque_arc_visual_x->Load();

  torque_arc_visual_x->AttachMesh("x_torque_arc");
  Ogre::MovableObject *arc_obj_x =
      torque_arc_visual_x->GetSceneNode()->getAttachedObject(0);
  arc_obj_x->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);
  arc_obj_x->getUserObjectBindings().setUserAny(
      Ogre::Any(std::string(torque_visual_x->GetName())));

  // X Torque arrow
  rendering::VisualPtr torque_arrow_visual_x(new rendering::Visual(
      this->GetName() + "_TORQUE_HEAD_X_", torque_visual_x, false));
  torque_arrow_visual_x->Load();

  torque_arrow_visual_x->AttachMesh("axis_head");
  Ogre::MovableObject *torque_head_obj_x =
      torque_arrow_visual_x->GetSceneNode()->getAttachedObject(0);
  torque_head_obj_x->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);
  torque_head_obj_x->getUserObjectBindings().setUserAny(
      Ogre::Any(std::string(torque_visual_x->GetName())));

  torque_arrow_visual_x->SetScale(math::Vector3(3, 3, 1));
  torque_arrow_visual_x->SetPosition(math::Vector3(-0.04, 0.125, 0));
  math::Quaternion quat_x(0, -M_PI/2.0, 0);
  torque_arrow_visual_x->SetRotation(quat_x);

  torque_visual_x->SetMaterial(selected_material_);
  torque_visual_x->GetSceneNode()->setInheritScale(false);

  torque_visuals_.push_back(torque_visual_x);

  // Y Torque visual
  rendering::VisualPtr torque_visual_y;
  torque_visual_y.reset(new rendering::Visual(
      this->GetName() + "_TORQUE_VISUAL_Y_", shared_from_this(), false));
  torque_visual_y->Load();

  // Y Torque arc
  common::Mesh *arc_mesh_y = CreateArc("y_torque_arc", 0.1, 0.13, 0.03, 1, 24, 1.0 * M_PI);
  this->InsertMesh(arc_mesh_y);

  rendering::VisualPtr torque_arc_visual_y(new rendering::Visual(
      this->GetName() + "_TORQUE_ARC_Y_", torque_visual_y, false));
  torque_arc_visual_y->Load();

  torque_arc_visual_y->AttachMesh("y_torque_arc");
  Ogre::MovableObject *arc_obj_y =
      torque_arc_visual_y->GetSceneNode()->getAttachedObject(0);
  arc_obj_y->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);
  arc_obj_y->getUserObjectBindings().setUserAny(
      Ogre::Any(std::string(torque_visual_y->GetName())));

  // Y Torque arrow
  rendering::VisualPtr torque_arrow_visual_y(new rendering::Visual(
      this->GetName() + "_TORQUE_HEAD_Y_", torque_visual_y, false));
  torque_arrow_visual_y->Load();

  torque_arrow_visual_y->AttachMesh("axis_head");
  Ogre::MovableObject *torque_head_obj_y =
      torque_arrow_visual_x->GetSceneNode()->getAttachedObject(0);
  torque_head_obj_y->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);
  torque_head_obj_y->getUserObjectBindings().setUserAny(
      Ogre::Any(std::string(torque_visual_y->GetName())));

  torque_arrow_visual_y->SetScale(math::Vector3(3, 3, 1));
  torque_arrow_visual_y->SetPosition(math::Vector3(-0.04, 0.125, 0));
  math::Quaternion quat_y(0, -M_PI/2.0, 0);
  torque_arrow_visual_y->SetRotation(quat_y);

  torque_visual_y->SetMaterial(selected_material_);
  torque_visual_y->GetSceneNode()->setInheritScale(false);

  torque_visuals_.push_back(torque_visual_y);

  // Z Torque visual
  rendering::VisualPtr torque_visual_z;
  torque_visual_z.reset(new rendering::Visual(
      this->GetName() + "_TORQUE_VISUAL_Z_", shared_from_this(), false));
  torque_visual_z->Load();

  // Z Torque arc
  common::Mesh *arc_mesh_z = CreateArc("z_torque_arc", 0.1, 0.13, 0.03, 1, 24, 1.0 * M_PI);
  this->InsertMesh(arc_mesh_z);

  rendering::VisualPtr torque_arc_visual_z(new rendering::Visual(
      this->GetName() + "_TORQUE_ARC_Z_", torque_visual_z, false));
  torque_arc_visual_z->Load();

  torque_arc_visual_z->AttachMesh("z_torque_arc");
  Ogre::MovableObject *arc_obj_z =
      torque_arc_visual_z->GetSceneNode()->getAttachedObject(0);
  arc_obj_z->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);
  arc_obj_z->getUserObjectBindings().setUserAny(
      Ogre::Any(std::string(torque_visual_z->GetName())));

  // Z Torque arrow
  rendering::VisualPtr torque_arrow_visual_z(new rendering::Visual(
      this->GetName() + "_TORQUE_HEAD_Z_", torque_visual_z, false));
  torque_arrow_visual_z->Load();

  torque_arrow_visual_z->AttachMesh("axis_head");
  Ogre::MovableObject *torque_head_obj_z =
      torque_arrow_visual_z->GetSceneNode()->getAttachedObject(0);
  torque_head_obj_z->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);
  torque_head_obj_z->getUserObjectBindings().setUserAny(
      Ogre::Any(std::string(torque_visual_z->GetName())));

  torque_arrow_visual_z->SetScale(math::Vector3(3, 3, 1));
  torque_arrow_visual_z->SetPosition(math::Vector3(-0.04, 0.125, 0));
  math::Quaternion quat_z(0, -M_PI/2.0, 0);
  torque_arrow_visual_z->SetRotation(quat_z);

  torque_visual_z->SetMaterial(selected_material_);
  torque_visual_z->GetSceneNode()->setInheritScale(false);

  torque_visuals_.push_back(torque_visual_z);

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

  for (int i = 0; i < torque_visuals_.size(); i++) {
    math::Vector3 torque_norm;

    if (i == 0) {
      torque_norm = math::Vector3::UnitX;
      if (torque_vector_.x < 0)
        torque_norm *= -1.0;
    }
    else if (i == 1)
      torque_norm = math::Vector3::UnitY;
    else
      torque_norm = math::Vector3::UnitZ;

    rendering::VisualPtr visual = torque_visuals_[i];

    math::Quaternion quat = this->QuaternionFromVector(torque_norm);
    visual->SetRotation(quat * math::Quaternion(math::Vector3(0, M_PI/2.0, 0)));

    math::Vector3 position = torque_norm.GetAbs();
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

  // Close ends in case it's not a full circle
  /*if (!ignition::math::equal(arc, 2.0 * M_PI))
  {
    for (ring = 0; ring < rings; ++ring)
    {
      // Close beginning
      subMesh->AddIndex((segments+1)*(ring+1));
      subMesh->AddIndex((segments+1)*ring);
      subMesh->AddIndex((segments+1)*((rings+1)*2-2-ring));

      subMesh->AddIndex((segments+1)*((rings+1)*2-2-ring));
      subMesh->AddIndex((segments+1)*ring);
      subMesh->AddIndex((segments+1)*((rings+1)*2-1-ring));

      // Close end
      subMesh->AddIndex((segments+1)*((rings+1)*2-2-ring)+segments);
      subMesh->AddIndex((segments+1)*((rings+1)*2-1-ring)+segments);
      subMesh->AddIndex((segments+1)*(ring+1)+segments);

      subMesh->AddIndex((segments+1)*(ring+1)+segments);
      subMesh->AddIndex((segments+1)*((rings+1)*2-1-ring)+segments);
      subMesh->AddIndex((segments+1)*ring+segments);
    }
  }*/

  mesh->RecalculateNormals();

  return mesh;
}

}
