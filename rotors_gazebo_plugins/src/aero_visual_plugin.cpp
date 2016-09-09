#include "rotors_gazebo_plugins/aero_visual_plugin.h"

namespace gazebo {

AeroVisualPlugin::AeroVisualPlugin()
    : VisualPlugin(),
      line(NULL) {
}

AeroVisualPlugin::~AeroVisualPlugin() {
  this->rosnode_->shutdown();
  delete this->rosnode_;
}

void AeroVisualPlugin::Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf) {
  this->visual_ = _parent;

  // start ros node
  if (!ros::isInitialized()) {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "gazebo_aero_visual", ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  std::string visual_namespace;

  if (_sdf->HasElement("visualNamespace"))
    visual_namespace = _sdf->GetElement("visualNamespace")->Get<std::string>();
  else
    gzerr << "[aero_visual_plugin] Please specify a visualNamespace.\n";

  this->rosnode_ = new ros::NodeHandle(visual_namespace);
  this->force_sub_ = this->rosnode_->subscribe("fw_forces", 1, &AeroVisualPlugin::ForceCallback, this);

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectRender(boost::bind(&AeroVisualPlugin::OnUpdate, this));

  force_visual_->InsertMesh("axis_shaft");
}

void AeroVisualPlugin::OnUpdate() {
  ros::spinOnce();

  this->line = this->visual_->CreateDynamicLine(rendering::RENDERING_LINE_STRIP);

        //TODO: Get the current link position
        //link_pose = CurrentLinkPose();
        //TODO: Get the current end position
        //endpoint = CalculateEndpointOfForceVector(link_pose, force_msg);

  // Add two points to a connecting line strip from link_pose to endpoint
  this->line->AddPoint(math::Vector3(0.5, 0.0, 0.5));
  this->line->AddPoint(math::Vector3(1.5, 0.0, 0.5));
  this->line->AddPoint(math::Vector3(1.2, -0.2, 0.5));
  this->line->AddPoint(math::Vector3(1.2, 0.2, 0.5));
  this->line->AddPoint(math::Vector3(1.5, 0.0, 0.5));

  // set the Material of the line, in this case to purple
  this->line->setMaterial("Gazebo/Purple");
  this->line->setVisibilityFlags(GZ_VISIBILITY_GUI);
  this->visual_->SetVisible(true);

  // Force visual
  /*rendering::VisualPtr force;
  force.reset(new rendering::Visual("FORCE_VISUAL", force_visual_, false));
  force->Load();

  // Force shaft
  rendering::VisualPtr force_shaft_visual(new rendering::Visual("FORCE_SHAFT", force, false));
  force_shaft_visual->Load();

  force_shaft_visual->AttachMesh("axis_shaft");

  Ogre::SceneNode *scene_node = force_shaft_visual->GetSceneNode();

  std::cout << "Here" << std::endl;

  if (scene_node) {
    Ogre::MovableObject *shaftObj = scene_node->getAttachedObject(0);
    shaftObj->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);
    shaftObj->getUserObjectBindings().setUserAny(Ogre::Any(std::string(force->GetName())));
    std::cout << "Test" << std::endl;
    force_shaft_visual->SetPosition(math::Vector3(0, 0, 0.1));

    force->SetMaterial("Gazebo/DarkOrangeTransparentOverlay");
    force->GetSceneNode()->setInheritScale(false);
  }*/
}

void AeroVisualPlugin::ForceCallback(const geometry_msgs::WrenchStampedConstPtr &force_msg) {
}

GZ_REGISTER_VISUAL_PLUGIN(AeroVisualPlugin);
}
