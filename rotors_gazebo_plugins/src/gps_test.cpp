#include "rotors_gazebo_plugins/gps_test.h"

namespace gazebo {

GpsTest::GpsTest()
    : ModelPlugin() {}

GpsTest::~GpsTest() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
}

void GpsTest::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  world_ = _model->GetWorld();

  std::string link_name;

  if (_sdf->HasElement("linkName"))
    link_name = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_gps_plugin] Please specify a linkName.\n";


  // Get the pointer to the link that holds the sensor.
  link_ = boost::dynamic_pointer_cast<physics::Link>(world_->GetByName(link_name));
  if (link_ == NULL)
    gzerr << "[gazebo_gps_plugin] Couldn't find specified link \"" << link_name << "\"\n";

  angle_ = 0.0;

  // Connect to the sensor update event.
  this->updateConnection_ =
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GpsTest::OnUpdate, this));
}

void GpsTest::OnUpdate() {
  math::Pose pose = link_->GetWorldPose();

  double x = -1000.0 + 1000.0 * cos(angle_);
  double y = 0.0 + 1000.0 * sin(angle_);

  angle_ += 0.001;

  pose.pos.x = x;
  pose.pos.y = y;

  link_->SetWorldPose(pose);
}

GZ_REGISTER_MODEL_PLUGIN(GpsTest);
}
