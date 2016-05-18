#include <rotors_gazebo_plugins/gui_plugin.h>

namespace gazebo {

TestGUI::TestGUI():
    GUIPlugin(),
    cam_offset_(kChaseCamOffset) {
  // Set the frame background and foreground colors
  setStyleSheet("QFrame { background-color : rgba(100, 100, 100, 255); color : white; }");

  // Create the main layout
  QHBoxLayout *mainLayout = new QHBoxLayout;

  // Create the frame to hold all the widgets
  QFrame *mainFrame = new QFrame();

  // Create the layout that sits inside the frame
  QVBoxLayout *frameLayout = new QVBoxLayout();

  // Create a push button, and connect it to the OnButton function
  QPushButton *forward_button = new QPushButton(tr("Forward"));
  QPushButton *chase_button = new QPushButton(tr("Chase"));
  connect(forward_button, SIGNAL(clicked()), this, SLOT(OnForwardButton()));
  connect(chase_button, SIGNAL(clicked()), this, SLOT(OnChaseButton()));

  // Add the buttons to the frame's layout
  frameLayout->addWidget(forward_button);
  frameLayout->addWidget(chase_button);

  // Add frameLayout to the frame
  mainFrame->setLayout(frameLayout);

  // Add the frame to the main layout
  mainLayout->addWidget(mainFrame);

  // Remove margins to reduce space
  frameLayout->setContentsMargins(0, 0, 0, 0);
  mainLayout->setContentsMargins(0, 0, 0, 0);

  setLayout(mainLayout);

  // Position and resize this widget
  move(10, 10);
  resize(120, 55);

  // Get a pointer to the active user camera
  user_cam_ = gui::get_active_camera();

  // Set up the Gazebo transport
  gazebo_node_ = transport::NodePtr(new transport::Node());
  gazebo_node_->Init("techpod");
  std::string topicname = "~/mav/pose";
  model_pose_sub_ = gazebo_node_->Subscribe(topicname, &TestGUI::OnPoseMsg, this);
}

TestGUI::~TestGUI() {
}

void TestGUI::OnForwardButton() {
  cam_offset_ = kForwardCamOffset;
}

void TestGUI::OnChaseButton() {
  cam_offset_ = kChaseCamOffset;
}

void TestGUI::OnPoseMsg(ConstPosePtr &msg) {
  math::Pose new_cam_pose;
  new_cam_pose.rot.w = msg->orientation().w();
  new_cam_pose.rot.x = msg->orientation().x();
  new_cam_pose.rot.y = msg->orientation().y();
  new_cam_pose.rot.z = msg->orientation().z();

  math::Vector3 cam_offset_body_ = new_cam_pose.rot.RotateVector(cam_offset_);

  new_cam_pose.pos.x = msg->position().x() + cam_offset_body_.x;
  new_cam_pose.pos.y = msg->position().y() + cam_offset_body_.y;
  new_cam_pose.pos.z = msg->position().z() + cam_offset_body_.z;

  user_cam_->SetWorldPose(new_cam_pose);
}

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(TestGUI);
}

