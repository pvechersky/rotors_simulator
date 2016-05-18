#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/gui.hh>
#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/transport/transport.hh>

namespace gazebo
{
// Constants
static const math::Vector3 kChaseCamOffset(-5.0, 0.0, 1.0);
static const math::Vector3 kForwardCamOffset(0.5, 0.0, 0.5);

class GAZEBO_VISIBLE TestGUI : public GUIPlugin {
 Q_OBJECT

 public:
  TestGUI();
  virtual ~TestGUI();

 protected slots:
  void OnForwardButton();
  void OnChaseButton();

 private:
  void OnPoseMsg(ConstPosePtr &msg);

  rendering::UserCameraPtr user_cam_;

  transport::NodePtr gazebo_node_;
  transport::SubscriberPtr model_pose_sub_;

  math::Vector3 cam_offset_;
};
}
