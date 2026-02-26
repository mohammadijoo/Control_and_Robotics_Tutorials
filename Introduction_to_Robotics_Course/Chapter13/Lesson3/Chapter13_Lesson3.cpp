// Minimal Gazebo model plugin (classic-style API)
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo {
class HelloSim : public ModelPlugin {
  physics::ModelPtr model;
  event::ConnectionPtr updateConn;

public:
  void Load(physics::ModelPtr _model, sdf::ElementPtr) {
    model = _model;
    updateConn = event::Events::ConnectWorldUpdateBegin(
      std::bind(&HelloSim::OnUpdate, this));
  }

  void OnUpdate() {
    // Read pose each sim step
    auto pose = model->WorldPose();
    std::cout << "z=" << pose.Pos().Z() << std::endl;
  }
};

GZ_REGISTER_MODEL_PLUGIN(HelloSim)
}
