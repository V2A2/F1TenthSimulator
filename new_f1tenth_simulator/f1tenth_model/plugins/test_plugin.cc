#ifndef _TESTWHEEL_PLUGIN_HH_
#define _TESTWHEEL_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <thread>

namespace gazebo
{
  class TestPlugin : public ModelPlugin
  { 
    public:
    physics::ModelPtr model;

    TestPlugin() {}
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      gazebo::physics::CollisionPtr test = _model->GetLink(_model->GetName() +"::base")->GetChildCollision("collision");
      //  std::cerr<< "\nX: " << test->BoundingBox().XLength();
      //  std::cerr<< "\nY: " << test->BoundingBox().YLength();
      //  std::cerr<< "\nZ: " << test->BoundingBox().ZLength();
       
      //  std::cerr<< _model->GetWorld()->Models().at(0)->GetName();
      //  std::cerr<< _model->GetWorld()->Models().at(1)->GetName();
    }
  };
  
  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(TestPlugin)
}
#endif
