#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include "road_description/road_config.h"

namespace gazebo
{
class TrackPlugin : public WorldPlugin
{
  public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
  {
    Road road = get_default_road();
    
    sdf::SDF sphereSDF;
    sphereSDF.SetFromString(road.getRoadModel());
    // Demonstrate using a custom model name.
    sdf::ElementPtr model = sphereSDF.Root()->GetElement("model");
    //model->GetAttribute("name")->SetFromString("unique_sphere"); change name
    _parent->InsertModelSDF(sphereSDF);
  }
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(TrackPlugin)
}
