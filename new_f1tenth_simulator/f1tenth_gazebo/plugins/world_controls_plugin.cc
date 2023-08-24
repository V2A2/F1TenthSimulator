#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <std_msgs/Int64.h>
namespace gazebo
{
class WorldControlsPlugin : public WorldPlugin
{
    /// \brief A node use for ROS transport
    std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief ROS subscriber
    ros::Subscriber rosCommandSub;

    /// \brief A ROS callbackqueue that helps process messages
    ros::CallbackQueue rosQueue;

    /// \brief A thread the keeps running the rosQueue
    std::thread rosQueueThread;

    std::string subscribe_topic = "/simulator/command/input";

    physics::WorldPtr world;
  public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
  {
      this->world = _parent;
    if (!ros::isInitialized())
    {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client",
            ros::init_options::NoSigintHandler);
    }

    // Create our ROS node. This acts in a similar manner to
    // the Gazebo node
    this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
    ros::SubscribeOptions commandDataSubscriberOptions =
    ros::SubscribeOptions::create<std_msgs::Int64>(
        this->subscribe_topic,
        1,
        boost::bind(&WorldControlsPlugin::CommandCallback, this, _1),
        ros::VoidPtr(), &this->rosQueue);
    this->rosCommandSub = this->rosNode->subscribe(commandDataSubscriberOptions);
    // Spin up the queue helper thread.
    this->rosQueueThread =
    std::thread(std::bind(&WorldControlsPlugin::QueueThread, this));
  }
    /// \brief ROS helper function that processes messages
    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }
    void CommandCallback(const std_msgs::Int64ConstPtr &msg){
        if(((int)msg->data) == 32){
            physics::pause_world(this->world,!this->world->IsPaused());
        }
    }
};
// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(WorldControlsPlugin)
}