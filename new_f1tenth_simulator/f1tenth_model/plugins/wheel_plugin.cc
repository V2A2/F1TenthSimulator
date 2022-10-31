#ifndef _BACKWHEEL_PLUGIN_HH_
#define _BACKWHEEL_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
namespace gazebo
{
  class WheelPlugin : public ModelPlugin
  { 
    public:
    physics::ModelPtr model;
    common::PID pid;

    double velocity = 0;
    double steering_angle=0;
    double previous_angle=0;
    std::string modelName = "f1tenth_car";
    std::string upload_topic = "/vesc/high_level/ackermann_cmd_mux/input/nav_0";
    physics::JointPtr frontLeftWheelTurning;
    physics::JointPtr frontRightWheelTurning;
    physics::JointPtr frontLeftWheel;
    physics::JointPtr backLeftWheel;
    physics::JointPtr frontRightWheel;
    physics::JointPtr backRightWheel;
    
    /// \brief A node use for ROS transport
    std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief ROS subscriber
    ros::Subscriber rosVelocitySub;
    ros::Subscriber rosSteeringAngleSub;
    ros::Subscriber rosDrivingDataSub;

    /// \brief A ROS callbackqueue that helps process messages
    ros::CallbackQueue rosQueue;

    /// \brief A thread the keeps running the rosQueue
    std::thread rosQueueThread;
    ros::Rate r = ros::Rate(10);
    WheelPlugin() {
    }
    public: void SetVelocity(double velocity)
    {
      if(isnan(velocity)){
        velocity = 0;
      }
      if(isinf(velocity)){
        velocity = 0;
      }
      if(velocity<0){
        velocity = 0;
      }
      if(velocity> 7){
        velocity = 7;
      }
      this->model->GetJoint((this->modelName +"::back_left_joint"))->SetParam("fmax", 0, 2.8);
      this->model->GetJoint((this->modelName +"::back_left_joint"))->SetParam("vel", 0, 20 * velocity); // 20 converts m/s input to rad/s that wheels need
      this->model->GetJoint((this->modelName +"::back_right_joint"))->SetParam("fmax", 0, 2.8);
      this->model->GetJoint((this->modelName +"::back_right_joint"))->SetParam("vel", 0, 20 * velocity);
      this->model->GetJoint((this->modelName +"::front_left_joint"))->SetParam("fmax", 1, 2.8);
      this->model->GetJoint((this->modelName +"::front_left_joint"))->SetParam("vel", 0, 20 * velocity);
      this->model->GetJoint((this->modelName +"::front_right_joint"))->SetParam("fmax", 1, 2.8);
      this->model->GetJoint((this->modelName +"::front_right_joint"))->SetParam("vel", 0, 20 * velocity);
    }
    public: void SetSteeringAngle(double angle){
      if(isnan(angle)){
        angle = 0;
      }
      if(isinf(angle)){
        angle = 0;
      }
      double maximum_angle = 0.9;
      double turning_angle = angle;
      if(abs(angle)>=maximum_angle){//51 degrees
         turning_angle = angle / abs(angle) *maximum_angle;
      }
      (this->model)->GetJointController()->SetJointPosition((this->modelName +"::front_left_joint_turning"), turning_angle);
      (this->model)->GetJointController()->SetJointPosition((this->modelName +"::front_right_joint_turning"), turning_angle);
   }
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Safety check
      if (_model->GetJointCount() == 0)
      {
        std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
        return;
      }
      this->modelName = _model->GetName();
      if (_sdf->HasElement("velocity")){
         this->velocity = _sdf->Get<double>("velocity");
      }
      if (_sdf->HasElement("steering_angle")){
         this->steering_angle = _sdf->Get<double>("steering_angle");
      }
      if (_sdf->HasElement("upload_topic")){
         this->upload_topic = _sdf->Get<std::string>("upload_topic");
      }

      // Store the model pointer for convenience.
      this->model = _model;
      //Set PID Controller Values
      // gazebo::common::PID pid(3,1,0);
      // (this->model)->GetJointController()->SetPositionPID((this->modelName +"::front_left_joint_turning"), pid);
      // (this->model)->GetJointController()->SetPositionPID((this->modelName +"::front_right_joint_turning"), pid);
      // this->SetVelocity(this->velocity);
      // this->SetSteeringAngle(this->steering_angle);
      // Initialize ros, if it has not already bee initialized.
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
      ros::SubscribeOptions drivingDataSubscriberOptions =
        ros::SubscribeOptions::create<ackermann_msgs::AckermannDriveStamped>(
            this->upload_topic,
            1,
            boost::bind(&WheelPlugin::OnDrivingDataCallback, this, _1),
            ros::VoidPtr(), &this->rosQueue);
      this->rosDrivingDataSub = this->rosNode->subscribe(drivingDataSubscriberOptions);
      // Spin up the queue helper thread.
      this->rosQueueThread =
        std::thread(std::bind(&WheelPlugin::QueueThread, this));
    }
    public: void OnDrivingDataCallback(const ackermann_msgs::AckermannDriveStampedConstPtr & msg){

      this->SetSteeringAngle(msg->drive.steering_angle);
      this->SetVelocity(msg->drive.speed);
    }
    /// \brief ROS helper function that processes messages
    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
        this->r.sleep();
      }
    }
  };
  
  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(WheelPlugin)
}
#endif
