#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/LinkState.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ModelState.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <sensor_msgs/LaserScan.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <fstream>
#include <std_msgs/Int64.h>
#include <filesystem>
#include <road_description/road_config.h>

using namespace std;

ofstream followingFile;
ofstream laneStateFile;
ofstream turningDirectionFile;

int followingImageCount = 0;
int laneStateImageCount = 0;
int turningDirectionCount = 0;
bool turningLeft = false;

void setFileCount(){
   std::filesystem::path followingFilePath { "/home/ryan/TrainingData/simulator_following_data/photos" };
   for (auto& p : std::filesystem::directory_iterator(followingFilePath))
   {
      ++followingImageCount;
   }
   std::filesystem::path laneStateFilePath { "/home/ryan/TrainingData/simulator_lane_state/photos" };
   for (auto& p : std::filesystem::directory_iterator(laneStateFilePath))
   {
      ++laneStateImageCount;
   }
   std::filesystem::path turningDirectionFilePath { "/home/ryan/TrainingData/simulator_turning_direction/photos" };
   for (auto& p : std::filesystem::directory_iterator(turningDirectionFilePath))
   {
      ++turningDirectionCount;
   }
 }
class OvertakingRecorder{
  private:
    ros::NodeHandle n;
    ros::Publisher drive_pub;
    ros::Subscriber model_sub;
    ros::Subscriber scan_sub;
    ros::Subscriber controls_sub;
    image_transport::ImageTransport image_transport;
    image_transport::Subscriber image_sub_;
    std::string carName = "";
    Road road = get_road(2);
    std::vector<int> overtakingLane;
    std::vector<int> previousLane;
    std::vector<float> filteredLidarReadings;
    sensor_msgs::LaserScanConstPtr lidar;
    cv::Mat currentSemanticSegmentationImage;
    long timeOfLastCall = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    bool switchingLanes = false; 
    long actionTimer = 0;
    double lastRecordedSteeringAngle = 0;
    double lastRecordedSpeed = -1;
    bool worldRunning = false;
  public:
    OvertakingRecorder():image_transport(n){
        n = ros::NodeHandle("~");
        std::string drive_topic, model_topic, scan_topic;
        n.getParam("nav_drive_topic", drive_topic);
        n.getParam("scan_topic", scan_topic);
        n.getParam("/model_topic",model_topic);
        n.getParam("car_name",carName);
        
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 1);
        model_sub = n.subscribe(model_topic, 1, &OvertakingRecorder::model_state_callback,this);
        scan_sub = n.subscribe(scan_topic, 1, &OvertakingRecorder::scan_callback,this);
        image_sub_ = image_transport.subscribe("/semantic_segmentation/car_output", 1, &OvertakingRecorder::image_callback, this);
        controls_sub = n.subscribe("/simulator/command/input",1000,&OvertakingRecorder::command_callback,this);
    }
    ~OvertakingRecorder()
    {
      followingFile.close();
      laneStateFile.close();
      turningDirectionFile.close();
    }
    void followRoad(Point currentCarLocation){
      ackermann_msgs::AckermannDriveStamped drive_st_msg;
      ackermann_msgs::AckermannDrive drive_msg;
      drive_msg.steering_angle = 0;
      drive_msg.speed = 0;
      drive_st_msg.drive = drive_msg;
      drive_pub.publish(drive_st_msg);
      timeOfLastCall = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
      lastRecordedSteeringAngle = drive_msg.steering_angle;
      lastRecordedSpeed = drive_msg.speed;
    }
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
      lidar = msg;
      filteredLidarReadings.clear();
      for(int i = 0; i<static_cast<int>(lidar->ranges.size());i++){
          float filteredInput = lidar->ranges.at(i);
          if(!std::isfinite(filteredInput)){
            // may need to switch to zero for values lower than range min if this is a problem
            filteredInput = lidar->range_max;
          }
          filteredLidarReadings.push_back(filteredInput);
        }
    }
    void model_state_callback(const gazebo_msgs::ModelStatesConstPtr& msg){
      if(msg->pose.size()>0){
        for(int i = 0; i < static_cast<int>(msg->name.size());i++){
          if(msg->name.at(i)==carName){
            geometry_msgs::Quaternion orientation =  msg->pose[i].orientation;
            tf2::Quaternion quaternion(orientation.x,orientation.y,orientation.z,orientation.w);
            tf2::Matrix3x3 m(quaternion);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            geometry_msgs::Point position=  msg->pose[i].position;
            Point currentCarLocation;
            currentCarLocation.rotation = yaw;
            currentCarLocation.x = position.x;
            currentCarLocation.y = position.y;
            currentCarLocation.z = position.z;
            followRoad(currentCarLocation);
          }
        }
      }
    }
    void command_callback(const std_msgs::Int64ConstPtr &msg){
        if(((int)msg->data) == 116){
            if(worldRunning){
            }else{
              ROS_INFO("World Resumed: IOSTREAM OPENED");
              worldRunning = true;
            }
        }
    }
    void image_callback(const sensor_msgs::ImageConstPtr& msg){
      try
      {
        // for color camera
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1);
        cv::Mat semantic_segmentation_mask = cv_ptr->image;

      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
  }
};
int main(int argc, char ** argv) {
    setFileCount();
    ros::init(argc, argv, "overtaking_node", ros::init_options::AnonymousName);
    OvertakingRecorder rw;
    ros::spin();
    return 0;
}
