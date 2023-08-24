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
#include "road_description/road_config.h"

class SampleLaneFollower{
    private:
    ros::NodeHandle n;
    ros::Publisher drive_pub;
    ros::Subscriber model_sub;
    std::string carName = "";
    double speed = 0;
    Road road = get_default_road();
    public:
    SampleLaneFollower(){
        n = ros::NodeHandle("~");
        std::string drive_topic, model_topic;
        n.getParam("nav_drive_topic", drive_topic);
        n.getParam("/model_topic",model_topic);
        n.getParam("car_name",carName);
        n.getParam("speed",speed);
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 1);
        model_sub = n.subscribe(model_topic, 1, &SampleLaneFollower::model_state_callback,this);
    }
    ~SampleLaneFollower()
    {
    }

  double linear_distance_stearing_adjustments(Point car, Point firstPoint,Point secondPoint){

    double angleToRotateBy =-1 * (car.rotation - M_PI_2);

    double pointXBeforeRotationFirst = firstPoint.x - car.x;
    double pointYBeforeRotationFirst = firstPoint.y - car.y;
    double newXForPointFirst = pointXBeforeRotationFirst * cos(angleToRotateBy) - pointYBeforeRotationFirst *sin(angleToRotateBy);
    double newYForPointFirst = pointYBeforeRotationFirst * cos(angleToRotateBy) + pointXBeforeRotationFirst *sin(angleToRotateBy);

    double pointXBeforeRotationSecond = secondPoint.x - car.x;
    double pointYBeforeRotationSecond = secondPoint.y - car.y;
    double newXForPointSecond = pointXBeforeRotationSecond * cos(angleToRotateBy) - pointYBeforeRotationSecond *sin(angleToRotateBy);
    double newYForPointSecond = pointYBeforeRotationSecond * cos(angleToRotateBy) + pointXBeforeRotationSecond *sin(angleToRotateBy);

    double averageDistance = (newXForPointFirst + newXForPointSecond) / 2;
    int sign = 1;
    if(averageDistance > 0){
      sign = -1;
    }
    double adjustmentConstant = 1.6; //was 0.8
    double adjustment =abs(averageDistance) * adjustmentConstant;
    if(adjustment>.8){
      adjustment=.8;
    }
    adjustment*=sign;
    return adjustment;
  }
  double roadCurve(Point car, Point firstRoadLine,Point secondRoadLine){
    double angleToAdjustToCurvature = 0;
    double adjustedAngleForRoadLines = (firstRoadLine.rotation + secondRoadLine.rotation) /2;
    if(abs(firstRoadLine.rotation - secondRoadLine.rotation)>M_PI){
      adjustedAngleForRoadLines = firstRoadLine.rotation;
    }
    if(abs(car.rotation-adjustedAngleForRoadLines)<M_PI){
        angleToAdjustToCurvature = adjustedAngleForRoadLines - car.rotation;
    }else{
      if(car.rotation<adjustedAngleForRoadLines){
        angleToAdjustToCurvature = adjustedAngleForRoadLines - car.rotation - M_PI * 2;
      }else{
        angleToAdjustToCurvature = adjustedAngleForRoadLines - car.rotation + M_PI * 2;
      }
    }
    return angleToAdjustToCurvature;
  }
  double steering_angle_from_points(Point car, Point firstRoadLine,Point secondRoadLine){
    //For Curve adjustments
    double angleToAdjustToCurvature = 0;
    //To Keep Car in Center of Lane
    double angleToAdjustForCentering = 0;
    
    // Curve adjustments
    while(car.rotation  <0){
      car.rotation  += M_PI *2;
    }
    while(car.rotation  >M_PI *2){
      car.rotation  -= M_PI *2;
    }
    while(firstRoadLine.rotation  <0){
      firstRoadLine.rotation  += M_PI *2;
    }
    while(firstRoadLine.rotation  >M_PI *2){
      firstRoadLine.rotation  -= M_PI *2;
    }
    while(secondRoadLine.rotation <0){
      secondRoadLine.rotation += M_PI *2;
    }
    while(secondRoadLine.rotation >M_PI *2){
      secondRoadLine.rotation -= M_PI *2;
    }
    angleToAdjustToCurvature = roadCurve(car,firstRoadLine,secondRoadLine);
    Point reverseCarDirection;
    reverseCarDirection.rotation = car.rotation + M_PI;
    if(reverseCarDirection.rotation > M_PI*2){
      reverseCarDirection.rotation-= M_PI*2;
    }
    reverseCarDirection.x = car.x;
    reverseCarDirection.y = car.y;
    reverseCarDirection.z = car.z;
    double curveInOppositeDirection = roadCurve(reverseCarDirection,firstRoadLine,secondRoadLine);
    if(abs(curveInOppositeDirection)<abs(angleToAdjustToCurvature)){
      angleToAdjustToCurvature = curveInOppositeDirection;
    }
    //Lane Line Adjustments
    angleToAdjustForCentering = linear_distance_stearing_adjustments(car,firstRoadLine,secondRoadLine);
    if(abs(angleToAdjustForCentering)>0.8){
      if(angleToAdjustForCentering >0){
        angleToAdjustForCentering = 0.8;
      }else{
        angleToAdjustForCentering = -0.8;
      }
    }
    return angleToAdjustToCurvature + angleToAdjustForCentering;
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
          std::vector<int> closestLines = this->road.closestLineIndexes(currentCarLocation);
          Point closestPoint = this->road.roadLines.at(closestLines.at(0)).getClosestPointOnRoad(currentCarLocation);
          Point secondClosestPoint = this->road.roadLines.at(closestLines.at(1)).getClosestPointOnRoad(currentCarLocation);
          double desiredSteeringAngle = steering_angle_from_points(currentCarLocation,closestPoint,secondClosestPoint);
          ackermann_msgs::AckermannDriveStamped drive_st_msg;
          ackermann_msgs::AckermannDrive drive_msg;
          drive_msg.steering_angle = desiredSteeringAngle;
          drive_msg.speed = this->speed;
          drive_st_msg.drive = drive_msg;
          drive_pub.publish(drive_st_msg);
        }
      }
    }
  }
};
int main(int argc, char ** argv) {
    ros::init(argc, argv, "lane_follower_node", ros::init_options::AnonymousName);
    SampleLaneFollower rw;
    ros::spin();
    return 0;
}
