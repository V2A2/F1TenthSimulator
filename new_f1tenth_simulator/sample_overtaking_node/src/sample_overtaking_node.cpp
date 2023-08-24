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
#include <road_description/road_config.h>
using namespace std;
class SampleOvertaking{
  private:
    ros::NodeHandle n;
    ros::Publisher drive_pub;
    ros::Subscriber model_sub;
    ros::Subscriber scan_sub;
    std::string carName = "";
    Road road = get_default_road();
    std::vector<int> overtakingLane;
    std::vector<int> previousLane;
    std::vector<float> filteredLidarReadings;
    sensor_msgs::LaserScanConstPtr lidar;
    long timeOfLastCall = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    int drivingState = 0; 
    // States: 
    // 0 = normal driving
    // 1 = merging into side lane 
    // 2 = in side lane
    // 3 = merging back into normal lane
    long actionTimer = 0;
    bool canTurnBackIntoLane = false;
    int wait_time = 0;
    double max_safe_speed = 0;
  public:
    SampleOvertaking(){
        n = ros::NodeHandle("~");
        std::string drive_topic, model_topic, scan_topic;
        n.getParam("nav_drive_topic", drive_topic);
        n.getParam("scan_topic", scan_topic);
        n.getParam("/model_topic", model_topic);
        n.getParam("car_name", carName);
        n.getParam("wait_time", wait_time);
        n.getParam("/max_safe_speed", max_safe_speed);
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 1);
        model_sub = n.subscribe(model_topic, 1, &SampleOvertaking::model_state_callback,this);
        scan_sub = n.subscribe(scan_topic, 1, &SampleOvertaking::scan_callback,this);
    }

    ~SampleOvertaking()
    {
    }

    bool isCarInLane(Point car,Point firstPoint,Point secondPoint){
      double angleToRotateBy =-1 * (car.rotation - M_PI_2);

      double pointXBeforeRotationFirst = firstPoint.x - car.x;
      double pointYBeforeRotationFirst = firstPoint.y - car.y;
      double newXForPointFirst = pointXBeforeRotationFirst * cos(angleToRotateBy) - pointYBeforeRotationFirst *sin(angleToRotateBy);
      double newYForPointFirst = pointYBeforeRotationFirst * cos(angleToRotateBy) + pointXBeforeRotationFirst *sin(angleToRotateBy);

      double pointXBeforeRotationSecond = secondPoint.x - car.x;
      double pointYBeforeRotationSecond = secondPoint.y - car.y;
      double newXForPointSecond = pointXBeforeRotationSecond * cos(angleToRotateBy) - pointYBeforeRotationSecond *sin(angleToRotateBy);
      double newYForPointSecond = pointYBeforeRotationSecond * cos(angleToRotateBy) + pointXBeforeRotationSecond *sin(angleToRotateBy);

      double distanceFromCenter = abs((newXForPointFirst + newXForPointSecond) / 2);
      double distanceBetweenPoints = abs(newXForPointFirst) + abs(newXForPointSecond);
      return (distanceFromCenter / distanceBetweenPoints) < 0.2; // find good ratio
    }

    bool isItSafeToTurn(Point car, Point firstPoint,Point secondPoint){
      double safeVerticalDistanceInLane = 1;
      double heightAdjustment = safeVerticalDistanceInLane/2;
      Point middlePointOfLane;
      middlePointOfLane.rotation = firstPoint.rotation + secondPoint.rotation /2;
      middlePointOfLane.x = (firstPoint.x + secondPoint.x) /2;
      middlePointOfLane.y = (firstPoint.y + secondPoint.y) /2;
      middlePointOfLane.z = (firstPoint.z + secondPoint.z) /2;
      //transform middle position to car coordinate frame
      double angleToRotateBy =-1 * (car.rotation - M_PI_2);
      double pointXBeforeRotation = middlePointOfLane.x - car.x;
      double pointYBeforeRotation = middlePointOfLane.y - car.y;
      double newXForPoint = pointXBeforeRotation * cos(angleToRotateBy) - pointYBeforeRotation *sin(angleToRotateBy);
      double newYForPoint = pointYBeforeRotation * cos(angleToRotateBy) + pointXBeforeRotation *sin(angleToRotateBy);
      double hypotenuse = sqrt(pow(newXForPoint,2)+pow(newYForPoint,2));
      //find angle of middle point in car frame
      double angleOfLaneMiddlePointInCarFrame = acos(newXForPoint/hypotenuse);
      //find how far the lidar should sweep on both sides to check if a car is there
      double angleToConsiderRanges = atan(heightAdjustment/hypotenuse);
      // subtract M_PI_2 to change standard coordinate plane to lidar frame where angle of zero is in the top direction
      double angleUpperBound = angleOfLaneMiddlePointInCarFrame + angleToConsiderRanges - M_PI_2;
      double angleLowerBound = angleOfLaneMiddlePointInCarFrame - angleToConsiderRanges - M_PI_2;
      // find what range is safe in that sweeping area
      double safeSweepingRange = sqrt(pow(hypotenuse,2)+pow(heightAdjustment,2));
      //loop through lidar to see if safe
      bool isSafeToTurn = true;
      
      for(int i = 0; i < static_cast<int>(filteredLidarReadings.size());i++){
        double angleChecking = lidar->angle_min + lidar->angle_increment * i;
        if(angleChecking < angleUpperBound && angleChecking > angleLowerBound){
          if(filteredLidarReadings.at(i)<safeSweepingRange){
            isSafeToTurn = false;
          }
        }
      }
      return isSafeToTurn;

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
      if(adjustment>.4){
        adjustment=.4;
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
      return angleToAdjustToCurvature *1.4; //1.2 tries to account for future curvature
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

    void setSwitchLaneInformation(Point currentCarLocation){
      //eventually this will need to change for differnt road structures
      //not guarenteed to follow disjoint tracks / lines
      std::vector<int> leftLines = this->road.getLanesToTheLeftOfPoint(currentCarLocation);
      std::vector<int> rightLines = this->road.getLanesToTheRightOfPoint(currentCarLocation);
      std::vector<int> lines = this->road.closestLineIndexes(currentCarLocation);
      if(static_cast<int>(leftLines.size())>=2){
        overtakingLane.clear();
        overtakingLane.push_back(leftLines.at(0));
        overtakingLane.push_back(leftLines.at(1));
        previousLane.clear();
        previousLane.push_back(lines.at(0));
        previousLane.push_back(lines.at(1));
      }
      else if(static_cast<int>(rightLines.size())>=2){
        overtakingLane.clear();
        overtakingLane.push_back(rightLines.at(0));
        overtakingLane.push_back(rightLines.at(1));
        previousLane.clear();
        previousLane.push_back(lines.at(0));
        previousLane.push_back(lines.at(1));
      }else{
        drivingState = 0;
      }
    }

    double safeSpeedToFollowAt(Point carLocation){
      // naive version that needs to be improved
      if(lidar != nullptr){
        double minDistance = lidar->range_max;
        for(int i = 0; i < static_cast<int>(filteredLidarReadings.size());i++){
          if(filteredLidarReadings.at(i)<minDistance){
            minDistance = filteredLidarReadings.at(i);
          }
        }
        double minSafeLidarVelocity = sqrt(2*1.7*std::max(0.0,minDistance-0.5));
        double minSafeRoadVelocity = this->road.maxium_speed_for_car(carLocation);
        return min(minSafeRoadVelocity,minSafeLidarVelocity);
      }
      return 0;
    }

    void followRoad(Point currentCarLocation){
      double desiredSteeringAngle = 0;
      double desiredSpeed = 0;


      if(drivingState == 0){
        std::vector<int> closestLines = this->road.closestLineIndexes(currentCarLocation);
        if(closestLines.size()>=2){
          Point closestPoint = this->road.roadLines.at(closestLines.at(0)).getClosestPointOnRoad(currentCarLocation);
          Point secondClosestPoint = this->road.roadLines.at(closestLines.at(1)).getClosestPointOnRoad(currentCarLocation);
          desiredSteeringAngle = steering_angle_from_points(currentCarLocation,closestPoint,secondClosestPoint);
          desiredSpeed = safeSpeedToFollowAt(currentCarLocation);
          long currentTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
          if(desiredSpeed >=1.3){
            actionTimer = 0;
          }
          if(desiredSpeed<3 && lidar != nullptr){
            actionTimer += currentTime -timeOfLastCall;
            if(actionTimer>wait_time){
              // actionTimer=0;
              // drivingState = 1;
              // setSwitchLaneInformation(currentCarLocation);
            }
          }
        }
      }

      if(drivingState == 1){
        Point closestPoint = this->road.roadLines.at(overtakingLane.at(0)).getClosestPointOnRoad(currentCarLocation);
        Point secondClosestPoint = this->road.roadLines.at(overtakingLane.at(1)).getClosestPointOnRoad(currentCarLocation);
        desiredSteeringAngle = steering_angle_from_points(currentCarLocation,closestPoint,secondClosestPoint);
        desiredSpeed = 4;
        if(isCarInLane(currentCarLocation,closestPoint,secondClosestPoint)){
          drivingState = 2;
        }
      }

      if(drivingState == 2){
        Point closestPoint = this->road.roadLines.at(overtakingLane.at(0)).getClosestPointOnRoad(currentCarLocation);
        Point secondClosestPoint = this->road.roadLines.at(overtakingLane.at(1)).getClosestPointOnRoad(currentCarLocation);
        desiredSteeringAngle = steering_angle_from_points(currentCarLocation,closestPoint,secondClosestPoint);
        desiredSpeed = 4;
        Point closestPointInOtherLane = this->road.roadLines.at(previousLane.at(0)).getClosestPointOnRoad(currentCarLocation);
        Point secondClosestPointInOtherLane = this->road.roadLines.at(previousLane.at(1)).getClosestPointOnRoad(currentCarLocation);
        if(isItSafeToTurn(currentCarLocation,closestPointInOtherLane,secondClosestPointInOtherLane)){
          if(canTurnBackIntoLane){
            long currentTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            actionTimer += currentTime -timeOfLastCall;
            if(actionTimer>500){
              actionTimer=0;
              drivingState = 3;
              canTurnBackIntoLane = false;
            }
          }
        }else{
          canTurnBackIntoLane = true;
        }
      }

      if(drivingState == 3){
        Point closestPoint = this->road.roadLines.at(previousLane.at(0)).getClosestPointOnRoad(currentCarLocation);
        Point secondClosestPoint = this->road.roadLines.at(previousLane.at(1)).getClosestPointOnRoad(currentCarLocation);
        desiredSteeringAngle = steering_angle_from_points(currentCarLocation,closestPoint,secondClosestPoint);
        desiredSpeed = 4;
        if(isCarInLane(currentCarLocation,closestPoint,secondClosestPoint)){
          drivingState = 0;
        }
      }
      // ROS_INFO("Driving State: %s",std::to_string(drivingState).c_str());
      ackermann_msgs::AckermannDriveStamped drive_st_msg;
      ackermann_msgs::AckermannDrive drive_msg;
      drive_msg.steering_angle = desiredSteeringAngle;
      drive_msg.speed = std::min(desiredSpeed, max_safe_speed);
      // ROS_INFO("Steering Angle %s",std::to_string(steeringAngle).c_str());
      // ROS_INFO("Speed %s",std::to_string(carSpeed).c_str());
      drive_st_msg.drive = drive_msg;
      drive_pub.publish(drive_st_msg);
      timeOfLastCall = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
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
};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "overtaking_node", ros::init_options::AnonymousName);
    SampleOvertaking rw;
    ros::spin();
    return 0;
}
