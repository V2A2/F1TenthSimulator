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
using namespace std;
struct Point{
    double x;
    double y;
    double z;
    double rotation;
};
class RoadSegment{
    //TODO: Write some way to convert an xml like format to RoadSegments
    /*
    0:Line calulated by having one endpoint at x,y,z and then have the other endpoint at the end of the vector centered at x,y,z with length param1 and angle param2
    1:Circle Segment calulated by taking arc from circle centered at x,y,z with radius param1, between the angles start and end
    ALL angles will be in radians
    Designed from standard xy plane view where angle 0 is to the right
    */
    public:
        //measurements in meters
        double x;
        double y;
        double z;
        int classification; // 0 = line 1 = circle
        //Paramrs 1 for both cirlce and line, params 2-4 for circle creation
        double param1; // length for line radius for cirle
        double param2; // angle for line and start angle for circle
        double param3; // end angle for circle
        RoadSegment(double x, double y, double z, int classification, double param1, double param2, double param3){
            this->x = x;
            this->y = y;
            this->z = z;
            this->classification = classification;
            this->param1 = param1;
            this->param2 = param2;
            this->param3 = param3;
        }
        double getLengthOfSegment(){
            if(classification == 0){
                return param1;
            }
            if(classification == 1){
                return abs(param1 * (param2-param3));
            }
            return 0;
        }
        Point getPointXLengthInLine(double distanceIn){
            Point point;
            point.z = z;
            if(classification == 0){
                point.x = x + distanceIn * cos(param2);
                point.y = y + distanceIn * sin(param2);
                point.rotation = param2;
            }
            if(classification ==1){
                double angleToRevolve = distanceIn / param1;
                int direction = 1;
                if(param3<param2){
                    direction = -1;
                }
                double angleOfNewPoint = param2 + angleToRevolve * direction;
                point.x = x + param1 * cos(angleOfNewPoint);
                point.y = y + param1 * sin(angleOfNewPoint);
                
                point.rotation = angleOfNewPoint+ M_PI/2;
                if(param2>param3){
                    point.rotation = angleOfNewPoint - M_PI/2;
                }

            }
            return point;
        }
};
class RoadLine{
    public:
        std::vector<RoadSegment> roadSegments;
        bool dotted = false;
        bool yellowLine = false;
        int visualTagIndex = 0;
        std::vector<Point> points;
        double default_resolution_for_point_map = 0.01;
        RoadLine(std::vector<RoadSegment>segments,int elementTag){
            this->roadSegments = segments;
            dotted = false;
            yellowLine = false;
            visualTagIndex = elementTag;
            this->points = getRoadPoints(default_resolution_for_point_map);
        }
        RoadLine(std::vector<RoadSegment>segments,bool isDotted, bool isYellowLine, int elementTag){
            this->roadSegments = segments;
            dotted = isDotted;
            yellowLine = isYellowLine;
            visualTagIndex = elementTag;
            this->points = getRoadPoints(default_resolution_for_point_map);
        }
        std::vector<Point> getRoadPoints(double resolution){ // resolution is distance between points in meters
            std::vector<Point> listOfPoints;
            double lastDistance = 0;
            for(int i=0;i<static_cast<int>(roadSegments.size());i++){
                RoadSegment currentSegment = roadSegments.at(i);
                double currentLength = lastDistance;
                while(currentLength<=currentSegment.getLengthOfSegment()){
                    listOfPoints.push_back(currentSegment.getPointXLengthInLine(currentLength));
                    currentLength+=resolution;
                }
                lastDistance = currentLength -currentSegment.getLengthOfSegment();
            }
            return listOfPoints;
        }
        std::string getRoadSDFString(){
            if(this->dotted && this->yellowLine){
                return getRoadSDFStringDottedYellowLine();
            }
            if(this->dotted && !this->yellowLine){
                return getRoadSDFStringDottedWhiteLine();
            }
            if(!this->dotted && this->yellowLine){
                return getRoadSDFStringYellowLine();
            }
            return getRoadSDFStringWhiteLine();
        }
        Point getClosestPointOnRoad(Point referencePoint){
          Point closestPoint= this->points.at(0);
          double minDistanceToPoint = distanceBetweenPoints(closestPoint,referencePoint);
          for(int i = 0;i<static_cast<int>(this->points.size());i++){
            if(distanceBetweenPoints(referencePoint,this->points.at(i))<minDistanceToPoint){
              minDistanceToPoint = distanceBetweenPoints(referencePoint,this->points.at(i));
              closestPoint = this->points.at(i);
            }
          }
          return closestPoint;
        }

        double distanceToClosestPoint(Point reference){
          return distanceBetweenPoints(reference,getClosestPointOnRoad(reference));
        }

        std::string getRoadSDFStringYellowLine(){
            std::string roadString = "";
            std::vector<Point> pointsToDisplay = getRoadPoints(0.1);
            for (int index = 0; index<static_cast<int>(pointsToDisplay.size());index++){
                Point point = pointsToDisplay.at(index);
                roadString+="<visual name ='visual"+ std::to_string(this->visualTagIndex)+"-"+std::to_string(index)+"'><pose>";
                roadString+= "" +std::to_string(point.x) + " " + std::to_string(point.y) + " " + std::to_string(point.z) + " 0 0 " +std::to_string(point.rotation);
                roadString+="</pose><geometry><plane><normal>0 0 1</normal> <size>0.1 0.03</size></plane></geometry><material><ambient>0 0 0 1</ambient><diffuse>0 0 0 1</diffuse><specular>0 0 0 0</specular><emissive>1 0.9 0.2 1</emissive></material></visual>";
            }
            return roadString;
        }
        std::string getRoadSDFStringWhiteLine(){
            std::string roadString = "";
            std::vector<Point> pointsToDisplay = getRoadPoints(0.1);
            for (int index = 0; index<static_cast<int>(pointsToDisplay.size());index++){
                Point point = pointsToDisplay.at(index);
                roadString+="<visual name ='visual"+ std::to_string(this->visualTagIndex)+"-"+std::to_string(index)+"'><pose>";
                roadString+= "" +std::to_string(point.x) + " " + std::to_string(point.y) + " " + std::to_string(point.z) + " 0 0 " +std::to_string(point.rotation);
                roadString+="</pose><geometry><plane><normal>0 0 1</normal> <size>0.1 0.03</size></plane></geometry><material><ambient>0 0 0 1</ambient><diffuse>0 0 0 1</diffuse><specular>0 0 0 0</specular><emissive>1 1 1 1</emissive></material></visual>";
            }
            return roadString;
        }
        std::string getRoadSDFStringDottedYellowLine(){
            std::string roadString = "";
            std::vector<Point> pointsToDisplay = getRoadPoints(0.3);
            for (int index = 0; index<static_cast<int>(pointsToDisplay.size());index++){
                Point point = pointsToDisplay.at(index);
                roadString+="<visual name ='visual"+ std::to_string(this->visualTagIndex)+"-"+std::to_string(index)+"'><pose>";
                roadString+= "" +std::to_string(point.x) + " " + std::to_string(point.y) + " " + std::to_string(point.z) + " 0 0 " +std::to_string(point.rotation);
                roadString+="</pose><geometry><plane><normal>0 0 1</normal> <size>0.1 0.03</size></plane></geometry><material><ambient>0 0 0 1</ambient><diffuse>0 0 0 1</diffuse><specular>0 0 0 0</specular><emissive>1 0.9 0.2 1</emissive></material></visual>";
            }
            return roadString;
        }
        std::string getRoadSDFStringDottedWhiteLine(){
            std::string roadString = "";
            std::vector<Point> pointsToDisplay = getRoadPoints(0.3);
            for (int index = 0; index<static_cast<int>(pointsToDisplay.size());index++){
                Point point = pointsToDisplay.at(index);
                roadString+="<visual name ='visual"+ std::to_string(this->visualTagIndex)+"-"+std::to_string(index)+"'><pose>";
                roadString+= "" +std::to_string(point.x) + " " + std::to_string(point.y) + " " + std::to_string(point.z) + " 0 0 " +std::to_string(point.rotation);
                roadString+="</pose><geometry><plane><normal>0 0 1</normal> <size>0.1 0.03</size></plane></geometry><material><ambient>0 0 0 1</ambient><diffuse>0 0 0 1</diffuse><specular>0 0 0 0</specular><emissive>1 1 1 1</emissive></material></visual>";
            }

            return roadString;
        }
    private:
        double distanceBetweenPoints(Point first,Point second){
          return  sqrt(pow((first.x-second.x),2) + pow((first.y-second.y),2));
        }

};
class Road{
    public:
    std::vector<RoadLine> roadLines;
    Road(std::vector<RoadLine> lines){
        roadLines = lines;
    }
    Road(int trackNumber){
        if(trackNumber == 1){
            RoadSegment line1(0,0,0,0,5,M_PI/2,0);
            RoadSegment circle1(2,5,0,1,2,M_PI,0);
            RoadSegment line2(4,5,0,0,5,-M_PI/2,0);
            RoadSegment circle2(2,0,0,1,2,0,-M_PI);
            std::vector<RoadSegment> yellowSegments;
            yellowSegments.push_back(line1);
            yellowSegments.push_back(circle1);
            yellowSegments.push_back(line2);
            yellowSegments.push_back(circle2);
            RoadLine yellow(yellowSegments,false,true,0);

            RoadSegment line3(0-0.4,0,0,0,5,M_PI/2,0);
            RoadSegment circle3(2,5,0,1,2+0.4,M_PI,0);
            RoadSegment line4(4+0.4,5,0,0,5,-M_PI/2,0);
            RoadSegment circle4(2,0,0,1,2+0.4,0,-M_PI);
            std::vector<RoadSegment> innerWhiteSegments;
            innerWhiteSegments.push_back(line3);
            innerWhiteSegments.push_back(circle3);
            innerWhiteSegments.push_back(line4);
            innerWhiteSegments.push_back(circle4);
            RoadLine innerWhite(innerWhiteSegments,1);

            RoadSegment line5(0+0.4,0,0,0,5,M_PI/2,0);
            RoadSegment circle5(2,5,0,1,2-0.4,M_PI,0);
            RoadSegment line6(4-0.4,5,0,0,5,-M_PI/2,0);
            RoadSegment circle6(2,0,0,1,2-0.4,0,-M_PI);
            std::vector<RoadSegment> outerWhiteSegments;
            outerWhiteSegments.push_back(line5);
            outerWhiteSegments.push_back(circle5);
            outerWhiteSegments.push_back(line6);
            outerWhiteSegments.push_back(circle6);
            RoadLine outerWhite(outerWhiteSegments,2);

            std::vector<RoadLine> roadLines;
            roadLines.push_back(yellow);
            roadLines.push_back(innerWhite);
            roadLines.push_back(outerWhite);
            this->roadLines = roadLines;
        }
        if(trackNumber == 2){
            RoadSegment line1(-1,0,0,0,2,0,0);
            RoadSegment line2(5,4,0,0,4,0,0);
            RoadSegment line3(9,12,0,0,4,M_PI,0);
            RoadSegment line4(1,16,0,0,2,M_PI,0);
            RoadSegment line5(-5,12,0,0,4,M_PI,0);
            RoadSegment line6(-9,4,0,0,4,0,0);
            RoadSegment circle1(1,2,0,1,2,M_PI*3/2,M_PI*2);
            RoadSegment circle2(5,2,0,1,2,M_PI,M_PI_2);
            RoadSegment circle3(9,8,0,1,4,M_PI*3/2,M_PI*2);
            RoadSegment circle4(9,8,0,1,4,0,M_PI_2);
            RoadSegment circle5(5,14,0,1,2,M_PI*3/2,M_PI);
            RoadSegment circle6(1,14,0,1,2,0,M_PI_2);
            RoadSegment circle7(-1,14,0,1,2,M_PI_2,M_PI);
            RoadSegment circle8(-5,14,0,1,2,2*M_PI,M_PI*3/2);
            RoadSegment circle9(-9,8,0,1,4,M_PI_2,M_PI);
            RoadSegment circle10(-9,8,0,1,4,M_PI,M_PI*3/2);
            RoadSegment circle11(-5,2,0,1,2,M_PI_2,0);
            RoadSegment circle12(-1,2,0,1,2,M_PI,M_PI*3/2);
            std::vector<RoadSegment> yellowSegments;
            yellowSegments.push_back(line1);
            yellowSegments.push_back(circle1);
            yellowSegments.push_back(circle2);
            yellowSegments.push_back(line2);
            yellowSegments.push_back(circle3);
            yellowSegments.push_back(circle4);
            yellowSegments.push_back(line3);
            yellowSegments.push_back(circle5);
            yellowSegments.push_back(circle6);
            yellowSegments.push_back(line4);
            yellowSegments.push_back(circle7);
            yellowSegments.push_back(circle8);
            yellowSegments.push_back(line5);
            yellowSegments.push_back(circle9);
            yellowSegments.push_back(circle10);
            yellowSegments.push_back(line6);
            yellowSegments.push_back(circle11);
            yellowSegments.push_back(circle12);
            RoadLine yellow(yellowSegments,false,true,0);

            RoadSegment w1line1(-1,0+0.4,0,0,2,0,0);
            RoadSegment w1line2(5,4+0.4,0,0,4,0,0);
            RoadSegment w1line3(9,12-0.4,0,0,4,M_PI,0);
            RoadSegment w1line4(1,16-0.4,0,0,2,M_PI,0);
            RoadSegment w1line5(-5,12-0.4,0,0,4,M_PI,0);
            RoadSegment w1line6(-9,4+0.4,0,0,4,0,0);
            RoadSegment w1circle1(1,2,0,1,2-0.4,M_PI*3/2,M_PI*2);
            RoadSegment w1circle2(5,2,0,1,2+0.4,M_PI,M_PI_2);
            RoadSegment w1circle3(9,8,0,1,4-0.4,M_PI*3/2,M_PI*2);
            RoadSegment w1circle4(9,8,0,1,4-0.4,0,M_PI_2);
            RoadSegment w1circle5(5,14,0,1,2+0.4,M_PI*3/2,M_PI);
            RoadSegment w1circle6(1,14,0,1,2-0.4,0,M_PI_2);
            RoadSegment w1circle7(-1,14,0,1,2-0.4,M_PI_2,M_PI);
            RoadSegment w1circle8(-5,14,0,1,2+0.4,2*M_PI,M_PI*3/2);
            RoadSegment w1circle9(-9,8,0,1,4-0.4,M_PI_2,M_PI);
            RoadSegment w1circle10(-9,8,0,1,4-0.4,M_PI,M_PI*3/2);
            RoadSegment w1circle11(-5,2,0,1,2+0.4,M_PI_2,0);
            RoadSegment w1circle12(-1,2,0,1,2-0.4,M_PI,M_PI*3/2);
            std::vector<RoadSegment> innerWhiteSegments;
            innerWhiteSegments.push_back(w1line1);
            innerWhiteSegments.push_back(w1circle1);
            innerWhiteSegments.push_back(w1circle2);
            innerWhiteSegments.push_back(w1line2);
            innerWhiteSegments.push_back(w1circle3);
            innerWhiteSegments.push_back(w1circle4);
            innerWhiteSegments.push_back(w1line3);
            innerWhiteSegments.push_back(w1circle5);
            innerWhiteSegments.push_back(w1circle6);
            innerWhiteSegments.push_back(w1line4);
            innerWhiteSegments.push_back(w1circle7);
            innerWhiteSegments.push_back(w1circle8);
            innerWhiteSegments.push_back(w1line5);
            innerWhiteSegments.push_back(w1circle9);
            innerWhiteSegments.push_back(w1circle10);
            innerWhiteSegments.push_back(w1line6);
            innerWhiteSegments.push_back(w1circle11);
            innerWhiteSegments.push_back(w1circle12);
            RoadLine innerWhite(innerWhiteSegments,1);

            RoadSegment w2line1(-1,0-0.4,0,0,2,0,0);
            RoadSegment w2line2(5,4-0.4,0,0,4,0,0);
            RoadSegment w2line3(9,12+0.4,0,0,4,M_PI,0);
            RoadSegment w2line4(1,16+0.4,0,0,2,M_PI,0);
            RoadSegment w2line5(-5,12+0.4,0,0,4,M_PI,0);
            RoadSegment w2line6(-9,4-0.4,0,0,4,0,0);
            RoadSegment w2circle1(1,2,0,1,2+0.4,M_PI*3/2,M_PI*2);
            RoadSegment w2circle2(5,2,0,1,2-0.4,M_PI,M_PI_2);
            RoadSegment w2circle3(9,8,0,1,4+0.4,M_PI*3/2,M_PI*2);
            RoadSegment w2circle4(9,8,0,1,4+0.4,0,M_PI_2);
            RoadSegment w2circle5(5,14,0,1,2-0.4,M_PI*3/2,M_PI);
            RoadSegment w2circle6(1,14,0,1,2+0.4,0,M_PI_2);
            RoadSegment w2circle7(-1,14,0,1,2+0.4,M_PI_2,M_PI);
            RoadSegment w2circle8(-5,14,0,1,2-0.4,2*M_PI,M_PI*3/2);
            RoadSegment w2circle9(-9,8,0,1,4+0.4,M_PI_2,M_PI);
            RoadSegment w2circle10(-9,8,0,1,4+0.4,M_PI,M_PI*3/2);
            RoadSegment w2circle11(-5,2,0,1,2-0.4,M_PI_2,0);
            RoadSegment w2circle12(-1,2,0,1,2+0.4,M_PI,M_PI*3/2);
            std::vector<RoadSegment> outerWhiteSegments;
            outerWhiteSegments.push_back(w2line1);
            outerWhiteSegments.push_back(w2circle1);
            outerWhiteSegments.push_back(w2circle2);
            outerWhiteSegments.push_back(w2line2);
            outerWhiteSegments.push_back(w2circle3);
            outerWhiteSegments.push_back(w2circle4);
            outerWhiteSegments.push_back(w2line3);
            outerWhiteSegments.push_back(w2circle5);
            outerWhiteSegments.push_back(w2circle6);
            outerWhiteSegments.push_back(w2line4);
            outerWhiteSegments.push_back(w2circle7);
            outerWhiteSegments.push_back(w2circle8);
            outerWhiteSegments.push_back(w2line5);
            outerWhiteSegments.push_back(w2circle9);
            outerWhiteSegments.push_back(w2circle10);
            outerWhiteSegments.push_back(w2line6);
            outerWhiteSegments.push_back(w2circle11);
            outerWhiteSegments.push_back(w2circle12);
            RoadLine outerWhite(outerWhiteSegments,2);

            std::vector<RoadLine> roadLines;
            roadLines.push_back(yellow);
            roadLines.push_back(innerWhite);
            roadLines.push_back(outerWhite);
            this->roadLines = roadLines;
        }
    }
    std::vector<int> closestLineIndexes(Point referencePoints){
      std::vector<int> closestLines;
      std::vector<int> indexes;
      std::vector<double> distances;
      for(int i =0; i<static_cast<int>(roadLines.size());i++){
        indexes.push_back(i);
        distances.push_back(roadLines.at(i).distanceToClosestPoint(referencePoints));
      }
      while(indexes.size()>0){
        double minimumDistance = distances.at(0);
        int indexToRemoveAt = 0;
        for(int j =0; j<static_cast<int>(indexes.size());j++){
          if(distances.at(j)<minimumDistance){
            minimumDistance = distances.at(j);
            indexToRemoveAt = j;
          }
        }
        closestLines.push_back(indexes.at(indexToRemoveAt));
        indexes.erase(indexes.begin()+indexToRemoveAt);
        distances.erase(distances.begin()+indexToRemoveAt);
      }
      return closestLines;
    }
    std::string getRoadModel(){
        std::string roadString = "<sdf version ='1.5'><model name ='TrackLines'><static>true</static><pose>0 0 0 0 0 0</pose><link name ='link'><pose>0 0 0 0 0 0</pose>";
        for(int index=0;index<static_cast<int>(roadLines.size());index++){
            roadString+=roadLines.at(index).getRoadSDFString();
        }
        roadString+="</link></model></sdf>";
        return roadString;
    }
};
class SampleLaneFollower{
    private:
    ros::NodeHandle n;
    ros::Publisher drive_pub;
    ros::Subscriber model_sub;
    std::string carName = "";
    double speed = 0;
    Road road = Road(2);
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
          ROS_INFO("Car Angle %s",std::to_string(desiredSteeringAngle).c_str());
          int test = 1;
          ackermann_msgs::AckermannDriveStamped drive_st_msg;
          ackermann_msgs::AckermannDrive drive_msg;
          drive_msg.steering_angle = desiredSteeringAngle;
          drive_msg.speed = this->speed;
          // ROS_INFO("Steering Angle %s",std::to_string(steeringAngle).c_str());
          // ROS_INFO("Speed %s",std::to_string(carSpeed).c_str());
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
