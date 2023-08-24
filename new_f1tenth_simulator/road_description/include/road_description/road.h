#include <road_description/road_line.h>
#include <vector>
#include <string>

#ifndef cpp_road

class Road{
    public:
    std::vector<RoadLine> roadLines;
    double DISTANCE_TO_LEAVE_BETWEEN_CARS = 1.2;
    double GOOD_DECELERATION_CONSTANT = 1.7 * 0.6;
    double MAX_VELOCITY = 3;
    Road(){
      
    }
    
    Road(std::vector<RoadLine> lines){
        roadLines = lines;
    }
    
    Road(std::vector<RoadLine> lines, bool oneWayDirection){
      if(oneWayDirection){
        roadLines = lines;
      }else{
        for(size_t index=0; index<lines.size();index++){
          this->roadLines.push_back(lines.at(index));
        }
        for(size_t lineIndex=0; lineIndex<lines.size(); lineIndex++){
          RoadLine currentLine = lines.at(lineIndex);
          std::vector<RoadSegment> roadSegments = lines.at(lineIndex).roadSegments;
          std::vector<RoadSegment> reversedSegments;
          for(int segmentIndex = static_cast<int>(roadSegments.size())-1; segmentIndex>=0; segmentIndex--){
            RoadSegment normalSegment = roadSegments.at(segmentIndex);
            if(normalSegment.classification == 0){
              double x = normalSegment.x + normalSegment.param1 * cos(normalSegment.param2);
              double y = normalSegment.y + normalSegment.param1 * sin(normalSegment.param2);
              double z = normalSegment.z;
              int classification = normalSegment.classification; 
              double param1 = normalSegment.param1; 
              double param2 = normalSegment.param2 - M_PI;
              if (param2 < 0){
                param2 += 2*M_PI;
              } 
              double param3 = normalSegment.param3;
              RoadSegment reversedSegment = RoadSegment(x,y,z,classification,param1,param2,param3);
              reversedSegments.push_back(reversedSegment);
            }else if(normalSegment.classification == 1){
              double x = normalSegment.x;
              double y = normalSegment.y;
              double z = normalSegment.z;
              int classification = normalSegment.classification; 
              double param1 = normalSegment.param1; 
              double param2 = normalSegment.param3; 
              double param3 = normalSegment.param2;
              RoadSegment reversedSegment = RoadSegment(x,y,z,classification,param1,param2,param3);
              reversedSegments.push_back(reversedSegment);
            }
          }
          RoadLine reversedLine = RoadLine(reversedSegments,currentLine.dotted,currentLine.yellowLine,currentLine.visualTagIndex,false,true);
          this->roadLines.push_back(reversedLine);
        }
      }
    }
    
    std::vector<int> closestLineIndexes(Point referencePoint){
      std::vector<int> closestLines;
      std::vector<int> indexes;
      std::vector<double> distances;
      for(int i = 0; i<static_cast<int>(roadLines.size());i++){
        Point closestPoint = roadLines.at(i).getClosestPointOnRoad(referencePoint);
        double angleDifference = differenceInPointAngles(closestPoint.rotation, referencePoint.rotation);
        if(angleDifference<=M_PI_2){
          indexes.push_back(i);
          distances.push_back(distanceBetweenPoints(closestPoint,referencePoint));
        }
      }
      while(indexes.size()>0){
        double minimumDistance = distances.at(0);
        int indexToRemoveAt = 0;
        for(int j = 0; j<static_cast<int>(indexes.size());j++){
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
    
    std::vector<int> getLanesToTheLeftOfPoint(Point referencePoint){
      std::vector<int> closestLines = this->closestLineIndexes(referencePoint);
      std::vector<int> leftLanes;
      for(int i =0; i<static_cast<int>(closestLines.size());i++){
        if(this->isLaneToTheLeftOfPoint(referencePoint,closestLines.at(i))){
          leftLanes.push_back(closestLines.at(i));
        }
      }
      return leftLanes;
    }
    
    std::vector<int> getLanesToTheRightOfPoint(Point referencePoint){
      std::vector<int> closestLines = this->closestLineIndexes(referencePoint);
      std::vector<int> rightLanes;
      for(int i =0; i<static_cast<int>(closestLines.size());i++){
        if(!this->isLaneToTheLeftOfPoint(referencePoint,closestLines.at(i))){
          rightLanes.push_back(closestLines.at(i));
        }
      }
      return rightLanes;
    }
    
    bool isLaneToTheLeftOfPoint(Point referencePoint,int roadIndex){
      Point closestPointOnLine = this->roadLines.at(roadIndex).getClosestPointOnRoad(referencePoint);
      double angleToRotateBy =-1 * (referencePoint.rotation - M_PI_2);

      double pointXBeforeRotation = closestPointOnLine.x - referencePoint.x;
      double pointYBeforeRotation = closestPointOnLine.y - referencePoint.y;
      double newXForPoint = pointXBeforeRotation * cos(angleToRotateBy) - pointYBeforeRotation *sin(angleToRotateBy);
      if(newXForPoint < 0){
        return true;
      }
      return false;
    }
    
    double distanceBetweenPoints(Point first,Point second){
          return  sqrt(pow((first.x-second.x),2) + pow((first.y-second.y),2));
    }
    
    // later adapt to consider limitations in camera ( Should not slow down if car in front of it is out of view)
    //Used to Train NN
    double maxium_speed_for_car(Point car_location){
      std::vector<int> closest_lane_lines_indicies = closestLineIndexes(car_location);
      RoadLine firstLine = this->roadLines.at(closest_lane_lines_indicies.at(0));
      RoadLine secondLine = this->roadLines.at(closest_lane_lines_indicies.at(1));
      return std::min(MAX_VELOCITY,std::min(firstLine.getFastestSpeedAtCurrentCarPoint(car_location),secondLine.getFastestSpeedAtCurrentCarPoint(car_location)));
    }

    // adapt this function to later consider when cars are turning lanes
    // ^ basically make sure firstLine and secondLine can be changed when the current car you are getting this value for is turning
    // and
    // when another car you need to consider for safety is turning into your lane, but hasnt yet hit the point where the two lane lines are the same
    // ^ change the for loop that considers if a car is valid to consider to see if the car is partially in a lane by distance rather than seeing if that is the lane it follows
    double maxium_speed_for_car_with_other_car_points(Point car_location,std::vector<Point> other_cars){
      std::vector<int> closest_lane_lines_indicies = closestLineIndexes(car_location);
      RoadLine firstLine = this->roadLines.at(closest_lane_lines_indicies.at(0));
      RoadLine secondLine = this->roadLines.at(closest_lane_lines_indicies.at(1));
      std::vector<Point> other_cars_to_consider;
      // check if both cars are in the same lane and consider it if so
      for(size_t index = 0; index<other_cars.size();index++){
        std::vector<int> closest_lane_lines_indicies_for_other_car = closestLineIndexes(other_cars.at(index));
        if(closest_lane_lines_indicies_for_other_car.size()>=2 && closest_lane_lines_indicies.size()>=2){
          bool sameFirstTwoElementsInOrder = closest_lane_lines_indicies_for_other_car.at(0) == closest_lane_lines_indicies.at(0) && closest_lane_lines_indicies_for_other_car.at(1) == closest_lane_lines_indicies.at(1);
          bool sameFirstTwoElementsOutOfOrder = closest_lane_lines_indicies_for_other_car.at(0) == closest_lane_lines_indicies.at(1) && closest_lane_lines_indicies_for_other_car.at(1) == closest_lane_lines_indicies.at(0);
          if(sameFirstTwoElementsInOrder || sameFirstTwoElementsOutOfOrder){
            other_cars_to_consider.push_back(other_cars.at(index));
          }
        }
      }
      // loop through and find minimum speed for each car
      double min_value_from_track_alone = maxium_speed_for_car(car_location);
      double min_total_velocity = min_value_from_track_alone;
      for(size_t index=0;index<other_cars_to_consider.size();index++){
          Point other_car = other_cars_to_consider.at(index);
          bool is_other_car_in_front_of_this_car = firstLine.secondPointInFrontOfFirst(car_location,other_car) && secondLine.secondPointInFrontOfFirst(car_location,other_car);
          if(is_other_car_in_front_of_this_car){
            // Find Distances into each line segment and use standard kinematics to find ok 
            double min_distance = sqrt(pow(car_location.x -other_car.x,2) + pow(car_location.y -other_car.y,2));
            // ^ may want to later find the distance along the curves rather than absolute distance difference later
            double min_velocity_relative_to_other_car = sqrt(2* GOOD_DECELERATION_CONSTANT * std::max(min_distance-DISTANCE_TO_LEAVE_BETWEEN_CARS,0.0));
            if(min_velocity_relative_to_other_car < min_total_velocity){
              min_total_velocity = min_velocity_relative_to_other_car;
            }
          }
      }
      return std::min(min_total_velocity,MAX_VELOCITY);
    }
    
    std::string getRoadModel(){
        std::string roadString = "<sdf version ='1.5'><model name ='TrackLines'><static>true</static><pose>0 0 0 0 0 0</pose><link name ='link'><pose>0 0 0 0 0 0</pose>";
        for(int index=0;index<static_cast<int>(roadLines.size());index++){
            if(roadLines.at(index).visible == true){
              roadString+=roadLines.at(index).getRoadSDFString();
            }
        }
        roadString+="</link></model></sdf>";
        return roadString;
    }
    //Used to Train NN
    double steering_angle_from_point(Point car){
      std::vector<int> closestLines = closestLineIndexes(car);
      Point closestPoint = roadLines.at(closestLines.at(0)).getClosestPointOnRoad(car);
      Point secondClosestPoint = roadLines.at(closestLines.at(1)).getClosestPointOnRoad(car);
      return steering_angle_from_point(car, closestPoint, secondClosestPoint);
    }
    
    double steering_angle_from_point(Point car, Point firstRoadLine,Point secondRoadLine){
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
    //Used to Train NN
    bool isCarInLane(Point car){
      std::vector<int> closestLineIndexes = this->closestLineIndexes(car);
      RoadLine first = this->roadLines.at(closestLineIndexes.at(0));
      RoadLine second = this->roadLines.at(closestLineIndexes.at(1));
      Point firstPoint = first.getClosestPointOnRoad(car);
      Point secondPoint = second.getClosestPointOnRoad(car);
      return isCarInLane(car,firstPoint,secondPoint,0.2);
    }
    bool isCarInLaneDifferentRatio(Point car,double ratio){
      std::vector<int> closestLineIndexes = this->closestLineIndexes(car);
      RoadLine first = this->roadLines.at(closestLineIndexes.at(0));
      RoadLine second = this->roadLines.at(closestLineIndexes.at(1));
      Point firstPoint = first.getClosestPointOnRoad(car);
      Point secondPoint = second.getClosestPointOnRoad(car);
      return isCarInLane(car,firstPoint,secondPoint,ratio);
    }
    //Used to Train NN
    bool has_left_lane(Point car){
      return getLanesToTheLeftOfPoint(car).size()>1;
    }
    //Used to Train NN
    bool has_right_lane(Point car){
      return getLanesToTheRightOfPoint(car).size()>1;
    }
    
    bool isCarInLane(Point car,Point firstPoint,Point secondPoint, double ratio = 0.2){
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
        return (distanceFromCenter / distanceBetweenPoints) < ratio; // find good ratio
      }

    private:
    
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
};
#endif 