#include <road_description/road_line.h>
#include <vector>
#include <string>

#ifndef cpp_road

class Road{
    public:
    std::vector<RoadLine> roadLines;
    
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
    
    double maxium_speed_for_car(Point car_location){
      std::vector<int> closest_lane_lines_indicies = closestLineIndexes(car_location);
      RoadLine firstLine = this->roadLines.at(closest_lane_lines_indicies.at(0));
      RoadLine secondLine = this->roadLines.at(closest_lane_lines_indicies.at(1));
      return min(firstLine.getFastestSpeedAtCurrentCarPoint(car_location),secondLine.getFastestSpeedAtCurrentCarPoint(car_location));
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
};
#endif 