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
    std::vector<int> closestLineIndexes(Point referencePoint){
      std::vector<int> closestLines;
      std::vector<int> indexes;
      std::vector<double> distances;
      for(int i =0; i<static_cast<int>(roadLines.size());i++){
        indexes.push_back(i);
        distances.push_back(roadLines.at(i).distanceToClosestPoint(referencePoint));
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
    std::string getRoadModel(){
        std::string roadString = "<sdf version ='1.5'><model name ='TrackLines'><static>true</static><pose>0 0 0 0 0 0</pose><link name ='link'><pose>0 0 0 0 0 0</pose>";
        for(int index=0;index<static_cast<int>(roadLines.size());index++){
            roadString+=roadLines.at(index).getRoadSDFString();
        }
        roadString+="</link></model></sdf>";
        return roadString;
    }
};
#endif 