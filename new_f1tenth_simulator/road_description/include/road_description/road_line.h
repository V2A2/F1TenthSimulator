#include <road_description/road_segment.h>
#include <vector>
#include <string>

#ifndef cpp_road_line
class RoadLine{
    public:
        std::vector<RoadSegment> roadSegments;
        bool dotted = false;
        bool yellowLine = false;
        int visualTagIndex = 0;
        std::vector<Point> points;
        double default_resolution_for_point_map = 0.01;
        double default_lane_thickness = 0.0254;
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
                roadString+="</pose><geometry><plane><normal>0 0 1</normal> <size>0.1 "+std::to_string(default_lane_thickness)+"</size></plane></geometry><material><ambient>0 0 0 1</ambient><diffuse>0 0 0 1</diffuse><specular>0 0 0 0</specular><emissive>1 0.9 0.2 1</emissive></material></visual>";
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
                roadString+="</pose><geometry><plane><normal>0 0 1</normal> <size>0.1 "+std::to_string(default_lane_thickness)+"</size></plane></geometry><material><ambient>0 0 0 1</ambient><diffuse>0 0 0 1</diffuse><specular>0 0 0 0</specular><emissive>1 1 1 1</emissive></material></visual>";
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
                roadString+="</pose><geometry><plane><normal>0 0 1</normal> <size>0.1 "+std::to_string(default_lane_thickness)+"</size></plane></geometry><material><ambient>0 0 0 1</ambient><diffuse>0 0 0 1</diffuse><specular>0 0 0 0</specular><emissive>1 0.9 0.2 1</emissive></material></visual>";
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
                roadString+="</pose><geometry><plane><normal>0 0 1</normal> <size>0.1 "+std::to_string(default_lane_thickness)+"</size></plane></geometry><material><ambient>0 0 0 1</ambient><diffuse>0 0 0 1</diffuse><specular>0 0 0 0</specular><emissive>1 1 1 1</emissive></material></visual>";
            }

            return roadString;
        }
    private:
        double distanceBetweenPoints(Point first,Point second){
          return  sqrt(pow((first.x-second.x),2) + pow((first.y-second.y),2));
        }

};
#endif