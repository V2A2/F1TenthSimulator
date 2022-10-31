#include <road_description/road_segment.h>
#include <vector>
#include <string>

#ifndef cpp_road_line

class RoadLine{
    public:
        std::vector<RoadSegment> roadSegments;
        bool dotted = false;
        bool yellowLine = false;
        bool visible = true;
        bool enableInteractions = true;
        bool connected_at_ends = false;
        int visualTagIndex = 0;
        std::vector<Point> points;
        std::vector<int> roadSegmentIndexVector; // Index of road segment for corresponding point in point vector
        std::vector<double> distanceInSegmentVector;
        double default_resolution_for_point_map = 0.01;
        double default_lane_thickness = 0.0254;
        double CONSTANT_FOR_MAX_TURNING = 0.7; // some proportion of coefficient of friction times gravitational accaleration
        double MAX_CAR_DECELERATION = 1.7;
        double default_resolution_for_display = 0.1;
        double dotted_line_resolution = 0.6;
        double straight_line_length = 0.1;
        double dotted_line_length = 0.2;
        std::vector<double> roadSegmentEndVelocities;

        RoadLine(std::vector<RoadSegment>segments,int elementTag){
            this->roadSegments = segments;
            dotted = false;
            yellowLine = false;
            visualTagIndex = elementTag;
            getRoadPointsWithDetails(default_resolution_for_point_map,this->points,this->roadSegmentIndexVector,this->distanceInSegmentVector);
            setupSegmentEndLimits();
        }
        
        RoadLine(std::vector<RoadSegment>segments,bool isDotted, bool isYellowLine, int elementTag){
            this->roadSegments = segments;
            dotted = isDotted;
            yellowLine = isYellowLine;
            visualTagIndex = elementTag;
            getRoadPointsWithDetails(default_resolution_for_point_map,this->points,this->roadSegmentIndexVector,this->distanceInSegmentVector);
            setupSegmentEndLimits();
        }
        // ENABLE INTERACTIONS NOT SUPPORTED YET
        RoadLine(std::vector<RoadSegment>segments,bool isDotted, bool isYellowLine, int elementTag, bool visable, bool enableInteractions){
            this->roadSegments = segments;
            dotted = isDotted;
            yellowLine = isYellowLine;
            visualTagIndex = elementTag;
            this->visible = visible;
            this->enableInteractions = enableInteractions;
            getRoadPointsWithDetails(default_resolution_for_point_map,this->points,this->roadSegmentIndexVector,this->distanceInSegmentVector);
            setupSegmentEndLimits();
        }
        
        void setupSegmentEndLimits(){
            // check if the first and last points connect
            RoadSegment first = this->roadSegments.at(0);
            RoadSegment last = this->roadSegments.at(this->roadSegments.size()-1);
            Point startOfFirst = first.getPointXLengthInLine(0);
            Point endOfLast = last.getPointXLengthInLine(last.getLengthOfSegment());
            bool connectedEnds = abs(startOfFirst.x-endOfLast.x)<0.1 && abs(startOfFirst.y-endOfLast.y)<0.1;
            this->connected_at_ends=connectedEnds;
            if(connectedEnds){
                for(size_t i=0; i<roadSegments.size();i++){
                    double minVelocity = std::numeric_limits<double>::max();
                    double runningDistance = 0;
                    size_t j = i+1;
                    if(j >= roadSegments.size()){
                        j=0;
                    }
                    while(j != i){
                        //
                        RoadSegment futureRoadSegment = roadSegments.at(j);
                        Point firstPointInFutureSegment = futureRoadSegment.getPointXLengthInLine(0);
                        int closestPointIndex = getClosestPointIndexOnSegment(firstPointInFutureSegment,j);
                        double circleCurveVelocityLimit = this->getCircleMaxVelocity(futureRoadSegment);
                        size_t nextIndex = j+1;
                        if(nextIndex >= roadSegments.size()){
                            nextIndex=0;
                        }
                        double cornerVelocityLimit = this->getCornerMaxVelocity(closestPointIndex,futureRoadSegment,roadSegments.at(nextIndex));
                        if(circleCurveVelocityLimit>0){
                            double minVelocityAtEndPoint = sqrt(pow(circleCurveVelocityLimit,2) + 2* MAX_CAR_DECELERATION * runningDistance);
                            if(minVelocityAtEndPoint<minVelocity){
                                minVelocity = minVelocityAtEndPoint;
                            }
                        }
                        if(cornerVelocityLimit>0){
                            double minVelocityAtEndPoint = sqrt(pow(cornerVelocityLimit,2) + 2* MAX_CAR_DECELERATION * runningDistance);
                            if(minVelocityAtEndPoint<minVelocity){
                                minVelocity = minVelocityAtEndPoint;
                            }
                        }
                        runningDistance += futureRoadSegment.getLengthOfSegment();
                        //
                        j++;
                        if(j >= roadSegments.size()){
                            j=0;
                        }
                    }
                    roadSegmentEndVelocities.push_back(minVelocity);
                }
            }else{
                // dont include last index
                for(size_t i=0; i<roadSegments.size()-1;i++){
                    double minVelocity = std::numeric_limits<double>::max();
                    double runningDistance = 0;
                    for( size_t j = i+1;j<roadSegments.size()-1;j++){
                        RoadSegment futureRoadSegment = roadSegments.at(j);
                        Point firstPointInFutureSegment = futureRoadSegment.getPointXLengthInLine(0);
                        int closestPointIndex = getClosestPointIndexOnSegment(firstPointInFutureSegment,j);
                        double circleCurveVelocityLimit = this->getCircleMaxVelocity(futureRoadSegment);
                        double cornerVelocityLimit = this->getCornerMaxVelocity(closestPointIndex,futureRoadSegment,roadSegments.at(j+1));
                        if(circleCurveVelocityLimit>0){
                            double minVelocityAtEndPoint = sqrt(pow(circleCurveVelocityLimit,2) + 2* MAX_CAR_DECELERATION * runningDistance);
                            if(minVelocityAtEndPoint<minVelocity){
                                minVelocity = minVelocityAtEndPoint;
                            }
                        }
                        if(cornerVelocityLimit>0){
                            double minVelocityAtEndPoint = sqrt(pow(cornerVelocityLimit,2) + 2* MAX_CAR_DECELERATION * runningDistance);
                            if(minVelocityAtEndPoint<minVelocity){
                                minVelocity = minVelocityAtEndPoint;
                            }
                        }
                        runningDistance += futureRoadSegment.getLengthOfSegment();
                    }
                    roadSegmentEndVelocities.push_back(minVelocity);
                }
                // last index must be zero since it doesnt connect
                roadSegmentEndVelocities.push_back(0);
            }
        }
        
        void getRoadPointsWithDetails(double resolution, std::vector<Point>& points, std::vector<int>& roadSegmentIndexVector, std::vector<double>& distanceInSegmentVector){
            double lastDistance = 0;
            for(int i=0;i<static_cast<int>(roadSegments.size());i++){
                RoadSegment currentSegment = roadSegments.at(i);
                double currentLength = lastDistance;
                while(currentLength<=currentSegment.getLengthOfSegment()){
                    points.push_back(currentSegment.getPointXLengthInLine(currentLength));
                    roadSegmentIndexVector.push_back(i);
                    distanceInSegmentVector.push_back(currentLength);
                    currentLength+=resolution;
                }
                lastDistance = currentLength - currentSegment.getLengthOfSegment();
            }
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
        
        int getClosestPointIndex(Point referencePoint){
          Point closestPoint= this->points.at(0);
          int closestPointIndex = 0;
          double minDistanceToPoint = distanceBetweenPoints(closestPoint,referencePoint);
          for(int i = 0;i<static_cast<int>(this->points.size());i++){
            if(distanceBetweenPoints(referencePoint,this->points.at(i))<minDistanceToPoint){
              minDistanceToPoint = distanceBetweenPoints(referencePoint,this->points.at(i));
              closestPoint = this->points.at(i);
              closestPointIndex = i;
            }
          }
          return closestPointIndex;
        }
        
        int getClosestPointIndexOnSegment(Point referencePoint,int segmentIndex){
          Point closestPoint= this->points.at(0);
          int closestPointIndex = 0;
          double minDistanceToPoint = std::numeric_limits<double>::max();
          for(int i = 0;i<static_cast<int>(this->points.size());i++){
            if(distanceBetweenPoints(referencePoint,this->points.at(i))<minDistanceToPoint && this->roadSegmentIndexVector.at(i) == segmentIndex){
              minDistanceToPoint = distanceBetweenPoints(referencePoint,this->points.at(i));
              closestPoint = this->points.at(i);
              closestPointIndex = i;
            }
          }
          return closestPointIndex;
        }

        double getFastestSpeedAtCurrentCarPoint(Point carLocation){
            int closestPointIndex = getClosestPointIndex(carLocation);
            Point cloestPoint = this->points.at(closestPointIndex);
            int closestRoadSegmentIndex = this->roadSegmentIndexVector.at(closestPointIndex);
            RoadSegment closestSegment = this->roadSegments.at(closestRoadSegmentIndex);
            double distanceInRoadSegment = distanceInSegmentVector.at(closestPointIndex);
            double curveVelocity = getCircleMaxVelocity(closestSegment);
            double distanceToEndOfSegment = closestSegment.getLengthOfSegment() - distanceInRoadSegment;
            double nextSegmentVelocity = sqrt(pow(roadSegmentEndVelocities.at(closestRoadSegmentIndex),2)+ 2* MAX_CAR_DECELERATION * distanceToEndOfSegment);
            double cornerVelocity = -1;
            // when not last index
            if(closestRoadSegmentIndex < static_cast<int>(this->roadSegments.size())-1){
                cornerVelocity = getCornerMaxVelocity(closestPointIndex, closestSegment, this->roadSegments.at(closestRoadSegmentIndex+1));
            }
            if(closestRoadSegmentIndex == static_cast<int>(this->roadSegments.size())-1){
                RoadSegment first = this->roadSegments.at(0);
                RoadSegment last = this->roadSegments.at(this->roadSegments.size()-1);
                Point startOfFirst = first.getPointXLengthInLine(0);
                Point endOfLast = last.getPointXLengthInLine(last.getLengthOfSegment());
                bool connectedEnds = abs(startOfFirst.x-endOfLast.x)<0.1 && abs(startOfFirst.y-endOfLast.y)<0.1;
                if(connectedEnds){
                    cornerVelocity = getCornerMaxVelocity(closestPointIndex, closestSegment, this->roadSegments.at(0));
                }
            }
            double lowestVelocity = std::numeric_limits<double>::max();
            if(cornerVelocity>=0 && cornerVelocity < lowestVelocity){
                lowestVelocity = cornerVelocity;
            }
            if(nextSegmentVelocity>=0 && nextSegmentVelocity < lowestVelocity){
                lowestVelocity = nextSegmentVelocity;
            }
            if(curveVelocity>=0 && curveVelocity < lowestVelocity){
                lowestVelocity = curveVelocity;
            }
            return lowestVelocity;
        }
        
        std::string getRoadSDFStringYellowLine(){
            std::string roadString = "";
            std::vector<Point> pointsToDisplay = getRoadPoints(default_resolution_for_display);
            for (int index = 0; index<static_cast<int>(pointsToDisplay.size());index++){
                Point point = pointsToDisplay.at(index);
                roadString+="<visual name ='visual"+ std::to_string(this->visualTagIndex)+"-"+std::to_string(index)+"'><pose>";
                roadString+= "" +std::to_string(point.x) + " " + std::to_string(point.y) + " " + std::to_string(point.z) + " 0 0 " +std::to_string(point.rotation);
                roadString+="</pose><geometry><plane><normal>0 0 1</normal> <size>"+std::to_string(straight_line_length) + " " + std::to_string(default_lane_thickness)+"</size></plane></geometry><material><ambient>0 0 0 1</ambient><diffuse>0 0 0 1</diffuse><specular>0 0 0 0</specular><emissive>1 0.9 0.2 1</emissive></material></visual>";
            }
            return roadString;
        }
        
        std::string getRoadSDFStringWhiteLine(){
            std::string roadString = "";
            std::vector<Point> pointsToDisplay = getRoadPoints(default_resolution_for_display);
            for (int index = 0; index<static_cast<int>(pointsToDisplay.size());index++){
                Point point = pointsToDisplay.at(index);
                roadString+="<visual name ='visual"+ std::to_string(this->visualTagIndex)+"-"+std::to_string(index)+"'><pose>";
                roadString+= "" +std::to_string(point.x) + " " + std::to_string(point.y) + " " + std::to_string(point.z) + " 0 0 " +std::to_string(point.rotation);
                roadString+="</pose><geometry><plane><normal>0 0 1</normal> <size>"+std::to_string(straight_line_length) + " "+ std::to_string(default_lane_thickness)+"</size></plane></geometry><material><ambient>0 0 0 1</ambient><diffuse>0 0 0 1</diffuse><specular>0 0 0 0</specular><emissive>1 1 1 1</emissive></material></visual>";
            }
            return roadString;
        }
        
        std::string getRoadSDFStringDottedYellowLine(){
            std::string roadString = "";
            std::vector<Point> pointsToDisplay = getRoadPoints(dotted_line_resolution);
            for (int index = 0; index<static_cast<int>(pointsToDisplay.size());index++){
                Point point = pointsToDisplay.at(index);
                roadString+="<visual name ='visual"+ std::to_string(this->visualTagIndex)+"-"+std::to_string(index)+"'><pose>";
                roadString+= "" +std::to_string(point.x) + " " + std::to_string(point.y) + " " + std::to_string(point.z) + " 0 0 " +std::to_string(point.rotation);
                roadString+="</pose><geometry><plane><normal>0 0 1</normal> <size>"+std::to_string(dotted_line_length) + " " +std::to_string(default_lane_thickness)+"</size></plane></geometry><material><ambient>0 0 0 1</ambient><diffuse>0 0 0 1</diffuse><specular>0 0 0 0</specular><emissive>1 0.9 0.2 1</emissive></material></visual>";
            }
            return roadString;
        }
        
        std::string getRoadSDFStringDottedWhiteLine(){
            std::string roadString = "";
            std::vector<Point> pointsToDisplay = getRoadPoints(dotted_line_resolution);
            for (int index = 0; index<static_cast<int>(pointsToDisplay.size());index++){
                Point point = pointsToDisplay.at(index);
                roadString+="<visual name ='visual"+ std::to_string(this->visualTagIndex)+"-"+std::to_string(index)+"'><pose>";
                roadString+= "" +std::to_string(point.x) + " " + std::to_string(point.y) + " " + std::to_string(point.z) + " 0 0 " +std::to_string(point.rotation);
                roadString+="</pose><geometry><plane><normal>0 0 1</normal> <size>"+std::to_string(dotted_line_length) + " " +std::to_string(default_lane_thickness)+"</size></plane></geometry><material><ambient>0 0 0 1</ambient><diffuse>0 0 0 1</diffuse><specular>0 0 0 0</specular><emissive>1 1 1 1</emissive></material></visual>";
            }

            return roadString;
        }
        
        double getCornerMaxVelocity(int closestPointIndex, RoadSegment closestRoadSegment, RoadSegment nextRoadSegment){
            Point currentPosition = this->points.at(closestPointIndex);
            double distanceIn = this->distanceInSegmentVector.at(closestPointIndex);
            double minimumAngleDiffToConsider = 0.0873; // 5 degrees
            Point pointAtEndOfClosestSegment = closestRoadSegment.getPointXLengthInLine(closestRoadSegment.getLengthOfSegment());
            Point pointAtStartOfNextSegment = nextRoadSegment.getPointXLengthInLine(0);
            // ADJUST ANGLE BY AMOUNT IT STILL HAS LEFT TO TURN
            double angleDifference = abs(differenceInPointAngles(wrapAngle(currentPosition.rotation+closestRoadSegment.angleAdjustmentOverSegment(distanceIn)), averagePointAngles(pointAtEndOfClosestSegment.rotation, pointAtStartOfNextSegment.rotation)));
            
            if(angleDifference<= minimumAngleDiffToConsider){
                return -1;
            }
            double distanceToEndOfClosestSegment =  closestRoadSegment.getLengthOfSegment()-distanceIn;
            double requiredTurningRadius = distanceToEndOfClosestSegment * (sin((M_PI-angleDifference)/2)) / sin(angleDifference);
            return sqrt(CONSTANT_FOR_MAX_TURNING * requiredTurningRadius);
        }
        
        double getCircleMaxVelocity(RoadSegment closestRoadSegment){
            if(closestRoadSegment.radiusOfCurvature()<0){
                return -1;
            }
            return sqrt(CONSTANT_FOR_MAX_TURNING * closestRoadSegment.radiusOfCurvature());
        }
        
        bool secondPointInFrontOfFirst(Point first, Point second){
            int firstCarClosestPointIndex = getClosestPointIndex(first);
            int secondCarClosestPointIndex = getClosestPointIndex(second);

            // see if car is following the direction of the array or opposite
            bool followsDirectionOfArray = differenceInPointAngles(first.rotation,this->points.at(firstCarClosestPointIndex).rotation)<M_PI_2;

            if(!this->connected_at_ends){
                if(secondCarClosestPointIndex>=firstCarClosestPointIndex && followsDirectionOfArray){
                    return true;
                }else if(secondCarClosestPointIndex<firstCarClosestPointIndex && !followsDirectionOfArray){
                    return true;
                }
                return false;
            }else{
                // If array looked like | | | | F \ \ \ \ S | | | |
                // where F is first point and S is the second point
                // inner distance is \ \ \ \
                // outer distance is | | | | ... | | | |
                int innerDistance = secondCarClosestPointIndex - firstCarClosestPointIndex;
                int outerDistance = firstCarClosestPointIndex + (this->points.size()-1 - secondCarClosestPointIndex);
                if(followsDirectionOfArray && innerDistance < outerDistance){
                    return true;
                }
                if(!followsDirectionOfArray && outerDistance < innerDistance){
                    return true;
                }
                return false;
            }
            return false;
        }
    private:
        double distanceBetweenPoints(Point first,Point second){
          return  sqrt(pow((first.x-second.x),2) + pow((first.y-second.y),2));
        }

};
#endif