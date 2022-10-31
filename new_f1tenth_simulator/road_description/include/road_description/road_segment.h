#include <road_description/road_point.h>
#include <cmath>
#ifndef cpp_road_segment

class RoadSegment{
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
                while(point.rotation<0){
                    point.rotation+= 2*M_PI;
                }
                while(point.rotation>2*M_PI){
                    point.rotation-= 2*M_PI;
                }

            }
            return point;
        }

        double radiusOfCurvature(){
            if(this->classification == 1){
                return this->param1;
            }
            return -1;
        }
        
        double angleAdjustmentOverSegment(double distanceIn){
            if(classification ==1){
                return (param3-param2) * (getLengthOfSegment()-distanceIn)/(getLengthOfSegment());
            }
            return 0;
        }
        Point getEndPoint(){
            Point end;
            end.z = this->z;
            if(classification == 0){
                end.x = x + param1 * cos(param2);
                end.y = y + param1 * sin(param2);
                end.rotation = param2;
            }
            if(classification == 1){
                end.x = x + param1 * cos(param3);
                end.y = y + param1 * sin(param3);
                end.rotation = param3+ M_PI/2;
                if(param2>param3){
                    end.rotation = param3 - M_PI/2;
                }
                while(end.rotation<0){
                    end.rotation+= 2*M_PI;
                }
                while(end.rotation>2*M_PI){
                    end.rotation-= 2*M_PI;
                }
            }
            return end;
        }
};
#endif