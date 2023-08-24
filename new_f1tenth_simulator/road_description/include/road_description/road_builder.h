#include <road_description/road.h>

#ifndef cpp_road_builder
class RoadBuilder{
    public:
    static const int WHITE_LINE = 0;
    static const int YELLOW_LINE = 1;
    static const int DOTTED_LINE = 2;
    static const int SOLID_LINE = 0;
    static const int LINE_INVISIBLE = 4;

    static const int COLOR_BIT = 1;
    static const int LANE_FILL_BIT = 2;
    static const int LANE_VISIBILITY_BIT = 4;

    std::vector<std::vector<RoadSegment>> unfinishedRoadLines;
    std::vector<RoadLine> finishedRoadLines;
    std::vector<int> roadLineConfigurationSettings;

    double lane_thickness = 0.4572;
    double lane_line_thickness = 0.0254;
    int number_of_lane_lines = 2;
    bool one_way_road = false;
    double starting_x = 0;
    double starting_y = 0;
    double starting_z = 0;
    double starting_angle = 0;

    int elementTagCounter = -1;
    void addCurveFromRadiusOfCurvature(double length, double radius_of_curvature, bool left){
        Point starting_point = getStartingPointForSegment();
        if(this->unfinishedRoadLines.size()<=0){
            for(int i=0;i<this->number_of_lane_lines;i++){
                std::vector<RoadSegment> newLine;
                this->unfinishedRoadLines.push_back(newLine);
            }
        }
        double centerIndex = (this->unfinishedRoadLines.size()-1) / 2.0;
        if(centerIndex * this->lane_thickness <radius_of_curvature && length < 3 * M_PI_2 * radius_of_curvature){
            for(size_t index=0;index<this->unfinishedRoadLines.size();index++){
                double new_starting_angle = starting_point.rotation;
                double new_end_angle = starting_point.rotation;
                double radius_of_curvature_for_line = radius_of_curvature;
                if(left){
                    new_starting_angle = wrapAngle(new_starting_angle-M_PI_2);
                    new_end_angle = wrapAngle(new_starting_angle+(length/radius_of_curvature));
                    if(new_end_angle<new_starting_angle){
                        new_starting_angle = new_starting_angle - M_PI *2;
                    }
                    radius_of_curvature_for_line = radius_of_curvature + (index - centerIndex)*this->lane_thickness;
                }else{
                    new_starting_angle = wrapAngle(new_starting_angle+M_PI_2);
                    new_end_angle = wrapAngle(new_starting_angle-(length/radius_of_curvature));
                    if(new_end_angle>new_starting_angle){
                        new_end_angle = new_end_angle - M_PI *2;
                    }
                    radius_of_curvature_for_line = radius_of_curvature - (index - centerIndex)*this->lane_thickness;
                }
                double circle_x = starting_point.x - radius_of_curvature * cos(new_starting_angle);
                double circle_y = starting_point.y - radius_of_curvature * sin(new_starting_angle);
                double circle_z = starting_point.z;
                this->unfinishedRoadLines.at(index).push_back(RoadSegment(circle_x, circle_y, circle_z, 1, radius_of_curvature_for_line, new_starting_angle, new_end_angle));
            }
        }
    }
    //TODO
    // add roadcurves to each unfinished roadline
    void addCurveFromTurningRadius(double length, double radius_of_curvature, bool left){
    }

    void addStraightLine(double length){
        Point starting_point = getStartingPointForSegment();
        if(this->unfinishedRoadLines.size()<=0){
            for(int i=0;i<this->number_of_lane_lines;i++){
                std::vector<RoadSegment> newLine;
                this->unfinishedRoadLines.push_back(newLine);
            }
        }
        double centerIndex = (this->unfinishedRoadLines.size()-1) / 2.0;
        for(size_t index=0;index<this->unfinishedRoadLines.size();index++){
            double vertical_angle = wrapAngle(starting_point.rotation - M_PI_2);
            double starting_line_x = starting_point.x +(index-centerIndex)*this->lane_thickness * cos(vertical_angle);
            double starting_line_y = starting_point.y +(index-centerIndex)*this->lane_thickness * sin(vertical_angle);
            this->unfinishedRoadLines.at(index).push_back(RoadSegment(starting_line_x, starting_line_y, starting_point.z, 0, length, starting_point.rotation, 0));
        }
    }
    //TODO
    void addAngledStraightLine(double length){

    }
    Road getRoad(){
        if(unfinishedRoadLines.size()>0){
            buildRoadLines();
        }
        return Road(finishedRoadLines,one_way_road);
    }
    void buildRoadLines(){
        for(size_t index=0; index<this->unfinishedRoadLines.size();index++){
            int builder_settings = 0;
            if(index<this->roadLineConfigurationSettings.size()){
                builder_settings = roadLineConfigurationSettings.at(index);
            }
            bool isDotted =  builder_settings & LANE_FILL_BIT;
            bool isYellowLine = builder_settings & COLOR_BIT;
            bool visible = !(builder_settings & LANE_VISIBILITY_BIT);
            bool enableInteractions = true;
            this->elementTagCounter = this->elementTagCounter + 1;
            std::vector<RoadSegment> lines = this->unfinishedRoadLines.at(index);
            RoadLine newRoadLine(lines,isDotted, isYellowLine, this->elementTagCounter, visible, enableInteractions);
            this->finishedRoadLines.push_back(newRoadLine);
        }
    }
    void setNumberOfLanes(int numberOfLanes){
        this->number_of_lane_lines = numberOfLanes + 1;
    }
    void setRoadSpacing(double roadSpacing){
        this->lane_thickness = roadSpacing;
    }
    void setLaneThickness(double laneThickness){
        this->lane_line_thickness = laneThickness;
    }
    void setLaneOneWay(bool is_one_way){
        this->one_way_road = is_one_way;
    }
    void setStartingPoint(double x, double y, double z, double rotation){
        this->starting_x = x;
        this->starting_y = y;
        this->starting_z = z;
        this->starting_angle = rotation;
    }
    void addLineDescription(int config_settings){
        this->roadLineConfigurationSettings.push_back(config_settings);
    }
    private:
    Point getStartingPointForSegment(){
        double starting_x_for_segment = this->starting_x;
        double starting_y_for_segment = this->starting_y;
        double starting_z_for_segment = this->starting_z;
        double starting_angle_for_segment = this->starting_angle;
        if(this->unfinishedRoadLines.size()>0 && this->unfinishedRoadLines.at(0).size()>0){
            if(this->unfinishedRoadLines.size() % 2 == 0){
                int lowerMid = this->unfinishedRoadLines.size()/2-1;
                int higherMid = this->unfinishedRoadLines.size()/2;
                Point endOfLowerMid = this->unfinishedRoadLines.at(lowerMid).at(this->unfinishedRoadLines.at(lowerMid).size()-1).getEndPoint();
                Point endOfHigherMid = this->unfinishedRoadLines.at(higherMid).at(this->unfinishedRoadLines.at(higherMid).size()-1).getEndPoint();
                starting_x_for_segment = (endOfLowerMid.x + endOfHigherMid.x) /2;
                starting_y_for_segment = (endOfLowerMid.y + endOfHigherMid.y) /2;
                starting_z_for_segment = endOfLowerMid.z;
                starting_angle_for_segment = endOfLowerMid.rotation;
            }else{
                int mid = this->unfinishedRoadLines.size()/2;
                // assumes at least road segment added so could throw error later if this is not the case
                Point endOfMidPoint = this->unfinishedRoadLines.at(mid).at(this->unfinishedRoadLines.at(mid).size()-1).getEndPoint();
                starting_x_for_segment = endOfMidPoint.x;
                starting_y_for_segment = endOfMidPoint.y;
                starting_z_for_segment = endOfMidPoint.z;
                starting_angle_for_segment = endOfMidPoint.rotation;
            }
        }
        Point return_point;
        return_point.rotation = starting_angle_for_segment;
        return_point.x = starting_x_for_segment;
        return_point.y = starting_y_for_segment;
        return_point.z = starting_z_for_segment;
        return return_point;
    }
};
#endif