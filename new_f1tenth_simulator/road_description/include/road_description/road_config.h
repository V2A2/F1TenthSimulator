#include <road_description/road_builder.h>
#include <vector>
#include <cmath>
#include <ros/param.h>
#ifndef cpp_road_config
Road get_road(int trackNumber);
Road get_default_road(){
    int track_number = -1;
    ros::param::get("/track_number", track_number);
    return get_road(track_number); // switch to param in f1tenth_gazebo
}
Road get_road(int trackNumber){
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
        return Road(roadLines);
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
        RoadLine yellow(yellowSegments,true,true,0);

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
        return Road(roadLines,false);
    }
    if(trackNumber == 3){
        RoadBuilder roadBuilder;
        roadBuilder.setNumberOfLanes(2);
        int firstLine = RoadBuilder::WHITE_LINE | RoadBuilder::DOTTED_LINE;
        int secondLine = RoadBuilder::YELLOW_LINE | RoadBuilder::DOTTED_LINE;
        int thirdLine = RoadBuilder::WHITE_LINE | RoadBuilder::DOTTED_LINE;
        roadBuilder.addLineDescription(firstLine);
        roadBuilder.addLineDescription(secondLine);
        roadBuilder.addLineDescription(thirdLine);
        roadBuilder.setStartingPoint(0,0,0,0);
        roadBuilder.addStraightLine(2);
        roadBuilder.addCurveFromRadiusOfCurvature(1 * M_PI,1,true);
        roadBuilder.addStraightLine(5);
        roadBuilder.addCurveFromRadiusOfCurvature(2 * M_PI,2,false);
        roadBuilder.addStraightLine(8);
        roadBuilder.addCurveFromRadiusOfCurvature(3 * M_PI,3,true);
        roadBuilder.addCurveFromRadiusOfCurvature(2 * M_PI,2,false);
        roadBuilder.addStraightLine(5);
        roadBuilder.addCurveFromRadiusOfCurvature(1.5 * M_PI,1.5,true);
        roadBuilder.addCurveFromRadiusOfCurvature(10 * M_PI_2,10,false);
        roadBuilder.addCurveFromRadiusOfCurvature(1 * M_PI_2,1,true);
        roadBuilder.addCurveFromRadiusOfCurvature(2 * M_PI_2,2,true);
        roadBuilder.addCurveFromRadiusOfCurvature(3 * M_PI_2,3,false);
        roadBuilder.addCurveFromRadiusOfCurvature(4 * M_PI_2,4,true);
        roadBuilder.addStraightLine(21);
        roadBuilder.addCurveFromRadiusOfCurvature(2 * M_PI_2,2,true);
        roadBuilder.addCurveFromRadiusOfCurvature(1 * M_PI_2,1,true);
        roadBuilder.addCurveFromRadiusOfCurvature(1 * M_PI_2,1,false);
        roadBuilder.addCurveFromRadiusOfCurvature(1 * M_PI_2,1,false);
        roadBuilder.addStraightLine(1);
        roadBuilder.addCurveFromRadiusOfCurvature(1 * M_PI,1,true);
        roadBuilder.addStraightLine(1);
        roadBuilder.addCurveFromRadiusOfCurvature(1 * M_PI_2,1,false);
        roadBuilder.addStraightLine(2);
        return roadBuilder.getRoad();
    }
    if(trackNumber == 4){
        //ESTIMATION FOR CURRENT TRACK
        RoadBuilder roadBuilder;
        roadBuilder.setNumberOfLanes(2);
        int firstLine = RoadBuilder::WHITE_LINE | RoadBuilder::SOLID_LINE;
        int secondLine = RoadBuilder::YELLOW_LINE | RoadBuilder::SOLID_LINE;
        int thirdLine = RoadBuilder::WHITE_LINE | RoadBuilder::SOLID_LINE;
        roadBuilder.addLineDescription(firstLine);
        roadBuilder.addLineDescription(secondLine);
        roadBuilder.addLineDescription(thirdLine);
        roadBuilder.setStartingPoint(0,0,0,0);
        roadBuilder.addStraightLine(1.4224);
        roadBuilder.addCurveFromRadiusOfCurvature(0.889 * M_PI_2,0.889,true);
        roadBuilder.addStraightLine(0.1778);
        roadBuilder.addCurveFromRadiusOfCurvature(0.889 * M_PI_2,0.889,true);
        roadBuilder.addStraightLine(1.4224);
        roadBuilder.addCurveFromRadiusOfCurvature(0.889 * M_PI_2,0.889,true);
        roadBuilder.addStraightLine(0.1778);
        roadBuilder.addCurveFromRadiusOfCurvature(0.889 * M_PI_2,0.889,true);
        return roadBuilder.getRoad();
    }
    if(trackNumber == 5){
        //ESTIMATION FOR OLD TRACK
        RoadBuilder roadBuilder;
        roadBuilder.setNumberOfLanes(2);
        int firstLine = RoadBuilder::WHITE_LINE | RoadBuilder::SOLID_LINE;
        int secondLine = RoadBuilder::YELLOW_LINE | RoadBuilder::SOLID_LINE;
        int thirdLine = RoadBuilder::WHITE_LINE | RoadBuilder::SOLID_LINE;
        roadBuilder.addLineDescription(firstLine);
        roadBuilder.addLineDescription(secondLine);
        roadBuilder.addLineDescription(thirdLine);
        roadBuilder.setStartingPoint(0,0,0,0);
        roadBuilder.addStraightLine(2.7);
        roadBuilder.addCurveFromRadiusOfCurvature(1.15 * M_PI_2,1.15,true);
        roadBuilder.addStraightLine(1.55);
        roadBuilder.addCurveFromRadiusOfCurvature(1.15 * M_PI_2,1.15,true);
        roadBuilder.addStraightLine(2.7);
        roadBuilder.addCurveFromRadiusOfCurvature(1.15 * M_PI_2,1.15,true);
        roadBuilder.addStraightLine(1.55);
        roadBuilder.addCurveFromRadiusOfCurvature(1.15 * M_PI_2,1.15,true);
        return roadBuilder.getRoad();
    }
    if(trackNumber == 6){
        // More pixel perfect representation of new track
        RoadSegment line1(0.92,     0.92+0.46,   0,  0,  1.36,   M_PI_2, 0);
        RoadSegment circle1(0.92+0.46,  .92+2.28-0.46,   0,  1,  0.46,  M_PI,   M_PI/2);
        RoadSegment line2(0.92+0.46,3.20,0,0,0.2,0,0);
        RoadSegment circle2(0.92+1-0.34,  3.2-0.34,   0,  1,  0.34,  M_PI/2,   0);
        RoadSegment line3(0.92+1,3.20-0.34,0,0,1.54,-M_PI_2,0);
        RoadSegment circle3(0.92+1-0.40,  0.92+0.40,   0,  1,  0.40,  0,   -M_PI_2);
        RoadSegment line4(0.92+1-0.4,0.92,0,0,0.14,-M_PI,0);
        RoadSegment circle4(0.92+0.46,  0.92+0.46,   0,  1,  0.46,  3*M_PI / 2,   M_PI);
        std::vector<RoadSegment> innerWhiteSegments;
        innerWhiteSegments.push_back(line1);
        innerWhiteSegments.push_back(circle1);
        innerWhiteSegments.push_back(line2);
        innerWhiteSegments.push_back(circle2);
        innerWhiteSegments.push_back(line3);
        innerWhiteSegments.push_back(circle3);
        innerWhiteSegments.push_back(line4);
        innerWhiteSegments.push_back(circle4);
        RoadLine innerWhite(innerWhiteSegments,1);
        innerWhite.default_resolution_for_display = 0.05;
        RoadSegment yline1(0.46,     0.46+0.60,   0,  0,  2.0,   M_PI_2, 0);
        RoadSegment ycircle1(0.46+0.60,  .46+3.20-0.60,   0,  1,  0.60,  M_PI,   M_PI/2);
        RoadSegment yline2(0.46+0.60,0.46+3.20,0,0,0.8,0,0);
        RoadSegment ycircle2(0.46+1.92-0.50,  .46+3.2-0.50,   0,  1,  0.50,  M_PI/2,   0);
        RoadSegment yline3(0.46+1.92,0.46+3.20-0.50,0,0,2.2,-M_PI_2,0);
        RoadSegment ycircle3(0.46+1.92-0.50,  0.46+0.50,   0,  1,  0.50,  0,   -M_PI_2);
        RoadSegment yline4(0.46+1.92-0.5,0.46,0,0,0.8,-M_PI,0);
        RoadSegment ycircle4(0.46+0.60,  0.46+0.60,   0,  1,  0.60,  3*M_PI / 2,   M_PI);
        std::vector<RoadSegment> yellowSegments;
        yellowSegments.push_back(yline1);
        yellowSegments.push_back(ycircle1);
        yellowSegments.push_back(yline2);
        yellowSegments.push_back(ycircle2);
        yellowSegments.push_back(yline3);
        yellowSegments.push_back(ycircle3);
        yellowSegments.push_back(yline4);
        yellowSegments.push_back(ycircle4);
        RoadLine yellow(yellowSegments,false,true,0);
        yellow.default_resolution_for_display = 0.05;
        std::vector<RoadSegment> outerWhiteSegments;
        RoadSegment oline1(0,     0+0.76,   0,  0,  2.6,   M_PI_2, 0);
        RoadSegment ocircle1(0+0.76,  0+4.12-0.76,   0,  1,  0.76,  M_PI,   M_PI/2);
        RoadSegment oline2(0+0.76,0+4.12,0,0,1.32,0,0);
        RoadSegment ocircle2(0+2.84-0.76,  0+4.12-0.76,   0,  1,  0.76,  M_PI/2,   0);
        RoadSegment oline3(0+2.84,0+4.12-0.76,0,0,2.6,-M_PI_2,0);
        RoadSegment ocircle3(0+2.84-0.76,  0+0.76,   0,  1,  0.76,  0,   -M_PI_2);
        RoadSegment oline4(0+2.84-0.76,0,0,0,1.32,-M_PI,0);
        RoadSegment ocircle4(0+0.76,  0+0.76,   0,  1,  0.76,  3*M_PI / 2,   M_PI);
        outerWhiteSegments.push_back(oline1);
        outerWhiteSegments.push_back(ocircle1);
        outerWhiteSegments.push_back(oline2);
        outerWhiteSegments.push_back(ocircle2);
        outerWhiteSegments.push_back(oline3);
        outerWhiteSegments.push_back(ocircle3);
        outerWhiteSegments.push_back(oline4);
        outerWhiteSegments.push_back(ocircle4);
        RoadLine outerWhite(outerWhiteSegments,2);
        outerWhite.default_resolution_for_display = 0.05;
        std::vector<RoadLine> roadLines;
        roadLines.push_back(yellow);
        roadLines.push_back(innerWhite);
        roadLines.push_back(outerWhite);
        Road realTrack(roadLines,false);
        return realTrack;
    }
    if(trackNumber == 7){
        RoadBuilder roadBuilder;
        roadBuilder.setNumberOfLanes(2);
        int firstLine = RoadBuilder::WHITE_LINE | RoadBuilder::SOLID_LINE;
        int secondLine = RoadBuilder::YELLOW_LINE | RoadBuilder::DOTTED_LINE;
        int thirdLine = RoadBuilder::WHITE_LINE | RoadBuilder::SOLID_LINE;
        roadBuilder.addLineDescription(firstLine);
        roadBuilder.addLineDescription(secondLine);
        roadBuilder.addLineDescription(thirdLine);
        roadBuilder.setStartingPoint(0,0,0,0);
        roadBuilder.addStraightLine(5);
        roadBuilder.addCurveFromRadiusOfCurvature(1.5 * M_PI_2,1.5,true);
        roadBuilder.addStraightLine(4);
        roadBuilder.addCurveFromRadiusOfCurvature(1.75 * M_PI_2,1.75,true);
        roadBuilder.addStraightLine(5);
        roadBuilder.addCurveFromRadiusOfCurvature(1.5 * M_PI_2,1.5,true);
        roadBuilder.addStraightLine(4);
        roadBuilder.addCurveFromRadiusOfCurvature(1.75 * M_PI_2,1.75,true);
        return roadBuilder.getRoad();
    }
    if(trackNumber == 8){
        RoadBuilder roadBuilder;
        roadBuilder.setNumberOfLanes(2);
        int firstLine = RoadBuilder::WHITE_LINE | RoadBuilder::SOLID_LINE;
        int secondLine = RoadBuilder::YELLOW_LINE | RoadBuilder::SOLID_LINE;
        int thirdLine = RoadBuilder::WHITE_LINE | RoadBuilder::SOLID_LINE;
        roadBuilder.addLineDescription(firstLine);
        roadBuilder.addLineDescription(secondLine);
        roadBuilder.addLineDescription(thirdLine);
        roadBuilder.setStartingPoint(0,0,0,0);
        roadBuilder.addStraightLine(40);
        roadBuilder.addCurveFromRadiusOfCurvature(5 * M_PI,5,true);
        roadBuilder.addStraightLine(40);
        roadBuilder.addCurveFromRadiusOfCurvature(5 * M_PI,5,true);
        return roadBuilder.getRoad();
    }
    
    return Road();
}
#endif
