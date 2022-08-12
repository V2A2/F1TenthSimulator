#include <road_description/road.h>
#include <vector>
#include <cmath>

#ifndef cpp_road_config
Road get_road(int trackNumber);
Road get_default_road(){
    return get_road(2);
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
        return Road(roadLines,false);
    }
    return Road();
}
#endif