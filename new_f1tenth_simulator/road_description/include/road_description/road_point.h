#ifndef cpp_road_point

struct Point{
    double x;
    double y;
    double z;
    double rotation;
}; 

double wrapAngle(double angle){
    while(angle<0){
        angle+=2*M_PI;
    }
    while(angle>=2*M_PI){
        angle-=2*M_PI;
    }
    return angle;
}

double averagePointAngles(double first, double second){
    first= wrapAngle(first);
    second = wrapAngle(second);
    if(first>second){
        double temp = first;
        first = second;
        second = temp;
    }
    if(abs(second-first)>M_PI){
        second -= 2 *M_PI;
        double average = (first+second) /2;
        if(average <0){
            average += 2 *M_PI;
        }
        return average;
    }else{
        return (first+second) / 2;
    }

}

double differenceInPointAngles(double first, double second){
    first = wrapAngle(first);
    second = wrapAngle(second);
    if(first>second){
        double temp = first;
        first = second;
        second = temp;
    }
    if(abs(second-first)>M_PI){
        return 2*M_PI - (second-first);
    }else{
        return second-first;
    }
}
#endif