#include "road.h"

int main(int argc, char **argv){

    ros::init(argc, argv, "road_node");
    road_ns::ROAD road;
    road.RoadMainLoop();
    return 0;
}