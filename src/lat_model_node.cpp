#include "lat_model.h"

int main(int argc, char **argv){

    ros::init(argc, argv, "lat_model");
    lat_model::LatModel Lat_model;
    Lat_model.LatModelMainLoop();
    return 0;

}