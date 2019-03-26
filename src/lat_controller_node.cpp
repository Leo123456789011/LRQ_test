#include "lat_controller.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lat_controller");
    lat_controller::LatController lat_control;
    lat_control.LatControllerMainLoop();
    return 0;
}