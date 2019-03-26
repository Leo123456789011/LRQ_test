#include "road.h"

namespace road_ns{

ROAD::ROAD()
:private_nh_("~")
,LOOP_RATE_(20)
,ts_(1.0/LOOP_RATE_)
{   
    initForROS();
}

ROAD::~ROAD()
{}

void ROAD::initForROS(){

    //ros publisher
    pub_radius = private_nh_.advertise<std_msgs::Float32>("/road/R", 10);

    // start information
	ROS_INFO("Road Node Started!");

}

void ROAD::publish(){

    std_msgs::Float32 ros_Radius;
    ros_Radius.data = R;

    pub_radius.publish(ros_Radius);

}


void ROAD::RoadMainLoop(){

    ros::Rate loop_rate(LOOP_RATE_);
    while (ros::ok()){

        // mark start time
		start = clock();
        ros::spinOnce();

        publish();

        end = clock();
		time_duration = (end - start)/CLOCKS_PER_SEC;

		loop_rate.sleep();
    }
}

}