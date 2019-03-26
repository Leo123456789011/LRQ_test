#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include <time.h>

namespace road_ns{

    class ROAD
    {
        public:
            ROAD();
            ~ROAD();
            void RoadMainLoop();

        protected:
            ros::NodeHandle private_nh_;

            //subscriber
            //_

            //publisher
            ros::Publisher pub_radius;

            // time 
	        double start;
	        double end;
	        double time_duration;
            
            // variable
            float LOOP_RATE_;
            float ts_;

            double R = 100000; // unit: m 

            //function
            void initForROS();
            void publish();
    };

}