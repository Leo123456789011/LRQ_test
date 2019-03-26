#include <ros/ros.h>
#include <Eigen/Dense>
#include "std_msgs/Float32.h"
#include <time.h>

namespace lat_model{
    class LatModel 
    {
        public:
            LatModel();
            ~LatModel();
            void LatModelMainLoop();

        protected:
            // ros handle.
            ros::NodeHandle private_nh_;
            
            // ros subscriber
            ros::Subscriber sub_target_steer_angle; // the cmd steer angle from lateral controller
            ros::Subscriber sub_target_steer_angle_rate; // the cmd steer angle rate from lateral controller
            ros::Subscriber sub_R;  

            // ros publisher
            ros::Publisher pub_state1; 
            ros::Publisher pub_state2;
            ros::Publisher pub_state3;
            ros::Publisher pub_state4;
            ros::Publisher pub_current_vel;
            ros::Publisher pub_current_steer_angle_; // publish the current steer_angle;

            //the real vehicle model parameters
            float mass_fl_, mass_fr_, mass_rl_, mass_rr_;
            float mass_front_, mass_rear_;
            float mass_;
            float lf_, lr_;
            float wheel_base_;
            float steer_ratio_;
            float steer_single_direction_max_degree_;
            float iz_;
            float cf_;
            float cr_;

            // constant
	        int LOOP_RATE_; // Ros node's loop rate.
	        float ts_;

            // variables
            float current_vel_ = 0;
            float target_steer_angle_ =0; // The aim steer angle from lqr
            float steer_angle_ = 0; // The current steer angle
            float steer_angle_rate_ = 0; 
            
            // matrix related
	        const int state_size_ = 0;

            // time 
	        double start;
	        double end;
	        double time_duration;

            // Road 
            float R = 0; // radius of path.

            float minimum_speed_protection_ = 0.1;

            // vehicle state matrix
            Eigen::MatrixXd matrix_a_;
            // vehicle state matrix (discrete-time)
            Eigen::MatrixXd matrix_ad_;
            // control matrix1
            Eigen::MatrixXd matrix_b_1_;
            // control matrix1 (discrete-time)
            Eigen::MatrixXd matrix_bd_1_;
             // control matrixb_b_2
            Eigen::MatrixXd matrix_b_2_;
            // control matrix_b_2 (discrete-time)
            Eigen::MatrixXd matrix_bd_2_;
            // vehicle state matrix coefficients
            Eigen::MatrixXd matrix_a_coeff_;
            // matrix_b_2_ coefficients
            Eigen::MatrixXd matrix_b_2_coeff_;
             // 4 by 1 matrix; state matrix
            Eigen::MatrixXd matrix_state_;
            // 4 by 1 matrix; next state matrix
            Eigen::MatrixXd matrix_state_next_;
        
            // initializer
	        void initForROS();

            //callback of subscribers
            void callbackFromSteerAngle(const std_msgs::Float32 &msg);
            void callbackFromSteerAngleRate(const std_msgs::Float32 &msg);
            void callbackFromR(const std_msgs::Float32 &msg);
        
            void initParamter();
            void initSystemMatrix();
            void updateSystemMatrix();
            void updateCurrentSteerAngle();
            void ComputeNextState();
            void Publish();

    };
}