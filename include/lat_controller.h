#include <ros/ros.h>
#include <Eigen/Dense>
#include "std_msgs/Float32.h"
#include <time.h>

namespace lat_controller{
    class LatController
    {
    public:
        LatController();
  	    ~LatController();
  	    void LatControllerMainLoop();

    protected:
        // ros handle
	    ros::NodeHandle private_nh_;

	    // ros subscriber
        //ros::Subscriber sub_phi_dot; // phi_dot is V_x/R;
        ros::Subscriber sub_R; // the radius of path
        ros::Subscriber sub_state_1; // the current state1 from vehicle model;
        ros::Subscriber sub_state_2; // the current state2 from vehicle model;
        ros::Subscriber sub_state_3; // the current state3 from vehicle model;
        ros::Subscriber sub_state_4; // the current state4 from vehicle model;
        ros::Subscriber sub_current_v; // the current v from vehicle model;
       
        // ros publisher
	    ros::Publisher pub_cmd_steer_angle_; // cmd is the steer cmd from controller to vehicle model;
        ros::Publisher pub_cmd_steer_angle_rate_; //

        // vehicle parameters which the controller konws
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

         // lqr computation parameters
        float lqr_param1_low, lqr_param2_low, lqr_param3_low, lqr_param4_low;
	    float lqr_param1, lqr_param2, lqr_param3, lqr_param4;
	    float lqr_param1_high, lqr_param2_high, lqr_param3_high, lqr_param4_high;
        float max_lat_acc_;
    	float lqr_eps_;
    	int lqr_max_iteration_;

        // constant
	    int LOOP_RATE_; // Ros node's loop rate.
	    float ts_; 

        // variables
	    float current_vel_ = 0;
        float pre_steer_angle_ = 0.0;

        float minimum_speed_protection_ = 0.1;

        // matrix related
	    const int state_size_ = 4;

        // control command
	    float target_steer_angle_feedback = 0.0;
	    float target_steer_angle_forward = 0.0;
	    float target_steer_angle = 0.0;
        float steer_angel_rate = 0.5;

         // the road phi_dot
        //float phi_dot_;
        float R = 10000.0;

        // time 
	    double start;
	    double end;
	    double time_duration;

        // vehicle state matrix
        Eigen::MatrixXd matrix_a_;
        // vehicle state matrix (discrete-time)
        Eigen::MatrixXd matrix_ad_;
        // control matrix
        Eigen::MatrixXd matrix_b_;
        // control matrix (discrete-time)
        Eigen::MatrixXd matrix_bd_;
        // gain matrix
        Eigen::MatrixXd matrix_k_;
        // control authority weighting matrix
        Eigen::MatrixXd matrix_r_;
        // state weighting matrix
        Eigen::MatrixXd matrix_q_;
        // vehicle state matrix coefficients
        Eigen::MatrixXd matrix_a_coeff_;
         // 4 by 1 matrix; state matrix
        Eigen::MatrixXd matrix_state_;

        // callback of subscribers
        //void callbackFromPhiDot(const std_msgs::Float32 &msg);
        void callbackFromState1(const std_msgs::Float32 &msg);
        void callbackFromState2(const std_msgs::Float32 &msg);
        void callbackFromState3(const std_msgs::Float32 &msg);
        void callbackFromState4(const std_msgs::Float32 &msg);
        void callbackFromCurrentV(const std_msgs::Float32 &msg);
        void callbackFromR(const std_msgs::Float32 &msg);

        // initializer
	    void initForROS();
        // tool functions
        void initParamter();
        void initSystemMatrix();
        void updateLQRParams();
        void updateSystemMatrix();
        float Clamp(float value, float low, float high);

        // control solver
        void SolveLQRProblem(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
                     const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
                     const double tolerance, const uint max_num_iteration,
                     Eigen::MatrixXd *ptr_K);

        void forwardControl();

        void computeLateralCommand();

        void publishLateralCommand();


    };

}