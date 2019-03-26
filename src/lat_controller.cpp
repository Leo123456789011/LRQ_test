
#include "lat_controller.h"

namespace lat_controller{

LatController::LatController()
: private_nh_("~")
, LOOP_RATE_(10)
, ts_(1.0/LOOP_RATE_)
, state_size_(4)
, target_steer_angle_feedback(0.0)
, target_steer_angle_forward(0.0)
, target_steer_angle(0.0)
{
	// ros initialization
	initForROS();
	// parameter initialization
	initParamter();
	// matrix initialization
	initSystemMatrix();
}

LatController::~LatController()
{}

void LatController::initForROS()
{
    //ros subscriber
    //sub_phi_dot = private_nh_.subscribe("/model/phi_dot",10,&LatController::callbackFromPhiDot, this);
    sub_R = private_nh_.subscribe("/road/R",10,&LatController::callbackFromR, this);
    sub_state_1 = private_nh_.subscribe("/model/state1",10,&LatController::callbackFromState1, this);
    sub_state_2 = private_nh_.subscribe("/model/state2",10,&LatController::callbackFromState2, this);
    sub_state_3 = private_nh_.subscribe("/model/state3",10,&LatController::callbackFromState3, this);
    sub_state_4 = private_nh_.subscribe("/model/state4",10,&LatController::callbackFromState4, this);

    sub_current_v = private_nh_.subscribe("/model/current_v",10,&LatController::callbackFromCurrentV,this);

    //ros publisher
    pub_cmd_steer_angle_ = private_nh_.advertise<std_msgs::Float32>("/control/target_steer_angle", 10);
    pub_cmd_steer_angle_rate_ = private_nh_.advertise<std_msgs::Float32>("/control/steer_angle_rate",10);
    // start information
	ROS_INFO("Lateral Controller Started!");
}

void LatController::initParamter()
{
    // vehicle model parameter which the controller knows
	private_nh_.getParam("/lat_controller/mass_fl_", mass_fl_);
	private_nh_.getParam("/lat_controller/mass_fr_", mass_fr_);
	private_nh_.getParam("/lat_controller/mass_rl_", mass_rl_);
	private_nh_.getParam("/lat_controller/mass_rr_", mass_rr_);
	private_nh_.getParam("/lat_controller/wheel_base_", wheel_base_);
	private_nh_.getParam("/lat_controller/cf_", cf_);
	private_nh_.getParam("/lat_controller/cr_", cr_);
	private_nh_.getParam("/lat_controller/steer_ratio_", steer_ratio_);
	private_nh_.getParam("/lat_controller/steer_single_direction_max_degree_", steer_single_direction_max_degree_);

    // basic_state_size_
	mass_front_ = mass_fl_ + mass_fr_;
	mass_rear_ = mass_rl_ + mass_rr_;
	mass_ = mass_front_ + mass_rear_;

	lf_ = wheel_base_ * (1.0 - mass_front_ / mass_);
	lr_ = wheel_base_ * (1.0 - mass_rear_ / mass_);
	iz_ = lf_ * lf_ * mass_front_ + lr_ * lr_ * mass_rear_;

    // max_lat_acc_
    private_nh_.getParam("/lat_controller/max_lat_acc_",max_lat_acc_);
    
	// lqr computation parameter
	private_nh_.getParam("/lat_controller/lqr_eps_", lqr_eps_);
	private_nh_.getParam("/lat_controller/lqr_max_iteration_", lqr_max_iteration_);

    // lqr params
	private_nh_.getParam("/lat_controller/lqr_param1_low", lqr_param1_low);
	private_nh_.getParam("/lat_controller/lqr_param2_low", lqr_param2_low);
	private_nh_.getParam("/lat_controller/lqr_param3_low", lqr_param3_low);
	private_nh_.getParam("/lat_controller/lqr_param4_low", lqr_param4_low);

	private_nh_.getParam("/lat_controller/lqr_param1", lqr_param1);
	private_nh_.getParam("/lat_controller/lqr_param2", lqr_param2);
	private_nh_.getParam("/lat_controller/lqr_param3", lqr_param3);
	private_nh_.getParam("/lat_controller/lqr_param4", lqr_param4);

	private_nh_.getParam("/lat_controller/lqr_param1_high", lqr_param1_high);
	private_nh_.getParam("/lat_controller/lqr_param2_high", lqr_param2_high);
	private_nh_.getParam("/lat_controller/lqr_param3_high", lqr_param3_high);
	private_nh_.getParam("/lat_controller/lqr_param4_high", lqr_param4_high);
    
}

void LatController::initSystemMatrix(){
    matrix_a_ = Eigen::MatrixXd::Zero(state_size_, state_size_);
	matrix_ad_ = Eigen::MatrixXd::Zero(state_size_, state_size_);
    matrix_a_(0, 1) = 1.0;
	matrix_a_(1, 2) = (2 * cf_ + 2 * cr_) / mass_;
	matrix_a_(2, 3) = 1.0;
	matrix_a_(3, 2) = (lf_ * 2 * cf_ - lr_ * 2 * cr_) / iz_;

    matrix_a_coeff_ = Eigen::MatrixXd::Zero(state_size_, state_size_);
	matrix_a_coeff_(1, 1) = -(2 * cf_ + 2 * cr_) / mass_;
	matrix_a_coeff_(1, 3) = (lr_ * 2 * cr_ - lf_ * 2 * cf_) / mass_;
	matrix_a_coeff_(2, 3) = 1.0;
	matrix_a_coeff_(3, 1) = (lr_ * 2 * cr_ - lf_ * 2 * cf_) / iz_;
	matrix_a_coeff_(3, 3) = -1.0 * (lf_ * lf_ * 2 * cf_ + lr_ * lr_ * 2 * cr_) / iz_;

    matrix_b_ = Eigen::MatrixXd::Zero(state_size_, 1);
	matrix_bd_ = Eigen::MatrixXd::Zero(state_size_, 1);

    matrix_b_(1, 0) = 2 * cf_ / mass_ - 2 * cr_ / mass_;              // since the rear wheel is also controllable which is different from the bicycle model
	matrix_b_(3, 0) = lf_ * 2 * cf_ / iz_ + lr_ * 2 * cr_/ iz_;          // since the rear wheel is also controllable whchi is different from the bicycle model
	matrix_bd_ = matrix_b_ * ts_;

    matrix_state_ = Eigen::MatrixXd::Zero(state_size_, 1);
	matrix_k_ = Eigen::MatrixXd::Zero(1, state_size_);
	matrix_r_ = Eigen::MatrixXd::Identity(1, 1);
	matrix_q_ = Eigen::MatrixXd::Zero(state_size_, state_size_);

    matrix_q_(0,0) = lqr_param1;
	matrix_q_(1,1) = lqr_param2;
	matrix_q_(2,2) = lqr_param3;
	matrix_q_(3,3) = lqr_param4;
}

void LatController::updateLQRParams()
{
	if (fabs(current_vel_) < 16)
	{
		matrix_q_(0,0) = lqr_param1_low;
		matrix_q_(1,1) = lqr_param2_low;
		matrix_q_(2,2) = lqr_param3_low;
		matrix_q_(3,3) = lqr_param4_low;
	}
	else if (fabs(current_vel_) < 26)
	{
		matrix_q_(0,0) = lqr_param1;
		matrix_q_(1,1) = lqr_param2;
		matrix_q_(2,2) = lqr_param3;
		matrix_q_(3,3) = lqr_param4;
	}
	else if (fabs(current_vel_) < 36)
	{
		matrix_q_(0,0) = lqr_param1_high;
		matrix_q_(1,1) = lqr_param2_high;
		matrix_q_(2,2) = lqr_param3_high;
		matrix_q_(3,3) = lqr_param4_high;
	}
}

void LatController::updateSystemMatrix(){

    const float v = std::max(current_vel_,
                            minimum_speed_protection_);
    //std::cout << "v: " << v << std::endl;
    matrix_a_(1, 1) = matrix_a_coeff_(1, 1) / v;
	matrix_a_(1, 3) = matrix_a_coeff_(1, 3) / v;
	matrix_a_(3, 1) = matrix_a_coeff_(3, 1) / v;
	matrix_a_(3, 3) = matrix_a_coeff_(3, 3) / v;

    Eigen::MatrixXd matrix_i;
	matrix_i = Eigen::MatrixXd::Identity(matrix_a_.cols(), matrix_a_.cols());
	matrix_ad_ = (matrix_i + ts_ * 0.5 * matrix_a_) *
               (matrix_i - ts_ * 0.5 * matrix_a_).inverse();

    //std::cout << "matrix_a_: " << matrix_a_ << std::endl;
    //std::cout << "matrix_i: " << matrix_i << std::endl;
    //std::cout << "ts_: " << ts_ << std::endl;
    //std::cout << "matrix_ad_: " << matrix_ad_ << std::endl;

    // update lqr params
	updateLQRParams();
}

void LatController::SolveLQRProblem(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, const Eigen::MatrixXd &Q,
                     const Eigen::MatrixXd &R, const double tolerance,
                     const uint max_num_iteration, Eigen::MatrixXd *ptr_K) {
  if (A.rows() != A.cols() || B.rows() != A.rows() || Q.rows() != Q.cols() ||
      Q.rows() != A.rows() || R.rows() != R.cols() || R.rows() != B.cols()) {
    std::cout << "LQR solver: one or more matrices have incompatible dimensions." << std::endl;
    return;
  }

  Eigen::MatrixXd AT = A.transpose();
  Eigen::MatrixXd BT = B.transpose();

  // Solves a discrete-time Algebraic Riccati equation (DARE)
  // Calculate Matrix Difference Riccati Equation, initialize P and Q
  Eigen::MatrixXd P = Q;
  uint num_iteration = 0;
  double diff = std::numeric_limits<double>::max();
  while (num_iteration++ < max_num_iteration && diff > tolerance) {
    Eigen::MatrixXd P_next =
        AT * P * A - AT * P * B * (R + BT * P * B).inverse() * BT * P * A + Q;
    //std::cout << "(R + BT * P * B): " << ( (R + BT * P * B) ) << std::endl;
    //std::cout << "(R + BT * P * B).inverse(): " << ( (R + BT * P * B).inverse() ) << std::endl;
    //std::cout << "AT * P * A - AT * P * B * (R + BT * P * B).inverse() * BT * P * A + Q: " << ( AT * P * A - AT * P * B * (R + BT * P * B).inverse() * BT * P * A + Q ) << std::endl;
    //std::cout << "AT * P * A : " << ( AT * P * A  ) << std::endl;
    //std::cout << "AT * P * B * (R + BT * P * B).inverse() * BT * P * A: " << ( P_next ) << std::endl;
    //std::cout << "Q: " << ( Q ) << std::endl;
    //std::cout << "P_next: " << ( P_next ) << std::endl;
    // check the difference between P and P_next
    diff = fabs((P_next - P).maxCoeff());
    P = P_next;
  }
  //std::cout << "num_iteration: " << num_iteration << std::endl;
  //std::cout << "diff: " << diff << std::endl;
  //std::cout << "P: " << P << std::endl;

  if (num_iteration >= max_num_iteration) {
    //std::cout << "LQR solver cannot converge to a solution, "
    //         "last consecutive result diff. is:"
    //      << diff << std::endl;
  } else {
    //std::cout << "LQR solver converged at iteration: " << num_iteration
    //       << ", max consecutive result diff.: " << diff << std::endl;
  }

  *ptr_K = (R + BT * P * B).inverse() * BT * P * A;
}

void LatController::forwardControl()
{   
    //float ref_curvature = phi_dot_/current_vel_; // phi_dot_ is V_x/R, current_vel_ is V_x. 
    float ref_curvature = 1/R;
	float kv = lr_ * mass_ / 2 / cf_ / wheel_base_ - lf_ * mass_ / 2 / cr_ / wheel_base_;

	target_steer_angle_forward = (wheel_base_ * ref_curvature + kv * current_vel_ * current_vel_ * ref_curvature - 
		matrix_k_(0, 2) * (lr_ * ref_curvature - lf_ * mass_ * current_vel_ * current_vel_ * ref_curvature / 2 / cr_ / wheel_base_)) * 180
        / M_PI * steer_ratio_ / steer_single_direction_max_degree_;

}

float LatController::Clamp(float value, float low, float high)
{
	if (value < low) return low;
		else if (value > high) return high;
			else return value;
}

void LatController::computeLateralCommand()
{
    updateSystemMatrix();
    //std::cout << "matrix_ad_: " << matrix_ad_ << std::endl;
    //std::cout << "matrix_bd_: " << matrix_bd_ << std::endl;
    //std::cout << "matrix_q_: " << matrix_q_ << std::endl;
    //std::cout << "matrix_r_: " << matrix_r_ << std::endl;

    SolveLQRProblem(matrix_ad_,matrix_bd_,matrix_q_,matrix_r_,
    lqr_eps_,lqr_max_iteration_,&matrix_k_);

    //std::cout << "matrix_k_: " << matrix_k_ << std::endl;
    //std::cout << "matrix_state_: " << matrix_state_ << std::endl;
    target_steer_angle_feedback =  -(matrix_k_ * matrix_state_)(0, 0) * 180
        / M_PI * steer_ratio_ / steer_single_direction_max_degree_;

    //std::cout << "matrix_k_ * matrix_state_* 180 / M_PI: " << (matrix_k_ * matrix_state_ * 180 / M_PI) << std::endl;

    forwardControl();

	target_steer_angle = target_steer_angle_feedback + target_steer_angle_forward;
	target_steer_angle = Clamp(target_steer_angle, -0.84, 0.84);

    //std::cout << "target_steer_angle: " << target_steer_angle << std::endl;
    //std::cout << "target_steer_angle_feedback: " << target_steer_angle_feedback << std::endl;
    //std::cout << "target_steer_angle_forward: " << target_steer_angle_forward << std::endl;
    //std::cout << "matrix_state_: " << matrix_state_ << std::endl;

    bool set_steer_limit = true;
    if (set_steer_limit) {
    const double steer_limit =
        std::atan(max_lat_acc_ * wheel_base_ /
                  (current_vel_*current_vel_)) *
        steer_ratio_ * 180 / M_PI /
        steer_single_direction_max_degree_ * 100;

    // Clamp the steer angle
    double target_steer_angle_limited =
        Clamp(target_steer_angle, -steer_limit, steer_limit);
    //target_steer_angle_limited = digital_filter_.Filter(steer_angle_limited);
    target_steer_angle = target_steer_angle_limited;
  }
    float lock_steer_speed = 0.081;
    if (current_vel_ < lock_steer_speed) {
    target_steer_angle = pre_steer_angle_;
  }
  pre_steer_angle_ = target_steer_angle;

}

void LatController::publishLateralCommand()
{
std_msgs::Float32 ros_target_steer_angle;
ros_target_steer_angle.data = target_steer_angle;
//ros_target_steer_angle.data = 0;

std_msgs::Float32 ros_steer_angel_rate;
ros_steer_angel_rate.data = steer_angel_rate;

pub_cmd_steer_angle_.publish(ros_target_steer_angle);
pub_cmd_steer_angle_rate_.publish(ros_steer_angel_rate);
}


void LatController::LatControllerMainLoop()
{
    ros::Rate loop_rate(LOOP_RATE_);
    while (ros::ok()){

        // mark start time
		start = clock();
        ros::spinOnce();

        computeLateralCommand();

        publishLateralCommand();

        end = clock();
		time_duration = (end - start)/CLOCKS_PER_SEC;

		loop_rate.sleep();
    }

}

//void LatController::callbackFromPhiDot(const std_msgs::Float32 &msg){
//    phi_dot_ = msg.data; 
//}

void LatController::callbackFromR(const std_msgs::Float32 &msg){
    R = msg.data;
}
void LatController::callbackFromState1(const std_msgs::Float32 &msg){
    matrix_state_(0) =  msg.data; 
}
void LatController::callbackFromState2(const std_msgs::Float32 &msg){
    matrix_state_(1) =  msg.data; 
}
void LatController::callbackFromState3(const std_msgs::Float32 &msg){
    matrix_state_(2) =  msg.data; 
}
void LatController::callbackFromState4(const std_msgs::Float32 &msg){
    matrix_state_(3) =  msg.data; 
}
void LatController::callbackFromCurrentV(const std_msgs::Float32 &msg){
    current_vel_ =  msg.data;
    //std::cout << "current_vel_: " << current_vel_ << std::endl;
}

}