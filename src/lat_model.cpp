#include "lat_model.h"

namespace lat_model{

LatModel::LatModel()
:private_nh_("~")
,LOOP_RATE_(500)
,ts_(1.0/LOOP_RATE_)
,state_size_(4)
{
    // ros initialization
	initForROS();
	// parameter initialization
	initParamter();
	// matrix initialization
	initSystemMatrix();
}

LatModel::~LatModel()
{}

void LatModel::initForROS()
{   
    //ros subscriber
    sub_target_steer_angle = private_nh_.subscribe("/control/target_steer_angle", 10, &LatModel::callbackFromSteerAngle, this);
    sub_target_steer_angle_rate = private_nh_.subscribe("/control/steer_angle_rate",10, &LatModel::callbackFromSteerAngleRate,this);
    sub_R = private_nh_.subscribe("/road/R", 10, &LatModel::callbackFromR, this);

    //ros publisher
    pub_state1 = private_nh_.advertise<std_msgs::Float32>("/model/state1", 10);
    pub_state2 = private_nh_.advertise<std_msgs::Float32>("/model/state2", 10);
    pub_state3 = private_nh_.advertise<std_msgs::Float32>("/model/state3", 10);
    pub_state4 = private_nh_.advertise<std_msgs::Float32>("/model/state4", 10);
    pub_current_vel = private_nh_.advertise<std_msgs::Float32>("/model/current_v", 10);
    pub_current_steer_angle_ =private_nh_.advertise<std_msgs::Float32>("/model/current_steer_angle", 10);

    ROS_INFO("Lateral Model node Ready");
}

void LatModel::initParamter()
{
    // vehicle model parameter
	private_nh_.getParam("/lat_model/mass_fl_", mass_fl_);
	private_nh_.getParam("/lat_model/mass_fr_", mass_fr_);
	private_nh_.getParam("/lat_model/mass_rl_", mass_rl_);
	private_nh_.getParam("/lat_model/mass_rr_", mass_rr_);
	private_nh_.getParam("/lat_model/wheel_base_", wheel_base_);
	private_nh_.getParam("/lat_model/cf_", cf_);
	private_nh_.getParam("/lat_model/cr_", cr_);
	private_nh_.getParam("/lat_model/steer_ratio_", steer_ratio_);
	private_nh_.getParam("/lat_model/steer_single_direction_max_degree_", steer_single_direction_max_degree_);
    private_nh_.getParam("/lat_model/current_vel_", current_vel_ );
    private_nh_.getParam("/lat_model/initial_steer_angle_", steer_angle_ );

    // basic_state_size_
	mass_front_ = mass_fl_ + mass_fr_;
	mass_rear_ = mass_rl_ + mass_rr_;
	mass_ = mass_front_ + mass_rear_;

	lf_ = wheel_base_ * (1.0 - mass_front_ / mass_);
	lr_ = wheel_base_ * (1.0 - mass_rear_ / mass_);
	iz_ = lf_ * lf_ * mass_front_ + lr_ * lr_ * mass_rear_;
}

void LatModel::initSystemMatrix(){

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

    matrix_b_1_ = Eigen::MatrixXd::Zero(state_size_, 1);
	matrix_bd_1_ = Eigen::MatrixXd::Zero(state_size_, 1);
    matrix_b_1_(1, 0) = 2 * cf_ / mass_ - 2 * cr_ / mass_; // since the rear wheel is also controllable which is different from the bicycle model
	matrix_b_1_(3, 0) = lf_ * 2 * cf_ / iz_ + lr_ * 2 * cr_/ iz_;
	matrix_bd_1_ = matrix_b_1_ * ts_;

    matrix_b_2_ = Eigen::MatrixXd::Zero(state_size_, 1);
    matrix_bd_2_ = Eigen::MatrixXd::Zero(state_size_, 1);
    matrix_b_2_coeff_ = Eigen::MatrixXd::Zero(state_size_, 1);
    matrix_b_2_coeff_(1, 0) = (lr_ * 2 * cr_ - lf_ * 2 * cf_) / mass_;
    matrix_b_2_coeff_(3, 0) = -1.0 * (lf_ * lf_ * 2 * cf_ + lr_ * lr_ * 2 * cr_) / iz_;
    

    matrix_state_ = Eigen::MatrixXd::Zero(state_size_, 1);
    matrix_state_next_ = Eigen::MatrixXd::Zero(state_size_, 1);

    private_nh_.getParam("/lat_model/state1_", matrix_state_(0,0)); // 初始化模型状态
    private_nh_.getParam("/lat_model/state2_", matrix_state_(1,0)); // 初始化模型状态
    private_nh_.getParam("/lat_model/state3_", matrix_state_(2,0)); // 初始化模型状态
    private_nh_.getParam("/lat_model/state4_", matrix_state_(3,0)); // 初始化模型状态
    //std::cout << "matrix_state_: " << matrix_state_ << std::endl;

}

void LatModel::updateSystemMatrix(){

    const float v = std::max(current_vel_,
                            minimum_speed_protection_);
    matrix_a_(1, 1) = matrix_a_coeff_(1, 1) / v;
	matrix_a_(1, 3) = matrix_a_coeff_(1, 3) / v;
	matrix_a_(3, 1) = matrix_a_coeff_(3, 1) / v;
	matrix_a_(3, 3) = matrix_a_coeff_(3, 3) / v;

    Eigen::MatrixXd matrix_i;
	matrix_i = Eigen::MatrixXd::Identity(matrix_a_.cols(), matrix_a_.cols());
	matrix_ad_ = (matrix_i + ts_ * 0.5 * matrix_a_) *
               (matrix_i - ts_ * 0.5 * matrix_a_).inverse();
    //std::cout << "matrix_ad_: " << matrix_ad_ << std::endl;

    matrix_b_2_(1,0) = ( matrix_b_2_coeff_(1,0) / v ) - v;
    matrix_b_2_(3,0) = matrix_b_2_coeff_(3,0) / v;
    //std::cout << " v " <<  v  << std::endl;
    matrix_bd_2_ = matrix_b_2_ * ts_;

}

void LatModel::updateCurrentSteerAngle(){

    if(steer_angle_ == target_steer_angle_){
        steer_angle_ = target_steer_angle_;
    }
    else if(steer_angle_ < target_steer_angle_){
        steer_angle_ = steer_angle_ + steer_angle_rate_ * ts_;
    }
    else if(steer_angle_ > target_steer_angle_){
        steer_angle_ = steer_angle_ - steer_angle_rate_ * ts_;
    }
    //std::cout << "current_steer_angle_: " << steer_angle_ << std::endl;
    //std::cout << "steer_angle_rate_: " << steer_angle_rate_ << std::endl;

}

void LatModel::ComputeNextState(){

    float v = current_vel_;
    float wheel_steer_angle_  = steer_angle_ * steer_single_direction_max_degree_ / steer_ratio_ * M_PI / 180;  // using steer model;
    //float wheel_steer_angle_  = target_steer_angle_ * steer_single_direction_max_degree_ / steer_ratio_ * M_PI / 180; // without steer model;
    updateSystemMatrix();
    if( R == 0 ){
        matrix_state_next_ = matrix_ad_ * matrix_state_ + matrix_bd_1_ * wheel_steer_angle_;
    }  // At the vert beginnig, R may be 0. If R == 0, we do not consider the matrix_bd_2_ * (v/R) part.
    else 
    {
        matrix_state_next_ = matrix_ad_ * matrix_state_ + matrix_bd_1_ * wheel_steer_angle_ +  matrix_bd_2_ * (v/R);
    }
    //std::cout << "matrix_state_1: "<< matrix_state_(0, 0) << std::endl;
    std::cout << "matrix_state_: " << matrix_state_ << std::endl;
    matrix_state_ = matrix_state_next_;

}

void LatModel::Publish(){

    std_msgs::Float32 ros_state1_;
    ros_state1_.data = matrix_state_next_(0,0);
    
    std_msgs::Float32 ros_state2_;
    ros_state2_.data = matrix_state_next_(1,0);

    std_msgs::Float32 ros_state3_;
    ros_state3_.data = matrix_state_next_(2,0);

    std_msgs::Float32 ros_state4_;
    ros_state4_.data = matrix_state_next_(3,0);

    std_msgs::Float32 ros_current_vel_;
    ros_current_vel_.data = current_vel_;

    std_msgs::Float32 ros_steer_angle_;
    ros_steer_angle_.data = steer_angle_;

    pub_state1.publish(ros_state1_);
    pub_state2.publish(ros_state2_);
    pub_state3.publish(ros_state3_);
    pub_state4.publish(ros_state4_);
    pub_current_vel.publish(ros_current_vel_);
    pub_current_steer_angle_.publish(ros_steer_angle_);

}

void LatModel::LatModelMainLoop(){
    
    ros::Rate loop_rate(LOOP_RATE_);
    while (ros::ok()){

        // mark start time
        start = clock();
        ros::spinOnce();

        updateCurrentSteerAngle();
        ComputeNextState();
        Publish();

        end = clock();
        time_duration = (end - start)/CLOCKS_PER_SEC;

        loop_rate.sleep();

    }

}

void LatModel::callbackFromSteerAngle(const std_msgs::Float32 &msg){
    target_steer_angle_ = msg.data; 
    //std::cout << "steer_angle_: " << steer_angle_ << std::endl;
}

void LatModel::callbackFromSteerAngleRate(const std_msgs::Float32 &msg){
    steer_angle_rate_ = msg.data;
}

void LatModel::callbackFromR(const std_msgs::Float32 &msg){
    R = msg.data;
    //std::cout << "R: " << R << std::endl;
}

}