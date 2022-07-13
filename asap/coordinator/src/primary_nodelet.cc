/* primary_nodelet.cc

The primary coordinator, which derives from CoorindatorBase and adds methods for desired tests.
*/

#include "coordinator/primary_nodelet.h"
#include "coordinator/primary_tests.hpp"

/* ************************************************************************** */
void PrimaryNodelet::Initialize(ros::NodeHandle* nh) {
  /**
  * @brief This is called when the nodelet is loaded into the nodelet manager
  * 
  */
  // Create Multi-threaded NH
  MTNH = getMTNodeHandle();

  // Load Params
  load_params();

  // publishers
  pub_flight_mode_ = nh->advertise<ff_msgs::FlightMode>(TOPIC_MOBILITY_FLIGHT_MODE, 1, true);  // FlightMode
  pub_status_ = nh->advertise<coordinator::StatusPrimary>(TOPIC_ASAP_STATUS, 5, true);
  
  // subscribers
  sub_flight_mode_= nh->subscribe<ff_msgs::FlightMode>(TOPIC_MOBILITY_FLIGHT_MODE, 5,
    boost::bind(&PrimaryNodelet::flight_mode_callback, this, _1));  // flight mode getter
  sub_ekf_ = nh->subscribe<ff_msgs::EkfState>("gnc/ekf", 5,
    boost::bind(&PrimaryNodelet::ekf_callback, this, _1));;
  sub_test_number_ = nh->subscribe<coordinator::TestNumber>(TOPIC_ASAP_TEST_NUMBER, 5,
    boost::bind(&PrimaryNodelet::test_num_callback, this, _1));
  sub_flight_mode_= nh->subscribe<ff_msgs::FlightMode>(TOPIC_MOBILITY_FLIGHT_MODE, 5,
    boost::bind(&PrimaryNodelet::flight_mode_callback, this, _1));  // flight mode setter
  sub_ekf_ = nh->subscribe<ff_msgs::EkfState>("gnc/ekf", 5,
    boost::bind(&PrimaryNodelet::ekf_callback, this, _1));
  
  // services
  serv_ctl_enable_ = nh->serviceClient<std_srvs::SetBool>(SERVICE_GNC_CTL_ENABLE);

  // tracking points
  try{
    std::vector<double> param_data;
    nh->getParam("/asap/primary/point_a_granite", param_data);
    POINT_A_GRANITE = Eigen::Matrix<double, 7, 1>(param_data.data());  // init from std::vector param
    nh->getParam("/asap/primary/point_a_iss", param_data);
    POINT_A_ISS = Eigen::Matrix<double, 7, 1>(param_data.data());  
    nh->getParam("/asap/primary/point_b_granite", param_data);
    POINT_B_GRANITE = Eigen::Matrix<double, 7, 1>(param_data.data());  
    nh->getParam("/asap/primary/point_b_iss", param_data);
    POINT_B_ISS= Eigen::Matrix<double, 7, 1>(param_data.data());  
    nh->getParam("/asap/primary/point_c_granite", param_data);
    POINT_C_GRANITE = Eigen::Matrix<double, 7, 1>(param_data.data());  
    nh->getParam("/asap/primary/point_c_iss", param_data);
    POINT_C_ISS = Eigen::Matrix<double, 7, 1>(param_data.data()); 
  }
  catch (const std::exception &exc) {
    std::cerr << exc.what() << std::endl;
    std::cout << "[PRIMARY_COORD]: Input parameters are invalid!" << std::endl;
  }

  // Pass control to Run method (activate timers and spin)
  std::cout << "[PRIMARY_COORD] Initialized." << std::endl;
  thread_.reset(new std::thread(&CoordinatorBase::Run, this, nh));
}

/* ************************************************************************** */
void PrimaryNodelet::get_status_msg(coordinator::StatusPrimary& msg){
  /**
   * @brief Fills the StatusPrimary message with the state of the private
   * variables. Published by a coordinator Timer.
   * Inputs: base_status_ and primary_status_
   * 
   */
  msg.stamp = ros::Time::now();
  msg.test_number = base_status_.test_number;
  msg.default_control = base_status_.default_control;
  msg.flight_mode = base_status_.flight_mode;
  msg.test_finished = base_status_.test_finished;
  msg.coord_ok = base_status_.coord_ok;
  msg.regulate_finished = base_status_.regulate_finished;

  msg.control_mode = primary_status_.control_mode;
  msg.description = primary_status_.description;
}

/* ************************************************************************** */
void PrimaryNodelet::load_params(){
  // Get sim and ground flags
  std::string sim_str, ground_str;
  ros::param::get("/asap/sim", sim_str);
  sim_ = !std::strcmp(sim_str.c_str(), "true"); // convert to bool
  ros::param::get("/asap/ground", ground_str);
  ground_ = !std::strcmp(ground_str.c_str(), "true");  // convert to bool, 1 if it's true
  
  // regulation
  ros::param::getCached("/asap/primary/reg_time", reg_time_);
  ros::param::getCached("/asap/primary/x_start", x0_(0));
  ros::param::getCached("/asap/primary/y_start", x0_(1));
  ros::param::getCached("/asap/primary/z_start", x0_(2));
  ros::param::getCached("/asap/primary/qx_start", a0_(0));
  ros::param::getCached("/asap/primary/qy_start", a0_(1));
  ros::param::getCached("/asap/primary/qz_start", a0_(2));
  ros::param::getCached("/asap/primary/qw_start", a0_(3));
  ros::param::getCached("/asap/primary/pos_reg_thresh", pos_reg_thresh_);
  ros::param::getCached("/asap/primary/vel_reg_thresh", vel_reg_thresh_);
  ros::param::getCached("/asap/primary/att_reg_thresh", att_reg_thresh_);
  ros::param::getCached("/asap/primary/omega_reg_thresh", omega_reg_thresh_);
}
