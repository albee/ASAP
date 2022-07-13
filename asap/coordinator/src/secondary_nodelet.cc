/* primary_nodelet.cc

The primary coordinator, which derives from CoorindatorBase and adds methods for DMPC, RATTLE, and assembly tests.
*/

#include "coordinator/secondary_nodelet.h"

/* ************************************************************************** */
void SecondaryNodelet::Initialize(ros::NodeHandle* nh) {
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
  pub_status_ = nh->advertise<coordinator::StatusSecondary>(TOPIC_ASAP_STATUS, 5, true);
  
  // subscribers
  sub_flight_mode_= nh->subscribe<ff_msgs::FlightMode>(TOPIC_MOBILITY_FLIGHT_MODE, 5,
    boost::bind(&SecondaryNodelet::flight_mode_callback, this, _1));  // flight mode setter
  sub_ekf_ = nh->subscribe<ff_msgs::EkfState>("gnc/ekf", 5,
    boost::bind(&SecondaryNodelet::ekf_callback, this, _1));
  sub_test_number_ = nh->subscribe<coordinator::TestNumber>(TOPIC_ASAP_TEST_NUMBER, 5,
    boost::bind(&SecondaryNodelet::test_num_callback, this, _1));

  // services
  serv_ctl_enable_ = nh->serviceClient<std_srvs::SetBool>(SERVICE_GNC_CTL_ENABLE);
  
  // Pass control to Run method (activate timers and spin)
  NODELET_INFO_STREAM("[SECONDARY COORD] Initialized. Passing control to Run method...");
  thread_.reset(new std::thread(&CoordinatorBase::Run, this, nh));
}

/* ************************************************************************** */
void SecondaryNodelet::get_status_msg(coordinator::StatusSecondary& msg){
  /**
   * @brief Fills the StatusSecondary message with the state of the private
   * variables. Published by a coordinator Timer.
   * Inputs: base_status_ and secondary_status_
   * 
   */
  msg.stamp = ros::Time::now();
  msg.test_number = base_status_.test_number;
  msg.default_control = base_status_.default_control;
  msg.flight_mode = base_status_.flight_mode;
  msg.test_finished = base_status_.test_finished;
  msg.coord_ok = base_status_.coord_ok;
  msg.test_finished = base_status_.test_finished;
}

/* ************************************************************************** */
void SecondaryNodelet::load_params(){
  // Get sim and ground flags
  std::string sim_str, ground_str;
  ros::param::get("/asap/sim", sim_str);
  sim_ = !std::strcmp(sim_str.c_str(), "true"); // convert to bool
  ros::param::get("/asap/ground", ground_str);
  ground_ = !std::strcmp(ground_str.c_str(), "true");  // convert to bool
  
  NODELET_INFO_STREAM("[SECONDARY COORD] Parameters Loaded...");
}

/************************************************************************/
void SecondaryNodelet::RunTest0(ros::NodeHandle *nh){
    NODELET_INFO_STREAM("[SECONDARY]: Congratulations, you have passed quick checkout. " 
    "May your days be blessed with only warnings and no errors.");

    disable_default_ctl();
    ros::Duration(5.0).sleep();

    NODELET_DEBUG_STREAM("[SECONDARY COORD]: ...test complete!");
    base_status_.test_finished = true;
};
