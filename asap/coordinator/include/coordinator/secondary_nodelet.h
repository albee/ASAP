#pragma once
#include "coordinator/coordinator.tpp"

// status struct for secondary Astrobee
struct secondary_status_struct {
  // add struct parameters here!
};


class SecondaryNodelet :  public CoordinatorBase<coordinator::StatusSecondary>, public ff_util::FreeFlyerNodelet {
 public:
  SecondaryNodelet(): ff_util::FreeFlyerNodelet(true) {}  // don't do anything ROS-related in the constructor! (call the Nodelet constructor)
  ~SecondaryNodelet() {};

 private:
  ros::Subscriber sub_status_;
  secondary_status_struct secondary_status_;

  // Parameters
  std::string controller_ = "default";  // controller to send commands to
  std::string flight_mode_check_;

  // Status parameters  
  bool ground_ = false;  // whether or not this is a ground test
  bool sim_ = false;
  bool coord_ok_ = true;
  bool regulate_finished_ = false;

  void get_status_msg(coordinator::StatusSecondary& msg) override;

  void Initialize(ros::NodeHandle* nh);
  void load_params();
  void Run(ros::NodeHandle *nh);

  // quick checkout
  void RunTest0(ros::NodeHandle *nh) override;

  // Test list (overrides coordinator empty tests---add as desired!)
};

PLUGINLIB_EXPORT_CLASS(SecondaryNodelet, nodelet::Nodelet); // this is a nodelet!
