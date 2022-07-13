#pragma once
#include "coordinator/primary_nodelet.h"

/************************************************************************/
void PrimaryNodelet::RunTest0(ros::NodeHandle *nh){
    NODELET_INFO_STREAM("[PRIMARY_COORD]: Congratulations, you have passed quick checkout. " 
    "May your days be blessed with only warnings and no errors.");

    disable_default_ctl();
    ros::Duration(5.0).sleep();

    NODELET_DEBUG_STREAM("[PRIMARY COORD]: ...test complete!");
    base_status_.test_finished = true;
};


/************************************************************************/
void PrimaryNodelet::RunTest1(ros::NodeHandle *nh){
    /* RATTLE test: hand off control to RATTLE coordinator
    */
    primary_status_.control_mode = "regulate";
    ros::Duration(0.4).sleep(); // make sure controller gets the regulate settings before disabling default controller.

    NODELET_DEBUG_STREAM("[PRIMARY COORD]: Disabling default controller...");
    disable_default_ctl();

    // Additional test commands go here
    // Test commands can be anything you want! Talk to as many custom nodes as desired.

    NODELET_DEBUG_STREAM("[PRIMARY COORD]: ...test complete!");
    base_status_.test_finished = true;
}


/* ************************************************************************** */
void PrimaryNodelet::control_mode_callback(const std_msgs::String::ConstPtr msg) {
    /* Update control_mode form an external node.
    */
    primary_status_.control_mode = msg->data;
}
