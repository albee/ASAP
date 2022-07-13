#!/usr/bin/env python

"""
asap_config.py

Global variables for sim and hardware.

Keenan Albee and Charles Oestreich, 2021
MIT Space Systems Laboratory
"""
import rospy
import rospkg
from asap_helper import create_namespaced_topics


# Globals for sim and hardware.
rospack = rospkg.RosPack()
DATA_PATH = rospack.get_path("data")

# -------------------------------------------------------------------------------------------------------------
# directory for rosbags to go to in simulation (hardware uses Ames convention, located in /data/bags/...)
BAG_PATH_SIM = DATA_PATH + "/output/rosbags/"  # must end in /, dir must exist!

# rosbag name
ROSBAG_NAME = "asap_standard_rosbag"  # this gets overwritten to use Ames' standard naming convention!


# -------------------------------------------------------------------------------------------------------------
# Node launch files for ASAP nodes
ASAP_PRIMARY_LAUNCH_PATH = rospack.get_path("execute_asap") + "/launch/asap_primary.launch"
ASAP_SECONDARY_LAUNCH_PATH = rospack.get_path("execute_asap") + "/launch/asap_secondary.launch"


# -------------------------------------------------------------------------------------------------------------
# Node list for shutdown for ASAP nodes
NODE_LIST_SIM_PRIMARY = ["primary_coordinator", "my_node"]
NODE_LIST_HARDWARE_PRIMARY = ["primary_coordinator", "my_node"]
NODE_LIST_SIM_SECONDARY = ["secondary_coordinator", "my_node"]
NODE_LIST_HARDWARE_SECONDARY = ["secondary_coordinator", "my_node"]


# -------------------------------------------------------------------------------------------------------------
# Topics to record

# FSW topics
FSW_TOPICS = ["gnc/ctl/command", "gnc/ekf", "gnc/ctl/setpoint", "hw/pmc/command", "hw/imu", "mob/flight_mode", "mob/inertia"]
FSW_HW_TOPICS = create_namespaced_topics(FSW_TOPICS, sim=False)
FSW_SIM_TOPICS = create_namespaced_topics(FSW_TOPICS, sim=True, bee_name="queen")

# Sim only topics (ground truth)
SIM_ONLY = ["loc/truth/pose", "loc/truth/twist"]
SIM_ONLY_TOPICS = create_namespaced_topics(SIM_ONLY, sim=True, bee_name="queen")

# Localization topics
LOC_TOPICS = ["loc/ar/features", "loc/ml/features", "loc/of/features", "graph_loc/state", "sparse_mapping/pose"]
LOC_HW_TOPICS = create_namespaced_topics(LOC_TOPICS, sim=False)

# Sample topics---add your own!
SAMPLE_PRIMARY = ["rattle/test_instruct", "rattle/rrt_high_level/status", "rattle/nmpc_acado_planner/status",
                  "rattle/nmpc_ctl/status", "rattle/rrt/path/posearray", "rattle/rrt/path/twistarray", "rattle/local/path/posearray",
                  "rattle/local/path/twistarray", "rattle/local/path/wrencharray", "rattle/rrt/params", "rattle/obstacles",
                  "rattle/rrt/path/posearray", "rattle/rrt/path/twistarray", "rattle/local/info_plan_instruct", "rattle/local/path/posearray",
                  "rattle/local/path/twistarray", "rattle/local/path/wrencharray", "rattle/local/path_ctl/posearray", "rattle/local/path_ctl/twistarray",
                  "rattle/local/path_ctl/wrencharray", "rattle/local/info_plan_instruct_start", "rattle/local/psi", "rattle/local/weights"]
SAMPLE_HW_PRIMARY_TOPICS = create_namespaced_topics(SAMPLE_PRIMARY, sim=False)
SAMPLE_SIM_PRIMARY_TOPICS = create_namespaced_topics(SAMPLE_PRIMARY, sim=True, bee_name="queen")

# -------------------------------------------------------------------------------------------------------------
# All topics for recording
TOPICS_HARDWARE_PRIMARY = FSW_HW_TOPICS + LOC_HW_TOPICS + SAMPLE_HW_PRIMARY_TOPICS

TOPICS_SIM_PRIMARY = SIM_ONLY_TOPICS + FSW_SIM_TOPICS + SAMPLE_SIM_PRIMARY_TOPICS

TOPICS_HARDWARE_SECONDARY = FSW_HW_TOPICS + LOC_HW_TOPICS

TOPICS_SIM_SECONDARY = SIM_ONLY_TOPICS + FSW_SIM_TOPICS


if __name__ == "__main__":
    print(TOPICS_HARDWARE_PRIMARY)
