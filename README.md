# ASAP (Astrobee Science Application Package)
This is the ASAP interface, a framework for running autonomy science tests on the Astrobee robotic free-flyers developed by NASA Ames. ASAP has on-orbit flight heritage over multiple test sessions on the International Space Station.

ASAP was developed by the MIT Space Systems Lab by Keenan Albee, with contributions from Charles Oestreich, Pedro Roque, and Monica Ekal. 

Latest tested compatibility: FSW 16.3


## What does ASAP do
ASAP does the following to make your life easier when running code on the Astrobee hardware:

- Manages node startup/shutdown *on top of existing Astrobee FSW*: for most uses, no modifiation of Astrobee's flight software is needed!
- Performs automatic recording and shutoff of data at test start/stop, with descriptive file naming
- Provides GDS (ground station GUI) and command line interaces to start/stop your tests with minimal typing
- Interfaces with Astrobee FSW to correctly transfer over impeller and vent control to guest science code
- Provides a "test-based" template for you to easily add in autonomy experiments as new "tests"
- Considers multi-agent commannding and hardware role-setting (like proper simulation namespacing)

In short, ASAP takes care of low-level hardware and FSW interfacing messiness that gets in the way of just developing algorithms and testing them. ASAP is the product of a series of experiments and lessons learned during MIT's ROAM and ReSWARM ISS testing campaigns and has flight heritage from four ISS test sessions at the time of writing.


## When should I use ASAP?
ASAP is great if you are incorporating your code as a ROS node(let) and want to use Python/C++ and standard roscpp and rospy interfacing. It's especially helpful if you're doing robot autonomy experimentation, like planning, control, and localization. It's potentially unnecessary if you are aiming to use higher-level functions provided by NASA Ames, like moving from waypoint-to-waypoint using a default planner. Ames has documented that interface and API, Astrobee Android, here.


## Installation
ASAP installs overtop an existing Astrobee workspace. First, download and install the latest Astrobee flight software [here](https://github.com/nasa/astrobee).

Next, you will place ASAP as an additional set of nodes alongside the Astrobee workspace. You can just directly copy---no need to stay up-to-date with the ASAP repo.

```
git clone https://github.com/albee/ASAP
cp ASAP/asap $ASTROBEE_WS_DIR
```

Thats's it! ASAP should compile its nodes normally like any other set of ROS packages when you build the Astrobee workspace:

`catkin build`


## Usage
ASAP runs alongside the standard Astrobee FSW. First, start up the Astrobee simulator:

`roslaunch astrobee sim.launch`

Now, bring up the `execute_asap` node that will run continuously throughout your testing:

`roslaunch execute_asap asap.launch`

Your nodes can be launched at any time by sending a trigger to `execute_asap`. In simulation, you can do this using:

`rosrun execute_asap pub_gds_topics.py --sim --iss 1`

This will start Test1, for simulation, in the ISS environment. In this case, a single dummy node, `my_node` has been created. When you're ready, run:

`rosrun execute_asap pub_gds_topics.py -1`

This will send the -1 kill command to stop the test. Your node will get roskill'ed and a bag will be produced.


## Explanation
ASAP relies on two main components: an `execute_asap` node that runs persistently during testing and a `coordinator_asap` node that handles test execution. Each robot will have its own copy of these ASAP nodes running during testing.

Your own code, structured as a node or nodelet, lives in additional ROS packages in the `asap` directory of the Astrobee flight software.


## Configuration and adding your own code
Now you can start to add your own code, usualy structured as a ROS node or nodelet. New code goes in the `asap/` folder and should be structured as a standard ROS [package](http://wiki.ros.org/Packages#:~:text=A%20ROS%20package%20is%20simply,and%20the%20unit%20of%20release.).


## Moving to Hardware
There are a few special considerations to make when moving to hardware. See the section below.


## More Documentation

["A Brief Guide to Astrobee's Flight Software"](https://github.com/albee/a-brief-guide-to-astrobee) is highly recommended reading if you are planning to use Astrobee for autonomy science tests. That guide covers many common questions like "how do I integrate my own controller?," "how does the Astrobee launchfile sequence work?," and "how do I cross-compile for hardware?".


