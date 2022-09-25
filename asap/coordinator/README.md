# coordinator

High-level test logic for controlling ASAP experiments. Test calls are made
using `execute_asap`, and the test run is operated from coordinator.

`coordinator` nodelet : Handles operations between the planner, estimator, controller, etc. Versions exist for the primary and secondary Astrobees.

## Details

* A base class is defined in `coordinator_base.tpp`. `primary_nodelet.h` extends `CoordinatorBase` and implements virtual
test functions. 

* The resulting nodelet, instantiated by `primary_nodelet.cc` is the `primary_coordinator`, which takes in test numbers on `/asap/test_number`, and sends out
`/asap/status` msgs which command other nodes. 

Replace `primary` with `secondary` above for a second Astrobee. Note that the `nodelet_plugins.xml` file had to be defined for this nodelet.


## Adding Publishers, Subscribers, and Services to Coordinator

New publishers, subscribers, and services monitored/advertised by `primary_coordinator` are created in `primary_nodelet.cc`,

```C++
  sub_test_number_ = nh->subscribe<coordinator::TestNumber>(TOPIC_ASAP_TEST_NUMBER, 5,
    boost::bind(&PrimaryNodelet::test_num_callback, this, _1));
```

with declarations in `primary.h`. This essential subscriber gets the TestNumber from `execute_asap`, for example.


## Adding a New Test

1. Add declarations of any new tests to the headers,

* `coordinator.tpp`:
```C++
virtual void RunTest2(ros::NodeHandle *nh) {};
```

* `primary_nodelet.h`:
```C++
void RunTest2(ros::NodeHandle *nh) override;
```
.


2. Add an entry method for your test in `primary_nodelet.cc`, for example,

```C++
void PrimaryNodelet::RunTest2(ros::NodeHandle *nh){
    NODELET_INFO_STREAM("[PRIMARY_COORD]: Congratulations, you have made a test. ");
    ros::Duration(5.0).sleep();

    NODELET_DEBUG_STREAM("[PRIMARY COORD]: ...test complete!");
    base_reswarm_status_.test_finished = true;
};
```

(Optional) If you'd like to place your test numbers in specific header files instead:

Create a `primary_tests.hpp` file (or use an existing one). This will be the header-only definition of specific methods for your test number. Add this new .hpp to `primary_nodelet.cc`,

```C++
  #include "coordinator/primary_tests.hpp"
```
.


3. Finally, make sure your method actually gets called when its test number is received by adding it to 
the main `primary_coordinator` loop in `coordinator_base.tpp`, 

```C++
void CoordinatorBase<T>::Run(ros::NodeHandle *nh) {
  ...
  if (base_reswarm_status_.test_number == 2) {
    RunTest2(nh);
  }
```
.

4. (Optional) You may need to enable test number filtering for your test number from `execute_asap/scripts/execute_asap.py`. Verify the `test_num_okay()` function.


## Usage

See `execute_asap` README.md for usage instructions.