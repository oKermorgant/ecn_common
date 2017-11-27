# ecn_common
ROS package for ECN labs regrouping several tools.

## Token manager

This tool is designed to control single-access to a given ressource (here the Baxter robot during labs at ECN).
It is composed of:
* A main python executable (`rosrun ecn_common token_manager`) that should be running on a single computer and that advertises the `/token_manager/manager` service.
* Two `TokenHandle` classes (C++ and Python) that are wrappers for the service client. Instanciating these classes leads to a loop that checks whether the ressource is available and returns only when the user has access. In the main loop, calling tokenhandle.update() lets the manager knows that this user is still accessing the ressource.
* Two examples: C++ and Python with random group names.

## Color detector

This class allows versatile single-object color segmentation from RGB and tracking (x/y/area). Can display the segmented image to help threshold tuning and can be set to fit a circle.
