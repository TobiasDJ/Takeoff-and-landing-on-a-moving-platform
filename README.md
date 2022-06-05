# Takeoff and landing on a moving platform

This project revolves around expanding drone capabilities into challenging operations, by means of introducing a moveable platform for recharging purposes, which the UAVs can autonomously detect, track, and land on. This problem is made even more challenging, considering the aspect of difficult weather conditions, which often occur in real-world applications.

The project will require the development of a simulation-based system to demonstrate take-off and landing in challenging conditions. The platform’s position should be disturbed to simulate the surface of a ship or another moving platform. The drone itself is also to be disturbed in other ways to simulate challenging wind conditions.
The proposed solution will need to incorporate:
 1.	Simulate an UAV alongside its essential components and control. 
 2.	Simulate a physical platform and its movement. 
 3.	Initiate take off and maintain an altitude without adverse wind conditions.
 4.	Initiate take off and maintain altitude adverse wind conditions.
 5.	Detect a moving target.
 6.	Estimate the planned path and future positions for a moving target.
 7.	Align planned path to a UAV trajectory compliant with constraints for dynamics and time 
 8.	Land on a moving platform without adverse wind conditions.
 9.	Land on a moving platform with adverse wind conditions



The project consists of four packages:

 1. Simulation package
 2. Landing detector package
 3. Path estimator package
 4. Mission controller package 


![Demo](https://github.com/TobiasDJ/Takeoff-and-landing-on-a-moving-platform/blob/main/Demos/land_moving_platform_0.5ms.gif)

The solution can be tested following the environment setup described [PX4 ROS with Gazebo simulation](https://docs.px4.io/master/en/simulation/ros_interface.html) and [Custom models for PX4 SITL](https://discuss.px4.io/t/create-custom-model-for-sitl/6700/3)

The project utilizes the find_object package developed by introlab.

```
@misc{labbe11findobject,
   Author = {{Labb\'{e}, M.}},
   Howpublished = {\url{http://introlab.github.io/find-object}},
   Note = {accessed 29-05-2022},
   Title = {{Find-Object}},
   Year = 2011
}
```
