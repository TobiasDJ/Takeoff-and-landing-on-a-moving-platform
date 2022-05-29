# Take-off-and-landing-on-a-moving-platform

This project revolves around expanding drone capabilities into challenging operations. Quadrotor unmanned aerial vehicles (UAVs) have short flight times due to limited power source. To expand their capabilities, moving landing platforms/motherships can be introduced, housing, recharging, and deploying them as needed. A difficult problem made even more challenging, when considering the aspect of difficult weather conditions, which can often occur in real-world applications

The proposed solution is a autonomous control system, created and tested utilizing a Gazebo-based simulation, which includes the PX4 autopilot through its SITL. This allows the control off a computer modelled vehicle in a powerful 3D simulated world. Furthermore, the simulation will include a computer modelled moving platform.

Onboard the UAV a camera sensor is integrated to detect and track the moving platform. The platform is detected with feature-based object detection using KAZE features, an algorithm detecting and describing features in a nonlinear scale space by means of nonlinear diffusion filtering. 
Based on detected and tracked positions of the moving platform, predictions of future positions are calculated based on the formulas for constant movement and Kalman filtering. This is aligned with the drone specifications and current position, to land the UAV based on the tracking and estimations made. 

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
