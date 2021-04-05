# Object tracking using a MAV
## Description
This work involves the implementation of a receding horizon based trajectory planner for a drone. The ground robot is equipped with a April Tag which is used for the pose estimation. The pose observations are captured using the downward facing camera of the drone and a polynomial fitting is done to predict the target's trajectory. 
>
The target is always to be kept in the Field of View of the camera for continuous tracking hence the FOV constraints are to be included in the optimization problem. Also the sensor and actuator constraints of the drone are included. The nonlinear optimization is solved using the [NLopt](https://nlopt.readthedocs.io/en/latest/) package.


## Dependencies
- [NLopt](https://github.com/stevengj/nlopt)
- [jackal](https://www.clearpathrobotics.com/assets/guides/kinetic/jackal/simulation.html)
- [apriltag](https://github.com/versatran01/apriltag)
- [mrsl_quadrotor](https://github.com/KumarRobotics/mrsl_quadrotor)
- [qflight_descriptions](https://github.com/ATLFlight/qflight_descriptions)
- [waypoint_navigation_plugin](https://github.com/KumarRobotics/waypoint_navigation_plugin)
- [quadrotor_control](https://github.com/KumarRobotics/quadrotor_control)
- [multi_mav_manager](https://github.com/KumarRobotics/multi_mav_manager)

## Usage
```
$ roslaunch simulator drone_world.launch
```
## Gazebo View
*Click on to watch the video*
>
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/XNe5X765P6k/0.jpg)](https://www.youtube.com/watch?v=XNe5X765P6k)

## RViz View
*Click on to watch the video*
>
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/PovK8Gs4ods/0.jpg)](https://www.youtube.com/watch?v=PovK8Gs4ods)


## Reference
>> J. Thomas, J. Welde, G. Loianno, K. Daniilidis and V. Kumar, "Autonomous Flight for Detection, Localization, and Tracking of Moving Targets With a Small Quadrotor," in IEEE Robotics and Automation Letters, vol. 2, no. 3, pp. 1762-1769, July 2017, doi: 10.1109/LRA.2017.2702198.
>
>> [Link](https://www.researchgate.net/publication/316899622_Autonomous_Flight_for_Detection_Localization_and_Tracking_of_Moving_Targets_With_a_Small_Quadrotor)
