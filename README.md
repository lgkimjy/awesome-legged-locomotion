# Awesome-legged-locomotion  
[![Awesome](https://awesome.re/badge.svg)](https://awesome.re)

This is a curated list of awesome resources related to legged locomotion. It also includes helpful papers for those who are just starting to learn about the legged system. List will be updated gradually.

## Table of Contents
- [Awesome-legged-locomotion](#awesome-legged-locomotion)
	- [Table of Contents](#table-of-contents)
	- [Courses](#courses)
	- [Books & Lectures](#books--lectures)
	- [Tools](#tools)
	- [Libraries](#libraries)
	- [Papers](#papers)


## Courses
- [Modern Robotics: Mechanics, Planning, and Control Specialization](https://www.coursera.org/specializations/modernrobotics?) **Coursera**
- [Robotics Specialization by GRASP Lab](https://www.coursera.org/specializations/robotics) **Coursera**
- [Robotics Software Engineer](https://www.udacity.com/course/robotics-software-engineer--nd209) **Udacity**


## Books & Lectures
- [Underactuated Robotics](https://underactuated.mit.edu) **MIT**
- [Robotic Manipulation](https://manipulation.mit.edu) **MIT**
- [Robotic Systems (draft)](https://motion.cs.illinois.edu/RoboticSystems/) **UIUC**
- [Robot Dynamics](https://ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/documents/RobotDynamics2017/RD_HS2017script.pdf) [ [Lecture](https://www.youtube.com/playlist?list=PLE-BQwvVGf8GMY11YEONylkQNu0UWj0VE) ] **ETH** 
- [Legged Robotics](https://pab47.github.io/legs.html) **UIC**
- [Feedback Control of Dynamic Bipedal Robot Locomotion]() **Jessy Grizzle**
- [Stéphane Caron Blog](https://scaron.info) **Scaron Blog**


## Tools
- [Webots](https://cyberbotics.com) Webots provides a complete development environment to model, program and simulate robots, vehicles and mechanical systems.
- [MuJoCo](https://github.com/deepmind/mujoco) Multi-Joint dynamics with Contact. A general purpose physics simulator.
- [Gazebo](https://gazebosim.org/home) Gazebo is a collection of open source software libraries designed to simplify development of high-performance applications.
- [CoppeliaSim](https://www.coppeliarobotics.com) CoppeliaSim is used for fast algorithm development, factory automation simulations, fast prototyping and verification, robotics related education.
- [Plotjuggler](https://plotjuggler.io) Fast, intuitive and extensible time series visualization tool.


## Libraries
- [Pinocchio](https://github.com/stack-of-tasks/pinocchio)
- [RBDL](https://github.com/rbdl/rbdl)
- [OSQP](https://github.com/google/osqp-cpp)
- [Modern Robotics](https://github.com/Le0nX/ModernRoboticsCpp)
- [drake](https://github.com/RobotLocomotion/drake)
- [dm_control](https://github.com/deepmind/dm_control)
- [xpp](https://github.com/leggedrobotics/xpp)
- [towr](https://github.com/ethz-adrl/towr)


## Papers

### Whole body control
- [**Int. J. HR**] [Design of a Momentum-Based Control Framework and Application to the Humanoid Robot Atlas](https://www.researchgate.net/profile/Twan-Koolen/publication/280839675_Design_of_a_Momentum-Based_Control_Framework_and_Application_to_the_Humanoid_Robot_Atlas/links/59ea41d3aca272cddddb7ba3/Design-of-a-Momentum-Based-Control-Framework-and-Application-to-the-Humanoid-Robot-Atlas.pdf)
- [**IROS2010**] [Dynamic Balance Force Control for Compliant Humanoid Robots](https://ieeexplore.ieee.org/document/5648837)
- [**ijrr**] [Dynamic Locomotion For Passive-Ankle Biped Robots And Humanoids Using Whole-Body Locomotion Control](https://journals.sagepub.com/doi/full/10.1177/0278364920918014)

### State Estimation
- [State Estimation for a Humanoid Robot](https://arxiv.org/abs/1402.5450)
- [State Estimation for Humanoid Robots](https://www.ri.cmu.edu/pub_files/2015/8/main.pdf) **CMU**
- [Contact-aided Invariant Extended Kalman Filtering for Legged Robot State Estimation](https://arxiv.org/pdf/1805.10410.pdf) **Cassie**
- [Legged Robot State Estimation in Slippery Environments Using Invariant Extended Kalman Filter with Velocity Update](https://hybrid-robotics.berkeley.edu/publications/ICRA2021_InEKF-Cassie-Slip-Estimation.pdf)

### Humanoid Locomotion
* Preivew Control / MPC
	* [**ICRA2003**] [Biped walking pattern generation by using preview control of zero-moment point](https://ieeexplore.ieee.org/document/1241826)
	* [**Advanced Robotics**] [Online Walking Motion Generation with Automatic Foot Step Placement](https://inria.hal.science/inria-00391408v1/document)
* Capture Point / Divergent Component of Motion
	* [**Humanoids2006**] [Capture Point: A Step toward Humanoid Push Recovery](https://ieeexplore.ieee.org/document/4115602?arnumber=4115602)
	* [**ijrr**] [Capturability-based analysis and control of legged locomotion, Part 1: Theory and application to three simple gait models](https://journals.sagepub.com/doi/pdf/10.1177/0278364912452673?casa_token=tBjlmF4O-3gAAAAA:vjbn7YPsvdkgIDt7ro2nv5JXnNMo_1VMMeKKfEcmj5zvILR-DHCjbwzcOUyBY11nYnMuh3un8sDl)
	* [**IROS2011**] [Bipedal walking control based on Capture Point dynamics](https://ieeexplore.ieee.org/document/6094435)
	* [**T-Ro**] [Three-dimensional bipedal walking control using Divergent Component of Motion](https://ieeexplore.ieee.org/document/7063218)
	* [**Humanoids2018**] [DCM-based gait generation for walking on moving support surfaces](https://ieeexplore.ieee.org/document/8625006)
	* [**T-RO**] [Capturability-based Pattern Generation for Walking with Variable Height](https://ieeexplore.ieee.org/document/8766870)
* Raibert Heuristic
	* [**T-SMC**] [Hopping in Legged Systems-Modeling and Simulation for the Two-Dimensional One-Legged Case](https://ieeexplore.ieee.org/document/6313238)
	* [**IROS2018**] [MIT Cheetah 3: Design and Control of a Robust, Dynamic Quadruped Robot](https://ieeexplore.ieee.org/abstract/document/8593885)
* Time Velocity Reversal
	* [**ICARCV2014**] [Continuous Cyclic Stepping on 3D Point-Foot Biped Robots Via Constant Time to Velocity Reversal](https://ieeexplore.ieee.org/document/7064561)
	* [**ijrr**] [Dynamic Locomotion For Passive-Ankle Biped Robots And Humanoids Using Whole-Body Locomotion Control](https://arxiv.org/abs/1901.08100)
* Timing Adjustment
	* [Step Timing Adjustment: A Step toward Generating Robust Gaits](https://arxiv.org/abs/1610.02377)
	* [Walking Control Based on Step Timing Adaptation](https://arxiv.org/abs/1704.01271)
	* [Variable Horizon MPC with Swing Foot Dynamics for Bipedal Walking Control](https://arxiv.org/abs/2010.08198)
* Hybrid Zero Dynamics / Angular Momentum-based planning
	* [Angular Momentum about the Contact Point for Control of Bipedal Locomotion: Validation in a LIP-based Controller](https://arxiv.org/pdf/2008.10763.pdf)
	* [Zero Dynamics, Pendulum Models, and Angular Momentum in Feedback Control of Bipedal Locomotion](https://arxiv.org/pdf/2105.08170.pdf)
