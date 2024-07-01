# Awesome-legged-locomotion  <!-- omit in toc -->
[![Awesome](https://awesome.re/badge.svg)](https://awesome.re)

This is a curated list of awesome resources related to legged locomotion. It also includes helpful papers for those who are just starting to learn about the legged system. List will be updated gradually.

## Table of Content<!-- omit in toc -->
- [Courses](#courses)
- [Books \& Lectures](#books--lectures)
- [Simulation Tools](#simulation-tools)
- [Libraries](#libraries)
- [Papers](#papers)


## Courses
- [Modern Robotics: Mechanics, Planning, and Control Specialization](https://www.coursera.org/specializations/modernrobotics?) **Coursera**
- [Robotics Specialization by GRASP Lab](https://www.coursera.org/specializations/robotics) **Coursera**
- [Robotics Software Engineer](https://www.udacity.com/course/robotics-software-engineer--nd209) **Udacity**


## Books & Lectures
- [Underactuated Robotics](https://underactuated.mit.edu) **MIT, Russ Tedrake**
- [Robotic Manipulation](https://manipulation.mit.edu) **MIT, Russ Tedrake**
- [Robotic Systems (draft)](https://motion.cs.illinois.edu/RoboticSystems/) **UIUC, Kris Hauser**
- [Robot Dynamics](https://ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/documents/RobotDynamics2017/RD_HS2017script.pdf) [ [Lecture](https://www.youtube.com/playlist?list=PLE-BQwvVGf8GMY11YEONylkQNu0UWj0VE) ] **ETH** 
- [Legged Robotics](https://pab47.github.io/legs.html) **UIC**
- [Feedback Control of Dynamic Bipedal Robot Locomotion]() **Jessy Grizzle**
- [Stéphane Caron Blog](https://scaron.info) **Scaron Blog**


## Simulation Tools
- [MuJoCo](https://github.com/deepmind/mujoco) Multi-Joint dynamics with Contact. A general purpose physics simulator.
- [Gazebo](https://gazebosim.org/home) Gazebo is a collection of open source software libraries designed to simplify development of high-performance applications.
- [raisimLib](https://github.com/raisimTech/raisimLib) is a physics engine for robotics and artificial intelligence research that provides efficient and accurate simulations for robotic systems.
- [CoppeliaSim](https://www.coppeliarobotics.com) CoppeliaSim is used for fast algorithm development, factory automation simulations, fast prototyping and verification, robotics related education.
- [Webots](https://cyberbotics.com) Webots provides a complete development environment to model, program and simulate robots, vehicles and mechanical systems.
<!-- - [Plotjuggler](https://plotjuggler.io) Fast, intuitive and extensible time series visualization tool. -->


## Libraries
- Quadratic Programming (QP) solver: [OSQP](https://github.com/google/osqp-cpp), [qpOASES](https://github.com/coin-or/qpOASES)
- Rigid Body Dynamics Library : [RBDL](https://github.com/rbdl/rbdl), [Pinocchio](https://github.com/stack-of-tasks/pinocchio)
- Robotics Library: [Modern Robotics](https://github.com/Le0nX/ModernRoboticsCpp), [drake](https://github.com/RobotLocomotion/drake)
- Others: [Plotjuggler](https://plotjuggler.io), [dm_control](https://github.com/deepmind/dm_control), [xpp](https://github.com/leggedrobotics/xpp), [towr](https://github.com/ethz-adrl/towr), [hpp-fcl](https://github.com/humanoid-path-planner/hpp-fcl)


## Papers

### State Estimation<!-- omit in toc -->
- [**2021 arXiv**] [Legged Robot State Estimation in Slippery Environments Using Invariant Extended Kalman Filter with Velocity Update](https://hybrid-robotics.berkeley.edu/publications/ICRA2021_InEKF-Cassie-Slip-Estimation.pdf)
- [**2020 IJRR**] [Contact-aided Invariant Extended Kalman Filtering for Legged Robot State Estimation](https://journals.sagepub.com/doi/full/10.1177/0278364919894385) [ [arXiv ver.](https://arxiv.org/pdf/1805.10410.pdf) ] **Cassie**
- [**2015 PhD thesis**] [State Estimation for Humanoid Robots](https://www.ri.cmu.edu/pub_files/2015/8/main.pdf) **CMU**
- [**2014 IROS**] [State Estimation for a Humanoid Robot](https://arxiv.org/abs/1402.5450)

### Whole body control<!-- omit in toc -->
* **Whole-body Impulse Control**
  - [**2020 IJRR**] [Dynamic Locomotion For Passive-Ankle Biped Robots And Humanoids Using Whole-Body Locomotion Control](https://journals.sagepub.com/doi/full/10.1177/0278364920918014)
  - [**2019 arXiv**] [Highly Dynamic Quadruped Locomotion via Whole-Body Impulse Control and Model Predictive Control](https://arxiv.org/abs/1909.06586)
* **Hierachical Optimization Approach**
  - [**2016 Humanoids**] [Perception-less Terrain Adaptation through Whole Body Control and Hierarchical Optimization](https://ieeexplore.ieee.org/abstract/document/7803330/)
  - [**2016 Auton Robot**] [Momentum Control with Hierarchical Inverse Dynamics on a Torque-Controlled Humanoid](https://link.springer.com/article/10.1007/s10514-015-9476-6)
  - [**2010 SIGGRAPH**] [Feature-Based Locomotion Controllers](https://dl.acm.org/doi/10.1145/1778765.1781157)
* **Weighted Approach**
  - [**2016 IJHR**] [Design of a Momentum-Based Control Framework and Application to the Humanoid Robot Atlas](https://www.researchgate.net/profile/Twan-Koolen/publication/280839675_Design_of_a_Momentum-Based_Control_Framework_and_Application_to_the_Humanoid_Robot_Atlas/links/59ea41d3aca272cddddb7ba3/Design-of-a-Momentum-Based-Control-Framework-and-Application-to-the-Humanoid-Robot-Atlas.pdf)
  - [**2010 IROS**] [Dynamic Balance Force Control for Compliant Humanoid Robots](https://ieeexplore.ieee.org/document/5648837)

### Locomotion Control<!-- omit in toc -->
* **Conventional ZMP-based Preivew Control & MPC**
	* [**2010 Adv. Robotics**] [Online Walking Motion Generation with Automatic Foot Step Placement](https://inria.hal.science/inria-00391408v1/document)
	* [**2006 Humanoids**] [Trajectory Free Linear Model Predictive Control for Stable Walking in the Presence of Strong Perturbations](https://ieeexplore.ieee.org/document/4115592)
	* [**2003 ICRA**] [Biped walking pattern generation by using preview control of zero-moment point](https://ieeexplore.ieee.org/document/1241826)
* **Capture Point / Divergent Component of Motion**
	* [**2020 T-RO**] [Capturability-based Pattern Generation for Walking with Variable Height](https://ieeexplore.ieee.org/document/8766870)
	* [**2018 Humanoids**] [DCM-based gait generation for walking on moving support surfaces](https://ieeexplore.ieee.org/document/8625006)
	* [**2015 T-RO**] [Three-dimensional bipedal walking control using Divergent Component of Motion](https://ieeexplore.ieee.org/document/7063218)
	* [**2012 IJRR**] [Capturability-based analysis and control of legged locomotion, Part 1: Theory and application to three simple gait models](https://journals.sagepub.com/doi/pdf/10.1177/0278364912452673?casa_token=tBjlmF4O-3gAAAAA:vjbn7YPsvdkgIDt7ro2nv5JXnNMo_1VMMeKKfEcmj5zvILR-DHCjbwzcOUyBY11nYnMuh3un8sDl)
	* [**2011 IROS**] [Bipedal walking control based on Capture Point dynamics](https://ieeexplore.ieee.org/document/6094435)
	* [**2006 Humanoids**] [Capture Point: A Step toward Humanoid Push Recovery](https://ieeexplore.ieee.org/document/4115602?arnumber=4115602)
* **Step Time Adjustment**
	* [**2021 RAL**] [Variable Horizon MPC with Swing Foot Dynamics for Bipedal Walking Control](https://arxiv.org/abs/2010.08198)
	* [**2020 T-RO**] [Walking Control Based on Step Timing Adaptation](https://arxiv.org/abs/1704.01271)
	* [**2016 Humanoids**] [Step Timing Adjustment: A Step toward Generating Robust Gaits](https://arxiv.org/abs/1610.02377)
* **Raibert Heuristic**
	* [**2018 IROS**] [MIT Cheetah 3: Design and Control of a Robust, Dynamic Quadruped Robot](https://ieeexplore.ieee.org/abstract/document/8593885)
	* [**1984 T-SMC**] [Hopping in Legged Systems-Modeling and Simulation for the Two-Dimensional One-Legged Case](https://ieeexplore.ieee.org/document/6313238)
* **Time Velocity Reversal**
	* [**2020 IJRR**] [Dynamic Locomotion For Passive-Ankle Biped Robots And Humanoids Using Whole-Body Locomotion Control](https://arxiv.org/abs/1901.08100)
	* [**2014 ICARCV**] [Continuous Cyclic Stepping on 3D Point-Foot Biped Robots Via Constant Time to Velocity Reversal](https://ieeexplore.ieee.org/document/7064561)
* **Hybrid Zero Dynamics / Angular Momentum-based planning**
	* [**2022 ASME**] [Zero Dynamics, Pendulum Models, and Angular Momentum in Feedback Control of Bipedal Locomotion](https://arxiv.org/pdf/2105.08170.pdf)
	* [**2021 arXiv**] [Angular Momentum about the Contact Point for Control of Bipedal Locomotion: Validation in a LIP-based Controller](https://arxiv.org/pdf/2008.10763.pdf)
