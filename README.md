## README: Creating 2D Occupancy Grid Map using Overhead Infrastructure Cameras
# Table of Contents
1. Project Overview
2. Problem Statement
3. Concept & Motivation
4. Project Scope & Deliverables
5. Introduction to ROS & Gazebo
6. Setting up the ROS2 Environment
7. How to Acquire Data from Cameras
8. How to Evaluate the Generated Map
# Project Overview
This project demonstrates the creation of a 2D occupancy grid map using a network of overhead RGB cameras in an indoor environment. The aim is to map the environment in real-time for Autonomous Mobile Robots (AMRs) navigation, using simulation in ROS and Gazebo.

# Agenda
1. Problem, Concept, and Motivation
2. Project Scope and Expected Deliverables
3. Introduction to ROS & Gazebo
4. Setting up Simulation Environment
5. Acquiring Data from Simulated Cameras
6. Measuring Map Accuracy

# Problem Statement
Autonomous Mobile Robots (AMRs) typically use SLAM (Simultaneous Localization and Mapping) with on-board sensors to navigate environments. However, SLAM has limitations such as limited Field of View (FoV) and inability to track dynamic changes in real-time. This project aims to overcome these limitations by using overhead infrastructure cameras to map the environment.

# Concept & Motivation
Motivation:

+ Map entire environment in one shot
+ Track moving obstacles in real-time
+ Reduce AMR costs by eliminating expensive LiDAR or depth cameras
+ Provide non-line-of-sight data to AMRs
+ Enhance multi-robot path planning and coordination

Concept:

+ Use multiple RGB cameras arranged in a matrix to cover the environment
+ Fuse images from these cameras to create a composite 2D occupancy grid map
  
# Project Scope & Deliverables
Scope:

+ Add 4 RGB cameras in a Gazebo simulation environment in a 2x2 matrix at a height of ~8 meters.
+ Acquire images from the simulated cameras at 640x480 resolution.
+ Explore multi-camera calibration techniques.
+ Create a composite 2D occupancy grid map.
+ Benchmark map accuracy and computing latency.
  
Deliverables:

+ Detailed documentation of the solution, approach, and comparison with the state-of-the-art.
+ Fused map with accurate physical dimensions.
+ Error estimates and computing latency measurements.
+ Source code and algorithms.
+ Accuracy of the generated map.
+ Computational complexity (latency) of the algorithm.
+ Novelty, practicality, and efficiency of the solution.
  
# Introduction to ROS & Gazebo
# ROS (Robot Operating System):

Open-source framework for robot software development.
Uses a publisher/subscriber model for data exchange.
Extensive libraries and tools.

# Gazebo:

+ Simulation tool integrated with ROS.
+ Provides realistic physics, sensor models, and environment modeling.
+ Setting up the ROS2 Environment
+ Install Ubuntu 20.04 and ROS2 Foxy.
+ Set up the ROS2 workspace and install necessary packages.
+ Add overhead cameras in the simulation environment.
+ Source and build the workspace.
+ How to Acquire Data from Cameras
+ ROS operates on a publisher-subscriber model.
+ Use a Python script to access image data from camera topics.
+ Ensure to source the ROS2 setup script before running the Python script.
+ How to Evaluate the Generated Map
+ Use Rviz, a visualization tool for ROS, to evaluate the map.
+ Measure distances between key points on the map and compare with reference measurements.

