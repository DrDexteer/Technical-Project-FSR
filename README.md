# Technical Project - Field and Service Robotics

This repository contains the MATLAB/Simulink files developed for the Field and Service Robotics technical project.

The project focuses on trajectory tracking control of a standard quadrotor UAV. Three control architectures are implemented and compared:

* Geometric controller on (SO(3))
* Hierarchical controller based on RPY angles
* INDI-based attitude controller with geometric outer loop

A momentum-based external wrench estimator is also included to compensate external force and torque disturbances.

## Repository Contents

* `geometric_control_template.slx`
  Simulink model of the geometric controller.

* `hierarchical_controller.slx`
  Simulink model of the hierarchical controller.

* `indi_geometric_control_template.slx`
  Simulink model of the INDI-based attitude controller with geometric outer loop.

* `plot.m`
  MATLAB script used to generate and export the simulation plots.

## Project Features

The simulations include:

* nominal trajectory tracking tests;
* external force and torque disturbance tests;
* comparison with and without estimator-based compensation;
* inertia mismatch test between the geometric and INDI-based attitude controllers.

## Software

The project was developed using:

* MATLAB
* Simulink

## Author

Michele Vozza
Field and Service Robotics
Università degli Studi di Napoli Federico II
