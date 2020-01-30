# WPILibStateSpace

A modern controls library for FRC, now as a vendor dependency! This library is built on the Eigen (C++) and EJML (Java) matrix libraries, with the DARE solver from MIT's DRAKE project. Features include:
* Runtime gain calculation -- no more copy pasting from python!
* State Space's Linear Quadratic Regulator and Linear System, with functions to create models of:
  * Elevators
  * Flywheels
  * Arms
  * Anything identified with FRC Characterization
* Regular, Extended and (soon) Unscented Kalman Filters

As of right now this project is very much still in development. 
