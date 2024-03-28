# Induction Motor Sensored Control Using Integral-Proportional (IP) as Speed Controller & Rotor Field Oriented Control (RFOC) as Current Controller

In this project, I want to share to you guys, how I implemented integral-proportional (IP) as a speed controller and rotor field oriented control (RFOC) as current controller for induction motor. 

I use CMEX S-function in MATLAB as a tool for me to do the simulation. S-function is customizable block diagram in simulink. This special block can be created uses the C programming language which compiled into a MEX file using mex utility.

CMEX S-function can be used to describe dynamic systems using the C programming language. Basically, there are three simple basic structures in CMEX as illustrated below.
* picture
Structure A is used to process input data into output data without uses state during the calculation process. Structure B is used for discrete systems, and structure C is used for continuous systems. With the combination of structure A, B, and C, we can simulate the dynamic systems, such as motor control systems.

## Project Description

In this project, I used two controller, speed controller and current controller. Speed controller used to regulate the speed of the induction motor, and current controller used to regulate the current of the induction motor in rotating reference frame.  

## Induction Motor Model

## Integral-Proportional Algorithm

## Rotor Field Oriented Control Algorithm

## Simulation Result

## References
