# Induction Motor Sensored Control Using Integral-Proportional (IP) as Speed Controller & Rotor Field Oriented Control (RFOC) as Current Controller

In this project, I want to share to you guys, how I implemented integral-proportional (IP) as a speed controller and rotor field oriented control (RFOC) as current controller for induction motor. 

I use CMEX S-function in MATLAB as a tool for me to do the simulation. S-function is customizable block diagram in simulink. This special block can be created uses the C programming language which compiled into a MEX file using mex utility.

CMEX S-function can be used to describe dynamic systems using the C programming language. Using the C programming language, it will be easier for me when I try to implement the algorithm in inverter/microcontroller (check my other project when I implemented this algorithm using DRV8301 Texas Instrument Inverter & MyWay PE-Expert4 Inverter).

Basically, there are three simple basic structures in CMEX as illustrated below.
Structure A is used to process input data into output data without uses state during the calculation process. Structure B is used for discrete systems, and structure C is used for continuous systems. With the combination of structure A, B, and C, we can simulate the dynamic systems, such as motor control systems.

## Project Description
In this project, I used two controller, speed controller and current controller. Speed controller used to regulate the speed of the induction motor, and current controller used to regulate the current (in rotating reference frame/d-q frame) of the induction motor.

## Induction Motor Model
Here's the quick electrical and mechanical model of induction motor in stationary reference frame (alpha-beta).
![image](https://github.com/raihannagib/Induction-Motor-RFOC-Sensored/assets/102897878/14dd9bdc-fb57-421b-b7bf-00f3a1a93dbd)
![image](https://github.com/raihannagib/Induction-Motor-RFOC-Sensored/assets/102897878/baae731c-c22a-46af-a0d5-831bbcd3f17f)

## Integral-Proportional Algorithm
Below is block diagram of the IP controller.
![image](https://github.com/raihannagib/Induction-Motor-RFOC-Sensored/assets/102897878/6bfff749-dafc-4d7d-9fb8-d9a94ed1d870)

## Rotor Field Oriented Control Algorithm
The following is a simple block diagram of the RFOC controller.
![image](https://github.com/raihannagib/Induction-Motor-RFOC-Sensored/assets/102897878/8df33559-27d5-48f3-a4e6-a28b3eb37232)

## Clarke-Park Transform
To perform the algorithm, we need clarke-park transform. Clarke transform used to convert three-phase electrical signal (abc) in stationary reference frame into a two-phase electrical signal (alpha-beta) in stationary reference frame. And than, park transform used to convert two-phase electrical signal (alpha-beta) in stationary reference frame into a two-phase electrical signal (d-q) in rotating reference frame.

Here's the formula of clarke and park transform, including the inverse clarke and park transform.
![image](https://github.com/raihannagib/Induction-Motor-RFOC-Sensored/assets/102897878/ad0508c2-4a12-4f3c-a3f1-cf44f471b7cd)
![image](https://github.com/raihannagib/Induction-Motor-RFOC-Sensored/assets/102897878/e519dc75-a286-4f08-993c-2f896451048c)
![image](https://github.com/raihannagib/Induction-Motor-RFOC-Sensored/assets/102897878/b85c018a-6581-47e5-bf4b-0f7df38c1564)
![image](https://github.com/raihannagib/Induction-Motor-RFOC-Sensored/assets/102897878/fedc99e6-9687-4921-ba76-1ba90232766b)

## Block Diagram in Simulink
![image](https://github.com/raihannagib/Induction-Motor-RFOC-Sensored/assets/102897878/f7f67796-d8ba-4a60-baee-b2554475840f)

## Simulation Result
### Speed
![image](https://github.com/raihannagib/Induction-Motor-RFOC-Sensored/assets/102897878/34b8cf9d-5d18-4d1a-b5ba-69343944d838)
### Imr (Flux)
![image](https://github.com/raihannagib/Induction-Motor-RFOC-Sensored/assets/102897878/7c04aa9b-e546-4673-8765-f72326670271)

### Electrical Torque
![image](https://github.com/raihannagib/Induction-Motor-RFOC-Sensored/assets/102897878/debdc8a6-0d57-43db-aaff-1c93e7c8a8ae)
### Electrical Angle

### Isd
![image](https://github.com/raihannagib/Induction-Motor-RFOC-Sensored/assets/102897878/da37c9ed-93e3-43dc-9cab-a105b58ce74e)
### Isq
![image](https://github.com/raihannagib/Induction-Motor-RFOC-Sensored/assets/102897878/76c990e3-4b81-4452-8969-12509bd24903)

## Closing
 Feel free to reach out with any feedback or collaboration opportunities.
Finish!!! I'm glad you read my project till here. If you feel interested, want to ask more about my project, or want to give me any feedback or collaboration opportunities, feel free to contact me in raihannagib@gmail.com or https://linkedin.com/in/raihan-nagib. 

Thank you! 

## References
1. F. Yusivar and S. Wakao, “Minimum requirements of motor vector control modeling and simulation utilizing C MEX S-function in MATLAB/SIMULINK,” Proc. Int. Conf. Power Electron. Drive Syst., vol. 1, pp. 315–321, 2001, doi: 10.1109/peds.2001.975333.
