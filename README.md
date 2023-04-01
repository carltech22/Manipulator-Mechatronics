# Manipulator-Mechatronics
This repository showcases the statics, dynamics, and kinematics of a 2D robotic manipulator.
A two-link robot manipulator is modeled using a Proportional Derivative (PD) Controller for closed-loop operation of the arm. The PD gain parameters are hand-tuned to ensure optimal performance. 

This is the position of Joint 1 vs Time

![Q4_Theta1](https://user-images.githubusercontent.com/121901181/228431387-209df7f1-5b48-44cf-b15c-abb496460866.jpg)

This is the position of Joint 2 vs Time

![Q4_Theta2](https://user-images.githubusercontent.com/121901181/228431723-f153676e-acd9-4552-bc5e-1c42826e21b9.jpg)

This is the X-position of the robot's end-effector

![Q4_X(t)](https://user-images.githubusercontent.com/121901181/229015370-22af2d7b-42b6-4fab-b200-bbb16fcea3dc.jpg)

This is the Y-position of the robot's end-effector

![Q4_Y(t)](https://user-images.githubusercontent.com/121901181/229015377-0ca50144-8a05-4c01-8a1f-b7c7d1867fc4.jpg)

Below is animated simulation in Matlab of a 2-link robotic manipulator generated using the files provided above.

https://user-images.githubusercontent.com/121901181/228316299-16a42eea-277b-4b8b-8034-a350d61e669e.mp4

Below is an animation of the robotic arm maintaning a linear motion which is essential many industrial applicaitons
![Keep_Box_Steady](https://user-images.githubusercontent.com/121901181/229310656-9e99041b-a8ef-4a6d-bf52-8f7beecccbf4.gif)
