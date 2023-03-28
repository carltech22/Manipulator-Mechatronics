function dxdt = odeTwoLinkPlanarRobot(t,x,tau1,tau2)

% dxdt is 4X1 a vector consisting of the right hand side of the
% diferenetial equation to be solved; and 
% x=[theta1,theta2,dtheta1,dtheta2]' is the state vector
% To solve diff eqns in Matlab, you need to first convert your equations in
% into equivalent first order equations

% x = zeros(4,1);
theta1=x(1);theta2=x(2);
dtheta1=x(3);dtheta2=x(4);

% LOAD SYTEM PARAMETERS
paramTwoLinkPlanarRobot 

% SPECIFY INPUT TORQUES

% tau1=0;tau2=0;

Kp = 500;
Kd = 650;

tau1 = Kp*(theta1d-theta1)- Kd*(dtheta1);
tau2 = Kp*(theta2d-theta2)- Kd*(dtheta2);

% When doing closed-loop control, these zeros will change of PID control
% law -- look at the uploaded matlab folder on PID control

% DEFINE dxdt
% copy and past the expressions of E and MassMatrix you oobtain after
% running the file derive_twolink.m
%

MassMatrix = [[                           - m2*r2^2 - L1*m2*cos(theta2)*r2 - I2,                        - m2*r2^2 - I2]
[- m2*L1^2 - 2*m2*cos(theta2)*L1*r2 - m1*r1^2 - m2*r2^2 - I1 - I2, - m2*r2^2 - L1*m2*cos(theta2)*r2 - I2]];

E = [L1*m2*r2*sin(theta2)*dtheta1^2 - tau2 + g*m2*r2*cos(theta1 + theta2);
- L1*m2*r2*sin(theta2)*dtheta2^2 - 2*L1*dtheta1*m2*r2*sin(theta2)*dtheta2 - tau1 + g*m2*r2*cos(theta1 + theta2) + L1*g*m2*cos(theta1) + g*m1*r1*cos(theta1)];


% these will get evaluated numerically here because of lines 9 & 12
% compute ddTheta=[ddtheta1;ddtheta2] by ddTheta=MassMatrix\E
% Use this to compute dxdt which is = [dtheta1;dtheta2;ddtheta1;ddtheta2]
ddTheta = MassMatrix\E;
ddtheta1 = ddTheta(1);
ddtheta2 = ddTheta(2);
dxdt= [dtheta1,dtheta2,ddtheta1,ddtheta2]';

end