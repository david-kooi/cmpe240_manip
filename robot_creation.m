clc
clear all
close all

% Robot Parameters
global DR;
global L1;
global L2;
global M1;
global M2; 

DR = 1;
L1 = 1;
L2 = 1;
M1 = 1;
M2 = 1;

robot = robotics.RigidBodyTree('DataFormat','column','MaxNumBodies',3);
robot.Gravity = [0 0 -9.81];

body = robotics.RigidBody('link1');
joint = robotics.Joint('joint1', 'revolute');
setFixedTransform(joint,trvec2tform([DR 0 0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
body.Mass = M1;
addBody(robot, body, 'base');

body = robotics.RigidBody('link2');
joint = robotics.Joint('joint2','revolute');
setFixedTransform(joint, trvec2tform([0,L1,0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
body.Mass = M2;
addBody(robot, body, 'link1');

body = robotics.RigidBody('tool');
joint = robotics.Joint('fix1','fixed');
setFixedTransform(joint, trvec2tform([0, L2, 0]));
body.Joint = joint;
addBody(robot, body, 'link2');

showdetails(robot)
com = centerOfMass(robot);


%% Compuate Dynamics
% Let center of mass of each link be in middle


% State vector
th1_0     = 0;
th1_0_dt  = 0;  
th2_0     = pi/2;
th2_0_dt  = 0;

theta_0    = [th1_0;th2_0];
theta_0_dt = [th1_0_dt;th2_0_dt];
y_0 = [theta_0; theta_0_dt];

[t,y] = ode45(@manipulator_dynamics, [0,5], y_0); 
last = y(end,:);
theta_f = last(1:2)';

figure
show(robot, theta_0);
hold on
show(robot, theta_f);
hold on


%plot(com(1),com(2),'or');
view(2)
ax = gca;
ax.Projection = 'orthographic';
hold on
%axis([-0.1 0.7 -0.3 0.5])


function dydt = manipulator_dynamics(t, y)

    global DR;
    global L1;
    global L2;
    global M1;
    global M2; 

    y1 = y(1:2); % Joint Position
    y2 = y(3:4); % Joint Velocity
    tau = [-0.1;0];

    R1 = L1/2;
    R2 = L2/2;
    alpha = 1 + 1 + M1*R1^2 + M2*(L1^2 + R2^2);
    beta  = M2*L1*R2;
    delta = 1 + M2*R2^2;
    
    M = [alpha + 2*beta*cos(y1(2)), delta + beta*cos(y1(2));...
         delta + beta*cos(y1(2)), delta];
    C = [-beta*sin(y1(2))*y2(2), -beta*sin(y1(2))*(y2(1)+y2(2));...
         beta*sin(y1(2))*y2(1), 0];
 
    dydt = [y2; M\(tau - C*y2);];

end

