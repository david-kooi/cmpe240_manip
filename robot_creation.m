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
setFixedTransform(joint, trvec2tform([L1,0,0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
body.Mass = M2;
addBody(robot, body, 'link1');

body = robotics.RigidBody('tool');
joint = robotics.Joint('fix1','fixed');
setFixedTransform(joint, trvec2tform([L2, 0, 0]));
body.Joint = joint;
addBody(robot, body, 'link2');

showdetails(robot)
com = centerOfMass(robot);


%% Compuate Dynamics
% Let center of mass of each link be in middle


% State vector
th1_0     = pi/2;
th1_0_dt  = 0;  
th2_0     = pi/2;
th2_0_dt  = 0;

theta_0    = [th1_0;th2_0; th1_0_dt;th2_0_dt];
[x_0,y_0] = hand_forward_kin([th1_0;th2_0]);

t_0 = 0;
t_accum = []
theta_accum = [];
step_size = 0.01;
t_i = 0:0.005:0.01;
T = 2
t_total = 0:step_size:T;


%% Generate path

%% Scenerio 1
% Start (0,1) End: (1.29, 0.707)
th1_f = 3*pi/4;
th2_f = -3*pi/4;

%% Scenerio 2
% Start: (0,1) End: 
%th1_f = 0.26;
%th2_f = -0.26*2;

z0.x  = th1_0; z0.y = th2_0;
z0.xv = 0;     z0.yv = 0;
z0.xa = 0;     z0.ya = 0;

zf.x  = th1_f; zf.y = th2_f;
zf.xv = 0;     zf.yv = 0;
zf.xa = 0;     zf.ya = 0;

[th1_traj,th2_traj, arc_length] = gen_pp_traj(z0, zf, T, step_size)

global z_accum;
global tau_accum;
z_accum = [];
tau_accum = [];

for i = 0:step_size:T
    t = t_0 + t_i;
    
    th_ddt_ref = [th1_traj(i); th2_traj(i)];
    %th_ddt_ref = [0; 0];
    [t,theta] = ode45(@(t,theta)manipulator_dynamics(t,theta_0,th_ddt_ref,step_size), t, theta_0); 
    
    t_0 = t(end);
    theta_0 = theta(end,:).';
    
    t_accum = [t_accum; t];
    theta_accum = [theta_accum; theta];
end

last = theta(end,:);
theta_f = last(1:2)';

% figure
% ax = gca;
% ax.Projection = 'orthographic';
% axis([-2 2 -2 2])
% show(robot, [th1_0;th2_0]);
% hold on
% show(robot, theta_f);


%% Plot reference trajectory and real trajectory

subplot(3,2,1)
% %%Animate
ax = gca;
ax.Projection = 'orthographic';
%plot(points(:,1),points(:,2),'k')
axis([-0.5 3.5 -0.5 2])
hold on
fps = 15;
r = robotics.Rate(fps);
for i = 1:50:length(theta_accum)
    show(robot, theta_accum(i,1:2)','PreservePlot',true);
    drawnow
    hold on
    waitfor(r);
end
 title("Kinematic Motion");
 xlabel("X");
 ylabel("Y");

 
subplot(3,2,2);
time = 0:length(theta_accum(:,1))-1;
time = time * step_size/3;
plot(time, theta_accum(:,1));
hold on
plot(time, theta_accum(:,2));
hold on
title("Thetas");
xlabel("Time");
ylabel("Radians");
legend("Theta_1", "Theta_2");
axis([0 2 -3 3]);

subplot(3,2,3);
plot(z_accum(:,1), z_accum(:,2));
title("End Effector Position");
xlabel("X");
ylabel("Y");
axis([-0.5 3 0 2]);

subplot(3,2,4);
time = 0:length(tau_accum(1,:))-1;
time = time * step_size/60;
plot(time, tau_accum(1,:));
hold on;
plot(time, tau_accum(2,:));
hold on
title("Torque Applied");
xlabel("Time");
ylabel("Tau");
legend("Joint_1", "Joint_2");
axis([0 2 -30 10]);

subplot(3,2,[5,6]);
plot(t_total, th1_traj(t_total));
hold on
plot(t_total, th2_traj(t_total));
hold on
title("Acceleration Trajectories");
legend("Theta_1 ", "Theta_2")
xlabel("Time");
ylabel("rad/s^2");


function dydt = manipulator_dynamics(t, y, th_ddt_ref, dt)
    persistent J_inv_m1;
    persistent t_m1;
    persistent z_m1;
    if isempty(J_inv_m1)
        J_inv_m1 = zeros(2,2);
    end
    if isempty(t_m1)
        t_m1 = 0;
    end
    if isempty(z_m1)
        z_m1 = 0;
    end
    
    global z_accum;
    global tau_accum;
    
    global DR;
    global L1;
    global L2;
    global M1;
    global M2; 
    
    g  = -9.81;
    y1 = y(1:2); % Joint Position
    y2 = y(3:4); % Joint Velocity
    
    R1 = L1/2;
    R2 = L2/2;
    alpha = 1 + 1 + M1*R1^2 + M2*(L1^2 + R2^2);
    beta  = M2*L1*R2;
    delta = 1 + M2*R2^2;
    
    %% Joint Dynamics
    y1(1) = y1(1) - pi/2; %% Some sin error
    M = [alpha + 2*beta*cos(y1(2)), delta + beta*cos(y1(2));...
         delta + beta*cos(y1(2)), delta];
    C = [-beta*sin(y1(2))*y2(2), -beta*sin(y1(2))*(y2(1)+y2(2));...
         beta*sin(y1(2))*y2(1), 0];
    N = [0.5*L1*M1*g*cos(y1(1)) + M2*g*(L1*cos(y1(1))+0.5*L2*cos(y1(1)+y1(2)));...
         0.5*L2*M2*g*cos(y1(1) + y1(2))];
    %y1(1) = y1(1) + pi/2; %% Some sin error

    
    y1(1) = y1(1) + pi/2;
    [x,y] = hand_forward_kin(y1);
    z = [x,y];
    z_accum = [z_accum; z];
    
    tau = M*th_ddt_ref + C*y2 + N;
    tau_accum = [tau_accum, tau];
    
    %tau = [0;0];
    %% Return angular velocities
    dydt = [y2; M\(tau - C*y2 - N)];

end

function [x,y] = hand_forward_kin(theta)

global DR;
global L1;
global L2;

theta1 = theta(1);
theta2 = theta(2);

u_T_r   = [1, 0, DR; 0, 1, 0; 0, 0, 1];
r_T_j0  = [cos(theta1), -sin(theta1), 0; sin(theta1), cos(theta1), 0; 0, 0, 1]; 
j0_T_c1 = [1, 0, L1/2; 0, 1, 0; 0, 0, 1];
j0_T_j1 = [cos(theta2), -sin(theta2), L1; sin(theta2), cos(theta2), 0; 0, 0, 1]; 
j1_T_c2 = [1, 0, L2/2; 0, 1, 0; 0, 0, 1];
j1_T_h  = [1, 0, L2; 0, 1, 0; 0, 0, 1];

u_T_h  = u_T_r * r_T_j0 * j0_T_j1 * j1_T_h;


x = u_T_h(1,3);
y = u_T_h(2,3);




end

