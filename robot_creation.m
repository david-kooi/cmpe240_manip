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
t_i = 0:step_size:0.09;
T = 2
t_total = 0:step_size:T;

%% Generate path
x_f = 1;
y_f = 2;

z0.x  = x_0; z0.y = y_0;
z0.xv = 0;   z0.yv = 0;
z0.xa = 0;   z0.ya = 0;

zf.x  = x_f; zf.y = y_f;
zf.xv = 0;   zf.yv = 0;
zf.xa = 0;   zf.ya = 0;


[x_traj,y_traj, arc_length] = gen_pp_traj(z0, zf, T, step_size)

global z_accum;
global tau_accum;
z_accum = [];
tau_accum = [];
for i = 1:step_size:T
    t = t_0 + t_i;
    
    x_traj_a = x_traj(i);
    y_traj_a = y_traj(i);
    z_ddt_ref = [x_traj_a; y_traj_a];
    %z_ddt_ref = [1; 0];
    [t,theta] = ode45(@(t,theta)manipulator_dynamics(t,theta,z_ddt_ref,step_size), t, theta_0); 
    
    t_0 = t(end);
    theta_0 = theta(end,:);
    
    t_accum = [t_accum; t];
    theta_accum = [theta_accum; theta];
end

last = theta(end,:);
theta_f = last(1:2)';

figure
show(robot, [th1_0;th2_0]);
hold on
%hold on
show(robot, theta_f);
hold on
ax = gca;
ax.Projection = 'orthographic';
%plot(points(:,1),points(:,2),'k')
axis([-2 2 -2 2])
hold on

%% Plot reference trajectory and real trajectory
figure
plot(t_total, x_traj(t_total));
hold on

figure
plot(z_accum(1,:), z_accum(2,:));

figure
plot(tau_accum(:,1));
hold on;
plot(tau_accum(:,2));
hold on

% Animate
% figure
% ax = gca;
% ax.Projection = 'orthographic';
% %plot(points(:,1),points(:,2),'k')
% axis([-2 2 -2 2])
% hold on
% fps = 10;
% r = robotics.Rate(fps);
% for i = 1:length(theta_accum)
%     show(robot, theta_accum(i,1:2)','PreservePlot',false);
%     drawnow
%     waitfor(r);
% end



function dydt = manipulator_dynamics(t, y, z_ddt_ref, dt)
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
    %tau = [0;0];

    %y1(1) = y1(1) - pi/2; %% Some sin error
    
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
    y1(1) = y1(1) + pi/2; %% Some sin error

     
     
    %% Jacobian
    J1 = -L1*sin(y1(1)) - L2*sin(y1(1))*cos(y1(2)) - L2*sin(y1(2))*cos(y1(1));
    J2 = -L2*sin(y1(1))*cos(y1(2)) - L2*sin(y1(2))*cos(y1(1));
    J3 = L1*cos(y1(1)) - L2*sin(y1(1))*sin(y1(2)) + L2*cos(y1(1))*cos(y1(2));
    J4 = -L2*sin(y1(1))*sin(y1(2)) + L2*cos(y1(1))*cos(y1(2));
    J = [J1, J2; J3, J4];
    
    J_inv = inv(J);
    J_inv_T = J_inv.';

    
    %% Compute velocities
    J_inv_dt = dt*(J_inv - J_inv_m1);
    J_inv_m1 = J_inv;
    
    [x,y]    = hand_forward_kin([y1(1), y1(2)]);
    z = [x;y];
    z_accum = [z_accum, z];
    z_dt = dt*(z - z_m1);
    z_m1 = z;
    
    %% Workspace Dynamics
    Mt = J_inv_T*M*J_inv;
    Ct = J_inv_T*(C*J_inv + M*J_inv_dt);
    Nt = J_inv_T*N;
    
    %% Compute torque to follow the reference
    F  = Mt*z_ddt_ref + Ct*z_dt + Nt;
    tau = J.'*F;
 
    tau_accum = [tau_accum; tau.'];
    
    %% Return angular velocities
    dydt = [y2; M\(tau - C*y2 - N);];

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

