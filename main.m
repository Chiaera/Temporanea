%% Template Exam Modelling and Control of Manipulators
clc;
close all;
clear;
addpath('include'); % put relevant functions inside the /include folder 

%% Compute the geometric model for the given manipulator
iTj_0 = BuildTree();

disp('iTj_0')
disp(iTj_0);
jointType = [0 0 0 0 0 1 0]; % specify two possible link type: Rotational, Prismatic.
q = [pi/2, -pi/4, 0, -pi/4, 0, 0.15, pi/4]';

%% Define the tool frame rigidly attached to the end-effector
% Tool frame definition

eta_te = [pi/10 0 pi/6];
eRt = getYawPitchRollMatrix(eta_te); 
e_r_te =  [0.3; 0.1; 0];
eTt = [eRt, e_r_te;  0 0 0 1];

%% Initialize Geometric Model (GM) and Kinematic Model (KM)

% Initialize geometric model with q0
gm = geometricModel(iTj_0,jointType,eTt);

% Update direct geoemtry given q0
gm.updateDirectGeometry(q);

% Initialize the kinematic model given the goemetric model
km = kinematicModel(gm);

bTt = gm.getToolTransformWrtBase();

disp("eTt");
disp(eTt);
disp('bTt q = 0');
disp(bTt);

%% Define the goal frame and initialize cartesian control
% Goal definition 
bOg = [0.2; -0.7; 0.3];
eta_g = [0; 1.57; 0];
bRg = getYawPitchRollMatrix(eta_g);
bTg = [bRg bOg;0 0 0 1]; 
disp('bTg')
disp(bTg)

% control proportional gain 
k_a = 0.8; 
k_l = 0.8; 

% Cartesian control initialization
cc = cartesianControl(gm, k_a, k_l);

%% Initialize control loop 

% Simulation variables
samples = 100;
t_start = 0.0;
t_end = 10.0;
dt = (t_end-t_start)/samples;
t = t_start:dt:t_end; 

% preallocation variables
bTi = zeros(4, 4, gm.jointNumber);
bri = zeros(3, gm.jointNumber+1);

% joints upper and lower bounds 
qmin = -3.14 * ones(7,1);
qmin(6) = 0;
qmax = +3.14 * ones(7,1);
qmax(6) = 1;

show_simulation = true;
pm = plotManipulators(show_simulation);
pm.initMotionPlot(t, bTg(1:3,4));

k = 1; %counter

%%%%%%% Kinematic Simulation %%%%%%%
for i = t
    % Updating transformation matrices for the new configuration 
    gm.updateDirectGeometry(q);

    % Get the cartesian error given an input goal frame
    x_dot = cc.getCartesianReference(bTg); 

    % Update the jacobian matrix of the given model
    km.updateJacobian();
    J = km.J;

    %% INVERSE KINEMATICS
    % Compute desired joint velocities 
    q_dot = pinv(J) * x_dot; % used pseudo inverse to get the minimum norm vector

    desired_tool_vel(:,k) = x_dot;   % save desired tool velocity for print
    desired_qdot(:,k)     = q_dot;   % save desired q_dot for print


    % simulating the robot
    q = KinematicSimulation(q, q_dot, dt, qmin, qmax);
    
    pm.plotIter(gm, km, i, q_dot);

    if(norm(x_dot(1:3)) < 0.01 && norm(x_dot(4:6)) < 0.01)
        disp('Reached Requested Pose')
        break
    end
    %% Q2.5 – Compute EE and Tool velocities wrt base
    
    % End-effector Jacobian (last link)
    J_ee = km.getJacobianOfLinkWrtBase(gm.jointNumber);
    xdot_ee = J_ee * q_dot;   
    
    % Tool Jacobian 
    J_tool = km.getJacobianOfToolWrtBase();
    xdot_tool = J_tool * q_dot;
    
    % Extract linear and angular velocities
    lin_vel_ee = xdot_ee(4:6);
    ang_vel_ee   = xdot_ee(1:3);
    
    lin_vel_tool = xdot_tool(4:6);
    ang_vel_tool = xdot_tool(1:3);

    ee_matrix_lin_vel(:,k)   = lin_vel_ee;
    ee_matrix_ang_vel(:,k) = ang_vel_ee;
    tool_matrix_lin_vel(:,k) = lin_vel_tool;
    tool_matrix_ang_vel(:,k) = ang_vel_tool;

    k = k+1;

end

pm.plotFinalConfig(gm);

% Results section

last_column = k - 1; % to prevent errors 
if last_column < 1
    last_column = 1;
end

% print cartesian error
fprintf('\n Initial cartesian error\n');
fprintf('Angular error:\n');
fprintf('   %.8f\n', desired_tool_vel(1:3,1));
fprintf('Linear error:\n');
fprintf('   %.8f\n', desired_tool_vel(4:6,1));
fprintf('\n');
fprintf('\n final cartesian error\n');
fprintf('Angular error:\n');
fprintf('   %.8f\n', desired_tool_vel(1:3,last_column));
fprintf('Linear error:\n');
fprintf('   %.8f\n', desired_tool_vel(4:6,last_column));
fprintf('\n');

%print desired tool velocities
fprintf('\n Initial desired tool velocities\n');
fprintf('Angular vel:\n');
fprintf('   %.8f\n', desired_tool_vel(1:3,1));
fprintf('Linear vel:\n');
fprintf('   %.8f\n', desired_tool_vel(4:6,1));
fprintf('\n');
fprintf('\n Final desired tool velocities \n');
fprintf('Angular vel:\n');
fprintf('   %.8f\n', desired_tool_vel(1:3,last_column));
fprintf('Linear vel:\n');
fprintf('   %.8f\n', desired_tool_vel(4:6,last_column));
fprintf('\n');

% print desired joint velocities
fprintf('Initial joint velocities q_dot \n');
fprintf('   %.8f\n', desired_qdot(:,1));
fprintf('\n');
fprintf('\n Final joint velocities q_dot \n');
fprintf('   %.8f\n', desired_qdot(:,last_column));
fprintf('\n');

% print initial velocities
fprintf('Initial end effector linear vel:\n');
fprintf('   %.8f\n', ee_matrix_lin_vel(:,1));
fprintf('\n');
fprintf('Initial end effector angular vel:\n');
fprintf('   %.8f\n', ee_matrix_ang_vel(:,1));
fprintf('\n');
fprintf('Initial tool linear vel:\n');
fprintf('   %.8f\n',  tool_matrix_lin_vel(:,1));
fprintf('\n');
fprintf('Initial tool angular vel:\n');
fprintf('   %.8f\n',  tool_matrix_ang_vel(:,1) );
fprintf('\n');
% print final velocities
fprintf('Final end effector linear vel:\n');
fprintf('   %.8f\n', ee_matrix_lin_vel(:,last_column));
fprintf('\n');
fprintf('Final end effector angular vel:\n');
fprintf('   %.8f\n', ee_matrix_ang_vel(:,last_column));
fprintf('\n');
fprintf('Final tool linear vel:\n');
fprintf('   %.8f\n',  tool_matrix_lin_vel(:,last_column));
fprintf('\n');
fprintf('Final tool angular vel:\n');
fprintf('   %.8f\n',  tool_matrix_ang_vel(:,last_column) );

function [R]=getYawPitchRollMatrix(eta)
    
    psi = eta(1);
    theta = eta(2);
    phi = eta(3);
    
    R_yaw = [cos(phi) -sin(phi) 0;
          sin(phi)  cos(phi) 0;
          0         0        1];
    
    R_pitch = [cos(theta) 0 sin(theta);
          0          1 0;
         -sin(theta) 0 cos(theta)];
    
    R_roll = [1 0 0;
          0 cos(psi) -sin(psi);
          0 sin(psi)  cos(psi)];
    
    R = R_yaw * R_pitch * R_roll;
    
    % Check if eRt is a rotation matrix
    I = eye(3,3);
    
    if ((det(R) - 1 > 1e-10) || any(abs(R * R' - I) > 1e-10,"all"))
        error("R can't be a rotation matrix.")
    end
end

