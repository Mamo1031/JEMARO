%% Template Exam Modelling and Control of Manipulators
clc;
close all;
clear all;
addpath('include'); % put relevant functions inside the /include folder 

%% Compute the geometric model for the given manipulator
iTj_0 = BuildTree();

disp('iTj_0')
disp(iTj_0);

jointType = []; % specify two possible link type: Rotational, Prismatic.
q0 = [0,0,0,0,0,0,0]'; 

%% Define the tool frame rigidly attached to the end-effector
% Tool frame definition
eO_t = ...;
eRt = ...;

eTt(1:3, 1:3) = eRt;
eTt(1:3, 4) = eO_t;

diep('eTt')
disp(eTt);

%% Initialize Geometric Model (GM) and Kinematic Model (KM)

% Initialize geometric model with q0
gm = geometricModel(iTj_0,jointType,eTt);


% Update direct geoemtry given q0
gm.updateDirectGeometry(q0)

% Initialize the kinematic model given the goemetric model
km = kinematicModel(gm);

bTt = gm.getToolTransformWrtBase();

disp("eTt");
disp(eTt);
disp('bTt q = q0');
disp(bTt);

% Plot initial configuration - DO NOT CHANGE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for j=1:gm.jointNumber
    bTi(:,:,j) = gm.getTransformWrtBase(j); 
end

bri(:,1) = [0; 0; 0];
% Plot joints
for j = 1:gm.jointNumber
    bri(:,j+1) = bTi(1:3,4,j);              
end
bTt = gm.getToolTransformWrtBase();
bri(:,gm.jointNumber+2) = bTt(1:3,4); 
figure
grid on 
hold on
title('CONFIGURATION for q=q0')
xlabel('x')
ylabel('y')
zlabel('z')
axis equal
az = 48;
el = 25;
view(az,el)
cindex = 1;
for j = 1:gm.jointNumber+2
    plot3(bri(1,j), bri(2,j), bri(3,j),'bo')
    
end
view(0,0)
plot3(bTg(1,4),bTg(2,4),bTg(3,4),'ro')
line(bri(1,:), bri(2,:), bri(3,:), 'LineWidth', 1.5)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% INVERSE KINEMATIC CONTROL (IKC)
%% Initialize data structures and classes

% initial configuration 
q = [0, pi/12, pi/3, pi/12, pi/12, pi/6, 0]';

% Update direct geoemtry
gm.updateDirectGeometry(q)
disp('qi = ')
disp(q)
disp('bTt if q = qi');
bTt = gm.getToolTransformWrtBase();
disp(bTt);

% Goal definition 
bOg = ...; %(m) x-y-z
bRg = ...;

bTg = [bRg bOg;0 0 0 1]; 
disp('bTg')
disp(bTg)

% control proportional gain 
k_a = 0.4;
k_l = 0.4;

% Cartesian control initialization
cc = cartesianControl(gm,k_a,k_l);

%% Initialize control loop 

% Simulation variables
samples = 1000;
t_start = 0.0;
t_end = 20.0;
dt = (t_end-t_start)/samples;
t = t_start:dt:t_end; 

% preallocation variables
bTi = zeros(4, 4, gm.jointNumber);
bri = zeros(3, gm.jointNumber+1);

% joints upper and lower bounds 
qmin = -3.14 * ones(7,1);
qmin(5) = 0;
qmax = +3.14 * ones(7,1);
qmax(5) = 1;

% list for data plot
x_dot_hist = [];
t_hist = [];
err_lin_hist = [];
err_ang_hist = [];
q_hist = [];

% Show simulation ? %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
show_simulation = true;

% init plot
figure
grid on 
hold on
title('MOTION OF THE MANIPULATOR')
xlabel('x')
ylabel('y')
zlabel('z')
axis equal
az = 48;
el = 25;
view(az,el)
cindex = 1;
csize = length(t);
cmap = colormap(parula(csize));
color = cmap(mod(cindex,csize)+1,:);
plot3(bTg(1,4),bTg(2,4),bTg(3,4),'ro')

%%%%%%% Kinematic Simulation %%%%%%%
for i = t
    % Updating transformation matrices for the new configuration 

    % Get the cartesian error given an input goal frame (pos and ori)

    % Update the jacobian matrix of the given model

    %% INVERSE KINEMATIC
    % Compute desired joint velocities 


    %% Plot Script (do not change) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % computing the actual velocity and save the unitary direction for plot
    x_dot_actual = km.J*q_dot;
    
    % simulating the robot
    q = KinematicSimulation(q, q_dot, dt, qmin, qmax);

    % Get bTt for plots
    bTt = gm.getToolTransformWrtBase();

    % Save data for plots
    % tool directions
    dir_lin = x_dot_actual(4:6)/norm(x_dot_actual(4:6));
    dir_ang = x_dot_actual(1:3)/norm(x_dot_actual(4:6));
    x_dot_hist = [x_dot_hist; dir_ang' dir_lin'];

    % linear error
    err_linear = bTg(1:3,4) - bTt(1:3,4);
    err_lin_hist = [err_lin_hist; norm(err_linear)];

    % angular error
    [psi_g,theta_g,phi_g] = RotToYPR(bTg(1:3,1:3));
    [psi_t,theta_t,phi_t] = RotToYPR(bTt(1:3,1:3));
    err_angular = [psi_g,theta_g,phi_g]'- [psi_t,theta_t,phi_t]';
    err_ang_hist = [err_ang_hist; norm(err_angular)];

    t_hist = [t_hist; i];
    q_hist = [q_hist; q'];

    if (rem(i,0.1) == 0) % only every 0.1 sec
        hold on
        
        %  plot moving goal
        plot3(bTg(1,4),bTg(2,4),bTg(3,4),'ro')
        for j=1:gm.jointNumber
            bTi(:,:,j) = gm.getTransformWrtBase(j); 
        end
       
        bri(:,1) = [0; 0; 0];
        % Plot joints
        for j = 1:gm.jointNumber
            bri(:,j+1) = bTi(1:3,4,j);              
        end
        bTt = gm.getToolTransformWrtBase();
        bri(:,gm.jointNumber+2) = bTt(1:3,4); 
  
        % Plot links
        for j = 1:gm.jointNumber+1
            plot_links = plot3(bri(1,j), bri(2,j), bri(3,j),'bo');          
        end
    
        plot_joints = plot3(bri(1,gm.jointNumber+2),bri(2,gm.jointNumber+2),bri(3,gm.jointNumber+2),'go'); 
    
        color = cmap(mod(cindex,csize)+1,:);
        cindex = cindex + 1;
    
        plot_lines = line(bri(1,:), bri(2,:), bri(3,:), 'LineWidth', 1.5, 'Color', color);
        
    end

    hold off
    
    if show_simulation == true
        
        drawnow
    end    

    if(norm(x_dot(1:3)) < 0.001 && norm(x_dot(4:6)) < 0.001)
        disp('Reached Requested Pose')
        break
    end
end

%% Plot the final configuration of the robot and the direction of the end-effector velocities
figure
grid on 
hold on
title('FINAL CONFIGURATION')
xlabel('x')
ylabel('y')
zlabel('z')
axis equal
az = 48;
el = 25;
view(az,el)
cindex = 1;
for j = 1:gm.jointNumber+2
    plot3(bri(1,j), bri(2,j), bri(3,j),'bo')
    
end

color = cmap(mod(cindex,csize)+1,:);
cindex = cindex + 1;

line(bri(1,:), bri(2,:), bri(3,:), 'LineWidth', 1.5, 'Color', color)

figure;
title('Distance to Goal')
plot(t_hist, err_lin_hist,'LineWidth', 1.5)
legend('|error linear|')
xlabel('Time (s)')
ylabel('|error linear|')

figure;
title('Misalignment')
plot(t_hist, err_ang_hist,'LineWidth', 1.5)
legend('|error angular|')
xlabel('Time (s)')
ylabel('|error angular|')


figure
hold on;
title('DIRECTION OF THE END-EFFECTOR VELOCITIES')
plot(t_hist, x_dot_hist)
legend('omega x', 'omega y', 'omega z', 'xdot', 'ydot', 'zdot')

figure
hold on;
title('q')
plot(t_hist, q_hist)
legend('q_1', 'q_2', 'q_3', 'q_4', 'q_5', 'q_6', 'q_7')
