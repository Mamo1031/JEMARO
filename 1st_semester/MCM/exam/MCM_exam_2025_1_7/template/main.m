%% Template Exam Modelling and Control of Manipulators
clc;
close all;
clear all;
addpath('include'); % put relevant functions inside the /include folder 

%% Compute the geometric model for the given manipulator
iTj_0 = BuildTree();

disp('iTj_0')
disp(iTj_0);
jointType = [...]; % specify two possible link type: Rotational, Prismatic.
q0 = [0,0,0,0,0,0,0]'; 

%% Define the tool frame rigidly attached to the end-effector
% Tool frame definition

eTt = ...;
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
disp('bTt q = 0');
disp(bTt);


%% INVERSE KINEMATIC CONTROL
% Goal definition 
bOg = [...]; %(m) x-y-z
b_eta_g = [...]; %(rad) YAW-PITCH-ROLL
bVg = [...]; %(m/s) linear velocity of the goal

bTg = ...; 
disp('bTg')
disp(bTg)

% control proportional gain 
k_a = 0.4;
k_l = 0.4;

% Cartesian control initialization
cc = cartesianControl(gm,k_a,k_l);

% initial configuration 
q = [0, -pi/6, pi/3, 0, pi/2, pi/2, 0]';
gm.updateDirectGeometry(q)

%% Initialize control loop 

% Simulation variables
samples = 500;
t_start = 0.0;
t_end = 20.0;
dt = (t_end-t_start)/samples;
t = t_start:dt:t_end; 

% preallocation variables
bTi = zeros(4, 4, gm.jointNumber);
bri = zeros(3, gm.jointNumber+1);

% joints upper and lower bounds 
qmin = -3.14 * ones(7,1);
qmin(4) = 0;
qmax = +3.14 * ones(7,1);
qmax(4) = 1;

% list for data plot
x_dot_hist = [];
t_hist = [];
err_lin_hist = [];

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
    % Update the position of the goal fram

    % Update geometric and kinematic model and use the cartesian control ... to do

    %% INVERSE KINEMATIC
    % Compute desired joint velocities 
    q_dot = ...;

    % simulating the robot
    q = KinematicSimulation(q, q_dot, dt, qmin, qmax);
    
    %% Plot Script (do not change)
    % computing the actual velocity and saving the unitary direction for plot
    % do NOT change
    x_dot_actual = km.J*q_dot;
    

    x_dot_hist = [x_dot_hist; (x_dot_actual/norm(x_dot_actual))'];
    bTt = gm.getToolTransformWrtBase();
    err_linear = bTg(1:3,4) - bTt(1:3,4);
    err_lin_hist = [err_lin_hist; norm(err_linear)];
    t_hist = [t_hist; i];

    if (rem(i,0.1) == 0) % only every 0.1 sec
        
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
            plot3(bri(1,j), bri(2,j), bri(3,j),'bo')           
        end
        plot3(bri(1,gm.jointNumber+2),bri(2,gm.jointNumber+2),bri(3,gm.jointNumber+2),'go') 
    
        color = cmap(mod(cindex,csize)+1,:);
        cindex = cindex + 1;
    
        line(bri(1,:), bri(2,:), bri(3,:), 'LineWidth', 1.5, 'Color', color)

    end

    if show_simulation == true
        
        drawnow
    end    
    if(norm(x_dot(1:3)) < 0.01 && norm(x_dot(4:6)) < 0.0001)
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


figure
plot(t_hist,x_dot_hist)

figure;
title('Distance to Goal')
plot(t_hist, err_lin_hist,'LineWidth', 1.5)
legend('|error linear|')
xlabel('Time (s)')
ylabel('|error linear|')
