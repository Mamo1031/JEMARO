addpath('include');

% TO DO: Test assignment 1 MCM 2024-2025

% 1.1 Angle-axis to rot
disp('ex 1.2:');
disp(AngleAxisToRot([1,0,0]',pi/2))
disp('ex 1.3:');
disp(AngleAxisToRot([0,0,1]',pi/3))
disp('ex 1.4:');
disp(AngleAxisToRot([-2/3,-1/3,2/3]',pi/2))


% 1.2 Rot to angle-axis
R1 = [1 0 0; 0 0 -1; 0 1 0];
R2 = [1 -sqrt(3)/2 0; sqrt(3)/2 0.5 0; 0 0 1];
R3 = eye(3);
R4 = [-1 0 0; 0 -1 0; 0 0 1];

rotation_matrices = {R1, R2, R3, R4};
exercise_labels = {'Q2.2', 'Q2.3', 'Q2.4', 'Q2.5'};

for i = 1:length(rotation_matrices)
    R = rotation_matrices{i};
    disp(['Results for ', exercise_labels{i}]);
    try
        [h, theta] = RotToAngleAxis(R);
        fprintf('Rotation Angle (theta): %.4f radians\n', theta);
        fprintf('Rotation Axis (h): [%.4f, %.4f, %.4f]\n', h(1), h(2), h(3));
    catch ME
        disp(['Error: ', ME.message]);
    end
    disp('---------------------------');
end


% 1.3 Euler to rot
disp('ex 3.2:');
disp(YPRToRot(0,0,pi/2))

disp('ex 3.3:');
disp(YPRToRot(pi/3,0,0))

disp('ex 3.4:');
disp(YPRToRot(pi/3,pi/2,pi/4))

disp('ex 3.5:');
disp(YPRToRot(0,pi/2,-pi/12))


% 1.4 Rot to Euler
R1 = [1 0 0; 0 0 -1; 0 1 0];
R2 = [1/2 -sqrt(3)/2 0; sqrt(3)/2 1/2 0; 0 0 1];
R3 = [0 -sqrt(2)/2 sqrt(2)/2; 0.5 sqrt(2)*sqrt(3)/4 sqrt(2)*sqrt(3)/4; -sqrt(3)/2 sqrt(2)/4 sqrt(2)/4];

rotation_matrices = {R1, R2, R3};
exercise_labels = {'Q4.2', 'Q4.3', 'Q4.4'};

for i = 1:length(rotation_matrices)
    R = rotation_matrices{i};
    disp(['Results for ', exercise_labels{i}]);
    try
        [psi, theta, phi] = RotToYPR(R);
        fprintf('Yaw (psi): %.4f radians\n', psi);
        fprintf('Pitch (theta): %.4f radians\n', theta);
        fprintf('Roll (phi): %.4f radians\n', phi);
    catch ME
        disp(['Error: ', ME.message]);
    end
    disp('---------------------------');
end
