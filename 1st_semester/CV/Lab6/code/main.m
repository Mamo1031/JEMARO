clc;
close all;
clear;

%% Part1
% Load images
img1 = imread('../images/Mire/Mire1.pgm');
img2 = imread('../images/Mire/Mire2.pgm');

if size(img1, 1) ~= size(img2, 1)
    img2 = imresize(img2, [size(img1, 1), size(img2, 2)]);
end

% Load points (Nx2 matrices)
P1 = load('../images/Mire/Mire1.points'); % Nx2
P2 = load('../images/Mire/Mire2.points'); % Nx2

% Convert to homogeneous coordinates (3xN)
P1_homo = [P1'; ones(1, size(P1, 1))];  % 3xN
P2_homo = [P2'; ones(1, size(P2, 1))];  % 3xN

% Unnormalized 8-point algorithm
F1 = EightPointsAlgorithm(P1_homo, P2_homo);

% check the matrix and rank
disp('Fundamental Matrix F1 (Unnormalized):');
disp(F1);
disp('Rank of the Fundamental Matrix F1:');
disp(rank(F1));  % mist be 2

% Nnormalized 8-point algorithm
F2 = EightPointsAlgorithmN(P1_homo, P2_homo); 

% check the matrix and rank
disp('Fundamental Matrix F2 (Normalized):');
disp(F2);
disp('Rank of the Fundamental Matrix F2:');
disp(rank(F2));  % must be 2

% Epipolar constraint - F1
disp('Epipolar constraints for F1 (unnormalized):');
errors1 = zeros(1, size(P1_homo, 2));
for i = 1:size(P1_homo, 2)
    constraint1 = P2_homo(:, i)' * F1 * P1_homo(:, i);
    fprintf('Point %d: Unnormalized %.6f\n', i, constraint1);
    errors1(i) = abs(constraint1);
end

% Epipolar constraint - F2
disp('Epipolar constraints for F2 (normalized):');
errors2 = zeros(1, size(P1_homo, 2)); 
for i = 1:size(P1_homo, 2)
    constraint2 = P2_homo(:, i)' * F2 * P1_homo(:, i);
    fprintf('Point %d: Normalized %.6f\n', i, constraint2);
    errors2(i) = abs(constraint2);
end

% Visualize epipolar lines - F1
visualizeEpipolarLines(img1, img2, F1, P1, P2, 1);
title('Epipolar Lines (Unnormalized)');

% mean epipolar error for unnormalized
mean_error1 = mean(errors1);
disp('Mean epipolar error for F1 (unnormalized):');
disp(mean_error1);

pause(1);

% Visualize epipolar lines - F2
visualizeEpipolarLines(img1, img2, F2, P1, P2, 2);  % Pass original points (Nx2 matrices)
title('Epipolar Lines (Normalized)');

% mean epipolar error for normalized
mean_error2 = mean(errors2);
disp('Mean epipolar error for F2 (normalized):');
disp(mean_error2);

pause(1);

%% Part 2 - Acquire and match your own images
rng(42);
I1 = imread('../images/Books/Books1.jpg');
I2 = imread('../images/Books/Books2.jpg');

% 2. Match features between images
[P1, P2] = matchFeaturesBetweenImages(I1, I2);

% 3. Estimate the fundamental matrix using RANSAC
threshold = 1e-3; % Threshold for inliers
[bestF, consensus, ~] = ransacF(P1, P2, threshold);

% 4. Evaluate the results
evaluateFundamentalMatrix(bestF, consensus(1:3, :), consensus(4:6, :), I1, I2);
