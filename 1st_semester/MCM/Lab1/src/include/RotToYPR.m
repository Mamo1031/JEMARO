function [psi,theta,phi] = RotToYPR(R)
% Given a rotation matrix the function outputs the relative euler angles
% usign the convention YPR
% Input:
% R rotation matrix
% Output:
% psi angle around z axis (yaw)
% theta angle around y axis (theta)
% phi angle around x axis (phi)
% SUGGESTED FUNCTIONS
    % atan2()
    % sqrt()
    
    % Check if R is a 3x3 matrix
    [rows, cols] = size(R);
    if rows ~= 3 || cols ~= 3
        error('Input must be a 3x3 matrix');
    end

    % Check the determinant (det(R) = +1)
    if abs(det(R) - 1) > 1e-6
        error('Input matrix must have a determinant of +1.');
    end
    
    % Check if R is orthogonal (R' * R = I)
    if norm(R' * R - eye(3), 'fro') > 1e-6
        error('Input matrix is not orthogonal (R'' * R must be identity).');
    end
    
    % Calculate the pitch angle theta
    theta = asin(-R(3,1));
    
    % Check for gimbal lock (singularity)
    if abs(cos(theta)) > 1e-6  % cos(theta) != 0
        % Standard calculation
        phi = atan2(R(3,2), R(3,3));  % Roll
        psi = atan2(R(2,1), R(1,1));  % Yaw
    else
        % Gimbal lock case: cos(theta) = 0
        warning('Gimbal lock detected. Setting phi = 0.');
        phi = 0;
        psi = atan2(-R(1,2), R(2,2));  % Yaw in gimbal lock
    end
end

