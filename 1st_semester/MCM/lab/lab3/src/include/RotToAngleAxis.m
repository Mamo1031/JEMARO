function [h,theta] = RotToAngleAxis(R)
%EULER REPRESENTATION: Given a tensor rotation matrices this function
% should output the equivalent angle-axis representation values,
% respectively 'theta' (angle), 'h' (axis) 
% SUGGESTED FUNCTIONS
    % size()
    % eye()
    % abs()
    % det()
    % NB: Enter a square, 3x3 proper-orthogonal matrix to calculate its angle
    % and axis of rotation. Error messages must be displayed if the matrix
    % does not satisfy the rotation matrix criteria.

    % Check matrix R to see if its size is 3x3
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
    
    % Calculate the angle of rotation theta
    theta = acos((trace(R) - 1) / 2);

    % Calculate the axis of rotation h
    if abs(theta) < 1e-6
        h = [0; 0; 0];  % No rotation
    elseif abs(theta - pi) < 1e-6
        % Handle the 180-degree case separately
        h = sqrt((diag(R) + 1) / 2);
        h(1) = copysign(h(1), R(3,2) - R(2,3));
        h(2) = copysign(h(2), R(1,3) - R(3,1));
        h(3) = copysign(h(3), R(2,1) - R(1,2));
        h = h / norm(h);
    else
        % General case
        h = vex((R - R') / (2 * sin(theta)));
        h = h / norm(h);
    end
end

function s = copysign(a, b)
    if b >= 0
        s = abs(a);
    else
        s = -abs(a);
    end
end

 
function a = vex(S_a)
% input: skew matrix S_a (3x3)
% output: the original a vector (3x1)

    % Ensure the matrix is skew-symmetric
    if size(S_a, 1) ~= 3 || size(S_a, 2) ~= 3 || ~isequal(S_a, -S_a')
        error('Input matrix must be 3x3 and skew-symmetric');
    end

    % Extract the vector components
    a = [S_a(3, 2); S_a(1, 3); S_a(2, 1)];
end