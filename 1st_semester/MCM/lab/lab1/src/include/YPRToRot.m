function [R] = YPRToRot(psi, theta, phi)
% The function compute the rotation matrix using the YPR (yaw-pitch-roll)
% convention, given psi, theta, phi.
% Input:
% psi angle around z axis (yaw)
% theta angle around y axis (theta)
% phi angle around x axis (phi)
% Output:
% R rotation matrix

% rotate around the x axis (yaw)
R_z = [cos(psi),-sin(psi),0;
    sin(psi), cos(psi),0;
    0,0,1];

% rotate around the x axis (theta)
R_y = [cos(theta),0,sin(theta);
    0,1,0;
    -sin(theta),0,cos(theta)];

% rotate around the x axis (phi)
R_x = [1,0,0;
    0,cos(phi),-sin(phi);
    0,sin(phi),cos(phi)];

% Output the rotation matrix
R = R_z*R_y*R_x;

end
