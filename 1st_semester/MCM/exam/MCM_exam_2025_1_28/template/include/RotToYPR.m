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
anything = 0;

theta = atan2(-R(3,1),sqrt(R(1,1)^2+R(2,1)^2));
if abs(R(3,1)) > 1-1e-5
    phi = anything;
    if R(3,1) > 0
        psi = phi + atan2(R(1,2),R(1,3));
    else
        psi = - phi + atan2(-R(1,2),-R(1,3));
    end
else
    psi = atan2(R(2,1),R(1,1));
    phi = atan2(R(3,2),R(3,3));
end

end

