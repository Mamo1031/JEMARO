function R = AngleAxisToRot(h,theta)
% The fuction implement the Rodrigues Formula
% Input: 
% h is the axis of rotation
% theta is the angle of rotation (rad)
% Output:
% R rotation matrix
I = eye(3);
S_h = [0, -h(3),h(2);
    h(3),0,-h(1);
    -h(2), h(1), 0];

R = I + S_h*sin(theta)+S_h*S_h*(1-cos(theta));

end