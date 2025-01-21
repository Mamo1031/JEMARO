%% Kinematic Model Class - GRAAL Lab
classdef kinematicModel < handle
    % KinematicModel contains an object of class GeometricModel
    % gm is a geometric model (see class geometricModel.m)
    properties
        gm % An instance of GeometricModel
        J % Jacobian
    end

    methods
        % Constructor to initialize the geomModel property
        function self = kinematicModel(gm)
            if nargin > 0
                self.gm = gm;
                self.J = zeros(6, self.gm.jointNumber);
            else
                error('Not enough input arguments (geometricModel)')
            end
        end
        function updateJacobian(self)
            %% Update Jacobian function
            % The function update:
            % - J: end-effector jacobian matrix

            % TO DO
            % Get the total number of joints
            numJoints = self.gm.jointNumber;
        
            % Initialize the Jacobian matrix
            self.J = zeros(6, numJoints); % 6 rows (linear + angular), columns = joint number
        
            % Initialize the transformation matrix from the base to the current joint
            bTj = eye(4);
        
            % Extract the base-to-end-effector transformation matrix
            bTe = self.gm.getTransformWrtBase(numJoints);

            % Loop through each joint to calculate its contribution to the Jacobian
            for i = 1:numJoints
                % Get the transformation matrix from the base to the i-th joint
                bTj = self.gm.getTransformWrtBase(i);
        
                % Extract the rotation matrix from base to the i-th joint
                bRj = bTj(1:3, 1:3);
        
                % Extract the position vector of the i-th joint relative to the base
                bPj = bTj(1:3, 4);
        
                % Extract the end-effector position relative to the base
                bPe = bTe(1:3, 4);
        
                % Compute the z-axis of the current joint in the base frame
                zAxis = bRj(:, 3);
        
                % Check the type of the joint
                jointType = self.gm.jointType(i);
        
                if jointType == 0  % Rotational joint
                    % Compute the linear velocity contribution (cross product)
                    self.J(1:3, i) = zAxis;
        
                    % Compute the angular velocity contribution (z-axis of the joint)
                    self.J(4:6, i) = cross(zAxis, (bPe - bPj));
                elseif jointType == 1  % Prismatic joint
                    % Compute the linear velocity contribution (z-axis of the joint)
                    self.J(1:3, i) = [0; 0; 0];
        
                    % Prismatic joints do not contribute to angular velocity
                    self.J(4:6, i) = zAxis;
                else
                    error('Invalid joint type. Must be 0 (rotational) or 1 (prismatic).');
                end
            end
        end
    end
end

