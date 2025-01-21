%% Kinematic Model Class - GRAAL Lab
classdef cartesianControl < handle
    % KinematicModel contains an object of class GeometricModel
    % gm is a geometric model (see class geometricModel.m)
    properties
        gm % An instance of GeometricModel
        k_a
        k_l
    end

    methods
        % Constructor to initialize the geomModel property
        function self = cartesianControl(gm,angular_gain,linear_gain)
            if nargin > 2
                self.gm = gm;
                self.k_a = angular_gain;
                self.k_l = linear_gain;
            else
                error('Not enough input arguments (cartesianControl)')
            end
        end
        function [x_dot]=getCartesianReference(self,bTg)
            %% getCartesianReference function
            % Inputs :
            % bTg : goal frame
            % Outputs :
            % x_dot : cartesian reference for inverse kinematic control
            
            bTt = self.gm.getToolTransformWrtBase();
            % Compute orientation error
            bR_error = bTg(1:3, 1:3) * bTt(1:3, 1:3)';
            [h, theta] = RotToAngleAxis(bR_error);
            e_o = h * theta;

            % Compute position error
            e_p = bTg(1:3, 4) - bTt(1:3, 4);

            % Display Cartesian errors
            disp('Orientation Error (e_o):')
            disp(e_o)
            disp('Position Error (e_p):')
            disp(e_p)

            % Scale the errors with the respective gains
            angular_velocity = self.k_a * e_o;
            linear_velocity = self.k_l * e_p;

            % Combine linear and angular velocities into a single vector
            x_dot = [angular_velocity; linear_velocity];
        end
    end
end
