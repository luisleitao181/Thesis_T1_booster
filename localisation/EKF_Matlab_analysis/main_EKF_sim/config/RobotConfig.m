%% RobotConfig.m - Physical and Sensor Parameters for Humanoid Robot
classdef RobotConfig
    properties
        % KINEMATIC PARAMETERS (meters)
        % Link lengths defining the robot's kinematic chain
        l1 = 0.35   % Lower leg: ankle to knee joint
        l2 = 0.35   % Upper leg: knee to hip joint
        l4 = 0.50   % Torso: waist (hip) to neck joint
        l6 = 0.15   % Head: neck to camera mounting point
        z_base = 0.10  % Base height

        % PROCESS NOISE
        % APM 
        %[X, Y, Theta]
        %Q = diag([0.9^2, 0.9^2, 0.6^2]); 
        Q = diag([0.03^2, 0.03^2, 0.02^2]);
        % Per-step noise standard deviations
        noise_xy = 0.03    
        noise_theta = 0.02   

        % MEASUREMENT NOISE MODEL
        % APM
        %sdv_dist_base = 0.05      % Base range error σ_d = base + linear * distance
        sdv_dist_base = 0.1
        %sdv_dist_linear = 0.02    % Distance-dependent
        sdv_dist_linear = 0.05    % Distance-dependent
        % Angular measurement noise constant
        % sdv_ang_h = 0.05           % Horizontal angle std dev rad
        % sdv_ang_v = 0.02           % Vertical angle std dev rad 
        sdv_ang_h = 0.1           % Horizontal angle std dev rad
        sdv_ang_v = 0.1           % Vertical angle std dev rad 


        % CAMERA PARAMETERS, not used
        % Field of View (FOV) - defines visible frustum
        FOV_h = 87 * pi/180   % Horizontal FOV (left-right) [rad]
        FOV_v = 58 * pi/180   % Vertical FOV (up-down) [rad]
        
        % Detection range limits
        max_range = 5.0       % Maximum feature detection distance [m]
        min_range = 0.3       % Minimum detection distance (too close) [m]
    
    end
    
    methods
        function sigma_range = get_range_noise(obj, distance)
            sigma_range = obj.sdv_dist_base + obj.sdv_dist_linear * distance;
        end
        
        function R = measurement_covariance(obj, distance)
            sigma_d = obj.get_range_noise(distance);
            R = diag([sigma_d^2, obj.sdv_ang_h^2, obj.sdv_ang_v^2]);
        end
    end
end