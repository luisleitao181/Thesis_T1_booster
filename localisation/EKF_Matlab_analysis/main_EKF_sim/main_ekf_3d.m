clear; close all;

addpath('config', 'environment', 'utils', 'visualization', 'kinematics', 'robot');

%% Setup
N = 1000; 
dt = 0.040;
config = RobotConfig();

dh_model = HumanoidDH(config);

n_features = 4; %mudar o numero de features para a simulação

environment = EnvironmentBuilder.create_simple(n_features);

%% Generate control trajectory
[u_t, q_t] = generate_control_simple(N, dt);

%% Initialize states
%APM x = [0; 0; 45*pi/180];
x = [0; 0; 0];
%APM x_e = x + [0.5; 0.5; 10*pi/180];
x_e = x;

P = diag([0.05^2, 0.05^2, (2*pi/180)^2]);

%% Storage
x_t = zeros(3, N);
x_e_t = zeros(3, N);
P_history = cell(N, 1);
z_base = config.z_base;

%% EKF Loop
for i = 1:N
    x_t(:, i) = x;
    x_e_t(:, i) = x_e;
    
    v_cmd = u_t(1,i);
    vn_cmd = u_t(2,i); 
    omega_cmd = u_t(3,i);

    q_current = q_t(:, i);
    
    %% PREDICTION STEP
    %APM x = propagate_state_omni(x, v_cmd, vn_cmd, omega_cmd, dt, config, true);
    x = propagate_state_omni(x, v_cmd, vn_cmd, omega_cmd, dt, config, false);
    x_e = propagate_state_omni(x_e, v_cmd, vn_cmd, omega_cmd, dt, config, false);
    
    [F, W] = compute_jacobians_omni(x_e(3), v_cmd, vn_cmd, omega_cmd, dt);
    
    P = F * P * F' + W * config.Q * W';
    % APM 
    % P = (P + P') / 2;
    
    % Camera transforms
    x_4d = [x(1:2); z_base; x(3)];
    x_e_4d = [x_e(1:2); z_base; x_e(3)];
    T_cam_real = transform_camera_to_global(x_4d, dh_model, q_current);
    T_cam_est = transform_camera_to_global(x_e_4d, dh_model, q_current);
    
    cam_pos_real = T_cam_real(1:3, 4);
    R_cam_real = T_cam_real(1:3, 1:3);
    cam_pos_est = T_cam_est(1:3, 4);
    R_cam_est = T_cam_est(1:3, 1:3);
    
    %% UPDATE STEP - Sequential with validation
    for j = 1:environment.num_features
        feat_world = [environment.xp(j); environment.yp(j); environment.zp(j)];

        % Real measurement
        vec_world = feat_world - cam_pos_real;
        vec_cam = R_cam_real' * vec_world;
        dist = norm(vec_cam);
        az = atan2(vec_cam(2), vec_cam(1));
        el = atan2(vec_cam(3), sqrt(vec_cam(1)^2 + vec_cam(2)^2));
        
        sigma_d = config.sdv_dist_base + config.sdv_dist_linear*dist;
%         z = [dist + randn*sigma_d; 
%              NormalizeAng(az + randn*config.sdv_ang_h); 
%              el + randn*config.sdv_ang_v];
% APM
        z = [dist + randn*sigma_d; 
             NormalizeAng(az + randn*config.sdv_ang_h); 
             NormalizeAng(el + randn*config.sdv_ang_v)];
         
        % Predicted measurement
        vec_world_e = feat_world - cam_pos_est;
        vec_cam_e = R_cam_est' * vec_world_e;
        dist_e = norm(vec_cam_e);
        z_e = [dist_e; 
               atan2(vec_cam_e(2), vec_cam_e(1)); 
               atan2(vec_cam_e(3), sqrt(vec_cam_e(1)^2 + vec_cam_e(2)^2))];
           
        sigma_d_e = config.sdv_dist_base + config.sdv_dist_linear*dist_e;

        R = diag([sigma_d_e^2, config.sdv_ang_h^2, config.sdv_ang_v^2]);
        
        H = compute_H_analytical(x_e, feat_world, dh_model, q_current, z_base);
        
        % Innovation with angle wrapping
        innov = z - z_e;
        innov(2) = NormalizeAng(innov(2));
        innov(3) = NormalizeAng(innov(3));
        
        S = H * P * H' + R;
        
        % Kalman update
        K = P * H' / S;
        
        x_e = x_e + K * innov;
        x_e(3) = NormalizeAng(x_e(3));
        
        % Joseph form
        I_KH = eye(3) - K * H;
        P = I_KH * P * I_KH' + K * R * K';
        % APM
        %P = (P + P') / 2;
      
        % Update camera for next feature
        x_e_4d = [x_e(1:2); z_base; x_e(3)];
        T_cam_est = transform_camera_to_global(x_e_4d, dh_model, q_current);
        cam_pos_est = T_cam_est(1:3, 4);
        R_cam_est = T_cam_est(1:3, 1:3);
    end
    
    P_history{i} = P;
end

%% Plots
figure('Position', [100, 100, 1400, 900]);

subplot(2, 3, [1, 4]);
plot_3d_trajectory(x_t, x_e_t, environment, z_base, dh_model, q_t);

subplot(2, 3, 2);
plot_error_xy(x_t, x_e_t);

subplot(2, 3, 3);
plot_error_theta(x_t, x_e_t);

subplot(2, 3, [5, 6]);
plot_covariance(P_history);