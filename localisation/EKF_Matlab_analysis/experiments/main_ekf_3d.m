clear; close all;

addpath('config', 'environment', 'utils', 'visualization', 'kinematics', 'robot');

%% Configuration
N = 1000; 
dt = 0.040;
config = RobotConfig();
dh_model = HumanoidDH(config);

n_features = 4;
environment = EnvironmentBuilder.create_simple(n_features);

%% Generate control trajectory
[u_t, q_t] = generate_control_simple(N, dt);

%% Initialize states
x = [0; 0; 0];
x_e = x + [0.1; 0.1; (2*pi/180)];
P = diag([0.05^2, 0.05^2, (2*pi/180)^2]);

%% Storage
x_t = zeros(3, N);
x_e_t = zeros(3, N);
P_history = cell(N, 1);
z_base = config.z_base;

% NEES/NIS storage
nees_history = zeros(1, N);
nis_history = [];  % Variable length (one per measurement)
nis_timesteps = []; % Track which timestep each NIS belongs to

%% Chi-squared bounds for consistency test
% NEES: 3 DOF (state dimension)
alpha = 0.05;  % 95% confidence
nees_lower = chi2inv(alpha/2, 3);
nees_upper = chi2inv(1-alpha/2, 3);

% NIS: 3 DOF (measurement dimension)
nis_lower = chi2inv(alpha/2, 3);
nis_upper = chi2inv(1-alpha/2, 3);

%% EKF Loop
for i = 1:N
    x_t(:, i) = x;
    x_e_t(:, i) = x_e;
    
    v_cmd = u_t(1,i);
    vn_cmd = u_t(2,i); 
    omega_cmd = u_t(3,i);
    q_current = q_t(:, i);
    
    % Prediction step
    x = propagate_state_omni(x, v_cmd, vn_cmd, omega_cmd, dt, config, false);
    x_e = propagate_state_omni(x_e, v_cmd, vn_cmd, omega_cmd, dt, config, false);
    
    [F, W] = compute_jacobians_omni(x_e(3), v_cmd, vn_cmd, omega_cmd, dt);
    P = F * P * F' + W * config.Q * W';
    
    % Compute NEES after prediction
    error = x - x_e;
    error(3) = NormalizeAng(error(3));
    nees_history(i) = error' * (P \ error);
    
    % Camera transforms
    x_4d = [x(1:2); z_base; x(3)];
    x_e_4d = [x_e(1:2); z_base; x_e(3)];
    T_cam_real = transform_camera_to_global(x_4d, dh_model, q_current);
    T_cam_est = transform_camera_to_global(x_e_4d, dh_model, q_current);
    
    cam_pos_real = T_cam_real(1:3, 4);
    R_cam_real = T_cam_real(1:3, 1:3);
    cam_pos_est = T_cam_est(1:3, 4);
    R_cam_est = T_cam_est(1:3, 1:3);
    
    % Update step - sequential processing
    for j = 1:environment.num_features
        feat_world = [environment.xp(j); environment.yp(j); environment.zp(j)];

        % Real measurement
        vec_world = feat_world - cam_pos_real;
        vec_cam = R_cam_real' * vec_world;
        dist = norm(vec_cam);
        az = atan2(vec_cam(2), vec_cam(1));
        el = atan2(vec_cam(3), sqrt(vec_cam(1)^2 + vec_cam(2)^2));
        
        sigma_d = config.sdv_dist_base + config.sdv_dist_linear*dist;
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
        
        % Compute NIS
        nis = innov' * (S \ innov);
        nis_history = [nis_history, nis];
        nis_timesteps = [nis_timesteps, i];
        
        K = P * H' / S;
        
        % State update
        x_e = x_e + K * innov;
        x_e(3) = NormalizeAng(x_e(3));
        
        % Covariance update (Joseph form)
        I_KH = eye(3) - K * H;
        P = I_KH * P * I_KH' + K * R * K';
      
        % Update camera for next feature
        x_e_4d = [x_e(1:2); z_base; x_e(3)];
        T_cam_est = transform_camera_to_global(x_e_4d, dh_model, q_current);
        cam_pos_est = T_cam_est(1:3, 4);
        R_cam_est = T_cam_est(1:3, 1:3);
    end
    
    P_history{i} = P;
end

%% Consistency Analysis
fprintf('\nEKF Consistency Analysis\n');

% NEES statistics
nees_mean = mean(nees_history);
nees_inside = sum(nees_history >= nees_lower & nees_history <= nees_upper) / N * 100;
fprintf('NEES:\n');
fprintf('  Mean: %.2f (expected: 3.00 for 3-DOF system)\n', nees_mean);
fprintf('  Inside 95%% bounds: %.1f%% (expected: ~95%%)\n', nees_inside);
fprintf('  Bounds: [%.2f, %.2f]\n', nees_lower, nees_upper);

% NIS statistics
nis_mean = mean(nis_history);
nis_inside = sum(nis_history >= nis_lower & nis_history <= nis_upper) / length(nis_history) * 100;
fprintf('\nNIS:\n');
fprintf('  Mean: %.2f (expected: 3.00 for 3-DOF measurements)\n', nis_mean);
fprintf('  Inside 95%% bounds: %.1f%% (expected: ~95%%)\n', nis_inside);
fprintf('  Bounds: [%.2f, %.2f]\n', nis_lower, nis_upper);
fprintf('  Total measurements: %d\n', length(nis_history));


%% Visualization
time = (0:N-1) * dt;

figure('Position', [50, 50, 1800, 1000]);

% Trajectory
subplot(2, 4, [1, 5]);
plot_3d_trajectory(x_t, x_e_t, environment, z_base, dh_model, q_t);

% Position errors
subplot(2, 4, 2);
plot_error_xy(x_t, x_e_t);

% Orientation error
subplot(2, 4, 3);
plot_error_theta(x_t, x_e_t);

% Covariance
subplot(2, 4, 6);
plot_covariance(P_history);

% NEES
subplot(2, 4, 4);
hold on; grid on;
plot(time, nees_history, 'b-', 'LineWidth', 1.5);
yline(nees_lower, 'r--', 'LineWidth', 1.5, 'Alpha', 0.7);
yline(nees_upper, 'r--', 'LineWidth', 1.5, 'Alpha', 0.7);
yline(3, 'g--', 'LineWidth', 1, 'Alpha', 0.5);
xlabel('Time (s)');
ylabel('NEES');
title(sprintf('NEES (%.1f%% in bounds)', nees_inside));
legend('NEES', '95% bounds', 'Expected (3)', 'Location', 'best');

% NIS
subplot(2, 4, [7, 8]);
hold on; grid on;
plot(nis_timesteps * dt, nis_history, 'b.', 'MarkerSize', 4);
yline(nis_lower, 'r--', 'LineWidth', 1.5, 'Alpha', 0.7);
yline(nis_upper, 'r--', 'LineWidth', 1.5, 'Alpha', 0.7);
yline(3, 'g--', 'LineWidth', 1, 'Alpha', 0.5);
xlabel('Time (s)');
ylabel('NIS');
title(sprintf('NIS (%.1f%% in bounds)', nis_inside));
legend('NIS', '95% bounds', 'Expected (3)', 'Location', 'best');
ylim([0, min(20, max(nis_history))]);

sgtitle('EKF Localization with Consistency Analysis', 'FontSize', 14, 'FontWeight', 'bold');