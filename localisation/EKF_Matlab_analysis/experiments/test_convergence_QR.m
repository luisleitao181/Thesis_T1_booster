%% test_convergence_QR_simplified.m
% Test: Influence of process noise (Q) on filter convergence
% Simplified version with manual steady-state definition

clear; close all;

addpath('config', 'environment', 'utils', 'kinematics', 'robot');

%% Configuration
N = 250;
dt = 0.040;
config = RobotConfig();
dh_model = HumanoidDH(config);

n_features = 4;
environment = EnvironmentBuilder.create_simple(n_features);

% Focused comparison: low Q vs high Q
Q_multipliers = [0.01, 100];
n_configs = length(Q_multipliers);
n_trials = 100;

% Initial error
initial_error = [0.25; 0.25; 90*pi/180];

% MANUAL STEADY-STATE DEFINITION (in seconds)
% Define quando cada configuração entra em steady-state
steady_state_time = [4, 2];  % [tempo para Q=0.01x, Q=100x]

[u_t, q_t] = generate_control_simple(N, dt);

%% Run tests
results = struct();
Q_base = config.Q;

fprintf('=== Simplified Q Trade-off Analysis ===\n\n');
fprintf('Configuration:\n');
fprintf('  Features: %d\n', n_features);
fprintf('  Initial error: %.2f m, %.2f m, %.1f °\n', ...
        initial_error(1), initial_error(2), initial_error(3)*180/pi);
fprintf('  Steady-state times: %.1f s (Q=%.2fx), %.1f s (Q=%.2fx)\n\n', ...
        steady_state_time(1), Q_multipliers(1), ...
        steady_state_time(2), Q_multipliers(2));

for idx = 1:n_configs
    Q_mult = Q_multipliers(idx);
    fprintf('Testing Q = %.2fx baseline:\n', Q_mult);
    
    config.Q = Q_base * Q_mult;
    
    % Storage for trial results
    rmse_x_trials = zeros(n_trials, 1);
    rmse_y_trials = zeros(n_trials, 1);
    rmse_theta_trials = zeros(n_trials, 1);
    
    for trial = 1:n_trials
        % Initialize
        x = [0; 0; 0];
        x_e = x + initial_error;
        P = diag([0.05^2, 0.05^2, 0.05^2]);
        
        x_t = zeros(3, N);
        x_e_t = zeros(3, N);
        P_history = zeros(3, N);
        z_base = config.z_base;
        
        % EKF loop
        for i = 1:N
            x_t(:, i) = x;
            x_e_t(:, i) = x_e;
            P_history(:, i) = [sqrt(P(1,1)); sqrt(P(2,2)); sqrt(P(3,3))];
            
            v_cmd = u_t(1,i);
            vn_cmd = u_t(2,i); 
            omega_cmd = u_t(3,i);
            q_current = q_t(:, i);
            
            % Prediction
            x = propagate_state_omni(x, v_cmd, vn_cmd, omega_cmd, dt, config, false);
            x_e = propagate_state_omni(x_e, v_cmd, vn_cmd, omega_cmd, dt, config, false);
            
            [F, W] = compute_jacobians_omni(x_e(3), v_cmd, vn_cmd, omega_cmd, dt);
            P = F * P * F' + W * config.Q * W';
            
            % Camera transforms
            x_4d = [x(1:2); z_base; x(3)];
            x_e_4d = [x_e(1:2); z_base; x_e(3)];
            T_cam_real = transform_camera_to_global(x_4d, dh_model, q_current);
            T_cam_est = transform_camera_to_global(x_e_4d, dh_model, q_current);
            
            cam_pos_real = T_cam_real(1:3, 4);
            R_cam_real = T_cam_real(1:3, 1:3);
            cam_pos_est = T_cam_est(1:3, 4);
            R_cam_est = T_cam_est(1:3, 1:3);
            
            % Update
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
                 
                % Prediction
                vec_world_e = feat_world - cam_pos_est;
                vec_cam_e = R_cam_est' * vec_world_e;
                dist_e = norm(vec_cam_e);
                z_e = [dist_e; 
                       atan2(vec_cam_e(2), vec_cam_e(1)); 
                       atan2(vec_cam_e(3), sqrt(vec_cam_e(1)^2 + vec_cam_e(2)^2))];
                   
                sigma_d_e = config.sdv_dist_base + config.sdv_dist_linear*dist_e;
                R = diag([sigma_d_e^2, config.sdv_ang_h^2, config.sdv_ang_v^2]);
                
                H = compute_H_analytical(x_e, feat_world, dh_model, q_current, z_base);
                
                innov = z - z_e;
                innov(2) = NormalizeAng(innov(2));
                innov(3) = NormalizeAng(innov(3));
                
                S = H * P * H' + R;
                K = P * H' / S;
                
                x_e = x_e + K * innov;
                x_e(3) = NormalizeAng(x_e(3));
                
                I_KH = eye(3) - K * H;
                P = I_KH * P * I_KH' + K * R * K';
              
                % Update camera for next feature
                x_e_4d = [x_e(1:2); z_base; x_e(3)];
                T_cam_est = transform_camera_to_global(x_e_4d, dh_model, q_current);
                cam_pos_est = T_cam_est(1:3, 4);
                R_cam_est = T_cam_est(1:3, 1:3);
            end
        end
        
        % Calculate RMSE
        err_x_all = x_t(1,:) - x_e_t(1,:);
        err_y_all = x_t(2,:) - x_e_t(2,:);
        err_theta_all = NormalizeAng(x_t(3,:) - x_e_t(3,:));
        
        rmse_x_trials(trial) = sqrt(mean(err_x_all.^2));
        rmse_y_trials(trial) = sqrt(mean(err_y_all.^2));
        rmse_theta_trials(trial) = sqrt(mean(err_theta_all.^2));
        
        % Save last trajectory for plots
        if trial == n_trials
            results(idx).x_t = x_t;
            results(idx).x_e_t = x_e_t;
            results(idx).P_history = P_history;
            results(idx).err_x = err_x_all;
            results(idx).err_y = err_y_all;
            results(idx).err_theta = err_theta_all;
        end
    end
    
    % Compute statistics
    results(idx).Q_mult = Q_mult;
    results(idx).rmse_x = mean(rmse_x_trials);
    results(idx).rmse_y = mean(rmse_y_trials);
    results(idx).rmse_theta = mean(rmse_theta_trials);
    
    % Compute steady-state error using manual definition
    steady_start_step = ceil(steady_state_time(idx) / dt);
    results(idx).steady_start_step = steady_start_step;
    results(idx).steady_start_time = steady_state_time(idx);
    
    results(idx).steady_err_x = mean(abs(results(idx).err_x(steady_start_step:end)));
    results(idx).steady_err_y = mean(abs(results(idx).err_y(steady_start_step:end)));
    results(idx).steady_err_theta = mean(abs(results(idx).err_theta(steady_start_step:end)));
    
    % Print results
    fprintf('  RMSE: X=%.3fm, Y=%.3fm, θ=%.2f °\n', ...
            results(idx).rmse_x, results(idx).rmse_y, results(idx).rmse_theta*180/pi);
    fprintf('  Steady-state (from %.1fs): X=%.2fcm, Y=%.2fcm, θ=%.2f °\n\n', ...
            steady_state_time(idx), ...
            results(idx).steady_err_x*100, results(idx).steady_err_y*100, ...
            results(idx).steady_err_theta*180/pi);
end

%% PLOTS
time = (0:N-1) * dt;
colors = [0.2, 0.4, 0.8; 0.8, 0.2, 0.2];  % Blue, Red

% Figure 1: Trajectories (Full time) - 2 subplots side by side
figure('Position', [50, 50, 1200, 500]);

for idx = 1:n_configs
    subplot(1, 2, idx);
    hold on; grid on; axis equal;
    
    % Plot full trajectory
    plot(results(idx).x_t(1,:), results(idx).x_t(2,:), ...
         'k-', 'LineWidth', 2, 'DisplayName', 'Real');
    plot(results(idx).x_e_t(1,:), results(idx).x_e_t(2,:), ...
         'Color', colors(idx,:), 'LineWidth', 2, 'DisplayName', 'Estimated');
    plot(environment.xp, environment.yp, 'r*', 'MarkerSize', 12, ...
         'LineWidth', 2, 'DisplayName', 'Features');
    plot(results(idx).x_e_t(1,1), results(idx).x_e_t(2,1), 'go', ...
         'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', 'g', ...
         'DisplayName', 'Start');
    
    xlabel('X (m)', 'FontSize', 11); 
    ylabel('Y (m)', 'FontSize', 11);
    title(sprintf('Q = %.2f × Q_{base}', Q_multipliers(idx)), ...
          'FontWeight', 'bold', 'FontSize', 12);
    legend('Location', 'best', 'FontSize', 9);
end

sgtitle('Trajectories Comparison', ...
        'FontSize', 14, 'FontWeight', 'bold');

% Figure 2: Error Evolution - 4 subplots (2 convergence + 2 steady-state)
figure('Position', [100, 100, 1200, 800]);

for idx = 1:n_configs
    % CONVERGENCE PHASE (top row)
    subplot(2, 2, idx);
    hold on; grid on;
    
    conv_end_step = results(idx).steady_start_step;
    time_conv = time(1:conv_end_step);
    
    % Calculate absolute position error
    err_pos_cm = sqrt(results(idx).err_x(1:conv_end_step).^2 + ...
                      results(idx).err_y(1:conv_end_step).^2) * 100;
    err_theta_deg = abs(results(idx).err_theta(1:conv_end_step)) * 180/pi;
    
    yyaxis left
    h1 = plot(time_conv, err_pos_cm, 'b-', 'LineWidth', 2, 'DisplayName', 'Position');
    ylabel('Position error (cm)', 'FontSize', 11);
    ylim([0 30]);

    yyaxis right
    h2 = plot(time_conv, err_theta_deg, 'r-', 'LineWidth', 2, 'DisplayName', 'θ');
    ylabel('Orientation error (°)', 'FontSize', 11);
    ylim([0 30]);

    xlabel('Time (s)', 'FontSize', 11);
    title(sprintf('CONVERGENCE: Q = %.2fx (0 to %.1fs)', ...
          Q_multipliers(idx), steady_state_time(idx)), ...
          'FontWeight', 'bold', 'FontSize', 11);
    legend([h1, h2], 'Location', 'best', 'FontSize', 9);
    
    % STEADY-STATE PHASE (bottom row)
    subplot(2, 2, idx + 2);
    hold on; grid on;
    
    ss_start = results(idx).steady_start_step + 1;
    time_ss = time(ss_start:end);
    
    % Calculate absolute position error for steady-state
    err_pos_cm_ss = sqrt(results(idx).err_x(ss_start:end).^2 + ...
                         results(idx).err_y(ss_start:end).^2) * 100;
    err_theta_deg_ss = abs(results(idx).err_theta(ss_start:end)) * 180/pi;

    yyaxis left
    h1 = plot(time_ss, err_pos_cm_ss, 'b-', 'LineWidth', 2, 'DisplayName', 'Position');
    ylabel('Position error (cm)', 'FontSize', 11);
    ylim([0 20]);  % Fixed scale for position

    yyaxis right
    h2 = plot(time_ss, err_theta_deg_ss, 'r-', 'LineWidth', 2, 'DisplayName', 'θ');
    ylabel('Orientation error (°)', 'FontSize', 11);
    ylim([0 8]);  % Fixed scale for orientation

    xlabel('Time (s)', 'FontSize', 11);
    title(sprintf('STEADY-STATE: Q = %.2fx (%.1fs to %.1fs)', ...
          Q_multipliers(idx), steady_state_time(idx), time(end)), ...
          'FontWeight', 'bold', 'FontSize', 11);
    legend([h1, h2], 'Location', 'best', 'FontSize', 9);
end

sgtitle('Error Evolution: Convergence vs Steady-State', ...
        'FontSize', 14, 'FontWeight', 'bold');

%% Summary
fprintf('\n=== RESULTS SUMMARY ===\n\n');
fprintf('%-10s | %-12s | %-12s | %-12s\n', ...
        'Q Mult', 'SS X (cm)', 'SS Y (cm)', 'SS θ (°)');
fprintf('-----------|--------------|--------------|-------------\n');
for idx = 1:n_configs
    fprintf('%8.2fx |   %9.2f |   %9.2f |   %9.2f\n', ...
            Q_multipliers(idx), ...
            results(idx).steady_err_x*100, ...
            results(idx).steady_err_y*100, ...
            results(idx).steady_err_theta*180/pi);
end
fprintf('\nSS = Steady-State\n');
fprintf('\n--- Trade-off Analysis ---\n');
fprintf('Low Q (%.2fx):    Lower steady-state noise\n', Q_multipliers(1));
fprintf('High Q (%.2fx):   Higher steady-state noise\n', Q_multipliers(2));

config.Q = Q_base;