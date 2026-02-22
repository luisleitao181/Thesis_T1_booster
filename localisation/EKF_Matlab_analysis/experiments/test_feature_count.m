%% test_feature_count.m
% Test: Localization accuracy vs number of features
% Objective: Determine minimum number of features required

clear; close all;

addpath('config', 'environment', 'utils', 'kinematics', 'robot');

%% Configuration
N = 600;
dt = 0.040;
config = RobotConfig();
dh_model = HumanoidDH(config);

feature_counts = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10];
n_trials = 200;

target_pos_error = 0.10;        % 10 cm
target_ang_error = 10 * pi/180; % 10 degrees

[u_t, q_t] = generate_control_simple(N, dt);

%% Run trials
results = struct();

fprintf('=== Feature Count Test ===\n\n');

for idx = 1:length(feature_counts)
    n_feat = feature_counts(idx);
    fprintf('Testing with %d features:\n', n_feat);
    
    environment = EnvironmentBuilder.create_simple(n_feat);
    
    results(idx).n_features = n_feat;
    results(idx).pos_rmse_trials = zeros(n_trials, 1);
    results(idx).ang_rmse_trials = zeros(n_trials, 1);
    
    for trial = 1:n_trials
        % Initialize with initial error
        x = [0; 0; 0];
        x_e = x + [0.25; 0.25; (4*pi/180)];
        P = diag([0.05^2, 0.05^2, (2*pi/180)^2]);
        
        x_t = zeros(3, N);
        x_e_t = zeros(3, N);
        z_base = config.z_base;
        
        % EKF loop
        for i = 1:N
            x_t(:, i) = x;
            x_e_t(:, i) = x_e;
            
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
            if n_feat > 0
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
                  
                    % Update camera
                    x_e_4d = [x_e(1:2); z_base; x_e(3)];
                    T_cam_est = transform_camera_to_global(x_e_4d, dh_model, q_current);
                    cam_pos_est = T_cam_est(1:3, 4);
                    R_cam_est = T_cam_est(1:3, 1:3);
                end
            end
        end
        
        % Compute metrics
        pos_errors = sqrt((x_t(1,:) - x_e_t(1,:)).^2 + (x_t(2,:) - x_e_t(2,:)).^2);
        ang_errors = abs(NormalizeAng(x_t(3,:) - x_e_t(3,:)));
        
        results(idx).pos_rmse_trials(trial) = sqrt(mean(pos_errors.^2));
        results(idx).ang_rmse_trials(trial) = sqrt(mean(ang_errors.^2));
        
        % Save last trajectory
        if trial == n_trials
            results(idx).x_t = x_t;
            results(idx).x_e_t = x_e_t;
            results(idx).environment = environment;
        end
    end
    
    fprintf('  Mean RMSE: Pos = %.3f m, Ang = %.2f º\n', ...
            mean(results(idx).pos_rmse_trials), ...
            mean(results(idx).ang_rmse_trials)*180/pi);
end

%% Plots
pos_mean = arrayfun(@(r) mean(r.pos_rmse_trials), results);
pos_std = arrayfun(@(r) std(r.pos_rmse_trials), results);
ang_mean = arrayfun(@(r) mean(r.ang_rmse_trials), results);
ang_std = arrayfun(@(r) std(r.ang_rmse_trials), results);

% Figure 1: Summary
figure('Position', [50, 50, 1400, 500]);

subplot(1, 3, 1);
hold on; grid on;
errorbar(feature_counts, pos_mean*100, pos_std*100, 'o-', ...
         'LineWidth', 2, 'MarkerSize', 8, 'MarkerFaceColor', 'b');
yline(target_pos_error*100, 'r--', 'LineWidth', 2);
xlabel('Number of Features');
ylabel('Absolute Position RMSE (cm)');
title('Aboslute Position Accuracy');

subplot(1, 3, 2);
hold on; grid on;
errorbar(feature_counts, ang_mean*180/pi, ang_std*180/pi, 'o-', ...
         'LineWidth', 2, 'MarkerSize', 8, 'MarkerFaceColor', 'r');
yline(target_ang_error*180/pi, 'r--', 'LineWidth', 2);
xlabel('Number of Features');
ylabel('Orientation RMSE (º)');
title('Orientation Accuracy');

subplot(1, 3, 3);
baseline_pos = pos_mean(2);  % Use 2 features as baseline (index 2)
gains = zeros(size(pos_mean));
for i = 1:length(pos_mean)
    if i <= 2
        gains(i) = 0;  % No gain for 1 and 2 features (baseline)
    else
        gains(i) = (baseline_pos - pos_mean(i)) / baseline_pos * 100;
    end
end
bar(feature_counts, gains);
xlabel('Number of Features');
ylabel('Improvement vs Baseline (%)');
title('Gain Relative to 2 Features');
grid on;
yline(0, 'k--', 'LineWidth', 1);

sgtitle(sprintf('Summary: Accuracy vs Number of Features (Targets: %.0f cm, %.0f º)', ...
        target_pos_error*100, target_ang_error*180/pi), 'FontSize', 12, 'FontWeight', 'bold');

% Figure 2: Individual trajectories (first 6)
figure('Position', [100, 100, 1400, 800]);

for idx = 1:min(6, length(results))
    subplot(2, 3, idx);
    hold on; grid on; axis equal;
    
    plot(results(idx).x_t(1,:), results(idx).x_t(2,:), 'k-', 'LineWidth', 2);
    plot(results(idx).x_e_t(1,:), results(idx).x_e_t(2,:), 'b-', 'LineWidth', 1.5);
    
    if results(idx).environment.num_features > 0
        plot(results(idx).environment.xp, results(idx).environment.yp, ...
             'r*', 'MarkerSize', 12, 'LineWidth', 2);
    end
    
    plot(results(idx).x_t(1,1), results(idx).x_t(2,1), 'go', ...
         'MarkerSize', 8, 'LineWidth', 2, 'MarkerFaceColor', 'g');
    plot(results(idx).x_t(1,end), results(idx).x_t(2,end), 'rs', ...
         'MarkerSize', 8, 'LineWidth', 2, 'MarkerFaceColor', 'r');
    
    xlabel('X (m)'); ylabel('Y (m)');
    title(sprintf('%d Feat | %.1f cm, %.1f º', ...
          feature_counts(idx), pos_mean(idx)*100, ang_mean(idx)*180/pi));
    
    if idx == 1
        legend('Real', 'Estimated', 'Features', 'Location', 'best', 'FontSize', 8);
    end
end

sgtitle('Trajectories by Feature Count', 'FontSize', 14, 'FontWeight', 'bold');

%% Print results
fprintf('\n=== RESULTS ===\n');
fprintf('Features | RMSE Pos (cm) | RMSE Ang (deg)\n');
fprintf('---------|---------------|---------------\n');
for i = 1:length(results)
    fprintf('%5d    |  %6.2f±%.2f  |  %6.2f±%.2f\n', ...
            feature_counts(i), pos_mean(i)*100, pos_std(i)*100, ...
            ang_mean(i)*180/pi, ang_std(i)*180/pi);
end

idx_pos = find(pos_mean <= target_pos_error, 1);
idx_ang = find(ang_mean <= target_ang_error, 1);
fprintf('\nMinimum features to reach targets:\n');
if ~isempty(idx_pos)
    fprintf('  Absolute Position: %d features\n', feature_counts(idx_pos));
end
if ~isempty(idx_ang)
    fprintf('  Orientation: %d features\n', feature_counts(idx_ang));
end