%% test_fiducial_marker.m
% Test: Docking station approach with optimal viewing geometry
% Evaluates localization accuracy as measurement noise increases

clear; close all;
addpath('config','utils','kinematics','robot');

%% Configuration
N_trajectories = 200;
N_steps = 600;
dt = 0.040;

alphas = [0.01 0.05 0.15 0.25 0.5 1];  % Measurement noise scale factors

pos_thr = 1;  % cm
ang_thr = 1;  % degrees

config = RobotConfig();
dh_model = HumanoidDH(config);

%% Environment setup - Docking Station Configuration
% Two markers at ground level, separated horizontally
marker_distance = 0.5;  % 2m apart horizontally
marker_height = 2.0;    % Height of markers

% Docking waypoint: positioned for 90° viewing angle to both markers
% This is the optimal geometry for localization
docking_waypoint = [0; 0.5];  % At distance = marker_distance for 90° angle

environment.xp = [-marker_distance; marker_distance];
environment.yp = [0; 0];
environment.zp = [marker_height; marker_height];
environment.num_features = 2;
environment.num_groups = 1;
environment.feature_groups = [1 1];
environment.feature_types = [1 1];

%% Storage
success_rate = zeros(length(alphas), 1);
mean_err_pos = zeros(length(alphas), 1);
mean_err_ang = zeros(length(alphas), 1);
all_traj_store = cell(length(alphas), 1);
max_err_pos = zeros(length(alphas), 1);
max_err_ang = zeros(length(alphas), 1);

%% Main loop
for a = 1:length(alphas)
    alpha = alphas(a);
    fprintf('Testing α = %.2f\n', alpha);

    final_errors = zeros(N_trajectories, 3);
    all_traj_store{a}.x = cell(N_trajectories, 1);
    all_traj_store{a}.xe = cell(N_trajectories, 1);
    
    max_err_pos_trials = zeros(N_trajectories,1);
    max_err_ang_trials = zeros(N_trajectories,1);


    for traj = 1:N_trajectories
        [u_t, q_t, x_init, ~] = ...
            generate_converging_trajectory(N_steps, dt, docking_waypoint);

        x = x_init;
        x_e = x_init;
        P = diag([0.1^2, 0.1^2, (5*pi/180)^2]);
        z_base = config.z_base;

        x_t = zeros(3, N_steps);
        x_e_t = zeros(3, N_steps);

        for i = 1:N_steps
            x_t(:, i) = x;
            x_e_t(:, i) = x_e;
            

            % Prediction

            v = u_t(1,i); 
            vn = u_t(2,i); 
            w = u_t(3,i);


            x = propagate_state_omni(x, v, vn, w, dt, config, false);
            x_e = propagate_state_omni(x_e, v, vn, w, dt, config, false);

            [F, W] = compute_jacobians_omni(x_e(3), v, vn, w, dt);
            P = F * P * F' + W * config.Q * W';

            % Camera pose
            q_cur = q_t(:, i);
            x4 = [x(1:2); z_base; x(3)];
            x4e = [x_e(1:2); z_base; x_e(3)];

            T_cam_real = transform_camera_to_global(x4, dh_model, q_cur);
            T_cam_est = transform_camera_to_global(x4e, dh_model, q_cur);

            cam_r = T_cam_real(1:3, 4);
            cam_e = T_cam_est(1:3, 4);
            R_r = T_cam_real(1:3, 1:3);
            R_e = T_cam_est(1:3, 1:3);

            % Update
            for j = 1:environment.num_features
                feat = [environment.xp(j); environment.yp(j); environment.zp(j)];

                vec = R_r' * (feat - cam_r);
                d = norm(vec);
                az = atan2(vec(2), vec(1));
                el = atan2(vec(3), hypot(vec(1), vec(2)));

                sigma_d = alpha * (config.sdv_dist_base + config.sdv_dist_linear * d);
                sigma_az = alpha * config.sdv_ang_h;
                sigma_el = alpha * config.sdv_ang_v;
                
                % Medição ruidosa (Simulação)
                z = [d + randn * sigma_d;
                     NormalizeAng(az + randn * sigma_az);
                     NormalizeAng(el + randn * sigma_el)];
                
                % Matriz de Covariância do Ruído (Filtro)
                R = diag([sigma_d^2, sigma_az^2, sigma_el^2]);


                vec_e = R_e' * (feat - cam_e);
                d_e = norm(vec_e);

                z_e = [d_e;
                       atan2(vec_e(2), vec_e(1));
                       atan2(vec_e(3), hypot(vec_e(1), vec_e(2)))];


                H = compute_H_analytical(x_e, feat, dh_model, q_cur, z_base);

                innov = z - z_e;
                innov(2:3) = NormalizeAng(innov(2:3));

                S = H * P * H' + R;
                K = P * H' / S;

                x_e = x_e + K * innov;
                x_e(3) = NormalizeAng(x_e(3));

                P = (eye(3) - K * H) * P * (eye(3) - K * H)' + K * R * K';

            end
        end

                % Erro ao longo do tempo
        err_pos_t = sqrt((x_t(1,:) - x_e_t(1,:)).^2 + ...
                         (x_t(2,:) - x_e_t(2,:)).^2) * 100; % cm
        
        err_ang_t = abs(NormalizeAng(x_t(3,:) - x_e_t(3,:))) * 180/pi; % deg
        
        % Erro absoluto máximo no ensaio
        m_pos = max(err_pos_t);
        m_ang = max(err_ang_t);
        if m_pos < 100 && m_ang < 90 %filtrar divergencia
            max_err_pos_trials(traj) = m_pos;
            max_err_ang_trials(traj) = m_ang;
        else
            max_err_pos_trials(traj) = NaN;
            max_err_ang_trials(traj) = NaN;
        end

        final_errors(traj, 1) = norm(x(1:2) - x_e(1:2)) * 100;
        final_errors(traj, 3) = abs(NormalizeAng(x(3) - x_e(3))) * 180/pi;

        all_traj_store{a}.x{traj} = x_t;
        all_traj_store{a}.xe{traj} = x_e_t;
    end
    
    max_err_pos(a) = median(max_err_pos_trials, 'omitnan');
    max_err_ang(a) = median(max_err_ang_trials, 'omitnan');


    success_rate(a) = mean(final_errors(:, 1) < pos_thr & ...
                            final_errors(:, 3) < ang_thr) * 100;
    %mean_err_pos(a) = mean(final_errors(:, 1));
    %mean_err_ang(a) = mean(final_errors(:, 3));
end


%% Visualization
% figure('Position', [100 100 800 400]);
% subplot(1,2,1);
% plot(alphas, max_err_pos, '-^', 'LineWidth', 2, 'MarkerSize', 8);
% grid on;
% xlabel('Noise scale \alpha');
% ylabel('Max absolute position error (cm)');
% title('Maximum Absolute Position Error');
% 
% subplot(1,2,2);
% plot(alphas, max_err_ang, '-^', 'LineWidth', 2, 'MarkerSize', 8);
% grid on;
% xlabel('Noise scale \alpha');
% ylabel('Max absolute orientation error (deg)');
% title('Maximum Absolute Orientation Error');

%%
figure('Position', [100 100 1800 1000]);

for a = 1:length(alphas)
    subplot(2, 3, a);
    hold on; grid on; axis equal;
    
    title(sprintf('\\alpha = %.2f (Success: %.1f%%)', ...
        alphas(a), success_rate(a)));
    xlabel('X (m)');
    ylabel('Y (m)');
    
     %Plot all trajectories
     for traj = 1:N_trajectories
         plot(all_traj_store{a}.x{traj}(1,:), all_traj_store{a}.x{traj}(2,:), '-', ...
             'Color', [0 0.4470 0.7410 0.3], 'LineWidth', 0.8, 'HandleVisibility','off');
     end
    %%%Plot all trajectories
    % for traj = 1:N_trajectories
    %     plot(all_traj_store{a}.xe{traj}(1,:), all_traj_store{a}.xe{traj}(2,:), '-', ...
    %         'Color', [0 0.4470 0.7410 0.3], 'LineWidth', 0.8, 'HandleVisibility','off');
    % end
    
    % Markers and docking waypoint
    plot(environment.xp, environment.yp, 'ks', 'MarkerSize', 15, ...
        'MarkerFaceColor', 'k', 'LineWidth', 2, 'HandleVisibility','off');  
    plot(docking_waypoint(1), docking_waypoint(2), 'r*', 'MarkerSize', 20, ...
        'LineWidth', 3, 'HandleVisibility','off');  
    
    % Legend entries (só no primeiro subplot)
    if a == 1  
        plot(NaN, NaN, 'ks', 'MarkerSize', 15, 'MarkerFaceColor', 'k', ...
            'LineWidth', 2, 'DisplayName', 'Fiducial Markers');
        plot(NaN, NaN, 'r*', 'MarkerSize', 20, 'LineWidth', 3, ...
            'DisplayName', 'Docking Waypoint');
        plot(NaN, NaN, '-', 'Color', [0 0.4470 0.7410], 'LineWidth', 2, ...
            'DisplayName', 'Trajectories');
        legend('Location', 'best');
    end  % ADICIONAR ESTA LINHA
    
    xlim([-1.5 1.5]);
    ylim([ 0 2]);
end

% Summary plots
figure('Position', [100 100 1200 400]);

subplot(1, 3, 1);
plot(alphas, success_rate, '-o', 'LineWidth', 2, 'MarkerSize', 8);
grid on;
xlabel('Noise scale \alpha');
ylabel('Success rate (%)');
title('Localization Success Rate');

% subplot(1, 3, 2);
% plot(alphas, mean_err_pos, '-s', 'LineWidth', 2, 'MarkerSize', 8);
% ylabel('Mean absolute position error (cm)');
% xlabel('Noise scale \alpha');
% title('Mean Position Error');
% grid on;
% 
% subplot(1, 3, 3);
% plot(alphas, mean_err_ang, '-d', 'LineWidth', 2, 'MarkerSize', 8);
% ylabel('Mean absolute orientation error (deg)');
% xlabel('Noise scale \alpha');
% title('Mean Orientation Error');
% grid on;

subplot(1, 3, 2);
plot(alphas, max_err_pos, '-s', 'LineWidth', 2, 'MarkerSize', 8);
grid on;
xlabel('Noise scale \alpha');
ylabel('Max absolute position error (cm)');
title('Maximum Absolute Position Error');

subplot(1, 3, 3);
plot(alphas, max_err_ang, '-d', 'LineWidth', 2, 'MarkerSize', 8);
grid on;
xlabel('Noise scale \alpha');
ylabel('Max absolute orientation error (deg)');
title('Maximum Absolute Orientation Error');

