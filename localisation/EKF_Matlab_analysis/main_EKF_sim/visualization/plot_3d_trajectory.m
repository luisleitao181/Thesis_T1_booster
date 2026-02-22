%% plot_3d_trajectory.m
function plot_3d_trajectory(x_t, x_e_t, environment, z_base, dh_model, q_t)
    hold on; grid on;
    title('3D Trajectory - Camera');
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    
    colors = lines(environment.num_groups);
    
    % Plot features (apenas um para legenda)
    legend_handles = [];
    legend_labels = {};
    
    for j = 1:environment.num_features
        marker = 's'; msize = 10;
        if environment.feature_types(j) == 2
            marker = 'o'; msize = 8;
        end
        gid = environment.feature_groups(j);
        h = plot3(environment.xp(j), environment.yp(j), environment.zp(j), marker, ...
              'MarkerSize', msize, 'MarkerEdgeColor', colors(gid,:), ...
              'MarkerFaceColor', colors(gid,:), 'LineWidth', 1.5);
        
        % Adiciona à legenda apenas o primeiro de cada grupo
        if j == 1 || ~ismember(gid, environment.feature_groups(1:j-1))
            legend_handles(end+1) = h;
            legend_labels{end+1} = sprintf('Features', gid);
        end
    end
    
    % Compute camera positions
    N = size(x_t, 2);
    cam_pos_real = zeros(3, N);
    cam_pos_est = zeros(3, N);
    
    for i = 1:N
        q_current = q_t(:, i);
        [~, cam_local, ~] = dh_model.compute_chain_positions(q_current);
        
        % Real trajectory
        T_base_real = build_base_transform(x_t(1:2, i), x_t(3, i), z_base);
        p_cam = T_base_real * [cam_local'; 1];
        cam_pos_real(:, i) = p_cam(1:3);
        
        % Estimated trajectory
        T_base_est = build_base_transform(x_e_t(1:2, i), x_e_t(3, i), z_base);
        p_cam_e = T_base_est * [cam_local'; 1];
        cam_pos_est(:, i) = p_cam_e(1:3);
    end
    
    % Plot trajectories
    h_real = plot3(cam_pos_real(1,:), cam_pos_real(2,:), cam_pos_real(3,:), ...
                   'b-', 'LineWidth', 2, 'DisplayName', 'Real Trajectory');
    h_est = plot3(cam_pos_est(1,:), cam_pos_est(2,:), cam_pos_est(3,:), ...
                  'r-', 'LineWidth', 1.5, 'DisplayName', 'Estimated Trajectory');
    
    % Start/end markers
    h_start = plot3(cam_pos_real(1,1), cam_pos_real(2,1), cam_pos_real(3,1), 'go', ...
          'MarkerSize', 12, 'MarkerFaceColor', 'g', 'DisplayName', 'Start');
    h_end = plot3(cam_pos_real(1,end), cam_pos_real(2,end), cam_pos_real(3,end), 'mo', ...
          'MarkerSize', 12, 'MarkerFaceColor', 'm', 'DisplayName', 'End');
    
    % Ground plane
    [X_floor, Y_floor] = meshgrid([-12, 12], [-11, 11]);
    surf(X_floor, Y_floor, zeros(size(X_floor)), 'FaceAlpha', 0.1, ...
         'EdgeColor', 'none', 'FaceColor', [0.8 0.8 0.8]);
    
    % Combina handles para legenda
    legend_handles = [legend_handles, h_real, h_est, h_start, h_end];
    legend_labels = [legend_labels, {'Real Trajectory', 'Estimated Trajectory', 'Start', 'End'}];
    
    % Cria legenda
    legend(legend_handles, legend_labels, 'Location', 'best', 'FontSize', 9);
    
    view(45, 30);
    axis equal;
    xlim([-2.5, 2.5]); ylim([-2.5, 5]); zlim([0, 4]);
    rotate3d on;
end

function T = build_base_transform(xy, theta, z_base)
    % Build base transformation matrix
    cos_th = cos(theta);
    sin_th = sin(theta);
    T = [cos_th, -sin_th, 0, xy(1);
         sin_th,  cos_th, 0, xy(2);
         0, 0, 1, z_base;
         0, 0, 0, 1];
end