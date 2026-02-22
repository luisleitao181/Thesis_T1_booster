classdef HumanoidDH
    properties
        alpha, a, d  % DH parameters
        theta_default, q_min, q_max  % Joint limits
        joint_names, n_joints
        T_base_alignment  % DH frame -> Standing upright
        T_camera_offset  % Head -> Camera transform
        config
    end
    
    methods
        function obj = HumanoidDH(config)
            if nargin < 1, config = RobotConfig(); end
            obj.config = config;
            obj.n_joints = 6;
            
            % DH table: Ankle -> Knee -> Hip -> Waist -> Neck -> Head
            %obj.alpha = [0; 0; -90; 90; -90; 0] * pi/180;
            % APM
            obj.alpha = [0; 0; 90; -90; 90; 0] * pi/180;
            obj.a = [config.l1; config.l2; 0; 0; 0; 0];
            obj.d = [0; 0; 0; config.l4; 0; config.l6];
            % APM ???
            obj.theta_default = [0; 0; 90; 0; 0; 0] * pi/180;  % Standing pose

            % Nova rotação = Rz(90°) * Original
            Rz_90 = [0, -1, 0, 0;
                     1,  0, 0, 0;
                     0,  0, 1, 0;
                     0,  0, 0, 1];
            
            T_original = [0,  0, -1,  0;
                          0,  1,  0,  0;
                          1,  0,  0,  0;
                          0,  0,  0,  1];
            
            obj.T_base_alignment = Rz_90 * T_original;
            
            % Camera mounted perpendicular on top of head
            % Points forward (X), left (Y), up (Z)
            obj.T_camera_offset = [
                1,  0,  0,  0;
                0, -1,  0,  0;
                0,  0, -1,  0;
                0,  0,  0,  1
            ];
            
            % Joint limits (degrees -> radians)
            obj.q_min = [-23; 0; -118; -58; -18; -58] * pi/180;
            obj.q_max = [49; 123; 118; 58; 47; 58] * pi/180;
            obj.joint_names = {'Ankle', 'Knee', 'Hip', 'Waist', 'Neck', 'Head'};
        end
        
        function T = dh_matrix(~, alpha, a, d, theta)
            % Standard DH transformation matrix
            ct = cos(theta); st = sin(theta);
            ca = cos(alpha); sa = sin(alpha);
            T = [ct, -st*ca,  st*sa, a*ct;
                 st,  ct*ca, -ct*sa, a*st;
                  0,     sa,     ca,    d;
                  0,      0,      0,    1];
        end
        
        function T = forward_kinematics(obj, q, apply_alignment)
            % Compute camera pose from joint angles
            if nargin < 2, q = zeros(6,1); end
            if nargin < 3, apply_alignment = true; end
            if length(q) ~= 6, error('q must have 6 elements'); end
            
            T = eye(4);
            q = q(:);
            
            % Chain DH transforms
            for i = 1:obj.n_joints
                theta = q(i) + obj.theta_default(i);  % Offset from default
                T = T * obj.dh_matrix(obj.alpha(i), obj.a(i), obj.d(i), theta);
            end
            
            % Align to world frame (optional)
            if apply_alignment
                T = obj.T_base_alignment * T;
            end
            
            % Apply camera offset
            T = T * obj.T_camera_offset;
        end
        
        function [joint_pos, cam_pos, cam_dir] = compute_chain_positions(obj, q)
            % Compute positions of all joints + camera + camera direction
            % Output:
            %   joint_pos: (n_joints+1) x 3 - [Ankle, ..., Head] positions
            %   cam_pos: 1 x 3 - camera position
            %   cam_dir: 1 x 3 - camera forward direction (unit vector)
            
            joint_pos = zeros(obj.n_joints + 1, 3);
            joint_pos(1,:) = [0, 0, 0];  % Ankle at origin
            
            T = eye(4);
            for k = 1:obj.n_joints
                theta = q(k) + obj.theta_default(k);
                T = T * obj.dh_matrix(obj.alpha(k), obj.a(k), obj.d(k), theta);
                T_aligned = obj.T_base_alignment * T;
                joint_pos(k+1,:) = T_aligned(1:3, 4)';  % Extract position
            end
            
            % Camera transform
            T_cam = obj.T_base_alignment * T * obj.T_camera_offset;
            cam_pos = T_cam(1:3, 4)';  % Camera position
            cam_dir = T_cam(1:3, 1)';  % X-axis = forward direction
        end
        
        function q_abs = get_absolute_angles(obj, q)
            % Convert relative offsets to absolute joint angles
            % APM ???
            q_abs = q(:) + obj.theta_default(:);
        end
        
        function visualize(obj, q)
            % Interactive joint angle visualizer with sliders
            if nargin < 2, q = zeros(6,1); end
            
            fig = figure('Position', [100, 100, 1400, 800]);
            ax = axes('Parent', fig, 'Position', [0.35, 0.1, 0.6, 0.85]);
            
            n = length(q);
            sliders = cell(n, 1);
            labels = cell(n, 1);
            
            % Create sliders for each joint
            for i = 1:n
                y = 0.85 - (i-1)*0.08;
                q_abs = (q(i) + obj.theta_default(i)) * 180/pi;
                
                labels{i} = uicontrol('Style', 'text', ...
                    'String', sprintf('%s: %.1f° (abs: %.1f°)', obj.joint_names{i}, q(i)*180/pi, q_abs), ...
                    'Units', 'normalized', 'Position', [0.02, y+0.015, 0.28, 0.03], ...
                    'HorizontalAlignment', 'left', 'FontSize', 10, 'FontWeight', 'bold');
                
                sliders{i} = uicontrol('Style', 'slider', ...
                    'Min', obj.q_min(i)*180/pi, 'Max', obj.q_max(i)*180/pi, ...
                    'Value', q(i)*180/pi, 'Units', 'normalized', ...
                    'Position', [0.02, y-0.015, 0.28, 0.04], ...
                    'Callback', @(~,~) update());
            end
            
            uicontrol('Style', 'pushbutton', 'String', 'Reset', ...
                'Units', 'normalized', 'Position', [0.02, 0.02, 0.1, 0.05], ...
                'Callback', @(~,~) reset_pose());
            
            update();
            
            function update()
                for j = 1:n
                    q(j) = sliders{j}.Value * pi/180;
                    q_abs = (q(j) + obj.theta_default(j)) * 180/pi;
                    labels{j}.String = sprintf('%s: %.1f° (abs: %.1f°)', obj.joint_names{j}, q(j)*180/pi, q_abs);
                end
                
                [joint_pos, cam_pos, cam_dir] = obj.compute_chain_positions(q);
                
                cla(ax); hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');
                
                % Plot joint chain
                plot3(ax, joint_pos(:,1), joint_pos(:,2), joint_pos(:,3), 'o-', ...
                    'LineWidth', 3.5, 'MarkerSize', 10, 'Color', [0.2 0.4 0.8], ...
                    'MarkerFaceColor', [0.3 0.5 1], 'MarkerEdgeColor', [0.1 0.2 0.5]);
                
                % Ankle marker
                plot3(ax, joint_pos(1,1), joint_pos(1,2), joint_pos(1,3), 'o', ...
                    'MarkerSize', 14, 'MarkerFaceColor', [0.2 0.8 0.2], 'MarkerEdgeColor', [0.1 0.5 0.1], 'LineWidth', 2);
                text(ax, joint_pos(1,1), joint_pos(1,2), joint_pos(1,3)-0.15, 'Ankle', ...
                    'HorizontalAlignment', 'center', 'FontSize', 9, 'FontWeight', 'bold');
                
                % Head marker
                plot3(ax, joint_pos(end,1), joint_pos(end,2), joint_pos(end,3), 'o', ...
                    'MarkerSize', 12, 'MarkerFaceColor', [0.1 0.2 0.6], 'MarkerEdgeColor', [0.05 0.1 0.3], 'LineWidth', 2);
                
                % Head -> Camera link
                plot3(ax, [joint_pos(end,1), cam_pos(1)], [joint_pos(end,2), cam_pos(2)], ...
                      [joint_pos(end,3), cam_pos(3)], 'm--', 'LineWidth', 1.5);
                
                % Camera marker
                plot3(ax, cam_pos(1), cam_pos(2), cam_pos(3), 'o', ...
                    'MarkerSize', 14, 'MarkerFaceColor', [0.9 0.2 0.2], 'MarkerEdgeColor', [0.6 0.1 0.1], 'LineWidth', 2);
                text(ax, cam_pos(1), cam_pos(2), cam_pos(3)+0.15, 'Camera', ...
                    'HorizontalAlignment', 'center', 'FontSize', 9, 'FontWeight', 'bold');
                
                % Camera direction arrow
                arrow_end = cam_pos + cam_dir * 0.4;
                plot3(ax, [cam_pos(1), arrow_end(1)], [cam_pos(2), arrow_end(2)], ...
                      [cam_pos(3), arrow_end(3)], 'r-', 'LineWidth', 4);
                
                % Ground plane
                [X_f, Y_f] = meshgrid([-1.5, 1.5], [-1.5, 1.5]);
                surf(ax, X_f, Y_f, zeros(size(X_f))-obj.config.z_base, ...
                     'FaceAlpha', 0.15, 'EdgeColor', 'none', 'FaceColor', [0.7 0.7 0.7]);
                
                % Ankle -> ground line
                plot3(ax, [joint_pos(1,1), joint_pos(1,1)], [joint_pos(1,2), joint_pos(1,2)], ...
                      [0, -obj.config.z_base], 'k--', 'LineWidth', 1.5);
                
                xlabel(ax, 'X (m)'); ylabel(ax, 'Y (m)'); zlabel(ax, 'Z (m)');
                title(ax, sprintf('Camera Height: %.2f m (from ground: %.2f m)', ...
                    cam_pos(3), cam_pos(3) + obj.config.z_base));
                view(ax, 45, 25);
                xlim(ax, [-0.8, 0.8]); ylim(ax, [-0.8, 0.8]); 
                zlim(ax, [-obj.config.z_base-0.1, max(1.5, cam_pos(3)+0.3)]);
                rotate3d(ax, 'on');
                drawnow;
            end
            
            function reset_pose()
                q = zeros(6,1);
                for j = 1:n
                    sliders{j}.Value = 0;
                end
                update();
            end
        end
    end
end