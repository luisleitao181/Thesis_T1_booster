function [u_t, q_t] = generate_control_simple(N, dt)
    % Generate simple circular trajectory
    % Only v (forward) and omega (rotation) - no lateral motion
    
    %% Circular motion parameters
    v = 0.5;           % Constant forward velocity [m/s]
    omega = 0.3;       % Constant angular velocity [rad/s]
    
    % Create constant control vectors
    v_t = ones(1, N) * v;
    vn_t = zeros(1, N);        % No lateral motion
    omega_t = ones(1, N) * omega;
    
    u_t = [v_t; vn_t; omega_t];
    
    %% Static joint configuration (standing pose)
    q_t = zeros(6, N);  % All joints at default position
end