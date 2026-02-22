function [u_t, q_t, x_init, x_final] = generate_converging_trajectory(N, dt, target_point)
% Docking approach trajectory: spawn from behind/above, converge to waypoint
% Simulates robot approaching a docking station with fiducial markers

    %% Spawn zone - Behind and above the docking waypoint
    % Randomize starting position closer to allow better convergence
    x0 = -1 + 2*rand;           % Spread horizontally: -1m to +1m
    y0 = 1 + 1*rand;        % Behind the waypoint: 
    
    P0 = [x0; y0];
    Pf = target_point(:);

    %% Time parameter
    t = linspace(0,1,N);

    %% Base straight-line path
    path = P0*(1-t) + Pf*t;

    %% Add smooth lateral deviation (zero at endpoints)
    % Creates a natural curved approach path
    dir = Pf - P0;
    perp = [-dir(2); dir(1)];
    if norm(perp) > 1e-6
        perp = perp / norm(perp);
    end

    amp = 0.25 * norm(dir) * (0.5 + rand);   % curvature strength
    offset = amp * sin(pi*t) .* (t .* (1-t));
    path = path + perp * offset;

    x = path(1,:);
    y = path(2,:);

    %% Velocities in world frame
    dx = gradient(x, dt);
    dy = gradient(y, dt);

    %% Orientation from velocity (pointing toward motion direction)
    theta = atan2(dy, dx);
    theta = unwrap(theta);

    %% Initial / final state
    x_init  = [x(1); y(1); theta(1)];
    x_final = [x(end); y(end); theta(end)];

    %% Angular velocity
    omega = [diff(theta)/dt, 0];
    omega(end) = omega(end-1);

    %% Transform velocities to robot frame (omni)
    v  =  dx .* cos(theta) + dy .* sin(theta);
    vn = -dx .* sin(theta) + dy .* cos(theta);

    %% Smooth commands (moving average filter)
    win = max(5, round(N/20));
    k = ones(1,win)/win;
    v  = conv(v,  k, 'same');
    vn = conv(vn, k, 'same');
    omega = conv(omega, k, 'same');

    %% Clamp velocities to realistic limits
    v = max(-0.8, min(0.8, v));
    vn = max(-0.6, min(0.6, vn));
    omega = max(-1.0, min(1.0, omega));

    %% Outputs
    u_t = [v; vn; omega];

    q_t = zeros(6, N);
    q_t(1,:) = x;
    q_t(2,:) = y;
    q_t(3,:) = theta;
end