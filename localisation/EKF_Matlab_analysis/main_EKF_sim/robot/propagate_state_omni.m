%% propagate_state_omni.m
function x_new = propagate_state_omni(x, v, vn, omega, dt, config, add_noise)
    % Propagate omnidirectional state [x, y, theta]
    
    xr = x(1); yr = x(2); theta_r = x(3);
    
    d = v * dt;  % Forward displacement
    dn = vn * dt;  % Lateral displacement
    delta_theta = omega * dt;  % Angular change
    
    % Compute local frame displacement
    if abs(delta_theta) < 1e-6  % Linear approximation
        dx_local = d;
        dy_local = dn;
    else  % Full nonlinear
        dx_local = d * sin(delta_theta)/delta_theta - dn * (1 - cos(delta_theta))/delta_theta;
        dy_local = d * (1 - cos(delta_theta))/delta_theta + dn * sin(delta_theta)/delta_theta;
    end
    
    % Rotate to global frame (using midpoint angle)
    theta_mid = theta_r + delta_theta/2;
    dx_global = dx_local * cos(theta_mid) - dy_local * sin(theta_mid);
    dy_global = dx_local * sin(theta_mid) + dy_local * cos(theta_mid);
    
    % Update state
    if add_noise
        xr = xr + dx_global + randn*config.noise_xy;
        yr = yr + dy_global + randn*config.noise_xy;
        theta_r = theta_r + delta_theta + randn*config.noise_theta;
    else
        xr = xr + dx_global;
        yr = yr + dy_global;
        theta_r = theta_r + delta_theta;
    end
    
    theta_r = NormalizeAng(theta_r);  % Wrap angle to [-pi, pi]
    x_new = [xr; yr; theta_r];
end