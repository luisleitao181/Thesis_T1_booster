%% compute_jacobians_omni.m
function [F, W] = compute_jacobians_omni(theta_e, v, vn, omega, dt)
    % Compute Jacobians for omnidirectional motion model
    % Outputs:
    %   F - State Jacobian ∂f/∂x (3x3)
    %   W - Control Jacobian ∂f/∂u (3x3) for [v, vn, omega]
    
    d = v * dt;  % Forward displacement
    dn = vn * dt;  % Lateral displacement
    delta_theta = omega * dt;  % Angular change
    theta_mid = theta_e + delta_theta/2;  % Midpoint angle
    
    % Local frame displacement and derivatives
    if abs(delta_theta) < 1e-6  % Linear approximation for small angles
        dx_local = d;
        dy_local = dn;
        
        ddx_dv = dt; ddx_dvn = 0; ddx_domega = 0;
        ddy_dv = 0; ddy_dvn = dt; ddy_domega = 0;
    else  % Full nonlinear model
        s = sin(delta_theta);
        c = cos(delta_theta);
        
        dx_local = d * s/delta_theta - dn * (1-c)/delta_theta;
        dy_local = d * (1-c)/delta_theta + dn * s/delta_theta;
        
        ddx_dv = dt * s/delta_theta;
        ddx_dvn = -dt * (1-c)/delta_theta;
        ddx_domega = dt * (d * (delta_theta*c - s)/(delta_theta^2) - ...
                           dn * (delta_theta*s - (1-c))/(delta_theta^2));
        
        ddy_dv = dt * (1-c)/delta_theta;
        ddy_dvn = dt * s/delta_theta;
        ddy_domega = dt * (d * (delta_theta*s - (1-c))/(delta_theta^2) + ...
                           dn * (delta_theta*c - s)/(delta_theta^2));
    end
    
    %% State Jacobian F = ∂f/∂X (3x3)
    cos_mid = cos(theta_mid);
    sin_mid = sin(theta_mid);
    
    ddxg_dtheta = -dx_local * sin_mid - dy_local * cos_mid;
    ddyg_dtheta = dx_local * cos_mid - dy_local * sin_mid;
    
    F = [1, 0, ddxg_dtheta;
         0, 1, ddyg_dtheta;
         0, 0, 1];
    
    %% Control Jacobian W = ∂f/∂U (3x3)
    % Global frame derivatives w.r.t. velocities
    ddxg_dv = ddx_dv * cos_mid - ddy_dv * sin_mid;
    ddyg_dv = ddx_dv * sin_mid + ddy_dv * cos_mid;
    
    ddxg_dvn = ddx_dvn * cos_mid - ddy_dvn * sin_mid;
    ddyg_dvn = ddx_dvn * sin_mid + ddy_dvn * cos_mid;
    
    ddxg_domega = ddx_domega * cos_mid - ddy_domega * sin_mid + ...
                  (-dx_local * sin_mid - dy_local * cos_mid) * dt/2;
    ddyg_domega = ddx_domega * sin_mid + ddy_domega * cos_mid + ...
                  (dx_local * cos_mid - dy_local * sin_mid) * dt/2;
    
    W = [ddxg_dv,    ddxg_dvn,    ddxg_domega;
         ddyg_dv,    ddyg_dvn,    ddyg_domega;
         0,          0,           dt];
end