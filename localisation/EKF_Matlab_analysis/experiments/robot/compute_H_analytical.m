%% compute_H_analytical.m
function H = compute_H_analytical(x_e, feat_world, dh_model, q, z_base)
    % Analytical Jacobian H = ∂h/∂X for measurements [dist; az; el] w.r.t. [x; y; θ]
    
    x = x_e(1); y = x_e(2); theta = x_e(3);
    xp = feat_world(1); yp = feat_world(2); zp = feat_world(3);
    
    % Forward kinematics
    T_cam_ankle = dh_model.forward_kinematics(q, true);
    pos_cam_ankle = T_cam_ankle(1:3, 4);  % Camera position in ankle frame
    R_cam_ankle = T_cam_ankle(1:3, 1:3);  % Camera rotation in ankle frame
    px = pos_cam_ankle(1); py = pos_cam_ankle(2); pz = pos_cam_ankle(3);
    
    % Camera position in world frame
    cos_th = cos(theta); sin_th = sin(theta);
    cam_x = x + cos_th*px - sin_th*py;
    cam_y = y + sin_th*px + cos_th*py;
    cam_z = z_base + pz;
    
    % Camera rotation in world frame
    R_base = [cos_th, -sin_th, 0; sin_th, cos_th, 0; 0, 0, 1];
    R_cam_global = R_base * R_cam_ankle;
    
    % Feature in camera frame
    vec_world = [xp - cam_x; yp - cam_y; zp - cam_z];
    vec_cam = R_cam_global' * vec_world;
    vcx = vec_cam(1); vcy = vec_cam(2); vcz = vec_cam(3);
    
    dist = sqrt(vcx^2 + vcy^2 + vcz^2);  % Range
    dist_xy = sqrt(vcx^2 + vcy^2);  % Horizontal distance
    
    % Derivatives of camera position w.r.t. theta
    dcam_x_dtheta = -sin_th*px - cos_th*py;
    dcam_y_dtheta = cos_th*px - sin_th*py;
    
    % Derivatives of world vector
    dvec_world_dx = [-1; 0; 0];
    dvec_world_dy = [0; -1; 0];
    dvec_world_dtheta = [-dcam_x_dtheta; -dcam_y_dtheta; 0];
    
    % Derivatives of rotation matrix
    dR_base_dtheta = [-sin_th, -cos_th, 0; cos_th, -sin_th, 0; 0, 0, 0];
    dR_cam_dtheta = dR_base_dtheta * R_cam_ankle;
    
    % Derivatives of camera frame vector
    dvec_cam_dx = R_cam_global' * dvec_world_dx;
    dvec_cam_dy = R_cam_global' * dvec_world_dy;
    dvec_cam_dtheta = dR_cam_dtheta' * vec_world + R_cam_global' * dvec_world_dtheta;
    
    H = zeros(3, 3);
    
    % ∂dist/∂[x,y,θ]
    if dist > 1e-6
        H(1,1) = (vcx*dvec_cam_dx(1) + vcy*dvec_cam_dx(2) + vcz*dvec_cam_dx(3)) / dist;
        H(1,2) = (vcx*dvec_cam_dy(1) + vcy*dvec_cam_dy(2) + vcz*dvec_cam_dy(3)) / dist;
        H(1,3) = (vcx*dvec_cam_dtheta(1) + vcy*dvec_cam_dtheta(2) + vcz*dvec_cam_dtheta(3)) / dist;
    end
    
    % ∂azimuth/∂[x,y,θ] where azimuth = atan2(vcy, vcx)
    if dist_xy^2 > 1e-6
        H(2,1) = (vcx*dvec_cam_dx(2) - vcy*dvec_cam_dx(1)) / (dist_xy^2);
        H(2,2) = (vcx*dvec_cam_dy(2) - vcy*dvec_cam_dy(1)) / (dist_xy^2);
        H(2,3) = (vcx*dvec_cam_dtheta(2) - vcy*dvec_cam_dtheta(1)) / (dist_xy^2);
    end
    
    % ∂elevation/∂[x,y,θ] where elevation = atan2(vcz, dist_xy)
    if dist^2 > 1e-6 && dist_xy > 1e-6
        ddist_xy_dx = (vcx*dvec_cam_dx(1) + vcy*dvec_cam_dx(2)) / dist_xy;
        ddist_xy_dy = (vcx*dvec_cam_dy(1) + vcy*dvec_cam_dy(2)) / dist_xy;
        ddist_xy_dtheta = (vcx*dvec_cam_dtheta(1) + vcy*dvec_cam_dtheta(2)) / dist_xy;
        
        H(3,1) = (dist_xy*dvec_cam_dx(3) - vcz*ddist_xy_dx) / (dist^2);
        H(3,2) = (dist_xy*dvec_cam_dy(3) - vcz*ddist_xy_dy) / (dist^2);
        H(3,3) = (dist_xy*dvec_cam_dtheta(3) - vcz*ddist_xy_dtheta) / (dist^2);
    end
end