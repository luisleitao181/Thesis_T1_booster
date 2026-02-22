%% transform_camera_to_global.m
function T_camera_global = transform_camera_to_global(x_base, dh_model, q)
    % Transform camera from local to global frame
    % Input:
    %   x_base: [x; y; z_base; theta] - base position in world
    %   dh_model: HumanoidDH object
    %   q: [6x1] joint angles
    % Output:
    %   T_camera_global: [4x4] camera transform in world frame
    
    cos_theta = cos(x_base(4));
    sin_theta = sin(x_base(4));
    
    % Base transform in world
    T_base_global = [cos_theta, -sin_theta, 0, x_base(1);
                     sin_theta,  cos_theta, 0, x_base(2);
                     0, 0, 1, x_base(3);
                     0, 0, 0, 1];
    
    % Camera transform in ankle frame (includes alignment and offset)
    T_camera_ankle = dh_model.forward_kinematics(q, true);
    
    % Chain transforms
    T_camera_global = T_base_global * T_camera_ankle;
end