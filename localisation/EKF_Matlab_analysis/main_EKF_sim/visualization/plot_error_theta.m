%% plot_error_theta.m
function plot_error_theta(x_t, x_e_t)
    hold on; grid on;
    
    N = size(x_t, 2);
    erro_theta = zeros(1, N);
    for i = 1:N
        erro_theta(i) = abs(NormalizeAng(x_t(3, i) - x_e_t(3, i))) * 180/pi;  % Angle error with wrap
    end
    
    plot(erro_theta, 'r-', 'LineWidth', 2);
    xlabel('Iteration'); ylabel('θ Error (°)');
    title('Orientation Error');
    ylim([0, max(erro_theta)*1.1]);
end