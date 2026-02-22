function plot_covariance(P_history)
    N = length(P_history);
    
    % Extract σ_x, σ_y, e σ_θ
    sigma_x = zeros(N, 1);
    sigma_y = zeros(N, 1);
    sigma_theta = zeros(N, 1);
    
    for i = 1:N
        P = P_history{i};
        sigma_x(i) = sqrt(P(1, 1));      % Desvio padrão em X
        sigma_y(i) = sqrt(P(2, 2));      % Desvio padrão em Y
        sigma_theta(i) = sqrt(P(3, 3));  % Desvio padrão em θ
    end
    
    % Dual-axis plot: σ_x e σ_y na esquerda, σ_θ na direita
    yyaxis left
    hold on;
    plot(1:N, sigma_x, 'b-', 'LineWidth', 2, 'DisplayName', '\sigma_x');
    plot(1:N, sigma_y, 'r-', 'LineWidth', 2, 'DisplayName', '\sigma_y');
    ylabel('\sigma_{x,y} (m)');
    ylim([0, max([max(sigma_x), max(sigma_y)])*1.1]);
    ax = gca;
    ax.YColor = 'k';
    
    yyaxis right
    plot(1:N, sigma_theta*180/pi, 'm-', 'LineWidth', 2, 'DisplayName', '\sigma_\theta');
    ylabel('\sigma_\theta (°)');
    ylim([0, max(sigma_theta*180/pi)*1.1]);
    ax.YColor = 'k';
    
    xlabel('Iteration');
    title('Covariance Evolution (X, Y, \theta separated)');
    grid on;
    legend('Location', 'best');
end