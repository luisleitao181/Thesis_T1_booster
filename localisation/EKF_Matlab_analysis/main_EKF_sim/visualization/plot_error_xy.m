%% plot_error_xy.m
function plot_error_xy(x_t, x_e_t)
    hold on; grid on;
    
    % Calcular erros separados
    erro_x = abs(x_t(1,:) - x_e_t(1,:));  % Erro em X
    erro_y = abs(x_t(2,:) - x_e_t(2,:));  % Erro em Y
    
    % Plot ambos
    plot(erro_x, 'b-', 'LineWidth', 2, 'DisplayName', 'Error X');
    plot(erro_y, 'r-', 'LineWidth', 2, 'DisplayName', 'Error Y');
    
    xlabel('Iteration'); 
    ylabel('Position Error (m)');
    title('Position Error (X and Y)');
    legend('Location', 'best');
    
    ylim([0, max([max(erro_x), max(erro_y)])*1.1]);
end