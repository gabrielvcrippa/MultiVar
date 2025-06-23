%% Q3 Item4 -> Estabilidade do equilíbrio (xe,ue)

% Exibição dos estados
t  = out.av(:,1); % tempo
x1 = out.av(:,2); % posição do carrinho
x2 = out.av(:,3); % velocidade do carrinho
x3 = out.av(:,4); % ângulo do pêndulo
x4 = out.av(:,5); % velocidade angular

figure(1)
plot(t,x1);
hold on;
plot(t,x2);
plot(t,x3);
plot(t,x4);
ylim([-10 10]);


%% Q3 Item5 -> Controlador

% Exibição dos estados
t  = out.desempenho(:,1); % tempo
w = out.desempenho(:,2); % perturbação
y = out.desempenho(:,3); % saída
yL = out.desempenho(:,5); % saída linearizado
r = out.desempenho(:,4); % referência

figure(1)
plot(t, y, 'LineWidth', 2, 'DisplayName', 'y');
hold on;
plot(t, yL, 'LineWidth', 2, 'DisplayName', 'y linearizado');
plot(t, r, 'LineWidth', 1, 'DisplayName', 'r');
plot(t, w, 'LineWidth', 1, 'DisplayName', 'w');
grid on;
ylim([-3 3]);
xlabel('Tempo (s)', 'FontSize', 12, 'FontWeight', 'bold');
%ylabel('X', 'FontSize', 12, 'FontWeight', 'bold');
legend('show', 'FontSize', 12, 'Location', 'best');
title('Desempenho do controlador', 'FontSize', 14, 'FontWeight', 'bold');
set(gca, 'FontSize', 11, 'FontWeight', 'bold');


