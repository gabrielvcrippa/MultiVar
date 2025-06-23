%% Q2 Item1 -> Visualização da estabilidade da origem

% Exibição dos estados
t  = out.av(:,1); % tempo
x1 = out.av(:,2); % posição do carrinho
x2 = out.av(:,3); % velocidade do carrinho
x3 = out.av(:,4); % ângulo do pêndulo
x4 = out.av(:,5); % velocidade angular

figure(1)
plot(t, x1, 'LineWidth', 2, 'DisplayName', 'x1');
hold on;
plot(t, x2, 'LineWidth', 2, 'DisplayName', 'x2');
plot(t, x3, 'LineWidth', 2, 'DisplayName', 'x3');
plot(t, x4, 'LineWidth', 2, 'DisplayName', 'x4');
hold off;

grid on;
ylim([-10 10]);

xlabel('Tempo (s)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('X', 'FontSize', 12, 'FontWeight', 'bold');

legend('show', 'FontSize', 12, 'Location', 'best');
title('Estados do pêndulo', 'FontSize', 14, 'FontWeight', 'bold');
set(gca, 'FontSize', 11, 'FontWeight', 'bold');



%% Q2 Item3 -> Resultado do controlador

t  = out.desempenho(:,1); % tempo
w = out.desempenho(:,2); % perturbação
y = out.desempenho(:,3); % saída
r = out.desempenho(:,4); % referência

figure(1)
plot(t, y, 'LineWidth', 2, 'DisplayName', 'y');
hold on;
plot(t, r, 'LineWidth', 1, 'DisplayName', 'r');
plot(t, w, 'LineWidth', 1, 'DisplayName', 'w');
hold off;

grid on;
ylim([-2 2]);

xlabel('Tempo (s)', 'FontSize', 12, 'FontWeight', 'bold');
%ylabel('X', 'FontSize', 12, 'FontWeight', 'bold');

legend('show', 'FontSize', 12, 'Location', 'best');
title('Desempenho do controlador', 'FontSize', 14, 'FontWeight', 'bold');
set(gca, 'FontSize', 11, 'FontWeight', 'bold');


% Dados filtrados para t >= 6.5 segundos
idx = t >= 7;  % Índices onde tempo >= 6.5s
t_filtrado = t(idx);
y_filtrado = y(idx);
r_filtrado = r(idx);
w_filtrado = w(idx);

% Figure 2
figure(2)
plot(t_filtrado, y_filtrado, 'LineWidth', 2, 'DisplayName', 'y');
hold on;
plot(t_filtrado, r_filtrado, 'LineWidth', 1, 'DisplayName', 'r');
plot(t_filtrado, w_filtrado, 'LineWidth', 1, 'DisplayName', 'w');
hold off;

grid on;
ylim([-0.5 1.5]);  % Mantém a mesma escala do eixo y para comparação

% Labels e título
xlabel('Tempo (s)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Amplitude', 'FontSize', 12, 'FontWeight', 'bold');  % Adicionei um label mais descritivo
legend('show', 'FontSize', 12, 'Location', 'best');
title('Desempenho do Controlador (t \geq 6.5 s)', 'FontSize', 14, 'FontWeight', 'bold');
set(gca, 'FontSize', 11, 'FontWeight', 'bold');



%% Q2 Item4 -> Resultado do controlador com estimador de estados

t  = out.desempenhoL(:,1); % tempo
w = out.desempenhoL(:,2); % perturbação
y = out.desempenhoL(:,3); % saída
r = out.desempenhoL(:,4); % referência

figure(1)
plot(t, y, 'LineWidth', 2, 'DisplayName', 'y');
hold on;
plot(t, r, 'LineWidth', 1, 'DisplayName', 'r');
plot(t, w, 'LineWidth', 1, 'DisplayName', 'w');
hold off;

grid on;
ylim([-2 4]);

xlabel('Tempo (s)', 'FontSize', 12, 'FontWeight', 'bold');
%ylabel('X', 'FontSize', 12, 'FontWeight', 'bold');

legend('show', 'FontSize', 12, 'Location', 'best');
title('Desempenho do controlador', 'FontSize', 14, 'FontWeight', 'bold');
set(gca, 'FontSize', 11, 'FontWeight', 'bold');


% Dados filtrados para t >= 6.5 segundos
idx = t >= 7;  % Índices onde tempo >= 6.5s
t_filtrado = t(idx);
y_filtrado = y(idx);
r_filtrado = r(idx);
w_filtrado = w(idx);

% Figure 2
figure(2)
plot(t_filtrado, y_filtrado, 'LineWidth', 2, 'DisplayName', 'y');
hold on;
plot(t_filtrado, r_filtrado, 'LineWidth', 1, 'DisplayName', 'r');
plot(t_filtrado, w_filtrado, 'LineWidth', 1, 'DisplayName', 'w');
hold off;

grid on;
ylim([-0.5 4]);  % Mantém a mesma escala do eixo y para comparação

% Labels e título
xlabel('Tempo (s)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Amplitude', 'FontSize', 12, 'FontWeight', 'bold');  % Adicionei um label mais descritivo
legend('show', 'FontSize', 12, 'Location', 'best');
title('Desempenho do Controlador (t \geq 6.5 s)', 'FontSize', 14, 'FontWeight', 'bold');
set(gca, 'FontSize', 11, 'FontWeight', 'bold');



%% Q2 Item5 -> Controle degrau + senoide

t  = out.desempenhoS(:,1); % tempo
w = out.desempenhoS(:,2); % perturbação
y = out.desempenhoS(:,3); % saída
r = out.desempenhoS(:,4); % referência

figure(1)
plot(t, y, 'LineWidth', 2, 'DisplayName', 'y');
hold on;
plot(t, r, 'LineWidth', 1, 'DisplayName', 'r');
plot(t, w, 'LineWidth', 1, 'DisplayName', 'w');
hold off;

grid on;
ylim([-1.5 2]);

xlabel('Tempo (s)', 'FontSize', 12, 'FontWeight', 'bold');
%ylabel('X', 'FontSize', 12, 'FontWeight', 'bold');

legend('show', 'FontSize', 12, 'Location', 'best');
title('Desempenho do controlador', 'FontSize', 14, 'FontWeight', 'bold');
set(gca, 'FontSize', 11, 'FontWeight', 'bold');


% Dados filtrados para t >= 6.5 segundos
idx = t >= 7;  % Índices onde tempo >= 6.5s
t_filtrado = t(idx);
y_filtrado = y(idx);
r_filtrado = r(idx);
w_filtrado = w(idx);