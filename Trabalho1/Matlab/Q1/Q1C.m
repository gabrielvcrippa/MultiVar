clear all; clc;

A = [0 0 1; 0 0 0; 92.8581 0 0];
B = [0; 1; -79.361];
C = eye(3);
D = zeros(3,1);

% Condicao inicial da moto -> [ρ θ ρ'], onde ρ é a inclinação da moto
x0 = [pi/3 0 0];

Con = ctrb(A,B);
svd(Con);


%% Modelo interno
Am = 0;
Bm = 1;

% Sistema aumentado
C = [1 0 0];
Aa = [A zeros(3,1); -Bm*C Am];
Ba = [B; zeros(1)];

Cona = ctrb(Aa,Ba);
svd(Cona);

%% Controle alocação de polos
%polos = [-9 -9.05 -9.1];
%K=place(A,B,polos);

%% Controle LQR
Q = diag([10, 1, 2]);
R = 3;
K = lqr(A,B,Q,R);
eig(A-B*K)

%% Observabilidade
C1 = [1 0 0];
C2 = [0 1 0];
C3 = [0 0 1];

Obs1 = obsv(A,C1);
Obs2 = obsv(A,C2);
Obs3 = obsv(A,C3);
% Nenhum par(A,C) é observável usando a saída como 1 estado apenas

%C = [1 0 0; 0 1 0];
%C = [0 1 0; 0 0 1];
%Obs = obsv(A,C); % y = [x1, x2] é observável

% Ganhos do observador
Ql = diag([1, 100, 100]);
Rl = 0.001;
L = lqr(A',C',Ql,Rl)';
eig(A-L*C)



%% Q1_5)

% Exibição dos estados
%t  = out.tempo(:); % tempo  
%x1 = out.ro(:); % ro
%x2 = out.theta(:); % theta
%x3 = out.dro(:); % ro'
%x1e = out.roe(:); % ro_e
%x2e = out.thetae(:); % theta
%x3e = out.droe(:); % ro'

%figure(1)
%plot(t, x1, 'LineWidth', 1, 'DisplayName', 'x1');
%hold on;
%plot(t, x2, 'LineWidth', 1, 'DisplayName', 'x2');
%plot(t, x3, 'LineWidth', 1, 'DisplayName', 'x3');
%plot(t, x1e, 'LineWidth', 1, 'DisplayName', 'x1e', 'LineStyle','--');
%plot(t, x2e, 'LineWidth', 1, 'DisplayName', 'x2e', 'LineStyle','--');
%plot(t, x3e, 'LineWidth', 1, 'DisplayName', 'x3e', 'LineStyle','--');
%grid on;
%ylim([-3 3]);
%xlim([0 2]);
%xlabel('Tempo (s)', 'FontSize', 12, 'FontWeight', 'bold');
%ylabel('X', 'FontSize', 12, 'FontWeight', 'bold');
%legend('show', 'FontSize', 12, 'Location', 'best');
%title('Estabilização com observador', 'FontSize', 14, 'FontWeight', 'bold');
%set(gca, 'FontSize', 11, 'FontWeight', 'bold');