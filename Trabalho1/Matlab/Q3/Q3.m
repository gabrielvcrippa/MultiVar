syms x1 x2 x3 x4 u M m L g
% x1 = y, x2 = y', x3 = theta, x4 = theta'

% Parâmetros do pêndulo invertido
M = 1; 
m = 0.5;
L = 1; 
g = 9.81;

x0 = [0 0 pi/4 0]; % condição inicial do pendulo
Dsim = [zeros(4,1)];

%% 1) Representação em espaço de estados

% Dinâmica dos estados
f1 = x2;
f2 = (-m*g*sin(x3)*cos(x3) + m*L*(x4^2)*sin(x3)) / (M+m-m*cos(x3)^2);
f3 = x4;
f4 = g*sin(x3)/L - f2*cos(x3)/L;

% Dinâmica do controle
g1 = 0;
g2 = u/(M+m-m*cos(x3)^2);
g3 = 0;
g4 = -cos(x3)/L * u/(M+m-m*cos(x3)^2);


f = [f1; f2; f3; f4];
g = [g1; g2; g3; g4];
x = [x1, x2, x3, x4];

%% 2) Determinar PE (xe, ue)
solve(f+g);

%% 3) Encontrar sistema linearizado da Q2 para (xe,ue)=(0,0)
xe = [0, 0, 0, 0];

A = double(subs(jacobian(f), x, xe));
B = double(subs(jacobian(g), x, xe));

A = [zeros(4,1) A];
B = B(:,1);
C = [1 0 0 0];

% O sistema é observável em y = x3?
C2 = [0 0 1 0];
Obs2 = obsv(A,C2);
svd(Obs2); % Rank 2 < 4, não observável

%% 4) Determinar estabilidade do PE (xe,ue)=(0,0)

eig(A); % Os autovalores indicam ponto de sela (é instável)

% Simular com os 2 valores de condição inicial dos estados
%x0 = [0 0 0 0];
x0 = [0 0 pi/4 0];

%% 5) Projeto de controlador para seguir ref e rejeitar pert tipo degrau
% Com estados medidos

% Definindo beta(s) = s, portanto K=1, alfa0 = 0 e alfa1 = 1
M = 0; % KxK <--> 1x1
N = 1; % KxK <--> 1x1

% Xm' = Am Xm + Bm e
Am = M; % pk x pk
Bm = N; % pk x p
% Xm' = e, portanto Xm é integral do erro

% Modelo aumentado Xa' = [X' Xm']
Aa = [A zeros(4,1); -Bm*C Am];
Ba = [B; zeros(1)];

Cona = ctrb(Aa,Ba);
svd(Cona);

% Projeto de controle via LQR:
%Q = diag ([1 0.1 0.1 1 40]);
%R = 0.01;

Q = diag([1 1 1 1 1]);
R = 1;

Ka = lqr(Aa,Ba,Q,R);
K = Ka (1:4);
Km = Ka(5);

%% 6) Projeto de controlador com OBSERVADOR, sinais tipo degrau

x0obs = [0 0 0 0]; % condicao inicial do observador

% Projeto do ganho L via LQR:
Ql = diag ([1 50 0.01 100000]);
Rl = 0.001;
L = lqr(A',C',Ql,Rl)';
eig(A-L*C);


%Q = diag([1 1 1 1 1]);
%R = 0.001;

%Ka = lqr(Aa,Ba,Q,R);
%K = Ka (1:4);
%Km = Ka(5);

%% 7) Referência degrau, perturbação senoide w=0.2rad/s

% Degrau -> beta(s) = s
% Seno   -> beta(s) = (s^2 + w^2) = (s^2+0.04)

% beta(s) = s*(s^2+0.04) = s^3 + 0.04*s
alfa0 = 0;
alfa1 = 0.04;
alfa2 = 0;
alfa3 = 1;

Ms = [0 1 0; 0 0 1; -alfa0 -alfa1 -alfa2];
Ns = [0; 0; 1];

% Modelo interno p/ rastrear degrau e rejeitar senoide
Ams = Ms;
Bms = Ns;

% Modelo aumentado Xa' = [X' Xm']
Aas = [A zeros(4,3); -Bms*C Ams];
Bas = [B; zeros(3,1)];

Conas = ctrb(Aas,Bas);
svd(Conas);

% Projeto de controle via LQR:
Qs = diag ([1 1 1 1 1 1 1]);
Rs = 0.001;

Kas = lqr(Aas,Bas,Qs,Rs);
Ks = Kas (1:4);
Kms = Kas(5:7);