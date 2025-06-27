%% Questão 1
clear all;
syms x1 x2 x3 x4 u real;

x = [x1;x2;x3;x4];

mV = 0.210;  % Massa do veículo sem giro [kg]
mG = 0.123;  % Massa do giro [kg]
mC = 0;
rG = 0.046;  % Raio do giro [m]
aG = 0.005;  % Espessura giro [m]
aV = 0.088;  % Altura veículo [m]
lV = 0.175;  % Largura veículo [m]
dG = 0.054;  % Distância entre centro de massa do giro e eixo de rotação [m]
dV = 0.025;  % Distância entre centro de massa do veículo e eixo de rotação[m]
omega = 2 * 3657 * 0.10472; % Velocidade de rotação do giro [rad/s]
g = 9.81;    % Gravidade [m/s^2]
d1 = 0.025;
d2 = 0.054;

yL = 0;
zL = 0;
mL = 0;

g = 9.81; % [m/s^2] aceleração da gravidade
IG11 = (mG*(rG^2)/4) + (mG*(dG^2)/12); % IG22 = IG11
IG33 = (mG*(rG^2))/2;
IB11 = (1/12) * mV * (lV^2 + aV^2); % eixo X (roll)

IG22 = IG11;
IL11 = 0;

k1 = IB11 + IL11;
k4 = IG11;     % (IC11 + IG11), IC11 = 0
k5 = IG22;     % (IC22 + IG22), IC22 = 0
k6 = IG33;     % (IC33 + IG33), IC33 = 0

k7 = d1 * mV + d2 * (mC + mG);
k8 = d1^2 * mV + d2^2 * (mC + mG);
k9 = k1 + k8 + (yL^2 + zL^2) * mL;
k10 = k4 - k6;

% Symbolic dynamics
c = cos(x2);

M_u = IG33 * omega * c * u;

% State-space model
f = [
    x3;
    x4;
    (k7*g*sin(x1)+2*x3*x4*sin(x2)*cos(x2)*k10-x4*cos(x2)*omega*IG33)/(k9+cos(x2)^2*k4+sin(x2)^2*k6);
    (M_u-x3^2*cos(x2)*sin(x2)*k10-x3*omega*cos(x2)*IG33)/(k5);
];

% Cálculo do par de equilíbrio
xe=[0;0;0;0];
ue= 0;

% Cálculo das matrizes do sistema linearizado
A = subs(subs(jacobian(f,x),x,xe),u,ue);
B = subs(jacobian(f,u), x, xe);

% Cálculo das matrizes do sistema discretizado
Ts = 5e-3;
[Ad,Bd] = c2d(double(A),double(B),Ts);
C = [1 0 0 0;
     0 1 0 0;
     0 0 1 0];

%% Item 1
Am = [0 0 1; 0 0 0; 92.8581 0 0];
Bm = [0; 1; -79.361];
Cm = eye(3);
Dm = zeros(3,1);

x0 = [pi/3 0 0]; % Condicao inicial -> [ρ θ ρ']

% 1) Planta discretizada

Tsm = 11e-3;
[Adm,Bdm] = c2d(Am,Bm,Tsm);

Cond = ctrb(Ad,Bd);
svd(Cond);

% Controle dLQR
Qm = diag([48, 25, 25]);
Rm = 90;
Kdm = dlqr(Adm,Bdm,Qm,Rm);
eig(Adm-Bdm*Kdm)

%% Item 3
% x = [psi alfa psi_dot alfa_dot]
% u = x4

% Controlabilidade e Observabilidade do sistema linearizado
Con = ctrb(Ad,Bd);
rCon = svd(Con);
Ob = obsv(Ad,C);
rOb = svd(Ob);

% Cálculo do Ganho Kd
Qd = diag([80, 10, 15, 3]);
Rd = 20;
Kd = dlqr(Ad,Bd,Qd,Rd);
eig(Ad-Bd*Kd)

% Cálculo do Ganho Ld
V1 = diag([0.01, 0.01, 0.1, 0.5]);  % modelo bem estimado, mas precessão menos certa
V2 = 1e-4*diag([0.001, 0.001, 0.05]); 
Ld = dlqr(Ad',C',V1,V2)';
eig(Ad-Ld*C)
x0obs = [deg2rad(0);0;0;0];

%% Item 4

%Cálculo dos ganhos utilizado no filtro de Kalman
Qkf = diag([10,5,8,1]);
Rkf = 90;
Kkf = dlqr(Ad,Bd,Qkf,Rkf);
x0kf = [0;0;0;0];
P0kf = 10*eye(4);
eig(Ad-Bd*Kkf)

%Cálculo dos ganhos utilizado no filtro de Kalman estendido
Ak = eye(4) + Ts * jacobian(f,x);
Qekf = diag([25,5,25,10]);
Rekf = 9;
Kekf = dlqr(Ad,Bd,Qekf,Rekf);
x0ekf = [deg2rad(0);0;0;0];
P0ekf = 1e8*eye(4);
eig(Ad-Bd*Kekf)