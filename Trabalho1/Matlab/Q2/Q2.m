s = tf('s');
% Parâmetros do pêndulo invertido
M = 1; 
m = 0.5;
L = 1; 
g = 9.81;

% Matrizes do modelo de estado
A = [0 1  0              0;
     0 0 -m*g/M          0;
     0 0  0              1;
     0 0 (m + M)*g/(M*L) 0];

B = [0; 1/M; 0; -1/(M*L)];

C = [1 0 0 0];

D = zeros(1,1);

Dsim = zeros(4,1); % usar no simulink como D

x0 = [0 0 deg2rad(51) 0]; % condição inicial do pendulo

%% 1) Determinar estabilidade interna da origem

eig(A); % Os autovalores indicam ponto de sela (é instável)

%% 2) Controlabilidade e observabilidade do sistema

Con = ctrb(A,B);
svd(Con); % Controlável

Obs = obsv(A,C);
svd(Obs); % Observável

% O sistema é observável em y = x3?
C2 = [0 0 1 0];
Obs2 = obsv(A,C2);
svd(Obs2); % NÃO observável

%% 3) Projetar controlador com modelo interno p/ degrau
% Teoria página 84

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
Q = diag ([1 0.1 0.1 1 40]);
R = 0.01;

Ka = lqr(Aa,Ba,Q,R);
K = Ka (1:4)
Km = Ka(5)

% Obter matriz de transferencia de malha fechada -> Lab5!!!!!!!!!!
E = B;
F = 0;

Ae = [A-B*K -B*Km; -Bm*C Am];
Be = [E zeros(4,1); -Bm*F Bm];
Ce = [C 0];

Gmf = Ce*(s*eye(5)-Ae)^-1*Be;

%% 4) Estimador de estados

x0obs = [0 0 0 0]; % condicao inicial do observador

% Projeto do ganho L via LQR:
Ql = diag ([1 50 0.01 1000000]);
Rl = 0.001;
L = lqr(A',C',Ql,Rl)';
eig(A-L*C)


% Obter matriz de transferencia de malha fechada com estimador
E = B;
F = 0;

Aetil = [A -B*Km -B*K; -Bm*C Am zeros(1,4); L*C -B*Km A-B*K-L*C];
Betil = [E zeros(4,1); -Bm*F Bm; L*F zeros(4,1)];
Cetil = [C zeros(1,5)];

GmfL = Cetil*(s*eye(9)-Aetil)^-1*Betil;

%% 5) Referência degrau, perturbação senoide w=0.2rad/s
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

% Obter matriz de transferencia de malha fechada -> Lab5!!!!!!!!!!
E = B;
F = 0;

Aes = [A-B*Ks -B*Kms; -Bms*C Ams];
Bes = [E zeros(4,1); -Bms*F Bms];
Ces = [C zeros(1,3)];

Gmfs = Ces*(s*eye(7)-Aes)^-1*Bes;