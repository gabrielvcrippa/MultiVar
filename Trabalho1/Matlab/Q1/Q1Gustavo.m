%% Dados do veículo
close all; clear all; clc;
syms x1 x2 x3 u;
mV = .430; % [kg] massa do veículo, excluindo giroscópio e objeto
mG = .190; % [kg] massa do giroscópio
rG = 2*.05; % [m] raio do giroscópio
aG = -1*.02; % [m] distância entre o centro de massa do giroscópio e ponto de rotação
aV = .1; % [m] altura do veículo
lV = .035; % [m] largura do veículo
dG = .1+.02;
dV = .1;
omega = 3000*10472; % rpm * conversion factor = rad/sec
g = 9.81; % [m/s^2] aceleração da gravidade
IG11 = (mG*(rG^2)/4) + (mG*(dG^2)/12); % IG22 = IG11
IG33 = (mG*(rG^2))/2;
IB11 = mV*((aV^2)+(lV^2))/12;

% Sistema linearizado na origem
a1 = omega*IG33/IG11;
a2 = -omega*IG33/(IB11+IG11+mV*(dV^2)+mG*(dG^2));
a3 = g*(mV*dV+mG*dG)/(IB11+IG11+mV*(dV^2)+mG*(dG^2));
b1 = 1/IG11;
f1 = x3;
f2 = 0;
f3 = ((mV*dV+mG*dG)*g*sin(x1))/((IB11+mV*(dV^2)+IG11*((cos(x2))^2))+mG*(dG^2)+IG33*((sin(x2))^2));
u1 = 0;
u2 = 1;
u3 = ((-2*cos(x2)*sin(x2)*x3*(IG33-IG11)-omega*cos(x2)*IG33))/((IB11+IG11*(cos(x2)^2)+mV*(dV^2)+mG*(dG^2)));
f = [f1;f2;f3];
u = [u1;u2;u3];
A = double(subs(jacobian(f),[x1 x2],[0 0]));
B = double(subs(u,[x1 x2],[0 0]));
C = [1 0 0];
D = 0;

%% Controle
close all;
tempo = 1;
Con = ctrb(A,B);
vsCon = svd(Con);
P1 = [-12 -12 -12];
K1 = acker(A,B,P1);
P2 = [-1.15 -1.175 -12];
K2 = place(A,B,P2);

%% Sistema Aumentado

Aa = [ A zeros(3,1);
    -C 0];
Ba = [B;0];
Cona = ctrb(Aa,Ba);
vsCona = svd(Cona);
%Ka = place(Aa,Ba,[-1.15 -1.175 -12 -12.05]);

