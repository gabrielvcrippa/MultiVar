%% Questão 2
% x = [theta1 theta2 theta1_dot theta2_dot]
% u = [tau1 tau2]

clear all;
syms x1 x2 x3 x4 u1 u2 y1 y2;

x = [x1;x2;x3;x4];
u = [u1;u2];

g = 9.81;
l1 = 1;
l2 = 1;

M = [3+2*cos(x2) 1+cos(x2);
     1+cos(x2) 1];

C = [-x4*(2*x3+x4)*sin(x2);
     x3^2*sin(x2)];

K = [2*g*sin(x1)+g*sin(x1+x2);
     g*sin(x1+x2)];

f = [x3;x4;-(M)\(C+K)] + [zeros(2,2); inv(M)]*u;

%% Item a
y1 = l1*cos(x1) + l2*cos(x1+x2);
y2 = l1*sin(x1) + l2*sin(x1+x2);
y = [y1;y2];

y_1 = jacobian(y,x)*f;
y_2 = jacobian(y_1,x)*f;

a = subs(y_2,u,[0;0]);
A = jacobian(y_2,u);

Agab = [(sin(x1)-sin(x1+2*x2))/(-3+cos(2*x2)), (-sin(x1)+3*sin(x1+x2)-sin(x1-x2)+sin(x1+2*x2))/(-3 +cos(2*x2));
         (-cos(x1)+cos(x1+2*x2))/(-3+cos(2*x2)), (cos(x1)-3*cos(x1+x2)+cos(x1-x2)-cos(x1+2*x2))/(-3+cos(2*x2))];

resp = simplify(A-Agab);

solve(det(A))