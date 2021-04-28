%dynamic draft
clear all
body(1).m = 2; % mass equals to one kg
l = 0.2; 
body(1).Ic = body(1).m * l^2 / 12; % mass moment of inertia along center of mass in kgm2
body(1).q = [0;0;0]; %initial coordinates
grav = [0; -9.81]; % gravitational acceleration
tspan = 0 : 0.1 : 5;
% body(2).m = 3; % mass equals to one kg
% l = 0.2; 
% body(2).Ic = body(2).m * l^2 / 12; % mass moment of inertia along center of mass in kgm2
% body(2).q = [0;3;0]; %initial coordinates
% 
% M1 = diag(mass_matrix(body(1)));
% M2 = diag(mass_matrix(body(2)));
% M3 = [M1 
%     M2];
% 
% Msys = diag(M3);
% Qsys = [body(1).q
%         body(2).q];
% force_vector = Msys*Qsys;
%% revolute joint and simple
% revolute(1).i = 1;
% revolute(1).j = 2;
% revolute(1).s_i = [0; 0];
% revolute(1).s_j = [0.1; 0];
% 
% simple(1).i = 1;
% simple(1).k = 1;
% simple(1).c_k = 0;
% 
% simple(2).i = 1;
% simple(2).k = 2;
% simple(2).c_k = 0;
% 
% simple(3).i = 1;
% simple(3).k = 3;
% simple(3).c_k = 0;
%%
% 
q0 = system_coordinates(body);
M = mass_matrix(body);
% C = constraint(revolute, simple, [], [], q0);
% Cq = constraint_dq(revolute, simple, [], [], q0);
% A = [Msys, Cq'
%     Cq, zeros(length(C))];
% Cdot = Cq * q0
% [T, Y] = ode45(@odefun, tspan, y0);
% dydt = odefun(t, y)

%%

% sforce.f =@(t) [1*t; 0];
sforce.f = [5; 0];
sforce.i = 1;
sforce.u_i = [0; 0.1];
F2 = force_vector(grav, sforce, body, q0);
acc_f2 = @(t, q, qp) system_accelerations(t, q, qp, M, sforce, grav, body);
[t2, u2, v2] = EulerCromer(acc_f2, 5, q0, zeros(size(q0)), 0.001);

hold on
figure(1)
plot(t2, u2(:, 3))
plot(t2, u2(:, 1))
plot(t2, u2(:, 2))
legend({'angle','x-coordinate','y-coordinate'},'Location','southwest')
hold off

figure(2)
plot(u2(:,1),u2(:,2))
title('position')