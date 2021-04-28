clear all
angle = 270;
% values from slider crank example
q1 = [0; 0; 0];
q2 = [-0.1 * cosd(angle)
    0.1 * sind(angle)
    -deg2rad(angle)];
q3 = [-0.2 * cosd(angle)
    0.2 * sind(angle)
    -deg2rad(angle)];

h_B = 0.2 * sind(angle); % y coordinate of point B
phi_l = asin(h_B / 0.5); % link's angle
q4 = [-0.2 * cosd(angle) - 0.3 * cos(phi_l)
    h_B - 0.3 * sin(phi_l)
    phi_l];
q_0 = [q1; q2; q3];
%% driving constraints and 
revolute(1).i = 1;
revolute(1).j = 2;
revolute(1).s_i = [0; 0];
revolute(1).s_j = [0.1; 0];

revolute(2).i = 2;
revolute(2).j = 3;
revolute(2).s_i = [-0.1; 0];
revolute(2).s_j = [0; 0];

simple(1).i = 1;
simple(1).k = 1;
simple(1).c_k = 0;

simple(2).i = 1;
simple(2).k = 2;
simple(2).c_k = 0;

simple(3).i = 1;
simple(3).k = 3;
simple(3).c_k = 0;

driving.i = 2;
driving.k = 3;
driving.d_k = @(t) deg2rad(angle) -  deg2rad(60) * t;
driving.d_k_t = @(t) -1.2;
driving.d_k_tt = @(t) 0;
%%
C_fun = @(t, q) constraint(revolute, simple, driving, t, q);
[T, Q] = position_fsolve(C_fun, 3, q_0, 0.1);

[g, Cr] = g_vec(q1, q2, q3, [], [], [], [], [])

%% Some verification plots
plot(Q(:, 4), Q(:, 5), ...
    Q(:, 7), Q(:, 8), ...
    Q(:, 1), Q(:, 2), ...
    0, 0, '*', 'LineWidth', 2);
axis equal