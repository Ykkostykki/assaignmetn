function [g, Cr] = g_vec(qi, qj,qc, revolute, simple, t, q, qp)
%revolute joint acceleration equation between two bodies i and j
%qi and qj are body coordinates
%qc is the loaction of revolute joint
q0 = [qi;qj];
Ai = [cos(qi(3)) -sin(qi(3))
    sin(qi(3)) cos(qi(3))];

Aj = [cos(qj(3)) -sin(qj(3))
    sin(qj(3)) cos(qj(3))];
commonpoint = [qc(1)
                qc(2)];

% Cr = ri + 2
si = Ai*    commonpoint ; %global components
sj = Aj*    commonpoint ; %global components
ri = [qi(1)
    qi(2)]+si;   %body 1 coordinates
rj = [qj(1)
    qj(2)]+sj ;   %body 2 coordinates

C_r = ri + rot(qi(3)) * si - rj - rot(qj(3)) * sj;
Cr = ri+Ai*si - rj-Aj*sj;
% C_r_dq = [eye(2), omega * rot(qi(3)) * si, ...
%     -eye(2), -omega * rot(qj(3)) * sj]

% rb = [qi(1)
%     qi(2)]+sb  %global coordinates for i
g = diff(C_r)
g2 = diff(diff(Cr))
qfori = [ri.' qi(3)]';
qforj = [rj.' qj(3)]';
qi_ii = diff(diff(qfori));
qj_ii = diff(diff(qfori));
g = [qi_ii; qj_ii];
% qi = [q1
%         phi_l]
% qidot = diff(qi)
end