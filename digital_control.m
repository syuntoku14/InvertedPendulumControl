clear all; close all;
load inv_param.mat

Ts = 1/100;
sys_d = c2d(sys_ss, Ts, 'zoh');  % discrete system

% check the controllability
co = ctrb(sys_d);
ob = obsv(sys_d);

controllability = rank(co)  % controllable
observability = rank(ob)  % observable

A = sys_d.a;
B = sys_d.b;
C = sys_d.c;
D = sys_d.D;

Q = C'*C;
R = 1;

K = dlqr(A, B, Q, R);

Ac = (A-B*K);

states = {'x', 'x_dot', 'phi', 'phi_dot'};
inputs = {'r'};
outputs = {'x'; 'phi'};

sys_cl = ss(Ac, B, C, D, Ts, 'statename', states, 'inputname', inputs, 'outputname', outputs);

figure(1)
t = 0:0.01:5;
r = 0.2*ones(size(t));
[y, t, x] = lsim(sys_cl, r, t);
yyaxis left
plot(t, y(:, 1)); 
ylabel('cart position')
yyaxis right
plot(t, y(:, 2));
ylabel('pendulum angle')
title('Discrete system Response with LQR Control')
z = y(:, 1); theta = -y(:, 2);


% Fixed the responsibility
Q(1, 1) = 5000;
Q(3, 3) = 100;
K = dlqr(A, B, Q, R);
Ac = (A-B*K);
sys_cl = ss(Ac, B, C, D, Ts, 'statename', states, 'inputname', inputs, 'outputname', outputs);

figure(2)
t = 0:0.01:5;
r = 0.2*ones(size(t));
[y, t, x] = lsim(sys_cl, r, t);
yyaxis left
plot(t, y(:, 1)); 
ylabel('cart position')
yyaxis right
plot(t, y(:, 2));
ylabel('pendulum angle')
title('Discrete system Response with LQR Control(Fixed)')
z = y(:, 1); theta = -y(:, 2);


% Add a preconpenstor
N_bar = -61.55;  % this can be found by try and error
K = dlqr(A, B, Q, R);
Ac = (A-B*K);
sys_cl = ss(Ac, B*N_bar, C, D, Ts, 'statename', states, 'inputname', inputs, 'outputname', outputs);

figure(3)
t = 0:0.01:5;
r = 0.2*ones(size(t));
[y, t, x] = lsim(sys_cl, r, t);
yyaxis left
plot(t, y(:, 1)); 
ylabel('cart position')
yyaxis right
plot(t, y(:, 2));
ylabel('pendulum angle')
title('Discrete system Response with LQR Control(preconpensator)')
z = y(:, 1); theta = -y(:, 2);


% Observer Design

poles = eig(A-B*K)

P = [-0.2, -0.21, -0.22, -0.23];
L = place(A', C', P)'

% Same as before
