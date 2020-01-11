clear all;
close all;
load('inv_param.mat')

% An eigen value is plus so this system is unstable.
poles = eig(A)

% Judge the controllability
co = ctrb(sys_ss)
controrability = rank(co)  % The rank is 4. Controllable.

% We will use LQR method to define feed back gain.
% First, we have to set the weight R and Q
R = 1;
Q = C' * C;
Q(1, 1) = 5000;  % cart position weight
Q(3, 3) = 100;  % angle weight

% Then, the feedback gain K can be obtained.
K = lqr(A, B, Q, R);

% Rebuild the system with the feedback gain K.
figure(1)
Ac = [(A-B*K)];
Bc = [B];
Cc = [C];
Dc = [D];

states = {'x' 'x_dot' 'phi' 'phi_dot'};
inputs = {'r'};
outputs = {'x'; 'phi'};

sys_cl = ss(Ac,Bc,Cc,Dc,'statename',states,'inputname',inputs,'outputname',outputs);

t = 0:0.01:5;
r = 0.2*ones(size(t));
[y, t, x] = lsim(sys_cl, r, t);
yyaxis left
plot(t, y(:, 1)); 
ylabel('cart position')
yyaxis right
plot(t, y(:, 2));
ylabel('pendulum angle')
title('Step Response with LQR Control')
z = y(:, 1); theta = -y(:, 2);
%  This system is not satisfactory. The cart position ends in oposit
%  direction and there are too much overshoot.


% We need to add a constant gain N after the reference.
figure(2)
Cn = [1 0 0 0];
sys_ss = ss(A, B, Cn, 0);
Nbar = rscale(sys_ss, K);

sys_cl = ss(Ac, Bc*Nbar, Cc, Dc, 'statename', states ...
    ,'inputname', inputs, 'outputname', outputs);
t = 0:0.01:5;
r = 0.2*ones(size(t));
[y, t, x] = lsim(sys_cl, r, t);
yyaxis left
plot(t, y(:, 1)); 
ylabel('cart position')
yyaxis right
plot(t, y(:, 2));
ylabel('pendulum angle')
title('Step Response with LQR Control and Precompansation')
z = y(:, 1); theta = -y(:, 2);

% We have done well!
% Let's enjoy more with it.
figure(3)
t1 = 0:0.01:2.49;
t2 = 2.5:0.01:4.99;
t3 = 5:0.01:7.5;
t = horzcat(t1, t2, t3);

r1 = 0.2 * ones(size(t1));
r2 = -0.1 * ones(size(t2));
r3 = -1.0 * ones(size(t3));
r = horzcat(r1, r2, r3);

[y, t, x] = lsim(sys_cl, r, t);
yyaxis left
plot(t, y(:, 1)); 
ylabel('cart position')
yyaxis right
plot(t, y(:, 2));
ylabel('pendulum angle')
title('Random Step Response with LQR Control and Precompansation')
z = y(:, 1); theta = -y(:, 2);


% The model so far is based on the assumption that the...
% full-state can be observed. Next, we will design a new
% model with observer.

figure(4)

% Verifying the system is observable.

ob = obsv(sys_ss);
observability = rank(ob)

% Next we will design the obserber. The poles of it should be 
% 4-10 times faster than the slowest controller pole. Based on this logic,
% we first have to find the controller poles.

poles = eig(Ac)  % The slowest is about -4.75

% So we will design the poles of the estimator as -40

P = [-40, -41, -42, -43];
L = place(A', C', P)';

% Then, design a new state-space system

Ace = [(A-B*K), B*K; zeros(size(A-L*C)), (A-L*C)];
Bce = [B*Nbar; zeros(size(B))];
Cce = [Cc zeros(size(Cc))];
Dce = [0; 0];

states = {'x' 'x_dot' 'phi' 'phi_dot' 'e1' 'e2' 'e3' 'e4'};
inputs = {'r'};
outputs = {'x'; 'phi'};

sys_est_cl = ss(Ace,Bce,Cce,Dce,'statename',states,'inputname',inputs,'outputname',outputs);

[y, t, x] = lsim(sys_est_cl, r, t);
yyaxis left
plot(t, y(:, 1)); 
ylabel('cart position')
yyaxis right
plot(t, y(:, 2));
ylabel('pendulum angle')
title('Random Step Response with LQR Control and Precompansation')
z = y(:, 1); theta = -y(:, 2);


