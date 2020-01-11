clear all;
close all;

load('inv_param.mat')

% zeros and polese of the unstable system
[zeros poles] = zpkdata(P_cart, 'v')


%-----------------------
% impulse response of inversed pendulum
figure(1);
t = 0:0.001:1.0;
[output, t] = impulse(sys_tf, t);
impulse(sys_tf, t)
z = output(:, 1);
phi = output(:, 2);
theta = -phi;

title('Open-Loop Impulse Response')

%-----------------------
% step response of inversed pendulum
figure(2);
u = ones(size(t));
[output, t] = lsim(sys_tf, u, t);
z = output(:, 1);
phi = output(:, 2);
theta = -phi;

subplot(2, 1, 1)
plot(t, z);
ylabel('x')
subplot(2, 1, 2)
title('Open-Loop Step Response')

plot(t, phi)
ylabel('phi')
xlabel('time t')
%-----------------------


