clear all;
close all;
load('inv_param.mat')

figure(1)
%-------------
% Native system
subplot(2, 2, 1)
rlocus(P_pend);
title('Root Locus of Plant(under Proportional Control)');

%-------------
% Add s to controller so that cancel the zero of plant.
subplot(2, 2, 2)
C = 1/s;
rlocus(P_pend * C);
title('Root Locus of Plant(under Integration Control)');

%------------
% Add 1/s to controller so that cancel the zero of plant.
subplot(2, 2, 3)
C = zpk(-0.1, 0, 1);
rlocus(P_pend * C);
title('Root Locus of Plant(under Deviation Control)');

%------------
% Add PID controller.
subplot(2, 2, 4)
z = [-3, -4];
p = 0;
k = 1;
C = zpk(z, p, k);
rlocus(P_pend * C);
title('Root Locus of Plant(under PID Control)');

%-----------
% By rlocfind, we can find the gain which meets our desired system
% requirement. And we got K = 20. Let's see what will happen on the system.
figure(2)
t = 0:0.01:8.0;
K = 20;
T = feedback(P_pend, K * C);
T2 = feedback(1, P_pend * C * K) * P_cart;
theta = -impulse(T, t);
z = impulse(T2, t);
subplot(2, 1, 1);
plot(t, z);
ylabel('z');
subplot(2, 1, 2);
plot(t, theta);
ylabel('theta');