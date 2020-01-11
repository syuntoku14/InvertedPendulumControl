close all;
clear all;
load('inv_param.mat')

% controlSystemDesigner('bode', minreal(P_pend*(1/s)))

%------
% The following controller is obtained as a result of controlSytemDesigner

t = 0:0.01:10;

figure(1)
C = zpk([-1, -1], 0, 10);
T = feedback(P_pend, C);
impulse(T, t); grid;
title('Before Adjusting Gain and Zero Position')

figure(2)
C = zpk([-1, -2], 0, 35);
T = feedback(P_pend, C); 
impulse(T, t); grid;
title('After Adjusting Gain and Zero Position')

figure(3)
T2 = feedback(1, P_pend*C) * P_cart;
T2 = minreal(T2);
impulse(T2, t); grid;
title('Cart position')

theta = - impulse(T, t);
z = impulse(T2, t);