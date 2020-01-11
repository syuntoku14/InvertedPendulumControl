close all;
clear all;
load('inv_param.mat')

% Define the PID Controller
t = 0:0.01:5;

%---------------------
subplot(3, 1, 1);
Kp = 1;
Kd = 1;
Ki = 1;

C = pid(Kp, Ki, Kd);

T = feedback(P_pend, C);
T2 = feedback(1, C*P_pend) * P_cart;

impulse(T, t);  % This is unstable.
% ylim([-0.05, 0.05]);
title(['[Kp, Kd, Ki] = ', '[', num2str(Kp), ', '...
    , num2str(Kd), ', ', num2str(Ki), ']' ]);

%--------------------
subplot(3, 1, 2);
Kp = 100;

C = pid(Kp, Ki, Kd);

T = feedback(P_pend, C);
T2 = feedback(1, C*P_pend) * P_cart;

impulse(T, t); 
ylim([-0.05, 0.05]);
title(['[Kp, Kd, Ki] = ', '[', num2str(Kp), ', '...
    , num2str(Kd), ', ', num2str(Ki), ']' ]);

%-------------------
subplot(3, 1, 3);
Kp = 100;
Kd = 20;
C = pid(Kp, Ki, Kd);

T = feedback(P_pend, C);
T2 = feedback(1, P_pend * C) * P_cart;

impulse(T, t);
ylim([-0.05, 0.05]);
title(['[Kp, Kd, Ki] = ', '[', num2str(Kp), ', '...
    , num2str(Kd), ', ', num2str(Ki), ']' ]);
[z, theta] = gen_animeData(@impulse, T, t);
z = impulse(T2, t);