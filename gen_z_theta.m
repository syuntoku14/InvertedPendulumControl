function [z, theta] = gen_z_theta(response, sys, t)
% Summery: 
%   Return z and theta required for cdip_anime.
%   response: Input function such as @impulse.

%   If the sys returns only phi data(In other words, sys is SISO),
%   returned z will be false.

[output, t] = response(sys, t);
try
    z = output(:, 1);
    phi = output(:, 2);
catch
    phi = output;
    z = zeros(size(phi));
theta = -phi;

end

