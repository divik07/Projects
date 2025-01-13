function T = LTOA_p( alpha_im1, a_im1, d_i, theta_i)
% creates transformation matrix from i to i-1
% i-1
%     T
%   1
% given 
%   a_(i-1)     link length
%   alpha_(i-1) link twist
%   d_i         joint offset
%   theta_i     joint angle

% assigned according to DH parameters

% all angles are assumed to be degrees, unless they are symbolic

if isnumeric(theta_i) || hasSymType(theta_i,'constant')
    t = deg2rad(theta_i);
else
    t = theta_i; % it's symbolic
end


ct = cos(t);
st = sin(t);

if isnumeric(alpha_im1) || hasSymType(alpha_im1,'constant')
    alph = deg2rad(alpha_im1);
else
    alph = alpha_im1; % it's symbolic
end

ca = cos(alph);
sa = sin(alph);

a = a_im1;
d = d_i;

T = [ ct       -st     0   a; ...
      st*ca   ct*ca  -sa -sa*d; ...
      st*sa   ct*sa   ca  ca*d; ...
      0         0      0   1];
end
