function [dF,dJ] = gradEb_hessEb3D(node0, node1, node2, ...
   curvature0, l_k, EI)
%
% This function returns the derivative of bending energy E_k^b with respect
% to x_{k-1}, y_{k-1}, z_{k-1}, x_k, y_k, z_k, x_{k+1}, y_{k+1}, and z_{k+1}.
%
% curvature0 is the "discrete" natural curvature [dimensionless] at node (xk, yk, zk).
% l_k is the voronoi length of node (xk, yk, zk).
% EI is the bending stiffness.
%

kappaBar = curvature0;

m1e = [0 1 0]; 
m1f = [0 1 0];
m2e = [1 0 0];
m2f = [1 0 0];

%% Computation of gradient of the two curvatures
gradKappa = zeros(9, 1);

ee = node1 - node0;
ef = node2 - node1;

norm_e = norm(ee);
norm_f = norm(ef);

te = ee / norm_e;
tf = ef / norm_f;

% Curvature binormal
kb = 2.0 * cross(te, tf) / (1.0 + dot(te, tf));

chi = 1.0 + dot(te, tf);
tilde_t = (te + tf) / chi;
tilde_d1 = (m1e + m1f) / chi;
tilde_d2 = (m2e + m2f) / chi;

% Curvatures
kappa1 = 0.5 * dot( kb, m2e + m2f); % CHECKED
kappa2 = -0.5 * dot( kb, m1e + m1f); % CHECKED

Dkappa1De = 1.0 / norm_e * (-kappa1 * tilde_t + cross(tf,tilde_d2));
Dkappa1Df = 1.0 / norm_f * (-kappa1 * tilde_t - cross(te,tilde_d2));

Dkappa2De = 1.0 / norm_e * (-kappa2 * tilde_t - cross(tf,tilde_d1));
Dkappa2Df = 1.0 / norm_f * (-kappa2 * tilde_t + cross(te,tilde_d1));

gradKappa(1:3, 1) = -Dkappa1De;
gradKappa(4:6, 1) = Dkappa1De - Dkappa1Df;
gradKappa(7:9, 1) = Dkappa1Df;

gradKappa(1:3, 2) = -Dkappa2De;
gradKappa(4:6, 2) = Dkappa2De - Dkappa2Df;
gradKappa(7:9, 2) = Dkappa2Df;

%% Computation of hessian of the two curvatures
DDkappa1 = zeros(9, 9);
DDkappa2 = zeros(9, 9);

norm2_e = norm_e^2;
norm2_f = norm_f^2;

tt_o_tt = tilde_t' * tilde_t; % must be 3x3. tilde_t is 1x3
tmp = cross(tf, tilde_d2);
tf_c_d2t_o_tt = tmp' * tilde_t; % must be 3x3
tt_o_tf_c_d2t = tf_c_d2t_o_tt'; % must be 3x3
kb_o_d2e = kb' * m2e; % must be 3x3
d2e_o_kb = kb_o_d2e'; % must be 3x3

Id3 = eye(3);
D2kappa1De2 ...
    = 1.0 / norm2_e * (2 * kappa1 * tt_o_tt - tf_c_d2t_o_tt - tt_o_tf_c_d2t) ...
    - kappa1 / (chi * norm2_e) * (Id3 - te'*te) ...
    + 1.0 / (4.0 * norm2_e) * (kb_o_d2e + d2e_o_kb);

tmp = cross(te, tilde_d2);
te_c_d2t_o_tt = tmp' * tilde_t;
tt_o_te_c_d2t = te_c_d2t_o_tt';
kb_o_d2f = kb' * m2f;
d2f_o_kb = kb_o_d2f';

D2kappa1Df2 ...
    = 1.0 / norm2_f * (2 * kappa1 * tt_o_tt + te_c_d2t_o_tt + tt_o_te_c_d2t) ...
    - kappa1 / (chi * norm2_f) * (Id3 - tf'*tf) ...
    + 1.0 / (4.0 * norm2_f) * (kb_o_d2f + d2f_o_kb);

D2kappa1DeDf ...
    = -kappa1/(chi * norm_e * norm_f) * (Id3 + te'*tf) ...
    + 1.0 / (norm_e*norm_f) * (2 * kappa1 * tt_o_tt - tf_c_d2t_o_tt + ...
    tt_o_te_c_d2t - crossMat(tilde_d2));
D2kappa1DfDe = D2kappa1DeDf';

tmp = cross(tf, tilde_d1);
tf_c_d1t_o_tt = tmp'*tilde_t; % must be 3x3
tt_o_tf_c_d1t = tf_c_d1t_o_tt'; % must be 3x3
kb_o_d1e = kb'*m1e; % must be 3x3
d1e_o_kb = kb_o_d1e'; % must be 3x3

D2kappa2De2 ...
    = 1.0 / norm2_e * (2 * kappa2 * tt_o_tt + tf_c_d1t_o_tt + tt_o_tf_c_d1t) ...
    - kappa2 / (chi * norm2_e) * (Id3 - te'*te) ...
    - 1.0 / (4.0 * norm2_e) * (kb_o_d1e + d1e_o_kb);

tmp = cross(te, tilde_d1);
te_c_d1t_o_tt = tmp'*tilde_t; % must be 3x3
tt_o_te_c_d1t = te_c_d1t_o_tt'; % must be 3x3
kb_o_d1f = kb'*m1f; % must be 3x3
d1f_o_kb =  kb_o_d1f'; % must be 3x3

D2kappa2Df2 ...
    = 1.0 / norm2_f * (2 * kappa2 * tt_o_tt - te_c_d1t_o_tt - tt_o_te_c_d1t) ...
    - kappa2 / (chi * norm2_f) * (Id3 - tf'*tf) ...
    - 1.0 / (4.0 * norm2_f) * (kb_o_d1f + d1f_o_kb); % must be 3x3

D2kappa2DeDf ...
    = -kappa2/(chi * norm_e * norm_f) * (Id3 + te'*tf) ...
    + 1.0 / (norm_e*norm_f) * (2 * kappa2 * tt_o_tt + tf_c_d1t_o_tt - tt_o_te_c_d1t + crossMat(tilde_d1));
% must be 3x3
D2kappa2DfDe = D2kappa2DeDf'; % must be 3x3

% Curvature terms
DDkappa1(1:3, 1:3) = D2kappa1De2(1:3, 1:3);
DDkappa1(1:3, 4:6) = -D2kappa1De2(1:3, 1:3) + D2kappa1DeDf(1:3, 1:3);
DDkappa1(1:3, 7:9) = -D2kappa1DeDf(1:3, 1:3);
DDkappa1(4:6, 1:3) = -D2kappa1De2(1:3, 1:3) + D2kappa1DfDe(1:3, 1:3);
DDkappa1(4:6, 4:6) = D2kappa1De2(1:3, 1:3) - D2kappa1DeDf(1:3, 1:3) - ...
    D2kappa1DfDe(1:3, 1:3) + D2kappa1Df2(1:3, 1:3);
DDkappa1(4:6, 7:9) = D2kappa1DeDf(1:3, 1:3) - D2kappa1Df2(1:3, 1:3);
DDkappa1(7:9, 1:3) = -D2kappa1DfDe(1:3, 1:3);
DDkappa1(7:9, 4:6) = D2kappa1DfDe(1:3, 1:3) - D2kappa1Df2(1:3, 1:3);
DDkappa1(7:9, 7:9) = D2kappa1Df2(1:3, 1:3);

% Curvature terms
DDkappa2(1:3, 1:3) =   D2kappa2De2;
DDkappa2(1:3, 4:6) = - D2kappa2De2 + D2kappa2DeDf;
DDkappa2(1:3, 7:9) =               - D2kappa2DeDf;
DDkappa2(4:6, 1:3) = - D2kappa2De2                + D2kappa2DfDe;
DDkappa2(4:6, 4:6) =   D2kappa2De2 - D2kappa2DeDf - D2kappa2DfDe + D2kappa2Df2;
DDkappa2(4:6, 7:9)=                 D2kappa2DeDf                - D2kappa2Df2;
DDkappa2(7:9, 1:3)=                              - D2kappa2DfDe;
DDkappa2(7:9, 4:6)=                                D2kappa2DfDe - D2kappa2Df2;
DDkappa2(7:9, 7:9)=                                               D2kappa2Df2;

%% Gradient of Eb
EIMat = [ EI 0; ...
    0 EI];
kappaVector = [kappa1 kappa2];
dkappaVector = kappaVector - kappaBar;
dF = gradKappa * EIMat * dkappaVector' / l_k;

%% Hessian of Eb
dJ = 1.0 / l_k * gradKappa * EIMat * transpose(gradKappa);
temp = 1.0 / l_k * dkappaVector * EIMat;
dJ = dJ + temp(1) * DDkappa1 + temp(2) * DDkappa2;

end
