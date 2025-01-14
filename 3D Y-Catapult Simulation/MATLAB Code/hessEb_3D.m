function dJ = hessEb(xkm1, ykm1, zkm1, xk, yk, zk, xkp1, ykp1, zkp1, curvature0, l_k, EI)

%
% This function returns the derivative of bending energy E_k^b with respect
% to x_{k-1}, y_{k-1}, z_{k-1}, x_k, y_k, z_k, x_{k+1}, y_{k+1}, and z_{k+1}.
%
% curvature0 is the "discrete" natural curvature [dimensionless] at node (xk, yk, zk).
% l_k is the voronoi length of node (xk, yk, zk).
% EI is the bending stiffness.
%

node0 = [xkm1, ykm1, zkm1];
node1 = [xk, yk, zk];
node2 = [xkp1, ykp1, zkp1];
m2e = [0 0 1];
m2f = [0 0 1];
kappaBar = curvature0;

% Computation of gradient of the two curvatures
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
tilde_d2 = (m2e + m2f) / chi;

% Curvatures
kappa1 = kb(3);

Dkappa1De = 1.0 / norm_e * (-kappa1 * tilde_t + cross(tf, tilde_d2));
Dkappa1Df = 1.0 / norm_f * (-kappa1 * tilde_t - cross(te, tilde_d2));

gradKappa(1:3, 1) = -Dkappa1De(1:3);
gradKappa(4:6, 1) = Dkappa1De(1:3) - Dkappa1Df(1:3);
gradKappa(7:9, 1) = Dkappa1Df(1:3);

% Computation of hessian of the two curvatures
DDkappa1 = zeros(9, 9);

norm2_e = norm_e^2;
norm2_f = norm_f^2;

tt_o_tt = tilde_t' * tilde_t;
tmp = cross(tf, tilde_d2);
tf_c_d2t_o_tt = tmp' * tilde_t;
tt_o_tf_c_d2t = tf_c_d2t_o_tt';

Id3 = eye(3);
D2kappa1De2 ...
    = 1.0 / norm2_e * (2 * kappa1 * tt_o_tt - tf_c_d2t_o_tt - tt_o_tf_c_d2t) ...
    - kappa1 / (chi * norm2_e) * (Id3 - te' * te) ...
    + 1.0 / (4.0 * norm2_e) * (kb' * m2e + m2e' * kb);

tmp = cross(te, tilde_d2);
te_c_d2t_o_tt = tmp' * tilde_t;
tt_o_te_c_d2t = te_c_d2t_o_tt';

D2kappa1Df2 ...
    = 1.0 / norm2_f * (2 * kappa1 * tt_o_tt + te_c_d2t_o_tt + tt_o_te_c_d2t) ...
    - kappa1 / (chi * norm2_f) * (Id3 - tf' * tf) ...
    + 1.0 / (4.0 * norm2_f) * (kb' * m2f + m2f' * kb);

D2kappa1DeDf ...
    = -kappa1 / (chi * norm_e * norm_f) * (Id3 + te' * tf) ...
    + 1.0 / (norm_e * norm_f) * (2 * kappa1 * tt_o_tt - tf_c_d2t_o_tt + ...
    tt_o_te_c_d2t - crossMat(tilde_d2));
D2kappa1DfDe = D2kappa1DeDf';

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

% Hessian of Eb
dkappa = kappa1 - kappaBar;
dJ = 1.0 / l_k * EI * gradKappa * gradKappa';
temp = 1.0 / l_k * dkappa * EI;
dJ = dJ + temp * DDkappa1;
end
