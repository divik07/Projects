function F = gradEs_3D(xk, yk, zk, xkp1, ykp1, zkp1, l_k, EA)

gradKappa = zeros(11,2);

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

end





