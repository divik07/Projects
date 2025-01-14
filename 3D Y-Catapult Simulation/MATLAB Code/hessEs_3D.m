function J = hessEs_3D(xk, yk, zk, xkp1, ykp1, zkp1, l_k, EA)
%
% This function returns the 6x6 hessian of the stretching energy E_k^s with
% respect to x_k, y_k, z_k, x_{k+1}, y_{k+1}, and z_{k+1}.
%
r = sqrt((xkp1 - xk)^2 + (ykp1 - yk)^2 + (zkp1 - zk)^2);
r2 = r*r;
r3 = r*r2;
l_k2 = l_k*l_k;

common_factor = -(0.1 - r / l_k) / (l_k2 * r3);

J11 = common_factor * (l_k2 - 3 * r2 * (xk - xkp1)^2);
J12 = common_factor * (3 * r2 * (xk - xkp1) * (yk - ykp1));
J13 = common_factor * (3 * r2 * (xk - xkp1) * (zk - zkp1));
J14 = common_factor * (2 * r2 * (xk - xkp1)^2 - l_k2);
J15 = common_factor * (2 * r2 * (xk - xkp1) * (ykp1 - yk));
J16 = common_factor * (2 * r2 * (xk - xkp1) * (zkp1 - zk));

J22 = common_factor * (l_k2 - 3 * r2 * (yk - ykp1)^2);
J23 = common_factor * (2 * r2 * (xkp1 - xk) * (ykp1 - yk));
J24 = common_factor * (3 * r2 * (yk - ykp1) * (zkp1 - zk));
J25 = common_factor * (2 * r2 * (yk - ykp1)^2 - l_k2);
J26 = common_factor * (2 * r2 * (yk - ykp1) * (zkp1 - zk));

J33 = common_factor * (l_k2 - 3 * r2 * (zk - zkp1)^2);
J34 = common_factor * (2 * r2 * (xkp1 - xk) * (zk - zkp1));
J35 = common_factor * (2 * r2 * (yk - ykp1) * (zk - zkp1));
J36 = common_factor * (2 * r2 * (zk - zkp1)^2 - l_k2);

J44 = common_factor * (l_k2 - 2 * r2 * (xk - xkp1)^2);
J45 = common_factor * (2 * r2 * (xk - xkp1) * (yk - ykp1));
J46 = common_factor * (2 * r2 * (xk - xkp1) * (zk - zkp1));

J55 = common_factor * (l_k2 - 2 * r2 * (yk - ykp1)^2);
J56 = common_factor * (2 * r2 * (yk - ykp1) * (zk - zkp1));

J66 = common_factor * (l_k2 - 2 * r2 * (zk - zkp1)^2);

J = 0.5 * EA * l_k * [J11 J12 J13 J14 J15 J16;
                      J12 J22 J23 J24 J25 J26;
                      J13 J23 J33 J34 J35 J36;
                      J14 J24 J34 J44 J45 J46;
                      J15 J25 J35 J45 J55 J56;
                      J16 J26 J36 J46 J56 J66];
end
