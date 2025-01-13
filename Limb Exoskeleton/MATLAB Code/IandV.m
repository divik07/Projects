function [i, ein] = IandV (kt,ddtheta, dtheta, J, N, n,T_l, R,L,time)

i = 1/kt*(J*ddtheta*N + 1/(N*n)*T_l);
ein = i*R + L*gradient(i,time) + kt*dtheta*N;

end

