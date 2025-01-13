function [i, ein] = IandV_sea (kt,ddtheta, dtheta, J, N, n,T_l, R,L,time,k, ddT_l)

i= 1/kt*(J*N*(ddtheta+ddT_l/k) + 1/(N*n)*T_l);
ein = i*R + L*gradient(i,time) + kt*dtheta*N;
end

