function [P,i,ein] = Power(kt,ddtheta, dtheta, J, N, n,T_l, R,L,time)

i = 1/kt*(J*ddtheta*N + 1/(N*n)*T_l);
ein = i*R + L*gradient(i,time) + kt*dtheta*N;
P = trapz(time, i.^2*R); %or V*I change accordingly
end