function x = Y_catapult
close all; 
%define variables and assign values
N = 50;
theta = 45; theta_release=0;
mu=0;

%% Material Parameters
rho = 7500;
E =10*1e9; %10Gpa for wood
r_out = 0.005; r_in =0.003;
A= pi*(r_out^2 - r_in^2); 

r_string = 0.004; rho_string = 1400;
E_string = 0.001*1e9;
A_string = pi*r_string^2; I_string = pi/4*(r_string^4);
EA_string = E_string*A_string;
EI_string = E_string*I_string;

%width = 0.03; depth=0.0085;
%A = width*depth
I = pi/4*(r_out^4 - r_in^4);
%I = width*depth^3/12;
EA = E*A; EI = E*I; %calculate area and Inertia
%% Define Y shape structure
l_v = 1; l_inc= 1; alpha_R = 45; 
% Calculate the coordinates of the Y-shaped structure
N_v= 25; N_inc=25; dl_v = l_v/(N_v-1); dl_inc= l_inc/(N_inc-1);
dm_v = rho*A*dl_v; dm_inc = rho*A*dl_inc; %defining dmass 

%coordinate of vertical line
z_v =  (dl_v)*(N_v-1:-1:0).';
x_v = zeros(size(z_v));
y_v = zeros(size(z_v));
node_v = zeros(3*N_v,1);
node_v(1:3:end) = x_v;  % x coordinates at odd places
node_v(2:3:end) = y_v;  % y coordinates at even places
node_v(3:3:end) = z_v;

z_incL = dl_inc*sind(alpha_R)*(0:N_inc-1) + z_v(1);
x_incL = -dl_inc*cosd(alpha_R)*(0:N_inc-1);
y_incL = zeros(size(z_incL));
node_incL = zeros(3*N_inc,1);
node_incL(1:3:end) = x_incL;  % x coordinates at odd places
node_incL(2:3:end) = y_incL;  % y coordinates at even places
node_incL(3:3:end) = z_incL;

z_incR = dl_inc*sind(alpha_R)*(0:N_inc-1) + z_v(1);
x_incR = dl_inc*cosd(alpha_R)*(0:N_inc-1);
y_incR = zeros(size(z_incR));
node_incR = zeros(3*N_inc,1);
node_incR(1:3:end) = x_incR;  % x coordinates at odd places
node_incR(2:3:end) = y_incR;  % y coordinates at even places
node_incR(3:3:end) = z_incR;

%% Plotting Y shape
%axis equal;
%plot3(node_incR(1:3:end), node_incR(2:3:end), node_incR(3:3:end));
%hold on;
%plot3(node_incL(1:3:end), node_incL(2:3:end),node_incL(3:3:end));
%hold on;
%plot3(node_v(1:3:end), node_v(2:3:end), node_v(3:3:end));
%hold on;

%% Defining String connecting two Y ends
l_string = abs(x_incL(end) - x_incR(end)); 
N_string = 25; dl_string = l_string/(N_string-1); dm_string=A_string*dl_string*rho_string;

x_U = linspace(x_incL(end), x_incR(end), N_string); % Angle values from 0 to pi
y_U = zeros(size(x_U));
z_U = zeros(size(x_U)); % Z-coordinate is set to 0
z_U = z_U + z_incR(end);

node_string = zeros(3*N_string,1);
node_string(1:3:end) =  x_U;
node_string(2:3:end) =  y_U;
node_string(3:3:end) =  z_U;


%% Voronoi length & Mass Vector (length associated with each node; used for bending and twisting)
%dl_v and dl_inc, N_v and N_inc
TotalN = 2*N_inc + N_v + N_string;
dm = zeros(TotalN,1);
voronoiLength = zeros(TotalN, 1); %starts with center node, goes to left(bottom to top)...
% right(bottom to top) and then center(top to bottom)
%1, N_inc+1 and 2*N_inc+1 denotes position of common node

for c=1: TotalN

    ind = [3*c-2, 3*c-1, 3*c];
    if c==1 || c==(N_inc+1) || c==(2*N_inc+1)
        voronoiLength(c) = 1/3*dl_v + 2/3*dl_inc;
        dm(ind) = 1/3*dm_v + 2/3*dm_inc;
    elseif (c==N_inc) || c==(2*N_inc) || c==TotalN || c==(TotalN-N_string+1)
        dm(ind) = 1/2*dm_inc + 1/2*dm_string;
        voronoiLength(c) = 1/2*dl_inc + 1/2*dl_string;
    elseif (c>=2 && c<N_inc) || (c>=N_inc+2 && c<2*N_inc) 
        voronoiLength(c) = dl_inc;
        dm(ind) = dm_inc;
    elseif(c>2*N_inc+1 && c<=(2*N_inc+N_v))
        voronoiLength(c) = dl_v;
        dm(ind) = dm_v;
        if(c==(2*N_inc+N_v))
         voronoiLength(c) = voronoiLength(c)/2;
         dm(ind) = dm_v/2;
        end
    else
        voronoiLength(c) = dl_string;
        dm(ind) = dm_string;
    end
end
M = diag(dm);
%% Initializing DOF vector
q0 = zeros(3*TotalN,1);
posV = 6*N_inc+3*N_v;

q0(1:3*N_inc) = node_incL;

q0(3*N_inc+1:6*N_inc) = node_incR;

q0(6*N_inc+1: posV) = node_v;

q0(posV+1:end) = node_string;


%% Defining Simulation parameters

eps = 1e-3*EI_string/dl_inc^2;
%setting the matrices
dt = 0.01;
maxTime = 10;
t = 0:dt:maxTime;
q = zeros(3*TotalN, numel(t));
uq = zeros(3*TotalN, numel(t));
q(:,1) = q0;

f = zeros(3*TotalN, 1);
J = zeros(3*TotalN, 3*TotalN);
fES = zeros(3*TotalN, 1);
tempFES = zeros(3*TotalN, 1);
tempFEB = zeros(3*TotalN, 1);
tempEES = zeros(3*TotalN, 1);
tempeEEB = zeros(3*TotalN, 1);

fEB = zeros(3*TotalN, 1);
eEB = 0; eEB_old=0;
eET = 0; eET_old=0;
tempjES = zeros(3*TotalN,3*TotalN);
tempjEB = zeros(3*TotalN,3*TotalN);
jES = zeros(3*TotalN, 3*TotalN);
jEB = zeros(3*TotalN, 3*TotalN);

freeDOF = [1:(6*N_inc + 3*N_v - 24) , 6*N_inc + 3*N_v+1: 3*TotalN];
freeDOF = setdiff(freeDOF, 1:3:length(freeDOF)); %Fixing x node, no twisting.

ind=1;
x_traj=0; y_traj=0;
for k=1:length(t)-1
    %setting parameters for q and uq
    q_old = q(:,k);
    uq_old = uq(:,k);
    q_new = q_old;
    err = eps*10;
    deltaQ = zeros(3*TotalN,1);

    while err > eps
        
        computeFandJ_YCatapult %calling computeF script
        deltaQ(freeDOF) = J(freeDOF, freeDOF) \ f(freeDOF);
        q_new(freeDOF) =  q_new(freeDOF) - deltaQ(freeDOF);
        k
        err = sum(abs(f(freeDOF)))    
        
    end

x = q_new(1:3:end);
y = q_new(2:3:end);
z = q_new(3:3:end);
breakS1 = N_inc;
breakE1 = N_inc+1;
breakS2 = 2*N_inc;
breakE2 = 2*N_inc+1;

plot3(x,y,z,'ro-');
ylim([-0.5,0.5]);


drawnow;
%pause(1);
%hold on;

 
    if(k<=30)
     q_new(3*88-1) =  q_new(3*88-1) - 0.01;
     q_new(3*87-1) = q_new(3*87-1) - 0.01;
      %q_new(6*N_inc+5) =  q_new(6*N_inc+5) - 0.01;
      %q_new(6*N_inc-4) = q_new(6*N_inc-4) - 0.01;
      %indices_to_remove = [3*N_inc-4, 6*N_inc-4];
      indices_to_remove = [3*88-1, 3*87-1];
      freeDOF = setdiff(freeDOF, indices_to_remove);
    else
        freeDOF = [1:(6*N_inc + 3*N_v - 24) , 6*N_inc + 3*N_v+1: 3*TotalN];
        freeDOF = setdiff(freeDOF, 1:3:length(freeDOF)); %Fixing x node, no twisting.
    end

    uq_new = (q_new - q_old)/dt;
    q(:, k+1) = q_new;
    uq(:, k+1) = uq_new;

    
end

    
%% Plotting Y shape


    
%% Plotting

figure (1);
xlabel('X Position(m)');
ylabel('Y Position (m)');
title('Position(m)');
plot(q_new(1:2:2*N), q_new(2:2:2*N),'ro');
grid on;
axis equal;
xlim([-3,4]);
ylim([-1,4]);


 if (ind <= length(x_traj))
    hold on
    plot(x_traj(ind) , y_traj(ind), 'bo');
  
    axis equal;
    drawnow;
    ind=ind+1;
    hold off;
end
          
end

%% Calculate stretching energy
function E_stretch = computeStretchingEnergy(xk, yk, xkp1, ykp1, l_k, EA)
%
% This function computes the stretching energy for a spring.
%
% Input:
% xk, yk, xkp1, ykp1 - Coordinates of nodes k and k+1.
% l_k - Rest length of the spring.
% EA - Stretching stiffness.
%
% Output:
% E_stretch - Stretching energy.
%

% Calculate the stretch of the spring
stretch = sqrt((xkp1 - xk)^2 + (ykp1 - yk)^2) - l_k;

% Calculate the stretching energy
E_stretch = 0.5 * EA * stretch^2;
end
%% calculate bending energy
function E_bend = computeBendingEnergy(xkm1, ykm1, xk, yk, xkp1, ykp1, curvature0, l_k, EI)
%
% This function computes the bending energy for a beam.
%
% Input:
% xkm1, ykm1, xk, yk, xkp1, ykp1 - Coordinates of nodes k-1, k, and k+1.
% curvature0 - Natural curvature at node k.
% l_k - Length of the beam.
% EI - Bending stiffness.
%
% Output:
% E_bend - Bending energy.
%

% Calculate the bending curvature
node0 = [xkm1, ykm1, 0];
node1 = [xk, yk, 0];
node2 = [xkp1, ykp1, 0];

ee = node1 - node0;
ef = node2 - node1;

norm_e = norm(ee);
norm_f = norm(ef);

te = ee / norm_e;
tf = ef / norm_f;

% Curvature binormal
kb = 2.0 * cross(te, tf) / (1.0 + dot(te, tf));

% Compute the bending energy
kappa = kb(3);
E_bend = 0.5 * EI * (kappa - curvature0)^2;
end
%%