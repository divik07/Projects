%% DH

% this script takes the DH parameters of the arm and calculates the forward kinematics
% 
% it saves two cell arrays, one with matrices and one with those matrices as functions that take joint position vectors as input
% 	first column corresponds to the transformation matrix between adjacent frames
% 	second column corresponds to the transformation from the first frame to the current frame
% 
% also saves the DH parameters used

%% DH Table

syms t1 t2 t3 t4 t5 
%
d0 = 10.2; % cm
a2 = 14.5; % cm
a3 = 14.5; % cm
dT = 14.8; %cm
%}

DHtab = [  0,  0, d0,  0; ...
           0,  0,  0, t1; ...
          90,  0,  0, t2;...
           0, a2,  0, t3;...
           0, a3,  0, t4;...
         -90,  0,  0, t5;...
           0,  0,  dT,  0];

%% Every Transformation Matrix
Ts = DH2transforms(DHtab);

%% Convert Transformation Matrices to Function Handles
numT = length(DHtab(:,1));
TsFcn = cell(numT, 2);
for i = 1:numT
    TsFcn{i,1} = matlabFunction(Ts{i,1}, "Vars", [t1 t2 t3 t4 t5]);
    TsFcn{i,2} = matlabFunction(Ts{i,2}, "Vars", [t1 t2 t3 t4 t5]);
end

%% Save
DHconst = [d0 a2 a3 dT];

filename = "DHTransforms.mat";
save(filename, "TsFcn", "Ts", "DHconst");

filename = "DHTransformsFcn.mat";
save(filename, "TsFcn", "DHconst");
    


