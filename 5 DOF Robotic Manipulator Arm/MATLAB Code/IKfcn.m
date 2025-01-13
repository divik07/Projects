%% Pure Inverse Kinematics
function [numSol, soln] = IKfcn(goal, DHconst)
% takes goal frame and DH constants of arm and solves inverse kinematics
% returns 5 x 4 solutions array and number of solutions as 2 or 4
% if there are infinite solutions, returns two unique solutions

soln = zeros(5,4); % maximum solutions we will output is 4
soln(:,:) = NaN;
r = goal;
x = r(1,4);
y = r(2,4);
z = r(3,4);

d0 = DHconst(1); % all in cm
a2 = DHconst(2);
a3 = DHconst(3);
dt = DHconst(4);

c = @cos;
s = @sin;

solved = 1; % success or not

%% Initial checks
tol = 1e-10;
if abs(r(3,1)^2 + r(3,2) + r(3,3)^2 - 1) > tol
    % does not meet s^2+c^2==1 requirement for th5
    numSol = 0;
    solved = 0;
end
if abs(r(1,3)^2 + r(2,3)^2 + r(3,3)^2 - 1) > tol 
    % does not meet s^2+c^2==1 requirement for th1
    numSol = 0;
    solved = 0;
end

%% Solvy Time
if solved == 1
for caseth1 = 1:2

    %% solve theta1 (1 per loop, 2 total)
    % atan2(0,0) could be undefined (it isn't in MATLAB, but we will explicitly
    % set th1 in this case
    if x == 0 && y == 0
        % only two unique solutions will be provided in this case
        th1 = 0;
        % if the value is really small, basically zero, we will still let
        % it calculate 4 solutions

        % there are two possible solutions for th1, 180 deg from each other
        % during implementation, only use case 1 (this will force J2 to be
        % positive)
    elseif caseth1 == 1
        th1 = atan2(-y,-x);
    else
        th1 = atan2(y,x);
    end

    %% solve theta5
    th5 = atan2(-r(1,1)*s(th1)+r(2,1)*c(th1), -r(1,2)*s(th1)+r(2,2)*c(th1));

    %% solve theta3 (x2)

    % start by solving th234
    th234 = atan2(-r(1,3)*c(th1)-r(2,3)*s(th1), r(3,3));

    g = x*c(th1)+y*s(th1)+dt*s(th234);
    h = z-d0-dt*c(th234);

    c3 =  (g^2+h^2-a3^2-a2^2)/(2*a2*a3);

    % sanity check
    if 1-c3^2 < 0
        solved = 0;
        numSol = 0;
        break
    end
    
    % there are two possible combos of th3, th2, th4
    th3_1 = atan2(sqrt(1-c3^2),c3);
    th3_2 = atan2(-sqrt(1-c3^2),c3);


    %% solve theta2 (x2)
    aa1 = a3*c(th3_1)+a2;
    cc1 = -a3*sin(th3_1);
    aa2 = a3*c(th3_2)+a2;
    cc2 = -a3*sin(th3_2);
    dd = g;

    ee1 = a3*sin(th3_1);
    ff1 = a3*c(th3_1)+a2;
    ee2 = a3*sin(th3_2);
    ff2 = a3*c(th3_2)+a2;
    gg = h;

    th2_1 = atan2(aa1*gg-dd*ee1, dd*ff1-cc1*gg); % 2 solutions, 1 for each option of th2
    th2_2 = atan2(aa2*gg-dd*ee2, dd*ff2-cc2*gg); % 2 solutions, 1 for each option of th2

    %% solve theta4 (x2)
    th4_1 = th234-th2_1-th3_1; % 2 solutions, 1 for each option of th2
    th4_2 = th234-th2_2-th3_2; % 2 solutions, 1 for each option of th2

    % no checks for uniqueness or obstacles
    % check if both solutions are equal
    soln(:,caseth1*2-1) = [th1; th2_1; th3_1; th4_1; th5];
    soln(:,caseth1*2) = [th1; th2_2; th3_2; th4_2; th5];
end
end

soln = wrapToPi(soln);

if (soln(:,1) == soln(:,2) & soln(:,3) == soln(:,4))
    numSol = 2;
else
    numSol = 4;
end

% % check
% for i = 1:4
%     fig = figure;
%     goal = FK(soln(:,i), TsFcn, 1, fig);
%     str = "\theta_1 = "+ rad2deg(soln(1,i)) + ", \theta_2 = "+rad2deg(soln(2,i))+", \theta_3 = "+ rad2deg(soln(3,i))+...
%         ", \theta_4 = "+rad2deg(soln(4,i))+", \theta_5 = "+rad2deg(soln(5,i));
%     title(str, 'Interpreter', 'tex')
% end

% end