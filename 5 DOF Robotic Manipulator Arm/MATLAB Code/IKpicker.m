function [soln, quality] = IKpicker(goal, DHconst)
%% Implement IK and Check
% Uses IK to generate all solutions, and then pick the most viable one
% Check that solution doesn't smash through the floor
% In function form take in current position and goal position
% Bias elbow up
% pick 1 theta 1 so there's only 2 solutions to check
% pick 1 theta1 consistently to limit theta 2
[numSol, soln] = IKfcn(goal, DHconst);
theta1choice = 2; %1, for arm goes away from +xbase, 2 for using arm goes toward from +xbase
soln = soln(:, (theta1choice*2-1):(theta1choice*2));

% check if arm hits floor
% use geometry of the arms?
d0 = DHconst(1); % all in cm
a2 = DHconst(2);
a3 = DHconst(3);
dt = DHconst(4);

solnL = length(soln(1,:));
for i=1:solnL
    % prevent elbow smashing into floor
    if (0 >= d0 + a2*sin(soln(2,i)))
        %fprintf('elbow in the ground \n');
        soln(:,i) = NaN;
    elseif (0 >= d0 + a2*sin(soln(2,i)) ...
            + a3*sin(soln(2,i)+soln(3,i)))
        %fprintf('wrist elbow in the ground \n');
        soln(:,i) = NaN;
    elseif (0 >= d0 + a2*sin(soln(2,i)) ...
            + a3*sin(soln(2,i)+soln(3,i)) ...
            + dt*sin(soln(2,i)+soln(3,i)+soln(4,i)+ pi/2))
        %fprintf('tool tip in the ground \n');
        soln(:,i) = NaN;
    end
end
% check if there's more than 1 solutions left
soln = soln(:,~isnan(soln(1,:)));
nsol = size(soln, 2);

if nsol == 0
    %fprintf('no solution');
    quality = 0;
    soln = [NaN NaN NaN NaN NaN];
elseif nsol == 1
    quality = 1;
else
    % elbow up bias
    if numSol == 2 || sin(180-soln(2,1)) < sin(180-soln(2,2))
        soln = soln(:,2);
    else
        soln = soln(:,1);
    end
    quality = 1;
end
    
end



