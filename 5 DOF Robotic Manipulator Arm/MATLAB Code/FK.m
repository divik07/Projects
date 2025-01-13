%% Forward Kinematics
function toolframe = FK(jointpos, TsFcn, pl, fig)
% takes joint angles/positions and cell of transforms
% solves arm position for given joint positions and plots the arm with
% coordinate frame
% pl is 0 or 1 indicating whether to plot

%% Joint Variables
t1 = jointpos(1);
t2 = jointpos(2);
t3 = jointpos(3);
t4 = jointpos(4);
t5 = jointpos(5);

%% Evaluate all frames
numT = size(TsFcn, 1);
evFrames = cell(numT, 1);

for i = 1:numT
    evFrames{i} = TsFcn{i,2}(t1, t2, t3, t4, t5);
end

%% Plot all frames with joints connected
if pl
    % plot base frame
    plotframe(diag([1 1 1 1]), fig);
    lastorigin = [0; 0; 0];

    for i = 1:numT
        plotframe(evFrames{i}, fig);
        origin = evFrames{i}(1:3,4);
        [x, y, z] = pt2xyz(lastorigin, origin);
        hold on
        plot3(x, y, z, 'k-')
        lastorigin = origin;
    end
    view([0 15])
    axis equal
    zlim([0 100])
    ylim([-50 50])
    xlim([-50 50])
    hold off
end

toolframe = evFrames{numT};
end


