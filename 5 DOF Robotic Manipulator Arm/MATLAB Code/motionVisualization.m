%% MotionVisualization
% motion visualization tool
% change pos to array
%   nPlaces x 5
%   each row is new joint position array
%   joint angles in degrees
% use convention that arm straight up is zero degree joint position
% plots ans saves video of arm moving through given path

load('DHTransformsFcn.mat')

t1offset = 180;
t2offset = 90;
t4offset = -90;

offset = [t1offset t2offset 0 t4offset 0];

%%% as thought by user, everything straight up is zero, degrees
pos = [0 70 20 90 0;
    -180 -10 90 100 0;
    -180 1.2 126.6 53.2 0 ];

viewtime = 3;
vid = 1;
steps = 30;

if vid
    vidfile = VideoWriter('FKmot', 'MPEG-4');
    vidfile.FrameRate = min(round(steps/3), 60);
    open(vidfile);
end

%% viewing settings
vw = [20 25];
%viewtime = 3; % seconds
fig = figure;
for i = 1:(size(pos,1)-1)
    toolframe = moveBetweenPosDeg(pos(i,:)+offset, pos(i+1,:)+offset, TsFcn, 1, fig, vidfile);
end


if vid
    close(vidfile);
end