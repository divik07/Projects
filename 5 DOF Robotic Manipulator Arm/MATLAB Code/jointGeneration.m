%% Joint Generation
% takes a series of tooltip points (either manually specified or by
% pickAndDrop) and generates the joint position path needed to achieve
% path. Also outputs simulated motion as a video

%% Load Arm Parameters
load('DHTransformsFcn.mat')

%% Feed Path
%   path is numpt x 4 consisting of ui position specified as [x y z phi]

% Pick and Drop Specification
% prompts user for pickup and dropoff point, returns a path to pickup then
% drop off
pickAndDrop

% Manual Input
%        x, y, z, phi (cm, degrees)
%path = [ 0 0 0 0; 0 0 0 0];

%% Solve Inverse Kinematics
numpt = size(path,1);
IKpath = zeros(size(path,1), 5); % to hold joint angles (path point, joint angle)

for pt = 1:numpt % for each goal position
    goalpos = path(pt, :);
    goalframe = pos2Goal(goalpos);
    [IKpath(pt,:), quality] = IKpicker(goalframe, DHconst); % eventually IKpicker(goalframe, currframe)
    if quality == 0
        fprintf('Path Not Possible Due to Point %i\n', pt)
        return
    end
end

%% FK Check
viewtime = 3;
vid = 1;
steps = 10;

if vid
    vidfile = VideoWriter('FKmot', 'MPEG-4');
    vidfile.FrameRate = 60;
    open(vidfile);
end

%% viewing settings
vw = [20 25];
%viewtime = 3; % seconds
fig = figure;
for i = 1:(size(IKpath,1)-1)
    toolframe = moveBetweenPosRad(IKpath(i,:), IKpath(i+1,:), TsFcn, 1, fig, vidfile);
end


if vid
    close(vidfile);
end


