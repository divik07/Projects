function [toolframe] = moveBetweenPosDeg(startJt, endJt, TsFcn, vid, fig, vidfile)
% takes initial and end joint positions specified in degrees
% and plots arm moving from first position to the next, saving to specified vidfile

% position 1
jtstart = deg2rad(startJt);

% position 2
jtend = deg2rad(endJt);

toolframe = moveBetweenPosRad(jtstart, jtend, TsFcn, vid, fig, vidfile);

end
