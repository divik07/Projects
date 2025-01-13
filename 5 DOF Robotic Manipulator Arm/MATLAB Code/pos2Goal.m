function goalframe = pos2Goal(pos)
% calculates goal frame given a position and orientation
% assumining gripper position of tool tip down

% pos = [x, y, z, phi]
% phi = (degrees) angle between gripping axis of gripper and base frame x axis
%   phi could be phi +/- 180 if you have the current phi and don't want to
%   spin wildly

cp = cosd(pos(4));
sp = sind(pos(4));
goalframe = [cp sp 0 pos(1);...
    sp -cp 0 pos(2); ...
    0 0 -1 pos(3); ...
    0 0 0 1];

end
