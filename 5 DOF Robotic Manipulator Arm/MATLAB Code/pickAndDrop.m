%% Path Generation
% returns an 6 x 4 array with each row corresponding to x, y, z, phi
% position of path
% also returns array of 1, 0, -1 indicating gripper behavior after the arm
% moves to the corresponding position 
%   1: gripper close
%   0: gripper does nothing
%   -1: gripper open

heightoff = 2; % cm, height offset for safe motion
%% Specify pickup path
pickprompt = input('Where do we pickup?\n Enter x y z (cm) and phi (deg) separated by spaces:\n', "s");
pickup = transpose(str2double(string(split(pickprompt))));
while length(pickup) ~= 4
    pickprompt = input('\n4 inputs please: Pickup?\n Enter x y z (cm) and phi (deg) separated by spaces:\n', "s");
    pickup = transpose(str2double(string(split(pickprompt))));
end

dropprompt = input('Where do we drop off?\n Enter x y z (cm) and phi (deg) separated by spaces:\n', "s");
dropoff = transpose(str2double(string(split(dropprompt))));
while length(dropoff) ~= 4
    dropprompt = input('\n4 inputs please: Pickup?\n Enter x y z (cm) and phi (deg) separated by spaces:\n', "s");
    dropoff = transpose(str2double(string(split(dropprompt))));
end

prepostpick = pickup + [0 0 heightoff 0];
prepostdrop = dropoff + [0 0 heightoff 0];

%% Smash Together
% orders the positions so process goes in
% prepickup => open => pickup => close => postpickup => predropoff =>
% dropoff => open => postdropoff => close

path = [prepostpick; pickup; prepostpick;...
    prepostdrop; dropoff; prepostdrop];
grip = [-1; 1; 0; 0; -1; 1]; 

