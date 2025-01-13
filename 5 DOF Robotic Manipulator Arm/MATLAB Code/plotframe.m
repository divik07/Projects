function [] = plotframe(frame, fig)
% takes 4x4 frame matrix and handle of existing figure, plots the frame
% on the figure
% x axis - red
% y axis - blue
% z axis - green

origin = frame(1:3, 4);
xax = frame(1:3, 1);
yax = frame(1:3, 2);
zax = frame(1:3, 3);

figure(fig);
hold on

% plot x axis
[x, y, z] = pt2xyz(origin, xax+origin);
plot3(x, y, z, 'r-')

% plot y axis
[x, y, z] = pt2xyz(origin, yax+origin);
plot3(x, y, z, 'b-')

% plot z axis
[x, y, z] = pt2xyz(origin, zax+origin);
plot3(x, y, z, 'g-')

hold off
end