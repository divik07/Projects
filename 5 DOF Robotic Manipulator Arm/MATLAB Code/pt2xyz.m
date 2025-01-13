function [xpt, ypt, zpt] = pt2xyz(pt1, pt2)
% takes two points and returns the x positions, y positions, and z
% positions in three arrays for easy plotting

xpt = [pt1(1) pt2(1)];
ypt = [pt1(2) pt2(2)];
zpt = [pt1(3) pt2(3)];

end