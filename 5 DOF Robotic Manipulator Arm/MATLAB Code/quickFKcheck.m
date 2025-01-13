%% easy FK visual
% this script takes joint positions, inputted below, and outputs the
% frame of the end effector of the arm, as well as a plot of the arm,
% represented as lines

%% Home Position

% for ui purposes (running the arm), positions are fed with this position as zero
%{
% diagram
^_^    <- gripper
 |
[ ]    <- J5
 |
 O     <- J4
 |
 |
 O     <- J3
 |
 |
 O     <- J2
 |
[ ]    <- J1

%}
% corresponds to DH joint positions of [180 90 0 -90 0]

%% Position Set

% from home position, degrees
t1c = 0;
t2c = 20;
t3c = 70;
t4c = 90;
t5c = 0;

t1offset = 180;
t2offset = 90;
t4offset = -90;

% set in degrees
t1 = t1c + t1offset;
t2 = t2c + t2offset;
t3 = t3c;
t4 = t4c + t4offset;
t5 = t5c;

jtpos = deg2rad([t1 t2 t3 t4 t5])

%{ 
% uncomment to override with DH specified joint positions
% straight up position
z1 = 0;
z2 = 90;
z3 = 0;
z4 = -90;
z5 = 0;
jtpos = deg2rad([z1 z2 z3 z4 z5]);
%}

%

load DHTransformsFcn.mat
fig = figure;
%clf(fig)
toolframe = FK(jtpos, TsFcn, 1, fig);

str = "FK: \theta_1 = "+ t1 + ", \theta_2 = "+t2+", \theta_3 = "+ t3+...
    ", \theta_4 = "+t4+", \theta_5 = "+t5;
title(str, 'Interpreter', 'tex')

fprintf('Tool Frame\n')
disp(toolframe)
