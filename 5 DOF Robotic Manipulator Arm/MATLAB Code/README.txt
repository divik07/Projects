README
%% List of Scripts and functions
 
% %%%%%%%%%%% TABLE OF CONTENTS %%%%%%%%%%%%%%%%%%%%

% -------------- Major Scripts%Functions -------------
% 1. DH.m (S)
% 2. DH2transforms.m (F)
% 3. FK.m (F)
% 4. quickFKcheck.m (S)
% 5. motionVisualization (S)
% 6. moveBetweenPosRad.m (F)
% 7. moveBetweenPosDeg.m (F)
% 8. IKfcn.m (F)
% 9. IKpicker.m (F)
% 10. jointGeneration.m (S)
% 11. pickAndDrop.m (S)
% 
% -------------- Helper Functions --------------------
% description not provided here, read file for information
% A1. LTOA_p.m
% A2. plotframe.m
% A3. pt2xyz.m
% A4. pos2goal.m
 
% %%%%%%%%%%%%%%%%%%%%% 1. %%%%%%%%%%%%%%%%%%%%%%%%%%

% 1. DH.m
% Use to construct a new set of DH parameters
% Fill out DH table and%or change parameters
% If number of DOF%definition of joint variables changes, update input vector to TsFcn

% Running script will save two mat files with variables listed below
%	DHTransforms.mat
%		Ts:	the transforms in a cell array, N x 2
%		TsFcn: 	the same transforms, as function handles in a cell array, N x 2
%	DHTransforms.mat 
%		TsFcn: 	the same transforms, as function handles in a cell array, N x 2

% Description of saved variables
% 	Format of Ts and TsFcn
%		first index is the transform from frame to frame (1->0, 2->1, etc)
% 		second index is the transform from base up to this frame (B->1, B->4, etc.)
% 			index:	   		                1   2   3   4   5   6
% 			transform from base to frame:   0   1   2   3   4   T
 
% %%%%%%%%%%%%%%%%%%%%% 2. %%%%%%%%%%%%%%%%%%%%%%%%%%

% 2. DH2transforms.m (F)
% this function takes a DH table and calculates the matrices for forward kinematics, 
% returning said matrices in a cell array
% 
% the cell array is organized thus
% 	first column corresponds to the transformation matrix between adjacent frames
% 	second column corresponds to the transformation from the first frame to the current frame
% 
% instructions:
% 	give DHtable input as nFrame x 4 array
% 
% %%%%%%%%%%%%%%%%%%%%% 3. %%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% 3. FK.m
% 
% takes joint angles%positions and cell of transforms (in format saved by DH.m)
% solves arm position for given joint positions and plots the arm with
% coordinate frame in the given frame (if chosen)
% pl is 0 or 1 indicating whether to plot
% 
% instructions:
% 	specify joint positions as 5 entry vector
% 	TsFcn should be in format as saved by DH.m
% 
% dependencies:
% 	requires plotframe.m
% 	requires pt2xyz.m
% 
% %%%%%%%%%%%%%%%%%%%%% 4. %%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% 4. quickFKcheck.m (S)
% 
% this script takes joint positions and outputs the frame of the end effector of the arm, as well as a plot of the arm, represented as lines
% 
% instructions:
% modify joint positions in the script and run
% joint positions are specified assuming the straight up position is zero for all joints
% joint positions can be specified with DH parameter angles instead (uncomment instructed section)
% 
% dependencies:
% 	requires FK.m
% 	requires DHTransformsFcn.mat as saved by DH.m
% 
% %%%%%%%%%%%%%%%%%%%%%%% 5. %%%%%%%%%%%%%%%%%%%%%%%%%%
%
% 5. MotionVisualization.m (S)
%
% motion visualization tool
% change pos to array
%   nPlaces x 5
%   each row is new joint position array
%   joint angles in degrees
% use convention that arm straight up is zero degree joint position
% plots ans saves video of arm moving through given path
%
% dependencies:
%	requires moveBetweenPosDeg.m
%
% %%%%%%%%%%%%%%%%%%%%%%% 6. %%%%%%%%%%%%%%%%%%%%%%%%%%
%
% 6. moveBetweenPosDeg.m (F)
%
% takes initial and end joint positions specified in degrees
% and plots arm moving from first position to the next, saving to specified vidfile
%
% dependencies: 
%	moveBetweenPosRad.m
%	FK.m
%
% %%%%%%%%%%%%%%%%%%%%%%% 7. %%%%%%%%%%%%%%%%%%%%%%%%%%
%
% 7. moveBetweenPosRad.m (F)
%
% takes initial and end joint positions specified in radians
% and plots arm moving from first position to the next, saving to specified vidfile
%
% dependencies: 
%	FK.m
%
% %%%%%%%%%%%%%%%%%%%%%%% 8. %%%%%%%%%%%%%%%%%%%%%%%%%%
%
% 8. IKfcn.m (F)
%
% takes goal frame and DH constants of arm and solves inverse kinematics
% returns 5 x 4 solutions array and number of solutions as 2 or 4
% if there are infinite solutions, returns two unique solutions
%
% input DHconst in format saved by DH.m
%
% %%%%%%%%%%%%%%%%%%%%%%% 9. %%%%%%%%%%%%%%%%%%%%%%%%%%
%
% 9. IKpicker.m (F)
%
% Uses IK to generate all solutions, and then pick the most viable one
% Check that solution doesn't smash through the floor
% In function form take in current position and goal position
% Bias elbow up
% pick 1 theta 1 so there's only 2 solutions to check
% pick 1 theta1 consistently to limit theta 2
%
% input DHconst in format saved by DH.m
%
% dependencies:
%	IKfcn.m
%
% %%%%%%%%%%%%%%%%%%%%%%% 10. %%%%%%%%%%%%%%%%%%%%%%%%%%
%
% 10. jointGeneration.m (S)
% takes a series of tooltip points (either manually specified or by
% pickAndDrop) and generates the joint position path needed to achieve
% path. Also outputs simulated motion as a video
%
% input of tooltip points is specified as needed by pos2Goal
%
% dependencies:
%	DHTransformsFcn.mat
%	pickAndDrop.m (if used)
%	pos2Goal.m
%	IKpicker.m
%	moveBetweenPosRad.m
% 
% %%%%%%%%%%%%%%%%%%%%%%% 11. %%%%%%%%%%%%%%%%%%%%%%%%%%
%
% 11. pickAndDrop.m (S)
%
% Path Generation
% returns an 6 x 4 array with each row corresponding to x, y, z, phi
% position of path
% also returns array of 1, 0, -1 indicating gripper behavior after the arm
% moves to the corresponding position 
%   1: gripper close
%   0: gripper does nothing
%   -1: gripper open
%
