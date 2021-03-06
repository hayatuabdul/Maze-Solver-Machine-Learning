%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% all rights reserved
% Author: Dr. Ian Howard
% Associate Professor (Senior Lecturer) in Computational Neuroscience
% Centre for Robotics and Neural Systems
% Plymouth University
% A324 Portland Square
% PL4 8AA
% Plymouth, Devon, UK
% howardlab.com
% 14/10/2020
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Move arm through maze
% you need to implement this script to run the assignment section 4

close all
clear all
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% your script from here onwards
load('Run_Grid_World.mat')% Load previous work into this part



% Specify endpoint trajectory of revolute arm
endpoint_maze = zeros(3, length(state_2d)); % Augment end effect positions

net2 = W1_tr*endpoint_maze;  % Internal activation
% Activation of the first layer
a2 = 1./(1+exp(-net2));
% Augment a2 to account for bias term in W2
ahat2 = [a2; ones(1, length(endpoint_maze))]; % Augment length of endpoint data
% Calculate output activations of layer 2
o = W2_tr*ahat2; % Joint angle as output


% Set correct scale for the maze
limits = [-0.6 1; -1 1;];

% build the maze
maze = CMazeMaze11x11(limits);

% Plot output
figure 
hold on
plot(o(1,:),o(2,:), 'r.')
title('10614230:Joint Angles of maze path as Output P4');

% Run forward kinematics function 
[ maze1 , maze2 ] = RevoluteForwardKinematics2D(armLen, o, origin);

% draw the maze.
maze.DrawMaze();
% Plot end effect positions
plot(maze2(1,:),maze2(2,:), 'r.')
title('10614230:Endpoint Data P4');



