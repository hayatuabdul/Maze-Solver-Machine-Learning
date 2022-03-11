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

% Generate arm data to train inverse model
% you need to implement this script to run the assignment section 1`

close all
clear all
clc

% Arm length 
armLen = [0.4 0.4];
a = 0;   % Range from 0
b = pi;  % To pi
theta = a+(b-a).* rand(2, 1000);  %Function that randomises 2 row and 1000 columns
origin = [0 0];   %Set origin so joint stays in place

% Call RevoluteForwardKinematics funtion to use in main
[ P1, P2 ] = RevoluteForwardKinematics2D(armLen, theta, origin);
 
 figure
 hold on
 plot(P2(1,:),P2(2,:), 'r.')  % Plot using dot formation
 title('10614230:Endpoint P1');
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Saves to load data onto next task
save('GenerateArmData.mat')
