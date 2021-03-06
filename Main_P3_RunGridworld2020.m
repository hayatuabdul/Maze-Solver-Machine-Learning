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
% 22/09/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Find path through maze
% you need to expand this script to run the assignment section 3

close all
clear all
clc


load('Train_Neural_Network.mat')% Load previous work into this part



% YOU NEED TO DEFINE THESE VALUES
limits = [0 1; 0 1;];

% build the maze
maze = CMazeMaze11x11(limits);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% YOU NEED TO DEFINE THESE VALUES
% init the q-table
minVal = 0.01;
maxVal = 0.1;
maze = maze.InitQTable(minVal, maxVal);


% test values
state = 1;
action = 1;
e = 0;  % Exploration rate for greedy action


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% this will be used by Q-learning as follows:
q = maze.QValues(state, action);   


            

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% YOU NEED TO FINISH OFF THIS FUNCTION
% get the reward from the action on the surrent state
% this will be used by Q-learning as follows:
reward = maze.RewardFunction(state, action);
reward

    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% YOU NEED TO FINISH OFF THIS FUNCTION
% build the transition matrix
maze = maze.BuildTransitionMatrix();
% print out values
maze.tm

% get the next state due to that action
% this will be used by Q-learning as follows:
resultingState = maze.tm(state, action);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% YOU NEED TO FINISH OFF THIS FUNCTION
% Loop 1000 times to generate 1000 starting states
for s = 1 : 1000

startingStateNew(s) = maze.RandomStartingState();

end

% print out value
startingStateNew

 figure
 hold on
 histogram(startingStateNew, 121)   % Display randomized data and bar charts
 title('10614230:Starting State');

 
learning_rate = 0.2;
gamma = 0.9;


% Experiment
% Run trial 100 times for Experiment
Trial_time = 100;

for t = 1 : Trial_time

[q, steps_mean,  steps_std, q_table, steps_no, states_visited] = maze.Trial(maze.QValues, learning_rate, gamma, e);

end

 figure
 hold on
 % Plot the mean and standard deviation for steps
 errorbar(steps_mean(1,:), steps_std(1,:),'b')
 title('10614230:Steps Plot');
 xlabel('Episodes')
 ylabel('Steps')
 


% Convert visited states into x and y coordinates cells
x = maze.stateX(states_visited);
y = maze.stateY(states_visited);
% Store coordinates in an array
state_xy = [x; y];
% Locks the x and y coordinates to the right cell
locX = maze.cursorCentre(state_xy);
locY = maze.cursorCentre(state_xy);
state_2d = [locX; locY];


 % draw the maze.
 maze.DrawMaze();
 % Plots the path to the maze
 p = plot(state_2d(1,:), state_2d(2,:), 'x-', 'LineWidth',8, 'MarkerSize',28);
 set(p,'Color','magenta')
 

% Save data from this script to use on other scripts
save('Run_Grid_World.mat')



