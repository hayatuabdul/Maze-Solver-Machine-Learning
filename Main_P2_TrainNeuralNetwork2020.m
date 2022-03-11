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

% Train a neural network to implement inverse model
% you need to implement this script to run the assignment section 2

close all
clear all 
clc


load('GenerateArmData.mat')% Load previous work into this part



% Initialize Hidden Nodes
hidnodes = 70; % This value seems to work well

% Select random weights to train using hidden units
W1_tr = 1 * randn(hidnodes,3); % W1 as a 10 x 3
W2_tr = 1 * randn(2,hidnodes+1); % W2 as 2 x 10+1

%Training Time
for j = 1 :8000  % j is for cycles the loop runs

 % Add '1's to the P2 input to accomodate for the 1000 size(Augment)
 X = [P2;ones(1,1000)];
    
 [r,c] = size(X); % Structure rows and columns in right order

    % Calculate Internal activations of layer 1
    % Augment input to account for bias in hidden layer 1
    net2 = W1_tr*X;  % Internal activation
    % Activation of the first layer
    a2 = 1./(1+exp(-net2));
    % Augment a2 to account for bias term in W2
    ahat2 = [a2; ones(1,c)];
    % Calculate output activations of layer 2
    o = W2_tr*ahat2;
    % Calculate output layer Delta term
    Delta3 = - (theta - o);
    % Back prop to calculate input (lower) layer delta term
    Delta2 = (W2_tr(:,hidnodes)'*Delta3.*a2.*(1-a2));
    % Calculate error gradient w.r.t. W1
    ErrorGW1 = Delta2 * X';
    % Calculate error gradient w.r.t. W2
    ErrorGW2 = Delta3* ahat2';
    
    %Learning Rate
    learning_r = 0.00011;
    W1_tr = W1_tr - (learning_r.*ErrorGW1);
    W2_tr = W2_tr - (learning_r.*ErrorGW2);
    
    
  %Error Plot
  %error(j) = sum(sum((o-theta).^2)); %#ok<SAGROW>
  error(j) = (theta(1,:) - o(1,:)) * (theta(1,:) - o(1,:))';  %#ok<SAGROW>
  error(j) = (theta(2,:) - o(2,:)) * (theta(2,:) - o(2,:))';  %#ok<SAGROW>
    
end

figure
hold on
plot (error, 'r.')
title('10614230:Error Data');
 

%Randomize second theta to test new dataset
theta2 = a+(b-a).* rand(2, 1000);

%%% First Dataset
[ Training_P1, Training_P2 ] = RevoluteForwardKinematics2D(armLen, theta2, origin);


 figure 
 hold on
 plot(theta2(1,:),theta2(2,:), 'r.')
 title('10614230:Random Joint Angle Data P2');

 figure
 hold on
 plot(Training_P2(1,:),Training_P2(2,:), 'r.')  % Plot using dot formation
 title('10614230:Endpoint Data P2');
 
 %Randomize third theta to test new dataset
 theta3 = a+(b-a).* rand(2, 1000);

 %%% Second Dataset
 [ DatasetP1, Dataset_P2 ] = RevoluteForwardKinematics2D(armLen, theta3, origin);

 
%  figure 
%  hold on
%  plot(theta3(1,:),theta3(2,:), 'r.')
%  title('10614230:Random Joint Angle Dataset 2');
% 
%  figure
%  hold on
%  plot(Dataset_P2(1,:),Dataset_P2(2,:), 'r.')  % Plot using dot formation
%  title('10614230:Endpoint Dataset 2');


 

% Inverse Kinematics with end points as input and joint angles as outputs

Train_P2 = [Dataset_P2;ones(1,1000)]; % Augment end effect positions
net2 = W1_tr*Train_P2;  % Internal activation
% Activation of the first layer
a2 = 1./(1+exp(-net2));
% Augment a2 to account for bias term in W2
ahat2 = [a2; ones(1,c)];
% Calculate output activations of layer 2
o = W2_tr*ahat2; % Joint angle as output

figure 
hold on
plot(o(1,:),o(2,:), 'r.')
title('10614230:Joint Angles as Output');

% Run forward kinematics function 
[ P1_trained , P2_trained ] = RevoluteForwardKinematics2D(armLen, o, origin);

% Plot end effect positions
figure
hold on
plot(P2_trained(1,:),P2_trained(2,:), 'r.')  % Plot using dot formation
title('10614230:Trained Endpoint Data');



% Save data from this script to use on other scripts
save('Train_Neural_Network.mat')
    

