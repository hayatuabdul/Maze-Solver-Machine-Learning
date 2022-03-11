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
% 07/10/2020
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


classdef CMazeMaze11x11
    % define Maze work for RL
    %  Detailed explanation goes here
    
    properties
        
        % parameters for the gmaze grid management
        %scalingXY;
        blockedLocations;
        cursorCentre;
        limitsXY;
        xStateCnt
        yStateCnt;
        stateCnt;
        stateNumber;
        totalStateCnt
        squareSizeX;
        cursorSizeX;
        squareSizeY;
        cursorSizeY;
        stateOpen;
        stateStart;
        stateEnd;
        stateEndID;
        stateX;
        stateY;
        xS;
        yS
        stateLowerPoint;
        textLowerPoint;
        stateName;
        
        % parameters for Q learning
        QValues;
        tm;
        actionCnt;
    end
    
    methods
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % constructor to specity maze
        function f = CMazeMaze11x11(limitsXY)
            
            % set scaling for display
            f.limitsXY = limitsXY;
            f.blockedLocations = [];
            
            % setup actions
            f.actionCnt = 4;
            
            % build the maze
            f = SimpleMaze11x11(f);
            
            % display progress
            disp(sprintf('Building Maze CMazeMaze11x11'));
            
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % build the maze
        function f = SetMaze(f, xStateCnt, yStateCnt, blockedLocations, startLocation, endLocation)
            
            % set size
            f.xStateCnt=xStateCnt;
            f.yStateCnt=yStateCnt;
            f.stateCnt = xStateCnt*yStateCnt;
            
            % compute state countID
            for x =  1:xStateCnt
                for y =  1:yStateCnt
                    
                    % get the unique state identified index
                    ID = x + (y -1) * xStateCnt;
                    
                    % record it
                    f.stateNumber(x,y) = ID;
                    
                    % also record how x and y relate to the ID
                    f.stateX(ID) = x;
                    f.stateY(ID) = y;
                end
            end
            
            % calculate maximum number of states in maze
            % but not all will be occupied
            f.totalStateCnt = f.xStateCnt * f.yStateCnt;
            
            
            % get cell centres
            f.squareSizeX= 1 * (f.limitsXY(1,2) - f.limitsXY(1,1))/f.xStateCnt;
            f.cursorSizeX = 0.5 * (f.limitsXY(1,2) - f.limitsXY(1,1))/f.xStateCnt;
            f.squareSizeY= 1 * (f.limitsXY(2,2) - f.limitsXY(2,1))/f.yStateCnt;
            f.cursorSizeY = 0.5 * (f.limitsXY(2,2) - f.limitsXY(2,1))/f.yStateCnt;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % init maze with no closed cell
            f.stateOpen = ones(xStateCnt, yStateCnt);
            f.stateStart = startLocation;
            f.stateEnd = endLocation;
            f.stateEndID = f.stateNumber(f.stateEnd(1),f.stateEnd(2));
            
            % put in blocked locations
            for idx = 1:size(blockedLocations,1)
                bx = blockedLocations(idx,1);
                by = blockedLocations(idx,2);
                f.stateOpen(bx, by) = 0;
            end
            
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % get locations for all states
            for x=1:xStateCnt
                for y=1:yStateCnt
                    
                    % start at (0,0)
                    xV = x-1;
                    yV = y-1;
                    
                    % pure scaling component
                    % assumes input is between 0 - 1
                    scaleX =  (f.limitsXY(1,2) - f.limitsXY(1,1)) / xStateCnt;
                    scaleY = (f.limitsXY(2,2) - f.limitsXY(2,1)) / yStateCnt;
                    
                    % remap the coordinates and add on the specified orgin
                    f.xS(x) = xV  * scaleX + f.limitsXY(1,1);
                    f.yS(y) = yV  * scaleY + f.limitsXY(2,1);
                    
                    % remap the coordinates, add on the specified orgin and add on half cursor size
                    f.cursorCentre(x,y,1) = xV * scaleX + f.limitsXY(1,1) + f.cursorSizeX/2;
                    f.cursorCentre(x,y,2) = yV * scaleY + f.limitsXY(2,1) + f.cursorSizeY/2;
                    
                    f.stateLowerPoint(x,y,1) = xV * scaleX + f.limitsXY(1,1);  - f.squareSizeX/2;
                    f.stateLowerPoint(x,y,2) = yV * scaleY + f.limitsXY(2,1); - f.squareSizeY/2;
                    
                    f.textLowerPoint(x,y,1) = xV * scaleX + f.limitsXY(1,1)+ 10 * f.cursorSizeX/20;
                    f.textLowerPoint(x,y,2) = yV * scaleY + f.limitsXY(2,1) + 10 * f.cursorSizeY/20;
                end
            end
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % draw rectangle
        function DrawSquare( f, pos, faceColour)
            % Draw rectagle
            rectangle('Position', pos,'FaceColor', faceColour,'EdgeColor','k', 'LineWidth', 3);
        end
        
        % draw circle
        function DrawCircle( f, pos, faceColour)
            % Draw rectagle
            rectangle('Position', pos,'FaceColor', faceColour,'Curvature', [1 1],'EdgeColor','k', 'LineWidth', 3);
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % draw the maze
        function DrawMaze(f)
            figure('position', [100, 100, 1200, 1500]);
            fontSize = 20;
            hold on
            h=title(sprintf('10614230: Maze wth %d x-axis X %d y-axis cells', f.xStateCnt, f.yStateCnt));
            set(h,'FontSize', fontSize);
            
            for x=1:f.xStateCnt
                for y=1:f.yStateCnt
                    pos = [f.stateLowerPoint(x,y,1)  f.stateLowerPoint(x,y,2)  f.squareSizeX f.squareSizeY];
                    
                    % if location open plot as blue
                    if(f.stateOpen(x,y))
                        DrawSquare( f, pos, 'b');
                        % otherwise plot as black
                    else
                        DrawSquare( f, pos, 'k');
                    end
                end
            end
            
            
            % put in start locations
            for idx = 1:size(f.stateStart,1)
                % plot start
                x = f.stateStart(idx, 1);
                y = f.stateStart(idx, 2);
                pos = [f.stateLowerPoint(x,y,1)  f.stateLowerPoint(x,y,2)  f.squareSizeX f.squareSizeY];
                DrawSquare(f, pos,'g');
            end
            
            % put in end locations
            for idx = 1:size(f.stateEnd,1)
                % plot end
                x = f.stateEnd(idx, 1);
                y = f.stateEnd(idx, 2);
                pos = [f.stateLowerPoint(x,y,1)  f.stateLowerPoint(x,y,2)  f.squareSizeX f.squareSizeY];
                DrawSquare(f, pos,'r');
            end
            
            % put on names
            for x=1:f.xStateCnt
                for y=1:f.yStateCnt
                    sidx=f.stateNumber(x,y);
                    stateNameID = sprintf('%s', f.stateName{sidx});
                    text(f.textLowerPoint(x,y,1),f.textLowerPoint(x,y,2), stateNameID, 'FontSize', 20)
                end
            end
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % setup 11x11 maze
        function f = SimpleMaze11x11(f)
            
            xCnt=11;
            yCnt=11;
            
            % specify start location in (x,y) coordinates
            % example only
            startLocation=[1 1];
            % YOUR CODE GOES HERE
            
            
            % specify end location in (x,y) coordinates
            % example only
            endLocation=[11 11];
            % YOUR CODE GOES HERE
            
            
            % specify blocked location in (x,y) coordinates
            % example only
            f.blockedLocations = [1 5; 1 8; 2 1; 2 4; 2 6; 2 10;
                                3 1; 3 2; 3 10; 4 1; 4 3; 4 4;
                                4 7; 4 8; 4 11; 5 7; 5 8; 5 11;
                                6 1; 6 2; 6 3; 6 4; 6 8; 7 3;
                                7 6; 7 8; 7 9; 8 1; 8 8; 9 4;
                                9 5; 9 9; 10 2; 10 4; 10 7;
                                10 9; 10 10; 11 4; 11 5; 11 7;];
            % YOUR CODE GOES HERE
            
            % build the maze
            f = SetMaze(f, xCnt, yCnt, f.blockedLocations, startLocation, endLocation);
            
            % write the maze state
            maxCnt = xCnt * yCnt;
            for idx = 1:maxCnt
                f.stateName{idx} = num2str(idx);
            end
            
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % reward function that takes a state and an action
        function reward = RewardFunction(f, state, action)
            
            % init to no reqard

             % Initalize action
             North = 1;
             East  = 2;
            
           if (state == 120 && action == East) || (state == 110 && action == North)
                    reward = 10;
                 
           else
                    reward = 0;  
           end
            
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % function that computes random starting states
        function startingState = RandomStartingState(f)
            
            %Generate 1000 Random Starting states  
            
                % Below are the movable state. Blocked states are not included
            a = [1;5;7;9;10;11;12;13;15;16;18;19;20;22;23;24;25;27;30;31;32;33;34;36;38;40;41;46;47;48;49;50;51;52;54;56;58;59;60;61;63;64;65;66;
                 67;68;69;72;73;74;75;79;80;86;87;88;89;90;91;92;93;94;96;99;100;103;104;105;106;107;108;110;111;112;113;116;117;118;119;120];
        
                 % Randomise and select one value from the array
                 rand_a = a(randi(length(a)));

                 startingState = rand_a; 
           
            
        end
     
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % look for end state
        function endState = IsEndState(f, x, y)
            
            % default to not found
            endState=0;
            
            % YOUR CODE GOES HERE ....
            
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % init the q-table
        function f = InitQTable(f, minVal, maxVal)
            
            
            % allocate
            f.QValues = zeros(f.xStateCnt * f.yStateCnt, f.actionCnt);
             
            range = maxVal - minVal;
            mean = (maxVal + minVal)/2;
            f.QValues = range * (rand(121, 4) - 0.5) + mean;
            
%              figure
%              hold on
%              surf(f.QValues);
%              axis([1 4 1 121 0 1])
%              view([35 45]);
             
        
            
      
        end
        
        
              %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Greedy Function
        function action = GreedyActionSelection(f, q, state, e)
            
            actionvalues = [1 2 3 4];
            
           % Randomly ask if e is 10%
          if rand() < e
           
           action = actionvalues(randi(length(actionvalues))); % Remaining e percent returns a random action
           
          else     
         
           [ Y, action] = ( max(q(state, :))); % Action gets updated with max Q values 90% of the time
        
          end 
       
        end
        
        
                 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Update Q function
        function q = UpdateQ(f, q, state, action, resultingState, reward, learning_rate, gamma)
           
              q(state,action) = q(state,action)+learning_rate*(reward+gamma*max(q(resultingState,action))-q(state,action));
               
       
        end
        
        
  
                  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Implement Q-Learning for Trial and Episodes
        function [q, steps_mean,  steps_std, q_table, steps_no, states_visited] = Trial(f, q, learning_rate, gamma, e)
        
        % Initialize Q table once for 121 states and 4 actions
        f = f.InitQTable(121,4);
        
         for episodes = 1 : 1000  % Run for 1000 episodes
             
                   
              % Choose a random starting state
              %state = f.RandomStartingState();
              % Choose green starting state
              state = 1;
              % Assign goal state
              goal = 121;
              steps = 0;

             % Loop that runs until the goal state is reached
          while (state ~= goal)
              % Make greedy action selection
              action = f.GreedyActionSelection(q, state, e);
              % Get the next state due to that action
              resultingState = f.tm(state, action);
              % Get the reward function from the action of previous state
              reward = f.RewardFunction(state, action);
              % Update Q-Table
              q = f.UpdateQ(q, state, action, resultingState, reward, learning_rate, gamma);
              % Choose what episode to plot in the maze
              recorded_episode = 1000;
                  if (episodes == recorded_episode)
                     states_visited(steps+1) = resultingState;  % Record every state for that episode
                  end
              % Termination if reaches goal
              if(resultingState == goal) 
                   break;
              end
              % Update steps
              steps = steps + 1;
              % Update the state
              state = resultingState;      
              
          end
      
          % Calculate mean and standard deviation for steps then record
          steps_mean(episodes)= mean(steps);
          steps_std(episodes)= std(steps);
          steps_no(episodes)= steps;
          
          
          

         end
           q_table = q; % Records Q table at the end of trial
               
       
        end
        
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % % build the transition matrix
        % look for boundaries on the grid
        % also look for blocked state
        function f = BuildTransitionMatrix(f)
           
            % allocate
            f.tm = zeros(f.xStateCnt * f.yStateCnt, f.actionCnt);
            
            %Make a for loop to go through each state
             for xstate = 1:f.xStateCnt
              for ystate = 1:f.yStateCnt   
                        % Assign direction of action
                        MoveNorth = ystate+1;
                        MoveEast  = xstate+1;
                        MoveSouth = ystate-1;
                        MoveWest  = xstate-1;
                        
                     % If the states are open on both axis   
                  if (f.stateOpen(xstate,ystate))
                            
                        % if state above is blocked or if it reaches top border                
                        if (ystate == 11) || (~f.stateOpen(xstate,ystate+1)) 
                              MoveNorth = ystate; % Don't go up
                        end
                        
                        % if state by the right is blocked or On the right border
                        if (xstate == 11) || (~f.stateOpen(xstate+1,ystate)) 
                              MoveEast = xstate; % Don't go right
                        end
                        
                        % if state below is blocked or On the bottom border
                        if (ystate == 1) || (~f.stateOpen(xstate,ystate-1)) 
                              MoveSouth = ystate; % Don't go down
                        end
                        
                        % if state by the left is blocked or On the left border
                        if (xstate == 1) || (~f.stateOpen(xstate-1,ystate)) 
                              MoveWest = xstate; % Don't go left
                        end
                       
                        % Convert and map state numbers to action
                        MoveNorth = f.stateNumber(xstate,MoveNorth);
                        MoveEast  = f.stateNumber(MoveEast,ystate);
                        MoveSouth = f.stateNumber(xstate,MoveSouth);
                        MoveWest  = f.stateNumber(MoveWest,ystate);
                        
                        % Automatically update the states each time
                        % Convert the state numbers into actual states
                        upd_state = f.stateNumber(xstate,ystate);
                        
                        % Run the algorithm to map the actions of the states
                        % Also maps the order of the matrix
                        f.tm(upd_state,:)=[MoveNorth, MoveEast, MoveSouth, MoveWest];
                  end
                
               end
             end
            
                        
                        
                  
            
        end
        
    end
end

