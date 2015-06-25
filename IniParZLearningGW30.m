options.input.GridSize = 10;
options.input.GoalState = options.input.GridSize^2;   %   Initializing the Goal State
                    
options.StateType = 1;	%	2: state pairs (for inertia and collisions)
		%	any other value: 1 position = 1 state

options.EstimPasType = 0;   %   1: random walk
                            %   any other value: random probabilities
                            %   (different than 0) to the adjacent states

options.MaxTimeStep = 1*10^3;
options.log.LogActSampFreq = 100;
options.log.ExibirSampFreq = 100;

options.pmode = 9;  % The policy to be followed by the agent:
                    % 1: purely greedy
                    % 2: "epsilon" greedy
                    % 3: softmax (not working, do not use)
                    % 4: random walk
                    % 5: equal to passive dynamics
                    % 6: use the policy which appears optimal given the 
                    % current estimate of z ("greedy Z learning" according 
                    % to Todorov's PNAS'09 paper)
                    % 7: Simulating symbolic actions, and taking a random
                    % symbolic action at every time
                    % 8: Simulating symbolic actions, and using the greedy
                    % policy with the action
                    % 9 using the policy which appears optimal given the
                    % current estimates of z and pd (greedy Z learning)
                    % 10 use a policy with random values as probabilities
                    % (normalized to sum up to one)

options.val.epsilon = 0.5;

options.AdaptEpsilon = 0;   %   if == 0 does not change the epsilon during execution
                            %   if == any other value: changes the epsilon
                            %   from more explorative to more greedy during
                            %   execution (and ignores the value in
                            %   "options.epsilon" in this file)

options.val.tau = 0.5;
options.val.DiscCostGamma = 0.9;
options.val.GradDesGamma = 10;
options.val.eta = 0.1;  % learning rate used in Z-learning
options.val.c = 100;  % parameter to be used if decay is enabled

options.endecay = 1;    % enable decay of the learning rate "eta" by (c/(c+t))
                        % where t is the current step
                        % 0: disable;
                        % 1: enable.
options.pnull = 1;  % determines what the program should do when the Z.*Legal is zero:
                    % 1: random walk;
                    % any other value: follow the passive dynamics.
options.passive = 3;    % determines the passive dynamics:
                        % Used only when the policy is calculated: if the
                        % policy is supposed to be random walk, and
                        % options.passive == 1, the "Passive" matrix is
                        % used to calculate the policy
                        % 1: random walk;
                        % 2: Newtonian mechanics with absorptive walls and obstacles (which absorb impacts);
                        % 3: Newtonian mechanics with reflexive walls and obstacles (the
                        % agent bounces when hits the wall).
                        
if options.pmode == 5                       
    options.ImpSamp = 0;    % determines if "Importance sampling" should be used or not:    
                            % options.ImpSamp = 0: do not use
                            % any other value: use
else
    options.ImpSamp = 1;
end

options.ImpSampType = 0;    %   determines how the "u" which appears optimal (used in the 
                            %   importance sampling formula in the Z update) should be calculated:
                            %   2:  force "epsilon-greedy" policy to be
                            %       used as the "u" 
                            %   any other value: use the current "policy"

options.LogBase = 1;   % determines the base of the log operation in the KL divergence calculation:
                        % 10: log base 10
                        % any other value: log base "e" (ln)
options.input.GoalStateCost = 0; % state cost of the goal state
options.input.OtherStatesCost = 1; % state cost of other states (which don't include the goal)
options.input.InitialValue = 1; % value to initialize the entire v function (except for the cost of the goal state)
options.RandomStart = 1;   %   Determines if the algorithm chooses a random position as the initial state
                            %   1: chooses a random position;
                            %   any other value: always starts at state 1.
options.val.HighProbVal = 0.9;   %   Value of the highest possible probability (given by the passive dynamics)
options.WallType = 0;	%   Type of wall
                        %   1: reflects impacts
                        %   any other value: absorbs impacts
options.log.RecErrorsSampFreq = 100; %   sampling rate for recording errors
options.val.HPChosenAction = 0.8;   %   when policy 7 is applied, the chosen
                                %   state (corresponding to the action)
                                %   will have this probability
                                
%   IMPORTANT: make some test to check if k does not have an absurd value?
%   It cannot be smaller than 1                                
options.val.k = 2;      %   the proportion to which, when policy 7 is applied,
                    %   in the remaining states (not the one with the
                    %   highest probability) the ones with a common
                    %   direction with the chosen one will have a
                    %   probability "k" times bigger
                    
options.ZOrVError = 0;  % if = 1 makes the figure with the error of V
                        % if = 2, makes the figure with the
                        % error of Z
                        % if == any other value, makes both figures

options.UseManualAxes = 1;  %   If == 1 uses the axes limits sent manually as a parameter to "CreateFigureWEta";
                            %   otherwise defines the axes limits automatically

                               
options.EqsApproachContEmb = 1; %   if == 1 uses the approach of the "Continuous Embedding" application (PNAS'09) to solve the equations
                                %   if == any other value uses the gradient
                                %   descent approach (with normalization at
                                %   each step)                      
                                
options.UseEstValues = 1;       %   use the estimated Pd and estimated q in the update equation of the Z estimate
                                %   if == 0 uses the correct Pd and q at
                                %   the Z update
                                %   if == any other value, uses the current
                                %   estimates

% options.ChangeUseEstVal = 1.5;
options.ChangeUseEstVal = 0;	%   if == 0 does not change 
                                %   options.UseEstValues automatically, and
                                %   always uses either the correct values 
                                %   or the estimated ones, according to 
                                %   options.UseEstValues   
                                %   if == any other value, changes from the
                                %   estimated values to the correct values
                                %   in (options.MaxTimeStep /
                                %   options.ChangeUseEstVal), by changing
                                %   the value recorded in
                                %   options.UseEstValues
                                
options.NumEqsNadjMult = 1; %   This number will multiply the number of
                            %   adjacent states to determine the number of
                            %   equations per state that will be gathered
                            %   and solved
                            %   This ALSO influences in the size of the
                            %   variable "Equations"
                            %   With value 3 this seems to freeze the
                            %   computer!!
                            %   Also, if options.UseTotalCosts == 0 then
                            %   this variable value will be substituted by
                            %   1 when the code starts
                            
options.NumEqsNadjSum = 2;  %   this number will be summed to the number of desired equations.                            
                            
options.ForceCorrectPdAndQ = 0; %   If == 0 does not force the correct Pd 
                                %   and Q values at the end of the
                                %   estimation: does the estimation 
                                %   normally
                                %   If == any other value forces the
                                %   correct Pd and Q (previousy known) when
                                %   the desired number of equations is
                                %   reached
% options.UseTotalCosts = 1;                                
options.UseTotalCosts = 1;  %   If == 0 estimates only the passive 
                            %   dynamics, and uses only action costs for
                            %   estimation.
                            %   If == any other value, uses total costs and
                            %   attempts to also estimate q

options.val.ErrorThreshold = 1e-10; %   Thershold to be considered by the function which counts the errors "CountErrors"

options.val.MinPolThreshold = 1e-10;    %   Threshold to be considered for the 
                                    %   minimum acceptable policy value 
                                    %   when using the estimated passive in
                                    %   the equation of the greedy policy
                                
%   Related to obstacles:
options.input.AuxObstacles = [3, 13, 23, 33, 38, 43, 48, 53, 58, 63, 68, 78, 88, 98];    %   auxiliary variable with the locations of the obstacles
[L, options.input.SizeAuxObstacles] = size(options.input.AuxObstacles);
options.input.Obstacles = zeros(1, (options.input.GridSize^2));  %   Location of obstacles (walls)
for i=1:options.input.SizeAuxObstacles
    Index = options.input.AuxObstacles(i);
    options.input.Obstacles(Index) = 1;
end
options.log.DebugStopFreq = 100;

options.RelPath = '.\Simulations\Temp\';
mkdir(options.RelPath);

options.GetMode = 1;    %   Mode which is used for getting the Last error
                        %   if == 0 gets the last non-zero element of the
                        %   line (from left to right);
                        %   if == any other value gets the element which
                        %   correspond to the number of equations, and this
                        %   element shows up on the second column (the
                        %   first column contains "-1" if
                        %   NumEqsPerState < NAdjPerState; or "1" otherwise
                        
options.val.Seed = 30;       %   Seed for generating random numbers
                        
%   Choosing the method to generate random numbers and the state of the
%   generator  
rand('state',options.val.Seed);