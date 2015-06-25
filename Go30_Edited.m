clear;
disp('Executing IniParZLearningGW');
IniParZLearningGW30;

%   Verifying is the position of the goal ("GoalState") is not an obstacle
%   position
disp('Checking if the goal position is not an obstacle');
GoalStateIsObs = CheckPosIsObstacle(options.input.GoalState, options.input.AuxObstacles);

%   Executing the program only if the Goal State position is not an
%   obstacle position, and generating an error message otherwise
if GoalStateIsObs == 0
    disp('Generating the state list');
    if options.StateType == 2
		GenerateStateList;
    else
        [States, NStates] = GenerateStateList_PosIsSt(options.input.GridSize);
	end

%   now that we have the variable NStatePairs, we will create the
%   variables necessary for the passive dynamics and state costs
%   estimation
	disp('Finding the index of the goal state');
    if options.StateType == 2
		FindGoalStatePosPairIndex;
    else
        options.input.GoalStatePosPairIndex = options.input.GoalState;
	end
	
%	Now solving the problem analytically
	disp('Solving analytically');
	if options.StateType == 2
		SolveClosedForm_StatePairs;
	else
		% SolveClosedForm_PosIsSt;
        [P, q, z_opt, v_opt, Matrix, Matrix_z_opt] = SolveClosedForm_PosIsSt(States, NStates, options.input.GridSize, options.input.GoalState, options.input.AuxObstacles, options.input.GoalStatePosPairIndex, options.input.OtherStatesCost, options.input.GoalStateCost);
    end

    disp('Calculating the optimal state transition distributions u');
    if options.StateType == 2
		[OptU] = CalculateOptimalU_StPosPair(States, NStates, options.input.AuxObstacles, options.input.SizeAuxObstacles, z_opt, q, P);
	else
        [OptU] = CalculateOptimalU_PosIsSt(States, NStates, options.input.AuxObstacles, options.input.SizeAuxObstacles, z_opt, q, P);
    end
    
    disp('Finding the maximum transition probability for each state');
    if options.StateType == 2
		MaxOptUs = FindMaxProbFromOptU(States, OptU);
    else
        % For MaxOptUs, the first column will be the maximum probability u, the
        % second column will be the corresponding future state which has this
        % highest probability according to u
        MaxOptUs = FindMaxProbFromOptU_PosIsSt(OptU);
    end
    
    disp('Generating the trajectories');
    [MapMatrix, TotalCost] = GenTrajGraph(GridSize, NStatePairs, q, P, GoalState, MaxOptUs, options, States, OptU, 1);
    
else % if GoalStateIsObs == 0
    disp('Error: goal state is located in an "obstacle-position": aborting execution');
end % if GoalStateIsObs == 0



