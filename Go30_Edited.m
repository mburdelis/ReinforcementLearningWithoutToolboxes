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
    
if options.StateType == 2
    %   now that we have the variable NStatePairs, we will create the
    %   variables necessary for the passive dynamics and state costs
    %   estimation
    disp('Finding the index of the goal state');
    FindGoalStatePosPairIndex;
   
    %	Now solving the problem analytically
	disp('Solving analytically');
    SolveClosedForm_StatePairs;
    
    disp('Calculating the optimal state transition distributions u');
    [OptU] = CalculateOptimalU_StPosPair(States, NStates, options.input.AuxObstacles, options.input.SizeAuxObstacles, z_opt, q, Passive);
    
    disp('Finding the maximum transition probability for each state');
    MaxOptUs = FindMaxProbFromOptU(States, OptU);
    
    disp('Generating the trajectories');
    [MapMatrix, TotalCost] = GenTrajGraph(GridSize, NStatePairs, q, Passive, GoalState, MaxOptUs, options, States, OptU, 1);
    
    %   Initializing the estimated passive dynamics matrix
    disp('Initializing the estimated passive dynamics and calculating the number of adjacent states per state');
    [EstimatedPassive, NAdjPerState] = GenEstimPas(States, NStatePairs, GridSize, GoalState, StateType, options);
    
    %   Initializing the vector which will contain the numbers
    %   corresponding to the numbers of obtained equations per state
    NumObtEq = -(NAdjPerState < 0);
    
    disp('Separating which indexes correspond to good states, and which ones do not');
    [GoodIndexes, BadIndexes] = SeparateIndexes(options, States, GoalState, NStatePairs);
    
    %%%%%%%%
    % Fazer aqui o Estimated_q
    
    disp('Running the algorithm');
    [ErrorsPd, LastErrorsPd, ErrorsQ, LastErrorsQ, v, Z, results, LogActions, ErrorZ, ErrorV, ErrorZSumAbs, ErrorZSumSqr, ErrorVSumAbs, ErrorVSumSqr, InitialStates, NumEqsPerState, EstimatedPassive, ErrorPdSumAbs, ErrorQSumAbs, ErrorPdCountLog, ErrorQCountLog, IndexesPdLog, IndexesQLog, ErrorsPdLog, ErrorsQLog]=ZlearningGWNonStop(M,T,options,States,NStatePairs,GoalState,GridSize,prev_or_next,z_opt,v_opt, EstimatedPassive, NAdjPerState);
else % if options.StateType == 2
    %   now that we have the variable NStatePairs, we will create the
    %   variables necessary for the passive dynamics and state costs
    %   estimation
    disp('Finding the index of the goal state');
    options.input.GoalStatePosPairIndex = options.input.GoalState;
    
    %	Now solving the problem analytically
	disp('Solving analytically');
    % SolveClosedForm_PosIsSt;
    [Passive, q, z_opt, v_opt, Matrix, Matrix_z_opt] = SolveClosedForm_PosIsSt(States, NStates, options.input.GridSize, options.input.GoalState, options.input.AuxObstacles, options.input.GoalStatePosPairIndex, options.input.OtherStatesCost, options.input.GoalStateCost);
    
    disp('Calculating the optimal state transition distributions u');
    [OptU] = CalculateOptimalU_PosIsSt(States, NStates, options.input.AuxObstacles, z_opt, q, Passive);
    
    disp('Finding the maximum transition probability for each state');
    % For MaxOptUs, the first column will be the maximum probability u, the
    % second column will be the corresponding future state which has this
    % highest probability according to u
    MaxOptUs = FindMaxProbFromOptU_PosIsSt(OptU);
    
    disp('Generating the trajectories');
    [MapMatrix, TotalCost] = GenTrajGraph_PosIsSt(States, NStates, options.input.GridSize, options.input.GoalState, options.input.AuxObstacles, options.input.SizeAuxObstacles, q, Passive, MaxOptUs, OptU, 1);
    
    %   Initializing the estimated passive dynamics matrix
    disp('Initializing the estimated passive dynamics and calculating the number of adjacent states per state');
    [EstimatedPassive, NAdjPerState] = GenEstimPas_PosIsSt(NStates, options.input.GridSize, options.input.GoalState, options.input.AuxObstacles, options.EstimPasType);
    
    %   Initializing the vector which will contain the numbers
    %   corresponding to the numbers of obtained equations per state
    NumObtEq = -(NAdjPerState < 0);
    
    disp('Separating which indexes correspond to good states, and which ones do not');
    [GoodIndexes, BadIndexes, OnesObstacles, OnesNonObstacles] = SeparateIndexes_PosIsSt(States, options.input.AuxObstacles);
    
    %   Initializing the estimated q vector
    disp('Initializing the estimated q');
    [Estimated_q, options.NumEqsNadjMult, options.NumEqsNadjSum] = GenEstimQ_PosIsSt(NStates, options.input.GoalState, options.UseTotalCosts, options.input.GoalStateCost, options.input.OtherStatesCost, options.NumEqsNadjMult, options.NumEqsNadjSum);
    
    disp('Running the algorithm');
    [ErrorsPd, LastErrorsPd, ErrorsQ, LastErrorsQ, v, Z, results, LogActions, ErrorZ, ErrorV, ErrorZSumAbs, ErrorZSumSqr, ErrorVSumAbs, ErrorVSumSqr, InitialStates, NumEqsPerState, EstimatedPassive, ErrorPdSumAbs, ErrorQSumAbs, ErrorPdCountLog, ErrorQCountLog, IndexesPdLog, IndexesQLog, ErrorsPdLog, ErrorsQLog]=ZlearningGWNonStop_PosIsSt(options,States,NStatePairs,GoalState,GridSize,prev_or_next,z_opt,v_opt, EstimatedPassive, NAdjPerState, Estimated_q, Passive, q, OnesObstacles, OnesNonObstacles);
end % if options.StateType == 2

    CheckResults;
    
else % if GoalStateIsObs == 0
    disp('Error: goal state is located in an "obstacle-position": aborting execution');
end % if GoalStateIsObs == 0



