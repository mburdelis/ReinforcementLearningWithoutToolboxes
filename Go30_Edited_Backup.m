%%%%%%%%%%%%%%%%%%%%%%%%
% Go30_Edited.m backup %
%%%%%%%%%%%%%%%%%%%%%%%%

clear;
disp('Executing IniParZLearningGW');
IniParZLearningGW30_Edited;
GoalStateIsObs = 0;

%   Verifying is the position of the goal ("GoalState") is not an obstacle
%   position
disp('Checking if the goal position is not an obstacle');
for i=1:options.SizeAuxObstacles
    if GoalState == options.AuxObstacles(i)
        GoalStateIsObs = 1;
        break;
    end
end

%   Executing the program only if the Goal State position is not an
%   obstacle position, and generating an error message otherwise
if GoalStateIsObs == 0
    disp('Generating the state list');
    GenerateStateList;
    %   now that we have the variable NStatePairs, we will create the
    %   variables necessary for the passive dynamics and state costs
    %   estimation
    disp('Finding the index of the goal state');
    FindGoalStatePosPairIndex;
    
    disp('Solving analytically');
    SolveClosedForm_StatePairs;
    
    disp('Calculating the optimal state transition distributions u');
    CalculateOptimalU;
    
    disp('Finding the maximum transition probability for each state');
    MaxOptUs = FindMaxProbFromOptU(States, OptU);
    
    disp('Generating the trajectories');
    [MapMatrix, TotalCost] = GenTrajGraph(GridSize, NStatePairs, q, P, GoalState, MaxOptUs, options, States, OptU, 1);
%     SolveClosedFormPRand_StatePairs;
%     CalculateOptimalUPRand;
%     MaxOptUsPRand = FindMaxProbFromOptU(States, OptUP_Rand);
%     [MapMatrix_PRand, TotalCost_PRand] = GenTrajGraph(GridSize, NStatePairs, q, P_Rand, GoalState, MaxOptUsPRand, options, States, OptUP_Rand, 1);
%     [MapMatrix_Comp, TotalCost_Comp] = GenTrajGraph(GridSize, NStatePairs, q, P, GoalState, MaxOptUsPRand, options, States, OptUP_Rand, 1);

    %   Initializing the estimated passive dynamics matrix
    disp('Initializing the estimated passive dynamics and calculating the number of adjacent states per state');
    [EstimatedPassive, NAdjPerState] = GenEstimPas(States, NStatePairs, GridSize, GoalState, StateType, options);
 
    %   Initializing the vector which will contain the numbers
    %   corresponding to the numbers of obtained equations per state
    NumObtEq = -(NAdjPerState < 0);
    
    disp('Separating which indexes correspond to good states, and which ones do not');
    [GoodIndexes, BadIndexes] = SeparateIndexes(options, States, GoalState, NStatePairs);
    
    disp('Running the algorithm');
    [ErrorsPd, LastErrorsPd, ErrorsQ, LastErrorsQ, v, Z, results, LogActions, ErrorZ, ErrorV, ErrorZSumAbs, ErrorZSumSqr, ErrorVSumAbs, ErrorVSumSqr, InitialStates, NumEqsPerState, EstimatedPassive, ErrorPdSumAbs, ErrorQSumAbs, ErrorPdCountLog, ErrorQCountLog, IndexesPdLog, IndexesQLog, ErrorsPdLog, ErrorsQLog]=ZlearningGWNonStop(M,T,options,States,NStatePairs,GoalState,GridSize,prev_or_next,z_opt,v_opt, EstimatedPassive, NAdjPerState);
    CheckResults;
    
    SumP = sum(sum(P));
    ErrorPdSumAbsNorm = ErrorPdSumAbs./SumP;
    
    SumQ = sum(q);
    ErrorQSumAbsNorm = ErrorQSumAbs./SumQ; 
    
    if options.ZOrVError == 1
        figure1 = CreateFigureWEta(ErrorV, options, 0, options.MaxTimeStep/options.RecErrorsSampFreq, 0, 1.5, 'V');
        saveas(figure1, strcat(options.RelPath, 'ErrorV30.fig'), 'fig');
        print('-djpeg', strcat(options.RelPath, 'ErrorV30.jpg'));
    elseif options.ZOrVError == 2
        figure1 = CreateFigureWEta(ErrorZ, options, 0, options.MaxTimeStep/options.RecErrorsSampFreq, 0, 40, 'Z');
        saveas(figure1, strcat(options.RelPath, 'ErrorZ30.fig'), 'fig');
        print('-djpeg', strcat(options.RelPath, 'ErrorZ30.jpg'));
    else
        figure1 = CreateFigureWEta(ErrorV, options, 0, options.MaxTimeStep/options.RecErrorsSampFreq, 0, 1.5, 'V');
        saveas(figure1, strcat(options.RelPath, 'ErrorV30.fig'), 'fig');
        print('-djpeg', strcat(options.RelPath, 'ErrorV30.jpg'));
        figure2 = CreateFigureWEta(ErrorZ, options, 0, options.MaxTimeStep/options.RecErrorsSampFreq, 0, 40, 'Z');
        saveas(figure2, strcat(options.RelPath, 'ErrorZ30.fig'), 'fig');
        print('-djpeg', strcat(options.RelPath, 'ErrorZ30.jpg'));
    end

    FigurePdQ = CreateFigureErrorsEst(ErrorPdSumAbsNorm, ErrorQSumAbsNorm, ErrorPdCountLog, ErrorQCountLog, options);
    saveas(FigurePdQ, strcat(options.RelPath, 'ErrorPdQ30.fig'), 'fig');
    print('-djpeg', strcat(options.RelPath, 'ErrorPdQ30.jpg'));
   
    figure
    FigureSemiPd = semilogy(ErrorPdSumAbsNorm);
    saveas(FigureSemiPd, strcat(options.RelPath, 'FigureSemiPd30.fig'), 'fig');
    print('-djpeg', strcat(options.RelPath, 'FigureSemiPd30.jpg'));
    
    figure
    FigureSemiQ = semilogy(ErrorQSumAbsNorm);
    saveas(FigureSemiQ, strcat(options.RelPath, 'FigureSemiQ30.fig'), 'fig');
    print('-djpeg', strcat(options.RelPath, 'FigureSemiQ30.jpg'));
    
    time = 1:1:size(ErrorV);
    eta = options.c./(options.c + time);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Generating the learning curves of the pd %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Classifying by number of adjacent states
    LearningCurvePd4 = ErrorsPdLog(find(NAdjPerState == 4), 1:9);
    LearningCurvePd6 = ErrorsPdLog(find(NAdjPerState == 6), 1:13);
    LearningCurvePd7 = ErrorsPdLog(find(NAdjPerState == 7), 1:15);
    LearningCurvePd8 = ErrorsPdLog(find(NAdjPerState == 8), 1:17);
    LearningCurvePd9 = ErrorsPdLog(find(NAdjPerState == 9), 1:19);
    
    % Calculating the average by number of adjacent states
    AvgLearningCurvePd4 = mean(LearningCurvePd4);
    AvgLearningCurvePd6 = mean(LearningCurvePd6);
    AvgLearningCurvePd7 = mean(LearningCurvePd7);
    AvgLearningCurvePd8 = mean(LearningCurvePd8);
    AvgLearningCurvePd9 = mean(LearningCurvePd9);
    
    % Creating and saving the graphics of the learning curves of the
    % passive dynamics
    figureAvgPd4 = CreateFigureLearningCurve(AvgLearningCurvePd4, 4, options, 0);
    figureAvgPd6 = CreateFigureLearningCurve(AvgLearningCurvePd6, 6, options, 0);
    figureAvgPd7 = CreateFigureLearningCurve(AvgLearningCurvePd7, 7, options, 0);
    figureAvgPd8 = CreateFigureLearningCurve(AvgLearningCurvePd8, 8, options, 0);
    figureAvgPd9 = CreateFigureLearningCurve(AvgLearningCurvePd9, 9, options, 0);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Generating the learning curves of the Q %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Classifying by number of adjacent states
    LearningCurveQ4 = ErrorsQLog(find(NAdjPerState == 4), 1:9);
    LearningCurveQ6 = ErrorsQLog(find(NAdjPerState == 6), 1:13);
    LearningCurveQ7 = ErrorsQLog(find(NAdjPerState == 7), 1:15);
    LearningCurveQ8 = ErrorsQLog(find(NAdjPerState == 8), 1:17);
    LearningCurveQ9 = ErrorsQLog(find(NAdjPerState == 9), 1:19);
    
    % Calculating the average by number of adjacent states
    AvgLearningCurveQ4 = mean(LearningCurveQ4);
    AvgLearningCurveQ6 = mean(LearningCurveQ6);
    AvgLearningCurveQ7 = mean(LearningCurveQ7);
    AvgLearningCurveQ8 = mean(LearningCurveQ8);
    AvgLearningCurveQ9 = mean(LearningCurveQ9);
    
    % Creating and saving the graphics of the learning curves of the
    % passive dynamics
    figureAvgQ4 = CreateFigureLearningCurve(AvgLearningCurveQ4, 4, options, 1);
    figureAvgQ6 = CreateFigureLearningCurve(AvgLearningCurveQ6, 6, options, 1);
    figureAvgQ7 = CreateFigureLearningCurve(AvgLearningCurveQ7, 7, options, 1);
    figureAvgQ8 = CreateFigureLearningCurve(AvgLearningCurveQ8, 8, options, 1);
    figureAvgQ9 = CreateFigureLearningCurve(AvgLearningCurveQ9, 9, options, 1);
    
  
    disp('Saving the results');
    save('ErrorV30.mat', 'ErrorV');
    save('ErrorZ30.mat', 'ErrorZ');
    save('ErrorsPd30.mat', 'ErrorsPd');
    save('ErrorsQ30.mat', 'ErrorsQ');
    save('LastErrorsPd30.mat', 'LastErrorsPd');
    save('LastErrorsQ30.mat', 'LastErrorsQ');
    save('v30.mat', 'v');
    save('Z30.mat', 'Z');
    save('ErrorPdSumAbs30.mat', 'ErrorPdSumAbs');
    save('ErrorQSumAbs30.mat', 'ErrorQSumAbs');
    save('ErrorPdSumAbsNorm30.mat', 'ErrorPdSumAbsNorm');
    save('ErrorQSumAbsNorm30.mat', 'ErrorQSumAbsNorm');
    save(strcat(options.RelPath, 'ErrorPdCountLog30.mat'), 'ErrorPdCountLog');
    save(strcat(options.RelPath, 'ErrorQCountLog30.mat'), 'ErrorQCountLog');
    save(strcat(options.RelPath, 'IndexesPdLog30.mat'), 'IndexesPdLog');
    save(strcat(options.RelPath, 'IndexesQLog30.mat'), 'IndexesQLog');
    save(strcat(options.RelPath, 'ErrorsPdLog30.mat'), 'ErrorsPdLog');
    save(strcat(options.RelPath, 'ErrorsQLog30.mat'), 'ErrorsQLog');
    save(strcat(options.RelPath, 'LearningCurvePd4'), 'LearningCurvePd4');
    save(strcat(options.RelPath, 'AvgLearningCurvePd4'), 'AvgLearningCurvePd4');
    save(strcat(options.RelPath, 'LearningCurvePd6'), 'LearningCurvePd6');
    save(strcat(options.RelPath, 'AvgLearningCurvePd6'), 'AvgLearningCurvePd6');
    save(strcat(options.RelPath, 'LearningCurvePd7'), 'LearningCurvePd7');
    save(strcat(options.RelPath, 'AvgLearningCurvePd7'), 'AvgLearningCurvePd7');
    save(strcat(options.RelPath, 'LearningCurvePd8'), 'LearningCurvePd8');
    save(strcat(options.RelPath, 'AvgLearningCurvePd8'), 'AvgLearningCurvePd8');
    save(strcat(options.RelPath, 'LearningCurvePd9'), 'LearningCurvePd9');
    save(strcat(options.RelPath, 'AvgLearningCurvePd9'), 'AvgLearningCurvePd9');
else
    ShowMessage = 'Error: goal state is located in an "obstacle-position": aborting execution'
end