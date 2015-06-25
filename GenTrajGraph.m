function [MapMatrix, TotalCost] = GenTrajGraph(GridSize, NStatePairs, q, P, GoalState, MaxOptUs, options, States, OptU, IniPosition)
    
    MapMatrix = zeros(GridSize, GridSize);
    TrajStep = 1;
    CurPosition = IniPosition;
    PrevPosition = IniPosition;
    TotalCost = 0;
    
    IniPosError = 0;
    %   Checking if the initial position exists in the map
    %   (the program can only proceed if it does)
    if (IniPosition < 0)||(IniPosition > (GridSize^2))
        IniPosError = 1;
    end
    
    if IniPosError == 0
        %   Checking if the initial position is an obstacle (the program can
        %   only proceed if not)
        for IndexObs=1:options.SizeAuxObstacles
            if IniPosition == options.AuxObstacles(IndexObs)
                IniPosError = 1;
                break;
            end    
        end
    end

    if IniPosError == 0
        %   Drawing the obstacles
        for IndexObs=1:options.SizeAuxObstacles
            ObsPosition = options.AuxObstacles(IndexObs);
            [lin, col] = GetLinCol(ObsPosition, GridSize);
            MapMatrix(lin, col) = -1;
        end
        
        %   Drawing the initial position (it will receive the value "1" in
        %   the matrix)
        [lin, col] = GetLinCol(CurPosition, GridSize);
        MapMatrix(lin, col) = TrajStep;
        

        
        %   Drawing the trajectory
        while CurPosition ~= GoalState
            %   Finding the state corresponding to the current position (obs:
            %   it is assumed that the agent was initially resting at the
            %   initial position i.e. the previous and the current position for
            %   the initial state are the same)
            for AuxIndexState=1:NStatePairs
                if States(AuxIndexState, 1)==PrevPosition && States(AuxIndexState, 2)==CurPosition
                    CurState = AuxIndexState;
                break;
                end
            end
            
            %   Now that the Current state was found, it is possible to
            %   calculate the cost
            KLDivOpt = CalcKLDivOpt(OptU, P, CurState, NStatePairs);
            TotalCost = TotalCost + q(CurState) + KLDivOpt;
            
            %   Now that the Current state was found, it is possible to
            %   find the most probable next position. So, the position can
            %   be updated. for that, the previous position is updated
            %   first and then the current position is also updated
            PrevPosition = CurPosition;
            CurPosition = MaxOptUs(CurState, 3);
            TrajStep = TrajStep + 1;
            
            %   Drawing the current position (it will receive the value in "TrajStep" in
            %   the matrix)
            [lin, col] = GetLinCol(CurPosition, GridSize);
            MapMatrix(lin, col) = TrajStep;
        end
    end 
end