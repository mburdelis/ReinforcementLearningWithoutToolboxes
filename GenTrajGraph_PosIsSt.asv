function [MapMatrix, TotalCost] = GenTrajGraph_PosIsSt(States, NStates, GridSize, GoalPos, ObstacleIndexes, NumObstacles, q, P, MaxOptUs, OptU, IniPosition)
    
    IniPosError = 0;
    %   Checking if the initial position exists in the map
    %   (the program can only proceed if it does)
    if (IniPosition < 0)||(IniPosition > (GridSize^2))
        IniPosError = 1;
    end
    
    %   Checking if the initial position is an obstacle (the program can
    %   only proceed if not)
    IniPosIsObs = CheckPosIsObstacle(IniPosition, ObstacleIndexes);
    
    IniPosError = or(IniPosError, IniPosIsObs);

    if IniPosError == 0
        
        MapMatrix = zeros(GridSize, GridSize);
        CurSt = IniPosition;
        TotalCost = 0;
        
        %   Drawing the obstacles
        for IndexObs=1:NumObstacles
            ObsPosition = ObstacleIndexes(IndexObs);
            [lin, col] = GetLinCol(ObsPosition, GridSize);
            MapMatrix(lin, col) = -1;
        end
        
        %   Drawing the initial position (it will receive the value "1" in
        %   the matrix)
        TrajStep = 1;
        [lin, col] = GetLinCol(CurSt, GridSize);
        MapMatrix(lin, col) = TrajStep;
        
        %   Drawing the trajectory
        while CurSt ~= GoalPos
           
            %   calculating the cost
            KLDivOpt = CalcKLDivOpt(NStates, P(CurSt, :), OptU(CurSt, :));
            TotalCost = TotalCost + q(CurSt) + KLDivOpt;

            %   Finding the most probable next position. The position is
            %   updated
            CurSt = MaxOptUs(CurSt, 2);
            TrajStep = TrajStep + 1;
            
            %   Drawing the current position (it will receive the value in "TrajStep" in
            %   the matrix)
            [lin, col] = GetLinCol(CurSt, GridSize);
            MapMatrix(lin, col) = TrajStep;
        end
    end 
end