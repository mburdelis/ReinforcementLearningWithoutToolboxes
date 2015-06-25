function    Passive = UpdatePassiveNewWObs_PosIsSt(CurSt, States, NStates, GridSize, GoalPos, ObstacleIndexes, GoalStateIndex)
    IndexFutState = 0;
    Passive = zeros(NStates, 1);
    TotalLowProbAdj = 0;
    CurStateObs = 0;    %   flag that indicates if the current state 
                        %   contains is an "obstacle" position
    
    CurStateObs = CheckPosIsObstacle(CurSt, ObstacleIndexes);

    %   The "if" command below has no "else", because when the condition is
    %   not true, the "Passive" must be all zeros, and it already is so
    %   before the "if" command
    if CurStateObs == 0
        %   If the current state already is the Goal,
        %   then the passive dynamics must be 100% probability
        %   of remaining there (0 to any other state)
        if CurSt == GoalPos
            Passive(GoalStateIndex, 1) = 1;
        else
            %   using random walk as passive dynamics - being careful with
            %   obstacle positions
            TotalAdj=0;
            %   Testing all possible positions (states), verifying if they 
            %   are adjacent and if they are not obstacles
            for IndexState=1:NStates
                % Here we have to check if it is an adjacent state:
                IsAdjacent=CheckAdj(IndexState, CurSt, GridSize);
                if IsAdjacent ~= 0
                    AdjStateObs = CheckPosIsObstacle(IndexState, ObstacleIndexes);
                    if AdjStateObs == 0
                        Passive(IndexState)=1;
                        TotalAdj = TotalAdj + 1;
                    end
                end
            end
            Passive = Passive./TotalAdj;
        end
    end
end