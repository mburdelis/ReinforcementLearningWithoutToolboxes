function [legal, TotalAdj, ExitCode] = GenLegalChecking_PosIsSt(NStates, CurSt, GridSize, GoalPos, ObstacleIndexes)
    legal = zeros(1, NStates);  %   initializing the "legal" column vector
    ExitCode = 0;   %   no error so far
                    %   the possibilities for "ExitCode" are:
                    %   0: no error, and it is NOT the case that the only
                    %   legal future state is the goal state                   
                    %   1: current state has an obstacle position (illegal)
                    %   3: the current state has the goal as the current
                    %   position, and therefore the only legal future state
                    %   is the goal state
    TotalAdj = 0;
    
    %   verifying if there is an obstacle position in the current state (illegal)
    CurStateObs = CheckPosIsObstacle(CurSt, ObstacleIndexes);

    %   The "if" command below would have no "else", because when the 
    %   condition is not true, the "legal" must be all zeros, and it 
    %   already is so before the "if" command, but there is an "else"
    %   only to update the "ExitCode"
    if (CurStateObs == 0)
        if CurSt == GoalPos
            legal(1, GoalPos) = 1;
            ExitCode = 3;
            TotalAdj = 1;
        else
            %   Testing all possible states to verify
            %   which are adjacent: 
            %   How this is done: getting the row and column of the
            %   current state and of each tested state, and checking if
            %   both displacements (horizontal and vertical) are not
            %   too big
            [LinCur, ColCur] = GetLinCol(CurSt, GridSize);
            for IndexState=1:NStates
                NextStateObs = CheckPosIsObstacle(IndexState, ObstacleIndexes);
                %   updating the value at the "legal" vector to 1
                %   only if the state is "adjacent" and does not
                %   contain an "obstacle-position"
                if NextStateObs == 0
                    adjacent=CheckAdj_LinCColC(LinCur, ColCur, IndexState, GridSize);
                    if adjacent == 1
                        legal(1, IndexState)=1;
                        TotalAdj = TotalAdj + 1;
                    end
                end %% if NextStateObs == 0
            end % for IndexState=1:NStates
        end % if CurSt == GoalPos
    else
        ExitCode = 1;
    end % if (CurStateObs == 0)
end