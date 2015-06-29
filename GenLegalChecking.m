function [legal, TotalAdj, ExitCode] = GenLegalChecking(States, NStates, CurStateIndex, GridSize, GoalPos, StateType, options)
    legal = zeros(NStates, 1);  %   initializing the "legal" column vector
    ExitCode = 0;  %   no error so far
                    %   the possibilities for "ExitCode" are:
                    %   0: no error, and it is not the case that the only
                    %   legal future state is the goal state                   
                    %   1: current state has an obstacle position (illegal)
                    %   2: current state is leaving the goal (illegal)
                    %   3: the current state has the goal as the current
                    %   position, and therefore the only legal future state
                    %   is the goal state
    TotalAdj = 0;
    
    StatePair = States(CurStateIndex, :);
    
    CurStateObs = 0; %   flag to indicate that there is an obstacle position in the current state (illegal)
    CurStateLG = 0; %   flag to indicate that the current state is actually leaving the goal (illegal)
    
    %   "StateType" informs if we are working with "1 state = 2 positions" 
    %   or not
    %   StateType = 2 means "1 state = 2 positions"
    %   StateType = any other value means "1 state = 1 position"
    if StateType == 2
        %   checking if the current state is not illegal
        for IndexObs=1:options.SizeAuxObstacles
            if StatePair(1)==options.AuxObstacles(IndexObs) || StatePair(2)==options.AuxObstacles(IndexObs)
                CurStateObs = 1;
                ExitCode = 1;
                break;
            end
        end
        if StatePair(1)==GoalPos && StatePair(2)~=GoalPos
            CurStateLG = 1;
            ExitCode = 2;
        end
    else
        for IndexObs=1:options.SizeAuxObstacles
            if CurStateIndex==options.AuxObstacles(IndexObs)
                CurStateObs = 1;
                ExitCode = 1;
                break;
            end
        end
    end
    
    %   The "if" command below has no "else", because when the condition is
    %   not true, the "legal" must be all zeros, and it already is so
    %   before the "if" command
    if (CurStateObs == 0) && (CurStateLG == 0)
        if StateType == 2
            if StatePair(2) == GoalPos
                legal(options.GoalStatePosPairIndex) = 1;
                ExitCode = 3;
                TotalAdj = 1;
            else
                %   Testing all possible states (position pairs) to verify
                %   which are adjacent: 
                %   How this is done: if the previous position of the tested state 
                %   pair equals the current position of the current state, then 
                %   it is adjacent
                for IndexState=1:NStates
                    if States(IndexState, 1) == StatePair(2)
                        %   Verify if the found "adjacent" state has an
                        %   "obstacle-position" inside
                        Position = States(IndexState, 2);
                        NextStateObs = CheckNextStateObsWPairs(options, Position);
                        %   updating the value at the "legal" vector to 1
                        %   only if the state is "adjacent" and does not
                        %   contain an "obstacle-position"
                        if NextStateObs == 0
                            legal(IndexState)=1;
                            TotalAdj = TotalAdj + 1;
                        end
                    end
                end
            end
        else
            if CurStateIndex == GoalPos
                legal(options.GoalStatePosPairIndex) = 1;
                ExitCode = 3;
                TotalAdj = 1;
            else
                %   Testing all possible states (position pairs) to verify
                %   which are adjacent: 
                %   How this is done: getting the row and column of the
                %   current state and of each tested state, and checking if
                %   both displacements (horizontal and vertical) are not
                %   too big
                [LinCurState, ColCurState] = GetLinCol(CurStateIndex, GridSize);
                for IndexState=1:NStates
                    [LinTestState, ColTestState] = GetLinCol(IndexState, GridSize);
                    DeltaHor = ColTestState - ColCurState;
                    DeltaVer = LinTestState - LinCurState;
                    %   this "if" is true if the states are adjacent and
                    %   false otherwise
                    if (abs(DeltaHor)<=1) && (abs(DeltaVer)<=1)
                        %   Verify if the found "adjacent" state is an
                        %   obstacle state
                        NextStateObs = CheckNextStateObsWPairs(options, IndexState);
                        %   updating the value at the "legal" vector to 1
                        %   only if the state is "adjacent" and does not
                        %   contain an "obstacle-position"
                        if NextStateObs == 0
                            legal(IndexState)=1;
                            TotalAdj = TotalAdj + 1;
                        end
                    end
                end
            end
        end
    end
end