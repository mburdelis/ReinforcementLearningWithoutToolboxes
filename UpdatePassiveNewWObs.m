function Passive = UpdatePassiveNewWObs(StatePair, GridSize, GoalState, States, options)
    [NLin, NCol] = size(States);
    IndexFutState = 0;
    Passive = zeros(NLin, 1);
    TotalLowProbAdj = 0;
    CurStateObs = 0;   %   flag that indicates if the current state contains is an "obstacle" position
    
    for IndexObs=1:options.SizeAuxObstacles
        if StatePair(1)==options.AuxObstacles(IndexObs) || StatePair(2)==options.AuxObstacles(IndexObs)
            CurStateObs = 1;
            break;
        end
    end

    %   The "if" command below has no "else", because when the condition is
    %   not true, the "Passive" must be all zeros, and it already is so
    %   before the "if" command
    if CurStateObs == 0

        % Assuming that StatePair(2) has the current position
        if StatePair(2) == GoalState
            Passive(options.GoalStatePosPairIndex) = 1;
        else
            %   If the current position equals the previous position: use
            %   random walk as passive dynamics - being careful with
            %   obstacle positions
            if StatePair(1) == StatePair(2)
                TotalAdj=0;
                %   Testing all possible states (position pairs) to verify
                %   which are adjacent: 
                %   How this is done: if the previous position of the tested state 
                %   pair equals the current position of the current state, then 
                %   it is adjacent
                for IndexState=1:NLin
                    if States(IndexState, 1) == StatePair(2)
                        %   Verify if the found "adjacent" state has an
                        %   "obstacle-position" inside
                        Position = States(IndexState, 2);
                        NextStateObs = CheckNextStateObsWPairs(options, Position);
                        %   updating the passive dynamics value at the
                        %   state only if it is "adjacent" and does not
                        %   contain an "obstacle-position"
                        if NextStateObs == 0
                            Passive(IndexState)=1;
                            TotalAdj = TotalAdj + 1;
                        end
                    end
                end
                Passive = Passive./TotalAdj;
            else

                %   Getting the line and column values for the current and
                %   previous position
                [LinCur, ColCur] = GetLinCol(StatePair(2), GridSize);
                [LinPrev, ColPrev] = GetLinCol(StatePair(1), GridSize);
                %   Calculating the "deltas" corresponding to the movement from
                %   the previous location to the current location
                DeltaVert = LinCur - LinPrev;
                DeltaHor = ColCur - ColPrev;

                %   Generating what the most probable NEW "delta" must be,
                %   based on the location (this is done for the line and the
                %   column independently):

%                 %   If the agent is close to the top wall or the bottom wall
%                 if (LinCur==1)||(LinCur==GridSize)
%                     %   If the wall is reflexive, the new most probable
%                     %   "DeltaVert" must be the opposite of the "DeltaVert" of
%                     %   the last movement
%                     if options.WallType == 1
%                         DeltaVert = -1*DeltaVert;
%                     %   If the wall is absorptive, the vertical movement must
%                     %   be absorbed
%                     else
%                         DeltaVert = 0;
%                     end
%                 end
% 
%                 %   If the agent is close to the left wall or the right wall
%                 if (ColCur==1)||(ColCur==GridSize)
%                     %   If the wall is reflexive, the new most probable
%                     %   "DeltaHor" must be the opposite of the "DeltaHor" of
%                     %   the last movement
%                     if options.WallType == 1
%                         DeltaHor = -1*DeltaHor;
%                     %   If the wall is absorptive, the horizontal movement must
%                     %   be absorbed
%                     else
%                         DeltaHor = 0;
%                     end
%                 end

                %   If the horizontal delta is to the left:
                if DeltaHor < 0
                    %   if the vertical delta is zero, the movement was
                    %   purely to the left
                    if DeltaVert == 0
                        %   if it is close to the left wall
                        if (ColCur==1)
                            %   If the wall is reflexive, the new most probable
                            %   "DeltaHor" must be the opposite of the "DeltaHor" of
                            %   the last movement
                            if options.WallType == 1
                                DeltaHor = -1*DeltaHor;
                            %   If the wall is absorptive, the horizontal movement must
                            %   be absorbed
                            else
                                DeltaHor = 0;
                            end
                        else
                            %   Verifying if the immediate position to the
                            %   left is an obstacle position
                            NextColLeft = ColCur-1;
                            NextPosLeft = (LinCur-1)*GridSize + NextColLeft;
                            LeftPosIsObs = CheckNextStateObsWPairs(options, NextPosLeft);
                            if LeftPosIsObs == 1
                                %   If the wall is reflexive, the new most probable
                                %   "DeltaHor" must be the opposite of the "DeltaHor" of
                                %   the last movement
                                if options.WallType == 1
                                    DeltaHor = -1*DeltaHor;
                                %   If the wall is absorptive, the horizontal movement must
                                %   be absorbed
                                else
                                    DeltaHor = 0;
                                end
                            end
                        end
                    else
                        %   If deltavert if up (and deltahor is to the
                        %   left) - then the movement is diagonal up left
                        if DeltaVert < 0
                            %   Verify if is there wall up
                            if (LinCur == 1)
                                %   Verify if is there wall to the left
                                if (ColCur == 1)
                                    %   Diagonal collision: both deltas
                                    %   must be changed
                                    if options.WallType == 1
                                        DeltaHor = -1*DeltaHor;
                                        DeltaVert = -1*DeltaVert;
                                    %   If the wall is absorptive, the horizontal movement must
                                    %   be absorbed
                                    else
                                        DeltaHor = 0;
                                        DeltaVert = 0;
                                    end
                                else
                                    %   Verify if is there an obstacle to
                                    %   the left:
                                    NextColLeft = ColCur-1;
                                    NextPosLeft = (LinCur-1)*GridSize + NextColLeft;
                                    LeftPosIsObs = CheckNextStateObsWPairs(options, NextPosLeft);
                                    if LeftPosIsObs == 1
                                        %   Diagonal collision: both deltas
                                        %   must be changed
                                        if options.WallType == 1
                                            DeltaHor = -1*DeltaHor;
                                            DeltaVert = -1*DeltaVert;
                                        %   If the wall is absorptive, the horizontal movement must
                                        %   be absorbed
                                        else
                                            DeltaHor = 0;
                                            DeltaVert = 0;
                                        end
                                    else
                                        %   There is no obstacle nor wall
                                        %   to the left, so only the
                                        %   "DeltaVert" must be changed
                                        if options.WallType == 1
                                            DeltaVert = -1*DeltaVert;
                                        %   If the wall is absorptive, the movement must
                                        %   be absorbed
                                        else
                                            DeltaVert = 0;
                                        end
                                    end
                                end
                            else
                                %   Verify if is there wall to the left
                                if (ColCur == 1)
                                    %   Verify if is there an obstacle up
                                    NextLineUp = LinCur-1;
                                    NextPosUp = (NextLineUp-1)*GridSize + ColCur;
                                    UpPosIsObs = CheckNextStateObsWPairs(options, NextPosUp);
                                    if UpPosIsObs == 1
                                        %   Diagonal collision: both deltas
                                        %   must be changed
                                        if options.WallType == 1
                                            DeltaHor = -1*DeltaHor;
                                            DeltaVert = -1*DeltaVert;
                                        %   If the wall is absorptive, the movement must
                                        %   be absorbed
                                        else
                                            DeltaHor = 0;
                                            DeltaVert = 0;
                                        end
                                    else
                                        %   Collision with the left:
                                        %   "DeltaHor" must be changed
                                        if options.WallType == 1
                                            DeltaHor = -1*DeltaHor;
                                        %   If the wall is absorptive, the movement must
                                        %   be absorbed
                                        else
                                            DeltaHor = 0;
                                        end
                                    end
                                else
                                    %   Verify if is there an obstacle up
                                    NextLineUp = LinCur-1;
                                    NextPosUp = (NextLineUp-1)*GridSize + ColCur;
                                    UpPosIsObs = CheckNextStateObsWPairs(options, NextPosUp);
                                    if UpPosIsObs == 1
                                        %   Verify if is there an obstacle to
                                        %   the left:
                                        NextColLeft = ColCur-1;
                                        NextPosLeft = (LinCur-1)*GridSize + NextColLeft;
                                        LeftPosIsObs = CheckNextStateObsWPairs(options, NextPosLeft);
                                        if LeftPosIsObs == 1
                                            %   Diagonal collision: both deltas
                                            %   must be changed
                                            if options.WallType == 1
                                                DeltaHor = -1*DeltaHor;
                                                DeltaVert = -1*DeltaVert;
                                            %   If the wall is absorptive, the movement must
                                            %   be absorbed
                                            else
                                                DeltaHor = 0;
                                                DeltaVert = 0;
                                            end
                                        else
                                            %   Collision with the
                                            %   obstacle up
                                            if options.WallType == 1
                                                DeltaVert = -1*DeltaVert;
                                            %   If the wall is absorptive, the movement must
                                            %   be absorbed
                                            else
                                                DeltaVert = 0;
                                            end
                                        end
                                    else
                                        %   Verify if is there an
                                        %   obstacle to the left
                                        NextColLeft = ColCur-1;
                                        NextPosLeft = (LinCur-1)*GridSize + NextColLeft;
                                        LeftPosIsObs = CheckNextStateObsWPairs(options, NextPosLeft);
                                        if LeftPosIsObs == 1
                                            %   Collision to the left
                                            if options.WallType == 1
                                                DeltaHor = -1*DeltaHor;
                                            %   If the wall is absorptive, the movement must
                                            %   be absorbed
                                            else
                                                DeltaHor = 0;
                                            end
                                        else
                                            %   Verify if is there an obstacle at
                                            %   the diagonal
                                            NextLineUp = LinCur-1;
                                            NextColLeft = ColCur-1;
                                            NextPosDiag = (NextLineUp-1)*GridSize + NextColLeft;
                                            DiagPosIsObs = CheckNextStateObsWPairs(options, NextPosDiag);
                                            if DiagPosIsObs == 1
                                                %   Diagonal collision: both deltas
                                                %   must be changed
                                                if options.WallType == 1
                                                    DeltaHor = -1*DeltaHor;
                                                    DeltaVert = -1*DeltaVert;
                                                %   If the wall is absorptive, the movement must
                                                %   be absorbed
                                                else
                                                    DeltaHor = 0;
                                                    DeltaVert = 0;
                                                end
                                            %   No "else for this "if",
                                            %   because if there is no
                                            %   collision then there is no
                                            %   need to chenge the deltas
                                            end
                                        end
                                    end
                                end
                            end 
                        else
                            %   As "DeltaVert" is not "=0" nor "<0", it can
                            %   only be ">0"
                            %   Verify if is there wall down
                            if (LinCur == GridSize)
                                %   Verify if is there wall to the left
                                if (ColCur == 1)
                                    %   Diagonal collision: both deltas
                                    %   must be changed
                                    if options.WallType == 1
                                        DeltaHor = -1*DeltaHor;
                                        DeltaVert = -1*DeltaVert;
                                    %   If the wall is absorptive, the horizontal movement must
                                    %   be absorbed
                                    else
                                        DeltaHor = 0;
                                        DeltaVert = 0;
                                    end
                                else
                                    %   Verify if is there an obstacle to
                                    %   the left:
                                    NextColLeft = ColCur-1;
                                    NextPosLeft = (LinCur-1)*GridSize + NextColLeft;
                                    LeftPosIsObs = CheckNextStateObsWPairs(options, NextPosLeft);
                                    if LeftPosIsObs == 1
                                        %   Diagonal collision: both deltas
                                        %   must be changed
                                        if options.WallType == 1
                                            DeltaHor = -1*DeltaHor;
                                            DeltaVert = -1*DeltaVert;
                                        %   If the wall is absorptive, the horizontal movement must
                                        %   be absorbed
                                        else
                                            DeltaHor = 0;
                                            DeltaVert = 0;
                                        end
                                    else
                                        %   There is no obstacle nor wall
                                        %   to the left, so only the
                                        %   "DeltaVert" must be changed
                                        if options.WallType == 1
                                            DeltaVert = -1*DeltaVert;
                                        %   If the wall is absorptive, the movement must
                                        %   be absorbed
                                        else
                                            DeltaVert = 0;
                                        end
                                    end
                                end
                            else
                                %   Verify if is there wall to the left
                                if (ColCur == 1)
                                    %   Verify if is there an obstacle down
                                    NextLineDown = LinCur+1;
                                    NextPosDown = (NextLineDown-1)*GridSize + ColCur;
                                    DownPosIsObs = CheckNextStateObsWPairs(options, NextPosDown);
                                    if DownPosIsObs == 1
                                        %   Diagonal collision: both deltas
                                        %   must be changed
                                        if options.WallType == 1
                                            DeltaHor = -1*DeltaHor;
                                            DeltaVert = -1*DeltaVert;
                                        %   If the wall is absorptive, the movement must
                                        %   be absorbed
                                        else
                                            DeltaHor = 0;
                                            DeltaVert = 0;
                                        end
                                    else
                                        %   Collision with the left:
                                        %   "DeltaHor" must be changed
                                        if options.WallType == 1
                                            DeltaHor = -1*DeltaHor;
                                        %   If the wall is absorptive, the movement must
                                        %   be absorbed
                                        else
                                            DeltaHor = 0;
                                        end
                                    end
                                else
                                    %   Verify if is there an obstacle
                                    %   down
                                    NextLineDown = LinCur+1;
                                    NextPosDown = (NextLineDown-1)*GridSize + ColCur;
                                    DownPosIsObs = CheckNextStateObsWPairs(options, NextPosDown);
                                    if DownPosIsObs == 1
                                        %   Verify if is there an obstacle to
                                        %   the left:
                                        NextColLeft = ColCur-1;
                                        NextPosLeft = (LinCur-1)*GridSize + NextColLeft;
                                        LeftPosIsObs = CheckNextStateObsWPairs(options, NextPosLeft);
                                        if LeftPosIsObs == 1
                                            %   Diagonal collision: both deltas
                                            %   must be changed
                                            if options.WallType == 1
                                                DeltaHor = -1*DeltaHor;
                                                DeltaVert = -1*DeltaVert;
                                            %   If the wall is absorptive, the movement must
                                            %   be absorbed
                                            else
                                                DeltaHor = 0;
                                                DeltaVert = 0;
                                            end
                                        else
                                            %   Collision with the
                                            %   obstacle down
                                            if options.WallType == 1
                                                DeltaVert = -1*DeltaVert;
                                            %   If the wall is absorptive, the movement must
                                            %   be absorbed
                                            else
                                                DeltaVert = 0;
                                            end
                                        end
                                    else
                                        %   Verify if is there an
                                        %   obstacle to the left
                                        NextColLeft = ColCur-1;
                                        NextPosLeft = (LinCur-1)*GridSize + NextColLeft;
                                        LeftPosIsObs = CheckNextStateObsWPairs(options, NextPosLeft);
                                        if LeftPosIsObs == 1
                                            %   Collision to the left
                                            if options.WallType == 1
                                                DeltaHor = -1*DeltaHor;
                                            %   If the wall is absorptive, the movement must
                                            %   be absorbed
                                            else
                                                DeltaHor = 0;
                                            end
                                        else
                                            %   Verify if is there an obstacle at
                                            %   the diagonal
                                            NextLineDown = LinCur+1;
                                            NextColLeft = ColCur-1;
                                            NextPosDiag = (NextLineDown-1)*GridSize + NextColLeft;
                                            DiagPosIsObs = CheckNextStateObsWPairs(options, NextPosDiag);
                                            if DiagPosIsObs == 1
                                                %   Diagonal collision: both deltas
                                                %   must be changed
                                                if options.WallType == 1
                                                    DeltaHor = -1*DeltaHor;
                                                    DeltaVert = -1*DeltaVert;
                                                %   If the wall is absorptive, the movement must
                                                %   be absorbed
                                                else
                                                    DeltaHor = 0;
                                                    DeltaVert = 0;
                                                end
                                            %   No "else for this "if",
                                            %   because if there is no
                                            %   collision then there is no
                                            %   need to update the deltas
                                            end  
                                        end
                                    end
                                end
                            end
                        end
                    end
                else
                    %   Here, the movement was not to the left, so we are
                    %   now testing if it was to the right (horizontal
                    %   component)
                    if DeltaHor > 0
                        %   if the vertical delta is zero, the movement was
                        %   purely to the right
                        if DeltaVert == 0
                            %   if it is close to the right wall
                            if (ColCur==GridSize)
                                %   If the wall is reflexive, the new most probable
                                %   "DeltaHor" must be the opposite of the "DeltaHor" of
                                %   the last movement
                                if options.WallType == 1
                                    DeltaHor = -1*DeltaHor;
                                %   If the wall is absorptive, the horizontal movement must
                                %   be absorbed
                                else
                                    DeltaHor = 0;
                                end
                            else
                                %   Verifying if the immediate position to the
                                %   right is an obstacle position
                                NextColRight = ColCur+1;
                                NextPosRight = (LinCur-1)*GridSize + NextColRight;
                                RightPosIsObs = CheckNextStateObsWPairs(options, NextPosRight);
                                if RightPosIsObs == 1
                                    %   If the wall is reflexive, the new most probable
                                    %   "DeltaHor" must be the opposite of the "DeltaHor" of
                                    %   the last movement
                                    if options.WallType == 1
                                        DeltaHor = -1*DeltaHor;
                                    %   If the wall is absorptive, the horizontal movement must
                                    %   be absorbed
                                    else
                                        DeltaHor = 0;
                                    end
                                end
                            end
                        else
                            %   If deltavert is up (and deltahor is to the
                            %   right) - then the movement is diagonal up right
                            if DeltaVert < 0
                                %   Verify if is there wall up
                                if (LinCur == 1)
                                    %   Verify if is there wall to the right
                                    if (ColCur == GridSize)
                                        %   Diagonal collision: both deltas
                                        %   must be changed
                                        if options.WallType == 1
                                            DeltaHor = -1*DeltaHor;
                                            DeltaVert = -1*DeltaVert;
                                        %   If the wall is absorptive, the horizontal movement must
                                        %   be absorbed
                                        else
                                            DeltaHor = 0;
                                            DeltaVert = 0;
                                        end
                                    else
                                        %   Verifying if the immediate position to the
                                        %   right is an obstacle position
                                        NextColRight = ColCur+1;
                                        NextPosRight = (LinCur-1)*GridSize + NextColRight;
                                        RightPosIsObs = CheckNextStateObsWPairs(options, NextPosRight);
                                        if RightPosIsObs == 1
                                            %   Diagonal collision: both deltas
                                            %   must be changed
                                            if options.WallType == 1
                                                DeltaHor = -1*DeltaHor;
                                                DeltaVert = -1*DeltaVert;
                                            %   If the wall is absorptive, the horizontal movement must
                                            %   be absorbed
                                            else
                                                DeltaHor = 0;
                                                DeltaVert = 0;
                                            end
                                        else
                                            %   There is no obstacle nor wall
                                            %   to the right, so only the
                                            %   "DeltaVert" must be changed
                                            if options.WallType == 1
                                                DeltaVert = -1*DeltaVert;
                                            %   If the wall is absorptive, the movement must
                                            %   be absorbed
                                            else
                                                DeltaVert = 0;
                                            end
                                        end
                                    end
                                else
                                    %   Verify if is there wall to the
                                    %   right
                                    if (ColCur == GridSize)
                                        %   Verify if is there an obstacle up
                                        NextLineUp = LinCur-1;
                                        NextPosUp = (NextLineUp-1)*GridSize + ColCur;
                                        UpPosIsObs = CheckNextStateObsWPairs(options, NextPosUp);
                                        if UpPosIsObs == 1
                                            %   Diagonal collision: both deltas
                                            %   must be changed
                                            if options.WallType == 1
                                                DeltaHor = -1*DeltaHor;
                                                DeltaVert = -1*DeltaVert;
                                            %   If the wall is absorptive, the movement must
                                            %   be absorbed
                                            else
                                                DeltaHor = 0;
                                                DeltaVert = 0;
                                            end
                                        else
                                            %   Collision with the right:
                                            %   "DeltaHor" must be changed
                                            if options.WallType == 1
                                                DeltaHor = -1*DeltaHor;
                                            %   If the wall is absorptive, the movement must
                                            %   be absorbed
                                            else
                                                DeltaHor = 0;
                                            end
                                        end
                                    else
                                        %   Verify if is there an obstacle up
                                        NextLineUp = LinCur-1;
                                        NextPosUp = (NextLineUp-1)*GridSize + ColCur;
                                        UpPosIsObs = CheckNextStateObsWPairs(options, NextPosUp);
                                        if UpPosIsObs == 1
                                            %   Verifying if the immediate position to the
                                            %   right is an obstacle position
                                            NextColRight = ColCur+1;
                                            NextPosRight = (LinCur-1)*GridSize + NextColRight;
                                            RightPosIsObs = CheckNextStateObsWPairs(options, NextPosRight);
                                            if RightPosIsObs == 1
                                                %   Diagonal collision: both deltas
                                                %   must be changed
                                                if options.WallType == 1
                                                    DeltaHor = -1*DeltaHor;
                                                    DeltaVert = -1*DeltaVert;
                                                %   If the wall is absorptive, the movement must
                                                %   be absorbed
                                                else
                                                    DeltaHor = 0;
                                                    DeltaVert = 0;
                                                end
                                            else
                                                %   Collision with the
                                                %   obstacle up
                                                if options.WallType == 1
                                                    DeltaVert = -1*DeltaVert;
                                                %   If the wall is absorptive, the movement must
                                                %   be absorbed
                                                else
                                                    DeltaVert = 0;
                                                end
                                            end
                                        else
                                            %   Verifying if the immediate position to the
                                            %   right is an obstacle position
                                            NextColRight = ColCur+1;
                                            NextPosRight = (LinCur-1)*GridSize + NextColRight;
                                            RightPosIsObs = CheckNextStateObsWPairs(options, NextPosRight);
                                            if RightPosIsObs == 1
                                                %   Collision to the
                                                %   right
                                                if options.WallType == 1
                                                    DeltaHor = -1*DeltaHor;
                                                %   If the wall is absorptive, the movement must
                                                %   be absorbed
                                                else
                                                    DeltaHor = 0;
                                                end
                                                
                                             else
                                                %   Verify if is there an obstacle at
                                                %   the diagonal
                                                NextLineUp = LinCur-1;
                                                NextColRight = ColCur+1;
                                                NextPosDiag = (NextLineUp-1)*GridSize + NextColRight;
                                                DiagPosIsObs = CheckNextStateObsWPairs(options, NextPosDiag);
                                                if DiagPosIsObs == 1
                                                    %   Diagonal collision: both deltas
                                                    %   must be changed
                                                    if options.WallType == 1
                                                        DeltaHor = -1*DeltaHor;
                                                        DeltaVert = -1*DeltaVert;
                                                    %   If the wall is absorptive, the movement must
                                                    %   be absorbed
                                                    else
                                                        DeltaHor = 0;
                                                        DeltaVert = 0;
                                                    end   
                                                    %   No "else for this "if",
                                                    %   because if there is no
                                                    %   obstacle to the right then
                                                    %   there is no collision
                                                end
                                            end
                                        end
                                    end
                                end 
                            else
                                %   As "DeltaVert" is not "=0" nor "<0", it can
                                %   only be ">0"
                                %   Verify if is there wall down
                                if (LinCur == GridSize)
                                    %   Verify if is there wall to the
                                    %   right
                                    if (ColCur == GridSize)
                                        %   Diagonal collision: both deltas
                                        %   must be changed
                                        if options.WallType == 1
                                            DeltaHor = -1*DeltaHor;
                                            DeltaVert = -1*DeltaVert;
                                        %   If the wall is absorptive, the horizontal movement must
                                        %   be absorbed
                                        else
                                            DeltaHor = 0;
                                            DeltaVert = 0;
                                        end
                                    else
                                        %   Verifying if the immediate position to the
                                        %   right is an obstacle position
                                        NextColRight = ColCur+1;
                                        NextPosRight = (LinCur-1)*GridSize + NextColRight;
                                        RightPosIsObs = CheckNextStateObsWPairs(options, NextPosRight);
                                        if RightPosIsObs == 1
                                            %   Diagonal collision: both deltas
                                            %   must be changed
                                            if options.WallType == 1
                                                DeltaHor = -1*DeltaHor;
                                                DeltaVert = -1*DeltaVert;
                                            %   If the wall is absorptive, the horizontal movement must
                                            %   be absorbed
                                            else
                                                DeltaHor = 0;
                                                DeltaVert = 0;
                                            end
                                        else
                                            %   There is no obstacle nor wall
                                            %   to the right, so only the
                                            %   "DeltaVert" must be changed
                                            if options.WallType == 1
                                                DeltaVert = -1*DeltaVert;
                                            %   If the wall is absorptive, the movement must
                                            %   be absorbed
                                            else
                                                DeltaVert = 0;
                                            end
                                        end
                                    end
                                else
                                    %   Verify if is there a wall to the
                                    %   right
                                    if (ColCur == GridSize)
                                        %   Verify if is there an obstacle down
                                        NextLineDown = LinCur+1;
                                        NextPosDown = (NextLineDown-1)*GridSize + ColCur;
                                        DownPosIsObs = CheckNextStateObsWPairs(options, NextPosDown);
                                        if DownPosIsObs == 1
                                            %   Diagonal collision: both deltas
                                            %   must be changed
                                            if options.WallType == 1
                                                DeltaHor = -1*DeltaHor;
                                                DeltaVert = -1*DeltaVert;
                                            %   If the wall is absorptive, the movement must
                                            %   be absorbed
                                            else
                                                DeltaHor = 0;
                                                DeltaVert = 0;
                                            end
                                        else
                                            %   Collision with the right:
                                            %   "DeltaHor" must be changed
                                            if options.WallType == 1
                                                DeltaHor = -1*DeltaHor;
                                            %   If the wall is absorptive, the movement must
                                            %   be absorbed
                                            else
                                                DeltaHor = 0;
                                            end
                                        end

                                    else
                                        %   Verify if is there an obstacle
                                        %   down
                                        NextLineDown = LinCur+1;
                                        NextPosDown = (NextLineDown-1)*GridSize + ColCur;
                                        DownPosIsObs = CheckNextStateObsWPairs(options, NextPosDown);
                                        if DownPosIsObs == 1
                                            %   Verifying if the immediate position to the
                                            %   right is an obstacle position
                                            NextColRight = ColCur+1;
                                            NextPosRight = (LinCur-1)*GridSize + NextColRight;
                                            RightPosIsObs = CheckNextStateObsWPairs(options, NextPosRight);
                                            if RightPosIsObs == 1
                                                %   Diagonal collision: both deltas
                                                %   must be changed
                                                if options.WallType == 1
                                                    DeltaHor = -1*DeltaHor;
                                                    DeltaVert = -1*DeltaVert;
                                                %   If the wall is absorptive, the movement must
                                                %   be absorbed
                                                else
                                                    DeltaHor = 0;
                                                    DeltaVert = 0;
                                                end
                                            else
                                                %   Collision with the
                                                %   obstacle down
                                                if options.WallType == 1
                                                    DeltaVert = -1*DeltaVert;
                                                %   If the wall is absorptive, the movement must
                                                %   be absorbed
                                                else
                                                    DeltaVert = 0;
                                                end
                                            end
                                        else
                                            %   Verifying if the immediate position to the
                                            %   right is an obstacle position
                                            NextColRight = ColCur+1;
                                            NextPosRight = (LinCur-1)*GridSize + NextColRight;
                                            RightPosIsObs = CheckNextStateObsWPairs(options, NextPosRight);
                                            if RightPosIsObs == 1
                                                %   Collision to the
                                                %   right
                                                if options.WallType == 1
                                                    DeltaHor = -1*DeltaHor;
                                                %   If the wall is absorptive, the movement must
                                                %   be absorbed
                                                else
                                                    DeltaHor = 0;
                                                end
                                                
                                            else
                                                %   Verify if is there an obstacle at
                                                %   the diagonal
                                                NextLineDown = LinCur+1;
                                                NextColLeft = ColCur-1;
                                                NextPosDiag = (NextLineDown-1)*GridSize + NextColLeft;
                                                DiagPosIsObs = CheckNextStateObsWPairs(options, NextPosDiag);
                                                if DiagPosIsObs == 1
                                                    %   Diagonal collision: both deltas
                                                    %   must be changed
                                                    if options.WallType == 1
                                                        DeltaHor = -1*DeltaHor;
                                                        DeltaVert = -1*DeltaVert;
                                                    %   If the wall is absorptive, the movement must
                                                    %   be absorbed
                                                    else
                                                        DeltaHor = 0;
                                                        DeltaVert = 0;
                                                    end
                                                    %   No "else for this "if",
                                                    %   because if there is no
                                                    %   collision then
                                                    %   there is no need to
                                                    %   update the deltas
                                                end    
                                            end
                                        end
                                    end
                                end
                            end
                        end    
                    else
                        %   Here, since "DeltaHor" is not ">0" nor "<0",
                        %   then it can only be "=0"
                        
                        %   if "DeltaVert < 0" here then the movement is
                        %   straight up
                        if DeltaVert < 0
                            %   Verifying if it is close to the upper wall
                            if (LinCur == 1)
                                %   Collision with the upper wall: the
                                %   "DeltaVert" must be changed
                                if options.WallType == 1
                                    DeltaVert = -1*DeltaVert;
                                %   If the wall is absorptive, the movement must
                                %   be absorbed
                                else
                                    DeltaVert = 0;
                                end
                            else
                                %   Since there is no wall up, verifying if
                                %   is there an obstacle up
                                NextLineUp = LinCur-1;
                                NextPosUp = (NextLineUp-1)*GridSize + ColCur;
                                UpPosIsObs = CheckNextStateObsWPairs(options, NextPosUp);
                                if UpPosIsObs == 1
                                    %   Collision with the upper obstacle: the
                                    %   "DeltaVert" must be changed
                                    if options.WallType == 1
                                        DeltaVert = -1*DeltaVert;
                                    %   If the wall is absorptive, the movement must
                                    %   be absorbed
                                    else
                                        DeltaVert = 0;
                                    end
                                    %   No "else for this "if",
                                    %   because if there is no
                                    %   obstacle up then
                                    %   there is no collision
                                end
                            end
                        else
                            %   Here, DeltaHor is already "=0", and both
                            %   DeltaHor and DeltaVert can't be "0" on the
                            %   same time, and here "DeltaVert" is not
                            %   "<0", so it can only be ">0" (movement
                            %   down)
                            
                            %   Verifying if it is close to the lower wall
                            if (LinCur == GridSize)
                                %   Collision with the lower wall: the
                                %   "DeltaVert" must be changed
                                if options.WallType == 1
                                    DeltaVert = -1*DeltaVert;
                                %   If the wall is absorptive, the movement must
                                %   be absorbed
                                else
                                    DeltaVert = 0;
                                end
                            else
                                %   Since there is no wall down, verifying if
                                %   is there an obstacle down
                                NextLineDown = LinCur+1;
                                NextPosDown = (NextLineDown-1)*GridSize + ColCur;
                                DownPosIsObs = CheckNextStateObsWPairs(options, NextPosDown);
                                if DownPosIsObs == 1
                                    %   Collision with the obstacle down: the
                                    %   "DeltaVert" must be changed
                                    if options.WallType == 1
                                        DeltaVert = -1*DeltaVert;
                                    %   If the wall is absorptive, the movement must
                                    %   be absorbed
                                    else
                                        DeltaVert = 0;
                                    end
                                    %   No "else for this "if",
                                    %   because if there is no
                                    %   obstacle down then
                                    %   there is no collision
                                end
                            end
                        end
                    end
                end
                
                %   Using the obtained new "deltas" and the current position to
                %   calculate the new most probable position (line and column)
                LinFut = LinCur + DeltaVert;
                ColFut = ColCur + DeltaHor;

                %   Converting the obtained position (line and column) to the
                %   corresponding position code
                FutState = (LinFut-1)*GridSize + ColFut;

                %   Verifying if the position in "FutState" is not an
                %   obstacle:
                FutStateIsObs = 0;
                for IndexObs=1:options.SizeAuxObstacles
                    if FutState == options.AuxObstacles(IndexObs)
                        FutStateIsObs = 1;
                        break;
                    end
                end

                %   verifying if are there obstacles around and updating
                if FutStateIsObs == 0
                    %   Updating the probabilities (passive dynamics)
                        %   Testing all possible states (position pairs) to verify
                        %   which are adjacent: 
                        %   How this is done: if the previous position of the tested state 
                        %   pair equals the current position of the current state, then 
                        %   it is adjacent
                    for IndexState = 1:NLin
                        if States(IndexState, 1)==StatePair(2)
                            if States(IndexState, 2)==FutState
                                %   the adjacent state that correspond to
                                %   "Futstate" must receive the high probability
                                IndexFutState = IndexState;
                                Passive(IndexState)=1;
                            else
                                %   Verify if the found "adjacent" state has an
                                %   "obstacle-position" inside
                                Position = States(IndexState, 2);
                                NextStateObs = CheckNextStateObsWPairs(options, Position);
                                %   updating the passive dynamics value at the
                                %   state only if it is "adjacent" and does not
                                %   contain an "obstacle-position"
                                if NextStateObs == 0
                                    Passive(IndexState)=1;
                                    TotalLowProbAdj = TotalLowProbAdj + 1;
                                end
                            end
                        end
                    end
                    LowProbVal = (1-options.HighProbVal)/TotalLowProbAdj;
                    %   Considering that all "adjacent" states should now
                    %   have value "1" in the matrix, and all other states should
                    %   have value "0" in the matrix, by multiplying by
                    %   "LowProbVal", all adjacent states should have "LowProbVal",
                    %   and all other states should have "0"
                    Passive = Passive*LowProbVal;
                    %   So, now the "HighProbVal" can be recorded at the
                    %   corresponding state
                    Passive(IndexFutState) = options.HighProbVal;
                else
                    %   Updating the probabilities (passive dynamics)
                        %   Testing all possible states (position pairs) to verify
                        %   which are adjacent: 
                        %   How this is done: if the previous position of the tested state 
                        %   pair equals the current position of the current state, then 
                        %   it is adjacent
                    for IndexState = 1:NLin
                        if States(IndexState, 1)==StatePair(2)
                            %   Verify if the found "adjacent" state has an
                            %   "obstacle-position" inside
                            Position = States(IndexState, 2);
                            NextStateObs = CheckNextStateObsWPairs(options, Position);
                            %   updating the passive dynamics value at the
                            %   state only if it is "adjacent" and does not
                            %   contain an "obstacle-position"
                            if NextStateObs == 0
                                Passive(IndexState)=1;
                                TotalLowProbAdj = TotalLowProbAdj + 1;
                            end
                        end
                    end
                    %   Considering that all "adjacent" states should now
                    %   have value "1" in the matrix, and all other states should
                    %   have value "0" in the matrix, by multiplying by
                    %   "LowProbVal", all adjacent states should have "LowProbVal",
                    %   and all other states should have "0"
                    Passive = Passive/TotalLowProbAdj;
                end
            end
        end
    end
end