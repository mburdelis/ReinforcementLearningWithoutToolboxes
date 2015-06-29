function NextStateObs = CheckNextStateObsWPairs(options, Position)
%   WARNING: this function must only be used when we are sure that the
%   current state does not contain obstacle positions
NextStateObs = 0;   %   flag that indicates if the found "adjacent" 
                    %   state has an "obstacle-position" inside
for IndexObs=1:options.SizeAuxObstacles
    %   We already know that the current state has
    %   no "obstacle-position" inside, and we also
    %   know that the currently tested "adjacent"
    %   state has its "source" position (col 1) equal to
    %   the "current" position (col 2) of the
    %   current state, so we only need to test the
    %   "current" position
    if Position == options.AuxObstacles(IndexObs)
        NextStateObs = 1;
        break;
    end
end