function [GoodIndexes, BadIndexes, OnesObstacles, OnesNonObstacles] = SeparateIndexes_PosIsSt(States, ObstacleIndexes)
%SeparateIndexes_PosIsSt Separates valid and invalid state indices
%   In the case of 1 state = 1 position, the only invalid states are the
%   obstacle positions
    BadIndexes = ObstacleIndexes';
    OnesObstacles = ismember(States, ObstacleIndexes);
    OnesNonObstacles = (OnesObstacles == 0);
    GoodIndexes = find(OnesNonObstacles);

end