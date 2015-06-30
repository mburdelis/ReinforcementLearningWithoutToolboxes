function [GoodIndexes, BadIndexes] = SeparateIndexes_PosIsSt(States, ObstacleIndexes)
%SeparateIndexes_PosIsSt Separates valid and invalid state indices
%   In the case of 1 state = 1 position, the only invalid states are the
%   obstacle positions
    BadIndexes = ObstacleIndexes';
    Temp = ismember(States, ObstacleIndexes);
    GoodIndexes = find(Temp == 0);

end