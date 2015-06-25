function [HasObs] = CheckStPosPairHasObs(Element, ObstacleIndexes)
%CheckStPosPairHasObs Check if a State=Position Pair has obstacle(s)
%   This function is for the implementation with 1 State = Position pair
%   It checks if any of the positions is an obstacle
%   returns 1 or 0
    HasObs1 = ismember(Element(1), ObstacleIndexes);
    HasObs2 = ismember(Element(2), ObstacleIndexes);
    HasObs = or(HasObs1, HasObs2);
%     for IndexObs=1:NumObstacles
%         if Element(1)==ObstacleIndexes(IndexObs) || Element(2)==ObstacleIndexes(IndexObs)
%             CurStateObs = 1;
%             break;
%         end
%     end
end

