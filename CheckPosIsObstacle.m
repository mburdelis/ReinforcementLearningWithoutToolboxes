function [IsObs] = CheckPosIsObstacle(Element, ObstacleIndexes)
%CheckPosIsObstacle 
%   checks if an element is present in the obstacle list
%   returns "1" if present, "0" if not

IsObs = ismember(Element, ObstacleIndexes);

end

