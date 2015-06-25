function [lin, col]=GetLinCol(ind, NumCols)
    if mod(ind, NumCols)~=0 % "mod" returns the rest of the division
        lin = floor(ind/NumCols) + 1;   % "floor" rounds the result of the division to the nearest integer
                                        % less than or equal to the result
    else
        lin = floor(ind/NumCols);
    end
    col = ind - NumCols*(lin-1);