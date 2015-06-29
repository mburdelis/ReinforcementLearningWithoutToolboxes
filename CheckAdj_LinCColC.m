function adjacent=CheckAdj_LinCColC(LinCur, ColCur, CodPropState, GridSize)
   
    % get the indexes of the line and the column of the grid world,
    % for the given coded proposed state
    [LinProp, ColProp] = GetLinCol(CodPropState, GridSize);
    
    % check if the absolute value of the difference is bigger than one
    if (abs(LinCur-LinProp)>1) || (abs(ColCur-ColProp)>1)
        adjacent = 0;
    else
        adjacent = 1;
    end
    
    