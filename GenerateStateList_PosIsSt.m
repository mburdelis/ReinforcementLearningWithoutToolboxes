function [States, NStates] = GenerateStateList_PosIsSt(GridSize)
%   GenerateStateList_PosIsSt
%   Generates the list of states and the variable NStates containing
%   the number of states
    States = (1:GridSize^2)';
    NStates = GridSize^2;
end

