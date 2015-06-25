function [qn, qt, Pnn, Pnt] = SeparateTerminal(P, q, GoalStateIndex)
%SeparateTerminal Separates Terminal and Non-Terminal States
%   Partitions the passive dynamics matrix and state costs vectors in
%   terminal and non-terminal states
    
    % Generating Pnn
    Pnn = P;
    % removing the column of the terminal state
    Pnn(:, GoalStateIndex) = [];
    % removing the row of the terminal state
    Pnn(GoalStateIndex, :) = [];
    
    % Generating Pnt
    Pnt = P(:, GoalStateIndex);
    Pnt(GoalStateIndex) = [];
    
    % Generating qn
    qn = q;
    qn(GoalStateIndex) = [];
   
    %Generating qt
    qt = q(GoalStateIndex, 1);
end

