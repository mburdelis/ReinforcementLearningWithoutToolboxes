function CurrentErrorsPd = UpdateErrorsPd(States, NStatePairs, GoalState, NAdjPerState, EstimatedPassive, GridSize, CurrentErrorsPd, options);

    for IndexState = 1:NStatePairs
        if (NAdjPerState(IndexState) ~= -1) && (States(IndexState, 2)~=GoalState)
            Passive = UpdatePassiveNewWObs(States(IndexState, :), GridSize, GoalState, States, options);
            CurrentErrorsPd(IndexState) = sum(abs(Passive - EstimatedPassive(IndexState, :)'));
        end
    end

end