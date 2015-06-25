for IndexStates = 1:NStatePairs
    if States(IndexStates, 1) == options.input.GoalState && States(IndexStates, 2) == options.input.GoalState
        options.input.GoalStatePosPairIndex = IndexStates;
        break;
    end
end
