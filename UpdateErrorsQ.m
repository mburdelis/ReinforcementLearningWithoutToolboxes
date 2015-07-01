function CurrentErrorsQ = UpdateErrorsQ(States, NStatePairs, GoalState, NAdjPerState, Estimated_q, CurrentErrorsQ, options)

    %   We will only update the error for the states which do not have the
    %   goal position as their current positions, because we are not
    %   estimating "q" for these cases
    for IndexState = 1:NStatePairs
        if (NAdjPerState(IndexState) ~= -1) && (States(IndexState, 2)~=GoalState)
            CurrentErrorsQ(IndexState) = (abs(options.OtherStatesCost - Estimated_q(IndexState)));
        end
    end

end