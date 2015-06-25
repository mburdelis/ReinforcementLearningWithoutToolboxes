function MaxOptUs = FindMaxProbFromOptU_PosIsSt(OptU)
%FindMaxProbFromOptU_PosIsSt Finds the future state with maximum
%probability

    % For MaxOptUs, the first column will be the maximum probability u, the
    % second column will be the corresponding future state which has this
    % highest probability according to u
    [MaxOptUs(:, 1), MaxOptUs(:, 2)] = max(OptU, [], 2);
end