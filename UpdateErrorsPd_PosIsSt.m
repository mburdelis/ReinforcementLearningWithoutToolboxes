function CurrentErrorsPd = UpdateErrorsPd_PosIsSt(EstimatedPassive, CorrectPassive)
%UpdateErrorsPd_PosIsSt Calculates the error of current estimated Pd
%   Calculated state-by-state, the sum of absolute differences
    CurrentErrorsPd = sum(abs(CorrectPassive - EstimatedPassive), 2);
end

