function CurrentErrorsQ = UpdateErrorsQ_PosIsSt(Estimated_q, Correct_q)
%UpdateErrorsQ_PosIsSt Calculates the error of the estimated q
%   The error for each state is the absolute value of the difference
    CurrentErrorsQ = sum(abs(Correct_q - Estimated_q), 2);

end

