function OptUForCurSt = CalcOptUWObsForCurSt_StPosPair(CurSt, States, NStates, IndexCurSt, ObstacleIndexes, NumObstacles, z_opt, StateCostCurSt, PCurSt)
%CalcOptUWObsForCurSt_StPosPair Calculates optimal u (1 state = 2 positions
%implementation)
%   The implementation was separated into different functions to avoid
%   processing too many "if" commands to check StateType    
    OptUForCurSt = zeros(NStates, 1);
    CurStateObs = 0;   %   flag that indicates if the current state contains is an "obstacle" position
    
    %   Checking if the current state ("x" in the paper) contains an obstacle position
    CurStateObs = CheckStPosPairHasObs(CurSt, ObstacleIndexes);

    %   The "if" command below has no "else", because when the condition is
    %   not true, the "OptU" must be all zeros, and it already is so
    %   before the "if" command
    if (CurStateObs == 0)&&(z_opt(IndexCurSt)~=0)
        % the "&&(z_opt(IndexCurSt)~=0)" part of the "if" was included
        % because we will not check "if P(IndexCurSt,IndexState) ~= 0"
        OptUForCurSt = exp(-StateCostCurSt)*PCurSt.*z_opt'/z_opt(IndexCurSt);
%         for IndexState=1:NLin
%             if P(IndexCurSt,IndexState) ~= 0
%                 %   updating the passive dynamics value at the
%                 %   state (only if the passive dynamics from the current
%                 %   state to the state is not zero)
%                 OptUForCurSt(IndexState)=exp(-StateCostCurSt)*P(IndexCurSt,IndexState)*z_opt(IndexState)/z_opt(IndexCurSt);
%             end
%         end
    end
end