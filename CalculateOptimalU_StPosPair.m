function [OptU] = CalculateOptimalU_StPosPair(States, NStates, ObstacleIndexes, NumObstacles, z_opt, q, P)
%CalculateOptimalU_StPosPair Calculates the optimal transition distributions u

    OptUForCurSt=zeros(NStates, 1);
    
    %   copying the first state pair to "CurSt"
    CurSt = States(1,:);
    OptUForCurSt = CalcOptUWObsForCurSt_StPosPair(CurSt, States, NStates, 1, ObstacleIndexes, NumObstacles, z_opt, q(1,1), P(1,:));
    OptU = OptUForCurSt;
    for i=2:(NStates)
        CurSt = States(i,:);
        OptUForCurSt = CalcOptUWObsForCurSt_StPosPair(CurSt, States, NStates, i, ObstacleIndexes, NumObstacles, z_opt, q(i,1), P(i,:));
        OptU(i, :)=OptUForCurSt;
    end
end