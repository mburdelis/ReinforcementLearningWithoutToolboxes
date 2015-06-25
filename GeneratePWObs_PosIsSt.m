function P = GeneratePWObs_PosIsSt(States, NStates, GridSize, GoalPos, ObstacleIndexes, GoalStateIndex)

    Passive=zeros(NStates, 1);

    %   copying the first state to "CurSt"
    CurSt = States(1, 1);
    Passive = UpdatePassiveNewWObs_PosIsSt(CurSt, States, NStates, GridSize, GoalPos, ObstacleIndexes, GoalStateIndex);
    P = Passive;
    for i=2:(NStates)
        CurSt = States(i,1);
        Passive = UpdatePassiveNewWObs_PosIsSt(CurSt, States, NStates, GridSize, GoalPos, ObstacleIndexes, GoalStateIndex);
        P(:,i)=Passive;
    end
    P = P';
end