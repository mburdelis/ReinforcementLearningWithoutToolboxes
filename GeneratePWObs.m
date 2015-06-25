% GridSize = 3;
% nstates = GridSize^2;
Passive=zeros(NStatePairs, 1);

%   copying the first state pair to "StatePair"
StatePair = States(1,:);
Passive = UpdatePassiveNewWObs(StatePair, options.input.GridSize, GoalState, States, options);
P = Passive;
for i=2:(NStatePairs)
    StatePair = States(i,:);
    Passive = UpdatePassiveNewWObs(StatePair, options.input.GridSize, GoalState, States, options);
    P(:,i)=Passive;
end
P = P';