NSingleStates = options.input.GridSize^2;
ind = 1;    %   "ind" corresponds to the index of a state (a position pair) in the list of states
for i=1:NSingleStates
    for j=1:NSingleStates
        X=CheckAdj(i,j,options.input.GridSize);
        if X==1
            States(ind, 1)=j;
            States(ind, 2)=i;
            ind = ind+1;
        end
    end
end
[NLin, NCol] = size(States);
NStatePairs = NLin;
NStates = NStatePairs;
