function [GoodIndexes, BadIndexes] = SeparateIndexes(options, States, GoalState, NStatePairs)
    
    IndexGood = 1;
    IndexBad = 1;
    
    for IndexState=1:NStatePairs
        FlagIsImpossible = 0;
        for IndexObs=1:options.SizeAuxObstacles
            if States(IndexState, 1)==options.AuxObstacles(IndexObs) || States(IndexState, 1)== GoalState || States(IndexState, 2)==options.AuxObstacles(IndexObs)
                FlagIsImpossible = 1;
                break;
            end
        end
        if FlagIsImpossible == 0
            GoodIndexes(IndexGood) = IndexState;
            IndexGood = IndexGood + 1;
        else
            BadIndexes(IndexBad) = IndexState;
            IndexBad = IndexBad + 1;
        end
    end
end