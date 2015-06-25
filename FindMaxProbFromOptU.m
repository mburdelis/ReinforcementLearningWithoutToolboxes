function MaxOptUs = FindMaxProbFromOptU(States, OptU)
    [NLin, NCol] = size(States);
    % For MaxOptUs, the first column will be the maximum probability u, the
    % second column will be the corresponding future state which has this
    % highest probability according to u and the third colum will be the
    % "present position" of this most probable state
    MaxOptUs = zeros(NLin, 3);
    CurrentMax = 0; %   auxiliary variable to calculate the maximum of each line
    CurrentMaxIndex = 1;    %   auxiliary variable to calculate the index of the maximum of each line (i.e. the index of the state)
    
    %   Checking if the current state ("x" in the paper) contains an obstacle position 
    for IndexCurState=1:NLin
        CurrentMax = OptU(IndexCurState, 1);
        CurrentMaxIndex = 1;
        for IndexFutState=1:NLin
            if (OptU(IndexCurState, IndexFutState) > CurrentMax)&&(OptU(IndexCurState, IndexFutState) ~= CurrentMax)
                CurrentMax = OptU(IndexCurState, IndexFutState);
                CurrentMaxIndex = IndexFutState;
            end
        end
        MaxOptUs(IndexCurState, 1)=CurrentMax;
        MaxOptUs(IndexCurState, 2)=CurrentMaxIndex;
        MaxOptUs(IndexCurState, 3)=States(CurrentMaxIndex,2);
    end
end