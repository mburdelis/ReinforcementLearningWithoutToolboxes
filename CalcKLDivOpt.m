function KLDivOpt = CalcKLDivOpt(OptU, P, CurState, NStatePairs)
    
    KLDivOpt = 0;
    
    for IndexPosFutState=1:NStatePairs
        if P(CurState, IndexPosFutState)  ~= 0
            KLDivOpt = KLDivOpt + OptU(CurState, IndexPosFutState)*log(OptU(CurState, IndexPosFutState)/P(CurState, IndexPosFutState));
        end    
    end
end