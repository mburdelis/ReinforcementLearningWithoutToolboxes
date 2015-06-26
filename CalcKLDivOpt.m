function [KLDivOpt] = CalcKLDivOpt(NStates, P, OptU)
    
    KLDivOpt = nansum(OptU.*log(OptU./P));
%     KLDivOpt = 0;
%     for IndexPosFutState=1:NStates
%         if P(IndexPosFutState)  ~= 0
%             KLDivOpt = KLDivOpt + OptU(IndexPosFutState)*log(OptU(IndexPosFutState)/P(IndexPosFutState));
%         end    
%     end
end