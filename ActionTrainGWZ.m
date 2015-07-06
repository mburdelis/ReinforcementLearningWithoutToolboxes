% NPC?iŠw?KƒvƒŒƒCƒ„?[?j‚Ì?s“®‘I‘ð‚Æ?CŠw?K‚·‚é?Û‚Ì‘Î?í‘ŠŽè‚Æ‚È‚é?l?H’m”\ƒvƒŒƒCƒ„?[‚Ìƒvƒ?ƒOƒ‰ƒ€

function [action,cost,state3,fin,CodedStatePair] = ActionTrainGWZ(policy,t,state3,States,GoalState,nactions,NStatePairs,prev_or_next,GoalStateCost,OtherStatesCost);
  cost = 1;

  % Verifying if the current (previous) state is the goal state
  % This implies in the cost that will be returned for the Z-Learning
  if prev_or_next==0 % if the value of prev_or_next is "0", the cost to be returned 
                     % must correspond the previous state
    TestState = CheckGoal(state3, GoalState);
  end
   
  
  % Šw?KƒvƒŒƒCƒ„?[
  % ?Å?‰‚ÌƒXƒeƒbƒv‚Å‚Í1ƒ}ƒX–Ú‚ð‘I‘ð
    % ?­?ôpolicy‚É?]‚¢ƒ‰ƒ“ƒ_ƒ€‚É?s“®‚ð‘I‘ð
    CodCurState = EncodeGW(state3);
    if(CodCurState ~= -1)
        
        %   Finding the concrete future state from the given controlled
        %   transition distribution "u" stored, for the current state
        %   visit, in the vector "policy"
        while(1)
          random = rand;

          cprob = 0;
          for a=1:NStatePairs      
            cprob = cprob + policy(a);
            if(random < cprob)
              break;
            end
          end

          %aqui esta faltando codificar os estados (atual e proposto)

          CodPropState = States(a,2);
          CodedStatePair = a;


          % Šù‚Éƒ}ƒX‚ª–„‚Ü‚Á‚Ä‚¢‚È‚¢‚©‚Ç‚¤‚©‚ðŠm”F
          % this check must be altered, to verify if the new position is
          % adjacent to the actual one
          adjacent=CheckAdj(CodCurState,CodPropState, sqrt(nactions));
          if adjacent==1 
            break;
          end
        end

      action = CodPropState;
      state3=zeros(1, nactions);
      state3(CodPropState)=1;
      fin = CheckGoal(state3, GoalState);
      if prev_or_next~=0    % if the value of prev_or_next is NOT "0", the cost to be returned 
                            % must correspond the next (destination) state
        TestState = CheckGoal(state3, GoalState);
      end
    else
        fin = -1;
    end
    
    % Assigning the cost, verifying first if the current state 
    % (before taking the action) is the goal state
    if(TestState == 1)   % if it is the goal state:
        cost = GoalStateCost;
        return;
    else  % if it is not:
        cost = OtherStatesCost;
        return;
    end
end