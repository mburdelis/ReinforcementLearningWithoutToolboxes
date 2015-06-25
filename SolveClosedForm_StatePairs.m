disp('     Generating the passive dynamics matrix');
GeneratePWObs;
clear q;

%   generating the vector of state costs
disp('     Generating the vector of state costs');
for IndexState=1:NStatePairs
      if States(IndexState, 2) == GoalState
          q(1,IndexState)=options.GoalStateCost;
      else
          q(1,IndexState)=options.OtherStatesCost;
      end
end
  
% for i=1:(NStatePairs-4)
%     q(1,i)=options.OtherStatesCost;
% end
% for i=0:3
%     q(1,NStatePairs-i)=options.GoalStateCost;
% end
% q = [1 1 1 1 1 1 1 1 0];

disp('     Generating a matrix for the calculations');
disp('          Clearing the variable Matrix');
clear Matrix;
disp('          Generating vector of exp(-q)');
Matrix = exp(-q);
disp('          Generating diagonal matrix of the vector');
Matrix = diag(Matrix);
disp('          Multiplying by P (passive dynamics)');
Matrix = Matrix*P;
disp('          Transposing q');
q = q';

disp('     Separating terminal and non-terminal states');
IndexRowPnt = 1;
IndexPnnRowQn = 1;
for IndexSeparation_Row = 1:NStatePairs
    if IndexSeparation_Row ~= options.GoalStatePosPairIndex
        IndexPnnCol = 1;
        for IndexSeparation_Col = 1:NStatePairs
            if IndexSeparation_Col ~= options.GoalStatePosPairIndex
                Pnn(IndexPnnRowQn, IndexPnnCol) = P(IndexSeparation_Row, IndexSeparation_Col);
                IndexPnnCol = IndexPnnCol + 1;
            else
                Pnt(IndexRowPnt, 1) = P(IndexSeparation_Row, IndexSeparation_Col);
                IndexRowPnt = IndexRowPnt + 1;
            end
        end
        qn(IndexPnnRowQn) = q(IndexSeparation_Row);
        IndexPnnRowQn = IndexPnnRowQn + 1;
    else
        qt = q(IndexSeparation_Row);
    end
end

% qn = q(1:(NStatePairs-1));
% qt = q(NStatePairs);
% Pnn = P(1:(NStatePairs-1), 1:(NStatePairs-1));
% Pnt = P(:, NStatePairs);

disp('     Generating the matrices of the system of linear equations');
Left = (diag(exp(qn))-Pnn);
Right = Pnt*exp(-qt);
disp('     Solving');
% Right = Right(1:(NStatePairs-1));
zn = Left\Right;
disp('     Organizing the obtained solution (terminal and non-terminal states)');
for IndexZ = 1:NStatePairs
    if IndexZ < options.GoalStatePosPairIndex
        z_opt(IndexZ, 1) = zn(IndexZ, 1);
    elseif IndexZ > options.GoalStatePosPairIndex
        z_opt(IndexZ, 1) = zn((IndexZ-1), 1);
    else
        z_opt(IndexZ, 1) = exp(-1*(q(IndexZ)));
    end
end


% z_opt = zn;
disp('     Obtaining the optimal v from the optimal z');
v_opt = -log(z_opt);
% v_opt(NStatePairs) = q(NStatePairs);
% z_opt(NStatePairs) = exp(-q(NStatePairs));

disp('     Checking the results, with an eigenvalue calculation');
disp('          Solving by calculating the eigenvectors and eigenvalues');
[eVec eVal] = eig(Matrix);

disp('          Multiplying the matrix by the obtained optimal z');
Matrixz_opt = Matrix*z_opt;