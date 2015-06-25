function [P, q, z_opt, v_opt, Matrix, Matrix_z_opt] = SolveClosedForm_PosIsSt(States, NStates, GridSize, GoalPos, ObstacleIndexes, GoalStateIndex, OtherStatesCost, GoalStateCost);

    disp('     Generating the passive dynamics matrix');
    P = GeneratePWObs_PosIsSt(States, NStates, GridSize, GoalPos, ObstacleIndexes, GoalStateIndex);
    clear q;

    disp('     Generating the vector of state costs');
    q = ones(NStates, 1)*OtherStatesCost;
    q(GoalStateIndex, 1) = GoalStateCost;

    disp('     Separating terminal and non-terminal states');
    [qn, qt, Pnn, Pnt] = SeparateTerminal(P, q, GoalStateIndex);

    disp('     Generating the matrices of the system of linear equations');
    Left = (diag(exp(qn))-Pnn);
    Right = Pnt*exp(-qt);

    disp('     Solving');
    zn = Left\Right;
    disp('     Organizing the obtained solution (terminal and non-terminal states)');
    zt = exp(-1*(q(GoalStateIndex)));
    z_opt = [zn(1:GoalStateIndex-1);zt;zn(GoalStateIndex:end)];

    disp('     Obtaining the optimal v from the optimal z');
    v_opt = -log(z_opt);

    disp('     Checking the results, with an eigenvalue calculation');
    disp('     Generating a matrix for the calculations');
    disp('          Clearing the variable Matrix');
    clear Matrix;
    disp('          Generating vector of exp(-q)');
    Matrix = exp(-q);
    disp('          Generating diagonal matrix of the vector');
    Matrix = diag(Matrix);
    disp('          Multiplying by P (passive dynamics)');
    Matrix = Matrix*P;

    % Verifying the solution: the Matrix_opt must be equal to z_opt
    % this program only does the multiplication but does not do any
    % comparison
    disp('          Multiplying the matrix by the obtained optimal z');
    Matrix_z_opt = Matrix*z_opt;

    %disp('          Solving by calculating the eigenvectors and eigenvalues');
    %[eVec eVal] = eig(Matrix);

end