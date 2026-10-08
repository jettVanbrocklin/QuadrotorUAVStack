
% True rotation = identity
R_true = eye(3);

% Choose a few easy unit vectors
vIMat = [1 0 0;
         0 1 0;
         0 0 1];   % x, y, z axes

% Since R_true = I, body vectors are the same
vBMat = vIMat;

% Equal weights
aVec = ones(3,1);

% Call Wahba solver
RBI_est = wahbaSolver(aVec, vIMat, vBMat);

% Display result
disp('Estimated RBI (identity test):');
disp(RBI_est);

% Simple checks
fprintf('Difference from identity (Frobenius norm): %e\n',...
    norm(RBI_est - eye(3), 'fro'));
