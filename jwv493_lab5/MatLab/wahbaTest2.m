
% True rotation: 90 deg about +z
R_true = [0 -1  0;
          1  0  0;
          0  0  1];

% Inertial vectors (x and y axes)
vIMat = [1 0 0;
         0 1 0];

% Corresponding body vectors = R_true * vI
vBMat = (R_true * vIMat')';

% Equal weights
aVec = ones(2,1);

% Call Wahba solver
RBI_est = wahbaSolver(aVec, vIMat, vBMat);

disp('Estimated RBI (90-deg test):');
disp(RBI_est);

% Compare with true rotation
diff_norm = norm(RBI_est - R_true, 'fro');
fprintf('Difference from R_true (Frobenius norm): %e\n', diff_norm);
