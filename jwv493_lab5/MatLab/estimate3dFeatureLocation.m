function [rXIHat,Px] = estimate3dFeatureLocation(M,P)
% estimate3dFeatureLocation : Estimate the 3D coordinates of a feature point
%                             seen by two or more cameras with known pose.
%
%
% INPUTS
%
% M ---------- Structure with the following elements:
%
%       rxArray = 1xN cell array of measured positions of the feature point
%                 projection on the camera's image plane, in pixels.
%                 rxArray{i} is the 2x1 vector of coordinates of the feature
%                 point as measured by the ith camera.  To ensure the
%                 estimation problem is observable, N must satisfy N >= 2 and
%                 at least two cameras must be non-colinear.
%
%      RCIArray = 1xN cell array of I-to-camera-frame attitude matrices.
%                 RCIArray{i} is the 3x3 attitude matrix corresponding to the
%                 measurement rxArray{i}.
%
%       rcArray = 1xN cell array of camera center positions.  rcArray{i} is
%                 the 3x1 position of the camera center corresponding to the
%                 measurement rxArray{i}, expressed in the I frame in meters.
%
% P ---------- Structure with the following elements:
%
%  sensorParams = Structure containing all relevant parameters for the quad's
%                 sensors, as defined in sensorParamsScript.m
%
% OUTPUTS
%
%
% rXIHat -------- 3x1 estimated location of the feature point expressed in I
%                 in meters.
%
% Px ------------ 3x3 error covariance matrix for the estimate rxIHat.
%
%+------------------------------------------------------------------------------+
% References:
%
%
% Author:  
%+==============================================================================+ 

%% Validate inputs
global INPUT_PARSING;
if INPUT_PARSING 
  issize =@(x,z1,z2) validateattributes(x,{'numeric'},{'size',[z1,z2]});
  ip = inputParser; ip.StructExpand = true; ip.KeepUnmatched = true;
  ip.addParameter('rxArray',[],@(x)issize(x{1},2,1));
  ip.addParameter('RCIArray',[],@(x)issize(x{1},3,3));
  ip.addParameter('rcArray',[],@(x)issize(x{1},3,1));
  ip.addParameter('sensorParams',[],@(x)isstruct(x));
  ip.parse(M,P);
end

%% Student code

%                       Insert your code here 

% rxArray contains my measured x_i and y_i needed for H
% P is the focal length matrix thing * [R_CI, -t_C] matrix thing, then I
% use the rows of that. Easy-peasy


N = numel(M.rxArray); % Finds N

if N < 2
    error('At least two measurements are required for a valid estimation.');
end

K = P.sensorParams.K; % Camera Intrinsic Matrix

H_prime = [];

ps = P.sensorParams.pixelSize;


for i=1:N
    % Calculate t_C = R_CI * tI, which is the vector between the inertial
    % frame and the camera frame center
    t_C = M.RCIArray{i} * M.rcArray{i};

    % fprintf("size of t_C")
    % size(t_C)
    % fprintf("--------")
    % 
    RCI_tc = [M.RCIArray{i}, -t_C];
    % fprintf("size of RCI_tC")
    % size(RCI_tc)
    % fprintf("--------")
    % % Calculate P and Extract Rows
    % fprintf("size of k")
    % size(K)
    % K
    % fprintf("--------")
    % size(RCI_tc)
    % fprintf("--------")
    P_matrix = K * RCI_tc;
    p1 = P_matrix(1, :);
    p2 = P_matrix(2, :);
    p3 = P_matrix(3, :);
    % Calculate 2 H_prime rows for each iteration
    x_delta = ps * M.rxArray{i}(1);
    y_delta = ps * M.rxArray{i}(2);
    H_prime = [H_prime; x_delta * p3 - p1; y_delta * p3 - p2];
end


% R matrix
Rc = P.sensorParams.Rc;
R = (ps*ps) * kron(eye(N), Rc);


% Extract H and -z from H_prime
H = H_prime(:, 1:3);
z = -H_prime(:, 4);

% Find rXIHat
R_inv = inv(R);
rXIHat = inv(H'*R_inv*H) * H'*R_inv*z; % Directly from Main.pdf page 82
% Find Covariance Matrix
Px = inv(H'*R_inv*H);



  
end % EOF estimate3dFeatureLocation.m