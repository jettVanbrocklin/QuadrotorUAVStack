function [zk] = h_meas(xk,wk,RBIBark,rXIMat,mcVeck,P)
% h_meas : Measurement model for quadcopter.
%
% INPUTS
%
% xk --------- 15x1 state vector at time tk, defined as 
% 
%              xk = [rI', vI', e', ba', bg']'
%
%              where all corresponding quantities are identical to those
%              defined for E.statek in stateEstimatorUKF.m and where e is the
%              3x1 error Euler angle vector defined such that for an estimate
%              RBIHat of the attitude, the true attitude is RBI = D(e)*RBIHat,
%              where D(e) is the DCM formed from the error Euler angle vector
%              e.
%
% wk --------- nz-by-1 measurement noise vector at time tk, defined as
%
%              wk = [wpIk', wbIk', w1C', w2C', ..., wNfkC']'
%
%              where nz = 6 + Nfk*3, with Nfk being the number of features
%              measured by the camera at time tk, and where all 3x1 noise
%              vectors represent additive noise on the corresponding
%              measurements.
%
% RBIBark ---- 3x3 attitude matrix estimate at time tk.
%
% rXIMat ----- Nf-by-3 matrix of coordinates of visual features in the
%              simulation environment, expressed in meters in the I
%              frame. rXIMat(i,:)' is the 3x1 vector of coordinates of the ith
%              feature.
%
% mcVeck ----- Nf-by-1 vector indicating whether the corresponding feature in
%              rXIMat is sensed by the camera: If mcVeck(i) is true (nonzero),
%              then a measurement of the visual feature with coordinates
%              rXIMat(i,:)' is assumed to be made by the camera.  mcVeck
%              should have Nfk nonzero values.
%
% P ---------- Structure with the following elements:
%
%    quadParams = Structure containing all relevant parameters for the
%                 quad, as defined in quadParamsScript.m 
%
%     constants = Structure containing constants used in simulation and
%                 control, as defined in constantsScript.m 
%
%  sensorParams = Structure containing sensor parameters, as defined in
%                 sensorParamsScript.m
%
%
% OUTPUTS
%
% zk --------- nz-by-1 measurement vector at time tk, defined as
%
%              zk = [rpItilde', rbItildeu', v1Ctildeu', ..., vNfkCtildeu']'
%
%              where rpItilde is the 3x1 measured position of the primary
%              antenna in the I frame, rbItildeu is the 3x1 measured unit
%              vector pointing from the primary to the secondary antenna,
%              expressed in the I frame, and viCtildeu is the 3x1 unit vector,
%              expressed in the camera frame, pointing toward the ith 3D
%              feature, which has coordinates rXIMat(i,:)'.
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
  ip.addRequired('xk',@(x)issize(x,15,1));
  ip.addRequired('wk',@(x)issize(x,NaN,1));
  ip.addRequired('RBIBark',@(x)issize(x,3,3));
  ip.addRequired('rXIMat',@(x)issize(x,NaN,NaN));
  ip.addRequired('mcVeck',@(x)issize(x,NaN,1));
  ip.addParameter('quadParams',[],@(x)isstruct(x));
  ip.addParameter('constants',[],@(x)isstruct(x));
  ip.addParameter('sensorParams',[],@(x)isstruct(x));
  ip.parse(xk,wk,RBIBark,rXIMat,mcVeck,P);
end

%% Student code

% ????? Insert your code here


%Pass in values to GNSS and Camera functions. Add noise to returned values

% Unpack xk
rI = xk(1:3);
vI = xk(4:6);
e = xk(7:9);
ba = xk(10:12);
bg = xk(13:15);


RBIk = euler2dcm(e)*RBIBark;

RCB = P.sensorParams.RCB;
rocB = P.sensorParams.rocB;
ra1B = P.sensorParams.raB(:,1);
rbB = P.sensorParams.raB(:,2)-P.sensorParams.raB(:,1);

rbBu = rbB/norm(rbB);

rpI = rI + RBIk'*ra1B;
rubI = RBIk'*rbBu;


rcI = rI + RBIk*rocB;


% vector of vuic's of each iteration
% --------------
% end_value = length(wk);
% i = 7;
% index = 0;
% vuiC = [];
% while i <= end_value
%     index = index + 1;
%     if mcVeck(index) == 0
%         continue; % skips
%     end
%     viI = rXIMat(index,:)' - rcI;
%     vuiC = [vuiC; RCB*RBIk*(viI/norm(viI))];
%     i = i + 3;
% end
end_value = (length(wk) - 6)/3;
index = 1;
vuiC = [];
while index <= end_value
    if mcVeck(index) == 0
        continue; % skips
    end
    viI = rXIMat(index,:)' - rcI;
    vuiC = [vuiC; RCB*RBIk*(viI/norm(viI))];
    index = index + 1;
end
% --------------

h_xk = [rpI; rubI; vuiC];


zk = h_xk + wk;

end 