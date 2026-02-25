function [NBk] = attitudeController(R,S,P)
% attitudeController : Controls quadcopter toward a reference attitude
%
%
% INPUTS
%
% R ---------- Structure with the following elements:
%
%       zIstark = 3x1 desired body z-axis direction at time tk, expressed as a
%                 unit vector in the I frame.
%
%       xIstark = 3x1 desired body x-axis direction, expressed as a
%                 unit vector in the I frame.
%
% S ---------- Structure with the following elements:
%
%        statek = State of the quad at tk, expressed as a structure with the
%                 following elements:
%                   
%                  rI = 3x1 position in the I frame, in meters
% 
%                 RBI = 3x3 direction cosine matrix indicating the
%                       attitude
%
%                  vI = 3x1 velocity with respect to the I frame and
%                       expressed in the I frame, in meters per second.
%                 
%              omegaB = 3x1 angular rate vector expressed in the body frame,
%                       in radians per second.
%
% P ---------- Structure with the following elements:
%
%    quadParams = Structure containing all relevant parameters for the
%                 quad, as defined in quadParamsScript.m 
%
%     constants = Structure containing constants used in simulation and
%                 control, as defined in constantsScript.m 
%
%
% OUTPUTS
%
% NBk -------- Commanded 3x1 torque expressed in the body frame at time tk, in
%              N-m.
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
  ip.addParameter('zIstark',[],@(x)issize(x,3,1));
  ip.addParameter('xIstark',[],@(x)issize(x,3,1));
  ip.addParameter('rI',[],@(x)issize(x,3,1));
  ip.addParameter('RBI',[],@(x)issize(x,3,3));
  ip.addParameter('vI',[],@(x)issize(x,3,1));
  ip.addParameter('omegaB',[],@(x)issize(x,3,1));
  ip.addParameter('quadParams',[],@(x)isstruct(x));
  ip.addParameter('constants',[],@(x)isstruct(x));
  ip.parse(R,S.statek,P);
end

%% Student code

%                       Insert your code here 
  

%Coeffecient Matrices
K = 1.2 .* [1 0 0;
              0 1 0;
              0 0 1];
% K = 8.2880 .* [1 0 0;
%               0 1 0;
%               0 0 1];
K_d =  0.45 * [1 0 0;
                0 1 0;
                0 0 1];
% K_d = 4.2101 .* [1 0 0;
%                 0 1 0;
%                 0 0 1];


% Determine Desired RBI
zstar = R.zIstark;
xstar = R.xIstark;


b = cross(zstar,xstar) / norm(cross(zstar,xstar));
a = cross(b,zstar);

RBI_star = [a b zstar]';

R_E = RBI_star * S.statek.RBI';

e_E = [ R_E(2,3) - R_E(3,2); R_E(3,1) - R_E(1,3); R_E(1,2) - R_E(2,1)];


% Define Values
omegaB = S.statek.omegaB;
omega_B_cross = crossProductEquivalent(omegaB);
J = P.quadParams.Jq;

% Compute commanded torque
NBk = K * e_E - K_d * omegaB + omega_B_cross * J * omegaB;








%NB = KeE − KdωB + [ωB×] JωB


end % EOF attitudeController.m