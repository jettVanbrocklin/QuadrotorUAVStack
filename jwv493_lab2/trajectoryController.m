function [Fk,zIstark] = trajectoryController(R,S,P)
% trajectoryController : Controls quadcopter toward a reference trajectory.
%
%
% INPUTS
%
% R ---------- Structure with the following elements:
%
%       rIstark = 3x1 vector of desired CM position at time tk in the I frame,
%                 in meters.
%
%       vIstark = 3x1 vector of desired CM velocity at time tk with respect to
%                 the I frame and expressed in the I frame, in meters/sec.
%
%       aIstark = 3x1 vector of desired CM acceleration at time tk with
%                 respect to the I frame and expressed in the I frame, in
%                 meters/sec^2.
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
% Fk --------- Commanded total thrust at time tk, in Newtons.
%
% zIstark ---- Desired 3x1 body z axis expressed in I frame at time tk.    
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
  ip.addParameter('rIstark',[],@(x)issize(x,3,1));
  ip.addParameter('vIstark',[],@(x)issize(x,3,1));
  ip.addParameter('aIstark',[],@(x)issize(x,3,1));
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


% Constants
m = P.quadParams.m;
g = P.constants.g;

e_3 = [0;0;1];

% Coeffecients
% k = 8.2880;
% kd = 4.2101;
k = 5;
kd = 5;

% Errors
e_r = R.rIstark - S.statek.rI;
e_r_dot = R.vIstark - S.statek.vI;

F_desired = k.*e_r + kd.*e_r_dot + m*g*e_3 + m*R.aIstark;

if norm(F_desired) > 1e-6
    zIstark = F_desired / norm(F_desired);
else
    zIstark = [0;0;1];
end
%display("----------")
%display(zIstark)
%display("===========")
Fk = F_desired' * S.statek.RBI' * e_3;
%I = ker + kd ˙er + mge3 + m¨r∗
  
end % EOF trajectoryController.m