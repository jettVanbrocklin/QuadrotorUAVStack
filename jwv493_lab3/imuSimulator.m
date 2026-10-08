function [ftildeB,omegaBtilde] = imuSimulator(S,P)
% imuSimulator : Simulates IMU measurements of specific force and
%                body-referenced angular rate.
%
% INPUTS
%
% S ---------- Structure with the following elements:
%
%        statek = State of the quad at tk, expressed as a structure with the
%                 following elements:
%                   
%                  rI = 3x1 position of CM in the I frame, in meters
%
%                  vI = 3x1 velocity of CM with respect to the I frame and
%                       expressed in the I frame, in meters per second.
%
%                  aI = 3x1 acceleration of CM with respect to the I frame and
%                       expressed in the I frame, in meters per second^2.
% 
%                 RBI = 3x3 direction cosine matrix indicating the
%                       attitude of B frame wrt I frame
%
%              omegaB = 3x1 angular rate vector expressed in the body frame,
%                       in radians per second.
%
%           omegaBdot = 3x1 time derivative of omegaB, in radians per
%                       second^2.
%
% P ---------- Structure with the following elements:
%
%  sensorParams = Structure containing all relevant parameters for the
%                 quad's sensors, as defined in sensorParamsScript.m 
%
%     constants = Structure containing constants used in simulation and
%                 control, as defined in constantsScript.m 
%
% OUTPUTS
%
% ftildeB ---- 3x1 specific force measured by the IMU's 3-axis accelerometer
%
% omegaBtilde  3x1 angular rate measured by the IMU's 3-axis rate gyro
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
  ip.addParameter('rI',[],@(x)issize(x,3,1))
  ip.addParameter('vI',[],@(x)issize(x,3,1))
  ip.addParameter('aI',[],@(x)issize(x,3,1))
  ip.addParameter('RBI',[],@(x)issize(x,3,3))
  ip.addParameter('omegaB',[],@(x)issize(x,3,1))
  ip.addParameter('omegaBdot',[],@(x)issize(x,3,1))
  ip.addParameter('sensorParams',[],@(x)isstruct(x));
  ip.addParameter('constants',[],@(x)isstruct(x));
  ip.parse(S.statek,P);
end

%% Student code

%                       Insert your code here 

% TO RETAIN ba AND bg values in between function calls.
persistent ba bg
if(isempty(ba))
% Set ba's initial value
QbaSteadyState = P.sensorParams.Qa2/(1 - P.sensorParams.alphaa^2);
ba = mvnrnd(zeros(3,1), QbaSteadyState)';
end
if(isempty(bg))
% Set bg's initial value
QbgSteadyState = P.sensorParams.Qg2/(1 - P.sensorParams.alphag^2);
bg = mvnrnd(zeros(3,1), QbgSteadyState)';
end


% accelerometer:
% locals
g = P.constants.g;
e_3 = [0 0 1]';
gterm = g*e_3;


Qa = P.sensorParams.Qa;

alphaa = P.sensorParams.alphaa;
Qa2 = P.sensorParams.Qa2;

% Noise for Acceleromter
va = mvnrnd(zeros(3,1), Qa)';
va2 = mvnrnd(zeros(3,1), Qa2)';

% Calculate Bias ba
ba = alphaa * ba + va2;
% Calculate ftildeB
ftildeB = S.statek.RBI*(S.statek.aI + gterm) + ba + va;
%-------------------%

% Rate Gyro:
% locals
alphag = P.sensorParams.alphag;
Qg = P.sensorParams.Qg;
Qg2 = P.sensorParams.Qg2;

% Noise
vg = mvnrnd(zeros(3,1), Qg)';
vg2 = mvnrnd(zeros(3,1), Qg2)';

% Calculate bg
bg = alphag*bg + vg2;
% Calculate omegaBtilde

omegaBtilde = S.statek.omegaB + bg + vg;

end % EOF imuSimulator.m