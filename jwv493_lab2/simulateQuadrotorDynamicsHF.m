function [P] = simulateQuadrotorDynamicsHF(S)
% simulateQuadrotorDynamicsHF : Simulates the dynamics of a quadrotor
%                               aircraft (high-fidelity version).
%
%
% INPUTS
%
% S ---------- Structure with the following elements:
%
%          tVec = Nx1 vector of uniformly-sampled time offsets from the
%                 initial time, in seconds, with tVec(1) = 0.
%
%  oversampFact = Oversampling factor. Let dtIn = tVec(2) - tVec(1). Then the
%                 output sample interval will be dtOut =
%                 dtIn/oversampFact. Must satisfy oversampFact >= 1.   
%
%         eaMat = (N-1)x4 matrix of motor voltage inputs.  eaMat(k,j) is the
%                 constant (zero-order-hold) voltage for the jth motor over
%                 the interval from tVec(k) to tVec(k+1).
%
%        state0 = State of the quad at tVec(1) = 0, expressed as a structure
%                 with the following elements:
%                   
%                   r = 3x1 position in the world frame, in meters
% 
%                   e = 3x1 vector of Euler angles, in radians, indicating the
%                       attitude
%
%                   v = 3x1 velocity with respect to the world frame and
%                       expressed in the world frame, in meters per second.
%                 
%              omegaB = 3x1 angular rate vector expressed in the body frame,
%                       in radians per second.
%
%       distMat = (N-1)x3 matrix of disturbance forces acting on the quad's
%                 center of mass, expressed in Newtons in the world frame.
%                 distMat(k,:)' is the constant (zero-order-hold) 3x1
%                 disturbance vector acting on the quad from tVec(k) to
%                 tVec(k+1).
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
% P ---------- Structure with the following elements:
%
%          tVec = Mx1 vector of output sample time points, in seconds, where
%                 P.tVec(1) = S.tVec(1), P.tVec(M) = S.tVec(N), and M =
%                 (N-1)*oversampFact + 1.
%                  
%  
%         state = State of the quad at times in tVec, expressed as a structure
%                 with the following elements:
%                   
%                rMat = Mx3 matrix composed such that rMat(k,:)' is the 3x1
%                       position at tVec(k) in the world frame, in meters.
% 
%                eMat = Mx3 matrix composed such that eMat(k,:)' is the 3x1
%                       vector of Euler angles at tVec(k), in radians,
%                       indicating the attitude.
%
%                vMat = Mx3 matrix composed such that vMat(k,:)' is the 3x1
%                       velocity at tVec(k) with respect to the world frame
%                       and expressed in the world frame, in meters per
%                       second.
%                 
%           omegaBMat = Mx3 matrix composed such that omegaBMat(k,:)' is the
%                       3x1 angular rate vector expressed in the body frame in
%                       radians, that applies at tVec(k).
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
  ip.addParameter('tVec',[],@(x)issize(x,NaN,1));
  ip.addParameter('oversampFact',[],@(x)issize(x,1,1));
  ip.addParameter('eaMat',[],@(x)issize(x,NaN,4));
  ip.addParameter('distMat',[],@(x)issize(x,NaN,3));
  ip.addParameter('quadParams',[],@(x)isstruct(x));
  ip.addParameter('constants',[],@(x)isstruct(x));
  ip.addParameter('r',[],@(x)issize(x,3,1));
  ip.addParameter('e',[],@(x)issize(x,3,1));
  ip.addParameter('v',[],@(x)issize(x,3,1));
  ip.addParameter('omegaB',[],@(x)issize(x,3,1));
  ip.parse(S,S.state0);
  if sum(cellfun(@isempty,struct2cell(ip.Results)))~=0
    warning('Input empty or missing.');
  end
end

%% Student code

%                       Insert your code here 


% Oversampling
dtIn = S.tVec(2) - S.tVec(1);
dtOut = dtIn/S.oversampFact;
N = length(S.tVec);

% Initial Rotor Rates



% Initialize Struct
P.tVec = [];
P.state.rMat = zeros(0,3);
P.state.eMat = zeros(0,3);
P.state.vMat = zeros(0,3);
P.state.omegaBMat = zeros(0,3);

%Parameters
parameters.quadParams = S.quadParams;
parameters.constants = S.constants;
%initial state
RBI = euler2dcm(S.state0.e);
omega0 = zeros(4,1);
Xbig = [S.state0.r',S.state0.v',RBI(1,1),RBI(2,1),RBI(3,1),RBI(1,2),RBI(2,2),RBI(3,2),RBI(1,3),RBI(2,3),RBI(3,3),S.state0.omegaB', omega0']';
for k=1:N-1
    tspan = [S.tVec(k):dtOut:S.tVec(k+1)]';
    eaVec = S.eaMat(k,:)'; % Transpose so it is a column vector;
    distVec = S.distMat(k,:)';
    [tVeck,XMatk] = ode45(@(t,X)quadOdeFunctionHF(t,X,eaVec,distVec,parameters), tspan, Xbig);
    % Store Values
    P.tVec = [P.tVec; tVeck(1:end-1)];
    P.state.rMat = [P.state.rMat; XMatk(1:end-1,1:3)];
    P.state.vMat = [P.state.vMat; XMatk(1:end-1,4:6)];
    P.state.omegaBMat = [P.state.omegaBMat; XMatk(1:end-1,16:18)];
    for i=1:length(tspan)-1
        RBI = reshape(XMatk(i,7:15),3,3);
        P.state.eMat = [P.state.eMat; dcm2euler(RBI)'];
    end

      % Ensure that RBI remains orthogonal
    
    % Prepare next iteration
    Xbig = XMatk(end,:)';
end
    P.tVec = [P.tVec; tVeck(end)];
    P.state.rMat = [P.state.rMat; XMatk(end,1:3)];
    P.state.vMat = [P.state.vMat; XMatk(end,4:6)];
    P.state.omegaBMat = [P.state.omegaBMat; XMatk(end,16:18)];
    RBI = reshape(XMatk(end,7:15), 3, 3);
    P.state.eMat = [P.state.eMat; dcm2euler(RBI)'];

end % EOF simulateQuadrotorDynamicsHF.m