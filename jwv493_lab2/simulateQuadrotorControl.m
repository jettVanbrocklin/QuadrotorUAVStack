function [Q] = simulateQuadrotorControl(R,S,P)
% simulateQuadrotorControl : Simulates closed-loop control of a quadrotor
%                            aircraft.
%
%
% INPUTS
%
% R ---------- Structure with the following elements:
%
%          tVec = Nx1 vector of uniformly-sampled time offsets from the
%                 initial time, in seconds, with tVec(1) = 0.
%
%        rIstar = Nx3 matrix of desired CM positions in the I frame, in
%                 meters.  rIstar(k,:)' is the 3x1 position at time tk =
%                 tVec(k).
%
%        vIstar = Nx3 matrix of desired CM velocities with respect to the I
%                 frame and expressed in the I frame, in meters/sec.
%                 vIstar(k,:)' is the 3x1 velocity at time tk = tVec(k).
%
%        aIstar = Nx3 matrix of desired CM accelerations with respect to the I
%                 frame and expressed in the I frame, in meters/sec^2.
%                 aIstar(k,:)' is the 3x1 acceleration at time tk =
%                 tVec(k).
%
%        xIstar = Nx3 matrix of desired body x-axis direction, expressed as a
%                 unit vector in the I frame. xIstar(k,:)' is the 3x1
%                 direction at time tk = tVec(k).
%  
% S ---------- Structure with the following elements:
%
%  oversampFact = Oversampling factor. Let dtIn = R.tVec(2) - R.tVec(1). Then
%                 the output sample interval will be dtOut =
%                 dtIn/oversampFact. Must satisfy oversampFact >= 1.
%
%        state0 = State of the quad at R.tVec(1) = 0, expressed as a structure
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
%                 disturbance vector acting on the quad from R.tVec(k) to
%                 R.tVec(k+1).
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
% Q ---------- Structure with the following elements:
%
%          tVec = Mx1 vector of output sample time points, in seconds, where
%                 Q.tVec(1) = R.tVec(1), Q.tVec(M) = R.tVec(N), and M =
%                 (N-1)*oversampFact + 1.
%  
%         state = State of the quad at times in tVec, expressed as a
%                 structure with the following elements:
%                   
%                rMat = Mx3 matrix composed such that rMat(k,:)' is the 3x1
%                       position at tVec(k) in the I frame, in meters.
% 
%                eMat = Mx3 matrix composed such that eMat(k,:)' is the 3x1
%                       vector of Euler angles at tVec(k), in radians,
%                       indicating the attitude.
%
%                vMat = Mx3 matrix composed such that vMat(k,:)' is the 3x1
%                       velocity at tVec(k) with respect to the I frame
%                       and expressed in the I frame, in meters per
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
  ip.addParameter('rIstar',[],@(x)issize(x,NaN,3));
  ip.addParameter('vIstar',[],@(x)issize(x,NaN,3));
  ip.addParameter('aIstar',[],@(x)issize(x,NaN,3));
  ip.addParameter('xIstar',[],@(x)issize(x,NaN,3));
  ip.addParameter('oversampFact',[],@(x)issize(x,1,1));
  ip.addParameter('distMat',[],@(x)issize(x,NaN,3));
  ip.addParameter('r',[],@(x)issize(x,3,1));
  ip.addParameter('e',[],@(x)issize(x,3,1));
  ip.addParameter('v',[],@(x)issize(x,3,1));
  ip.addParameter('omegaB',[],@(x)issize(x,3,1));
  ip.addParameter('quadParams',[],@(x)isstruct(x));
  ip.addParameter('constants',[],@(x)isstruct(x));
  ip.addParameter('sensorParams',[],@(x)isstruct(x));
  ip.parse(R,S,S.state0,P);
  if sum(cellfun(@isempty,struct2cell(ip.Results)))~=0
    warning('Input empty or missing.');
  end
end

%% Student code

%                       Insert your code here 
  
% FLOW CHART
% TrajectoryController + Attitude Controller -> VoltageConverter
% ->QuadODEHF



% Oversampling
dtIn = R.tVec(2) - R.tVec(1);
dtOut = dtIn/S.oversampFact;
N = length(R.tVec);

% Initialize Struct
Q.tVec = [];
Q.state.rMat = zeros(0,3);
Q.state.eMat = zeros(0,3);
Q.state.vMat = zeros(0,3);
Q.state.omegaBMat = zeros(0,3);

%Parameters
parameters.quadParams = P.quadParams;
parameters.constants = P.constants;


%initial state
RBI = euler2dcm(S.state0.e);
omega0 = zeros(4,1);
Xbig = [S.state0.r',S.state0.v',RBI(1,1),RBI(2,1),RBI(3,1),RBI(1,2),RBI(2,2),RBI(3,2),RBI(1,3),RBI(2,3),RBI(3,3),S.state0.omegaB', omega0']';

for k=1:N-1
    tspan = [R.tVec(k):dtOut:R.tVec(k+1)]';

    % Calculate the commanded Thrust and Torque -> Commanded Voltage every
    % iteration
    R_traj.rIstark = R.rIstar(k,:)';
    R_traj.vIstark = R.vIstar(k,:)';
    R_traj.aIstark = R.aIstar(k,:)';

    S_local.statek.rI = Xbig(1:3);
    S_local.statek.RBI = reshape(Xbig(7:15), 3, 3);
    S_local.statek.vI = Xbig(4:6);
    S_local.statek.omegaB = Xbig(16:18);

    [F_k, zIstark] = trajectoryController(R_traj, S_local, parameters);
    % display(F_k);
    % display(zIstark);
    R_att.zIstark = zIstark;
    R_att.xIstark = R.xIstar(k,:)';
    [NBk] = attitudeController(R_att, S_local, parameters);
    eak = voltageConverter(F_k, NBk, parameters);
    % ^^^^^^^^^^^^^^^^^^^^^ Calculates Commanded Voltage
    %eaVec = S.eaMat(k,:)'; % Transpose so it is a column vector;
    distVec = S.distMat(k,:)';
    [tVeck,XMatk] = ode45(@(t,X)quadOdeFunctionHF(t,X,eak,distVec,parameters), tspan, Xbig);
    % Store Values
    Q.tVec = [Q.tVec; tVeck(1:end-1)];
    Q.state.rMat = [Q.state.rMat; XMatk(1:end-1,1:3)];
    Q.state.vMat = [Q.state.vMat; XMatk(1:end-1,4:6)];
    Q.state.omegaBMat = [Q.state.omegaBMat; XMatk(1:end-1,16:18)];
    for i=1:length(tspan)-1
        RBI = reshape(XMatk(i,7:15),3,3);
        Q.state.eMat = [Q.state.eMat; dcm2euler(RBI)'];
    end


    % Prepare next iteration
    Xbig = XMatk(end,:)';


    if(mod(k,10) == 0)
      RBI = reshape(Xbig(7:15),3,3);
      [U,~,V] = svd(RBI);
      RBI = U*V';
      Xbig(7:15) = RBI(:);
    end
end
    Q.tVec = [Q.tVec; tVeck(end)];
    Q.state.rMat = [Q.state.rMat; XMatk(end,1:3)];
    Q.state.vMat = [Q.state.vMat; XMatk(end,4:6)];
    Q.state.omegaBMat = [Q.state.omegaBMat; XMatk(end,16:18)];
    RBI = reshape(XMatk(end,7:15), 3, 3);
    Q.state.eMat = [Q.state.eMat; dcm2euler(RBI)'];

end % EOF simulateQuadrotorControl.m