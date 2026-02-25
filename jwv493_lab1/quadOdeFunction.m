function [Xdot] = quadOdeFunction(t,X,omegaVec,distVec,P)
% quadOdeFunction : Ordinary differential equation function that models
%                   quadrotor dynamics.  For use with one of Matlab's ODE
%                   solvers (e.g., ode45).
%
%
% INPUTS
%
% t ---------- Scalar time input, as required by Matlab's ODE function
%              format.
%
% X ---------- Nx-by-1 quad state, arranged as 
%
%              X = [rI',vI',RBI(1,1),RBI(2,1),...,RBI(2,3),RBI(3,3),omegaB']'
%
%              rI = 3x1 position vector in I in meters
%              vI = 3x1 velocity vector wrt I and in I, in meters/sec
%             RBI = 3x3 attitude matrix from I to B frame
%          omegaB = 3x1 angular rate vector of body wrt I, expressed in B
%                   in rad/sec
%
% omegaVec --- 4x1 vector of rotor angular rates, in rad/sec.  omegaVec(i)
%              is the constant rotor speed setpoint for the ith rotor.
%
%  distVec --- 3x1 vector of constant disturbance forces acting on the quad's
%              center of mass, expressed in Newtons in I.
%
% P ---------- Structure with the following elements:
%
%    quadParams = Structure containing all relevant parameters for the
%                 quad, as defined in quadParamsScript.m 
%
%     constants = Structure containing constants used in simulation and
%                 control, as defined in constantsScript.m 
%
% OUTPUTS
%
% Xdot ------- Nx-by-1 time derivative of the input vector X
%
%+------------------------------------------------------------------------------+
% References:
%
%
% Author:  Jett Vanbrocklin
%+==============================================================================+

%% Validate inputs
global INPUT_PARSING;
if INPUT_PARSING
  issize =@(x,z1,z2) validateattributes(x,{'numeric'},{'size',[z1,z2]});
  ip = inputParser; ip.StructExpand = true; ip.KeepUnmatched = true;
  ip.addRequired('t',@(x)issize(x,1,1));
  ip.addRequired('X',@(x)issize(x,18,1));
  ip.addRequired('omegaVec',@(x)issize(x,4,1));
  ip.addRequired('distVec',@(x)issize(x,3,1));
  ip.addParameter('quadParams',[],@(x)isstruct(x));
  ip.addParameter('constants',[],@(x)isstruct(x));
  ip.parse(t,X,omegaVec,distVec,P);
end

%% Student code

%                       Insert your code here 

%constants
gravity = [0;0;P.constants.g;];
kF = P.quadParams.kF(1,1);
kN = P.quadParams.kN(1,1);
mass = P.quadParams.m;

% Extract values from state vector
position_cvector = X(1:3); % Column vector of position
velocity_cvector = X(4:6); % Column vector of velocity
RBI_MAT = reshape(X(7:15), 3, 3); % 3x3 DCM
omega_cvector = X(16:18); % Column vector of ang. velocity
omega_cross = crossProductEquivalent(omega_cvector);
%Fi
F = [0;0;0;];
N = [0;0;0;];
for i=1:4
    Fi = [0;0;kF*(omegaVec(i)^2);];
    Ni = [0;0;kN*(omegaVec(i)^2);];
    F = F + Fi;
    N = N + ((P.quadParams.omegaRdir(i))*(Ni + cross(P.quadParams.rotor_loc(:,i), Fi)));
end
% Equations
der_position = velocity_cvector;
der_velocity = -gravity + (RBI_MAT'*F)/mass + distVec/mass;
der_omega = inv(P.quadParams.Jq) * (N - omega_cross*P.quadParams.Jq*omega_cvector);
der_RBI = -omega_cross*RBI_MAT;

Xdot = [der_position',der_velocity',der_RBI(1,1),der_RBI(2,1),der_RBI(3,1),der_RBI(1,2),der_RBI(2,2),der_RBI(3,2),der_RBI(1,3),der_RBI(2,3),der_RBI(3,3),der_omega']';


end % EOF quadOdeFunction.m