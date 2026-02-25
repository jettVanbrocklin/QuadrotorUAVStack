function [Xdot] = quadOdeFunctionHF(t,X,eaVec,distVec,P)
% quadOdeFunctionHF : Ordinary differential equation function that models
%                     quadrotor dynamics -- high-fidelity version.  For use
%                     with one of Matlab's ODE solvers (e.g., ode45).
%
%
% INPUTS
%
% t ---------- Scalar time input, as required by Matlab's ODE function
%              format.
%
% X ---------- Nx-by-1 quad state, arranged as 
%
%              X = [rI',vI',RBI(1,1),RBI(2,1),...,RBI(2,3),RBI(3,3),...
%                   omegaB',omegaVec']'
%
%              rI = 3x1 position vector in I in meters
%              vI = 3x1 velocity vector wrt I and in I, in meters/sec
%             RBI = 3x3 attitude matrix from I to B frame
%          omegaB = 3x1 angular rate vector of body wrt I, expressed in B
%                   in rad/sec
%        omegaVec = 4x1 vector of rotor angular rates, in rad/sec.
%                   omegaVec(i) is the angular rate of the ith rotor.
%
%    eaVec --- 4x1 vector of voltages applied to motors, in volts.  eaVec(i)
%              is the constant voltage setpoint for the ith rotor.
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
% Author:  
%+==============================================================================+

%% Validate inputs
global INPUT_PARSING;
if INPUT_PARSING
  issize =@(x,z1,z2) validateattributes(x,{'numeric'},{'size',[z1,z2]});
  ip = inputParser; ip.StructExpand = true; ip.KeepUnmatched = true;
  ip.addRequired('t',@(x)issize(x,1,1));
  ip.addRequired('X',@(x)issize(x,22,1));
  ip.addRequired('eaVec',@(x)issize(x,4,1));
  ip.addRequired('distVec',@(x)issize(x,3,1));
  ip.addParameter('quadParams',[],@(x)isstruct(x));
  ip.addParameter('constants',[],@(x)isstruct(x));
  ip.parse(t,X,eaVec,distVec,P);
end

%% Student code

%                       Insert your code here 


%constants
kF = P.quadParams.kF;
kN = P.quadParams.kN;
mass = P.quadParams.m;
g = P.constants.g;
cm = P.quadParams.cm; % 4x1 vector
tm = P.quadParams.taum; % 4x1 vector
Cd = P.quadParams.Cd;
Ad = P.quadParams.Ad;
rho = P.constants.rho;


% Extract values from state vector
position_cvector = X(1:3); % Column vector of position
velocity_cvector = X(4:6); % Column vector of velocity
RBI_MAT = reshape(X(7:15), 3, 3); % 3x3 DCM
omega_cvector = X(16:18); % Column vector of ang. velocity
omega_cross = crossProductEquivalent(omega_cvector);

rotor_rates = X(19:22); % 4x1 column vector

%Fi
% F = [0;0;0;];
% N = [0;0;0;];
% for i=1:4
%     Fi = [0;0;kF(i)*(rotor_rates(i)^2);];
%     Ni = [0;0;kN(i)*(rotor_rates(i)^2);];
%     F = F + Fi;
%     N = N + ((P.quadParams.omegaRdir(i))*(Ni + cross(P.quadParams.rotor_loc(:,i), Fi)));
% end

F = [zeros(2,4);(P.quadParams.kF.*(rotor_rates.^2))'];
N = [zeros(2,4);(P.quadParams.kN.*(rotor_rates.^2).*(P.quadParams.omegaRdir)')'];
NB = sum(N,2) + sum(cross(P.quadParams.rotor_loc,F,1),2);

% F = [0; 0; sum(kF.*rotor_rates.^2)];
% N = zeros(3, 1); % Initialize moment vector
% N_vector = kN.*(rotor_rates.^2);
% N = N + (P.quadParams.omegaRdir .* Ni + cross(P.quadParams.rotor_loc(:), F));

% Equations
der_position = velocity_cvector;
% add new drag term to velocity

if norm(velocity_cvector) > 0
    v_hat = velocity_cvector / norm(velocity_cvector);
else
    v_hat = zeros(3,1); % or handle however you prefer
end

zI = RBI_MAT(:,3);
da = (1/2) * Cd * Ad * rho * ((zI'*velocity_cvector) * abs(zI'*velocity_cvector));


der_velocity = -da*v_hat + [0;0;-mass*g] + (RBI_MAT'*sum(F,2)) + distVec/mass;
der_omega = inv(P.quadParams.Jq) * (NB - omega_cross*P.quadParams.Jq*omega_cvector);
der_RBI = -omega_cross*RBI_MAT;

rotor_rate_dot = (-rotor_rates./tm) + (cm./tm).*eaVec;

Xdot = [der_position',der_velocity',der_RBI(1,1),der_RBI(2,1),der_RBI(3,1),der_RBI(1,2),der_RBI(2,2),der_RBI(3,2),der_RBI(1,3),der_RBI(2,3),der_RBI(3,3),der_omega', rotor_rate_dot']';


end % EOF quadOdeFunctionHF.m