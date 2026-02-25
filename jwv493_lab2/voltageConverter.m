function [eak] = voltageConverter(Fk,NBk,P)
% voltageConverter : Generates output voltages appropriate for desired
%                    torque and thrust.
%
%
% INPUTS
%
% Fk --------- Commanded total thrust at time tk, in Newtons.
%
% NBk -------- Commanded 3x1 torque expressed in the body frame at time tk, in
%              N-m.
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
% eak -------- Commanded 4x1 voltage vector to be applied at time tk, in
%              volts. eak(i) is the voltage for the ith motor.
%
%+------------------------------------------------------------------------------+
% References: Lab 2 Document, Main.pdf
%
%
% Author:  Jett Vanbrocklin
%+==============================================================================+  

%% Validate inputs
global INPUT_PARSING;
if INPUT_PARSING
  issize =@(x,z1,z2) validateattributes(x,{'numeric'},{'size',[z1,z2]});
  ip = inputParser; ip.StructExpand = true; ip.KeepUnmatched = true;
  ip.addRequired('Fk',@(x)issize(x,1,1));
  ip.addRequired('NBk',@(x)issize(x,3,1));
  ip.addParameter('quadParams',[],@(x)isstruct(x));
  ip.addParameter('constants',[],@(x)isstruct(x));
  ip.parse(Fk,NBk,P);
end

%% Student code

%                       Insert your code here 


% Set Up Constants
s1 = P.quadParams.omegaRdir(1);
s2 = P.quadParams.omegaRdir(2);
s3 = P.quadParams.omegaRdir(3);
s4 = P.quadParams.omegaRdir(4);
x1 = P.quadParams.rotor_loc(1,1);
y1 = P.quadParams.rotor_loc(2,1);
x2 = P.quadParams.rotor_loc(1,2);
y2 = P.quadParams.rotor_loc(2,2);
x3 = P.quadParams.rotor_loc(1,3);
y3 = P.quadParams.rotor_loc(2,3);
x4 = P.quadParams.rotor_loc(1,4);
y4 = P.quadParams.rotor_loc(2,4);

kF = P.quadParams.kF; % 4x1 Vector
kN = P.quadParams.kN;
kT = kN ./ kF;
cm = P.quadParams.cm; % 4x1 Vector
eamax = P.quadParams.eamax;

F_max = kF .* (cm .* eamax).^2;

conversion_matrix = [ 1 1 1 1;
                      y1 y2 y3 y4;
                      -x1 -x2 -x3 -x4;
                      s1*kT(1) s2*kT(2) s3*kT(3) s4*kT(4)];

inv_conv_matrix = inv(conversion_matrix);

% Begin Calculation for Desired Force
beta = 0.9;
alpha = 1;

is_calculated = 0;

while is_calculated == 0

    F_N_vec = [min(Fk,4*beta*F_max(1)); alpha*NBk(1); alpha*NBk(2); alpha*NBk(3)];
    Force_Vector = inv_conv_matrix * F_N_vec;

    if any(Force_Vector > F_max)
        alpha = alpha - 0.01;
    else
        is_calculated = 1;
    end
   
end

Force_Vector = max(Force_Vector, 0);

% I need to convert force to voltage
% force is dependent on rotor speed and force constant
% rotor speed is dependent on voltage

angular_rate = sqrt(Force_Vector ./ kF); % From Fi = kF * wi^2

% Using equation that I used to determine voltage needed for hover
% ea = angular_rate/cm
eak = angular_rate ./ cm;


end % EOF voltageConverter.m