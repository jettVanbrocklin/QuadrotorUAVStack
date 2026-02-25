function [R_BW] = euler2dcm(e)
% euler2dcm : Converts Euler angles phi = e(1), theta = e(2), and psi = e(3)
%             (in radians) into a direction cosine matrix for a 3-1-2 rotation.
%
% Let the world (W) and body (B) reference frames be initially aligned.  In a
% 3-1-2 order, rotate B away from W by angles psi (yaw, about the body Z
% axis), phi (roll, about the body X axis), and theta (pitch, about the body Y
% axis).  R_BW can then be used to cast a vector expressed in W coordinates as
% a vector in B coordinates: vB = R_BW * vW
%
% INPUTS
%
% e ---------- 3-by-1 vector containing the Euler angles in radians: phi =
%              e(1), theta = e(2), and psi = e(3)
%
%
% OUTPUTS
%
% R_BW ------- 3-by-3 direction cosine matrix 
% 
%+------------------------------------------------------------------------------+
% References: main.pdf, Lecture 2 notes
%
%
% Author: Jett Vanbrocklin
%+==============================================================================+  
  
%% Validate inputs
global INPUT_PARSING;
if INPUT_PARSING
  issize =@(x,z1,z2) validateattributes(x,{'numeric'},{'size',[z1,z2]});
  ip = inputParser; ip.StructExpand = true; ip.KeepUnmatched = true;
  ip.addRequired('e',@(x)issize(x,3,1));
  ip.parse(e);
end

%% Student code

%                       Insert your code here 

% I need to do the 3 - 1 - 2 sequence. Check for singularities. If
% singularities, throw an error/warning. 
% Check your Lecture 2 notes for information.

% Check/convert angles into proper ranges
% phi (x-axis): [-pi/2, pi/2]
e(1) = max(min(e(1),  pi/2), -pi/2);

% theta (y-axis): (-pi, pi]
e(2) = atan2(sin(e(2)), cos(e(2)));

% psi (z-axis): (-pi, pi]
e(3) = atan2(sin(e(3)), cos(e(3)));


% Check for singularities in the pitch angle (phi)
if abs(e(1)) == pi/2 % phi >= 
    error('Singularity detected: Phi >= +/- pi/2');
end


ev1 = [1;0;0;];
ev2 = [0;1;0;];
ev3 = [0;0;1;];

R1 = rotationMatrix(ev1, e(1));   % Use Ahat and e(1)
R2 = rotationMatrix(ev2, e(2));   % Use Ahat and e(2)
R3 = rotationMatrix(ev3, e(3));   % Use Ahat and e(3)

R_BW = R2*R1*R3;

end % EOF euler2dcm.m








