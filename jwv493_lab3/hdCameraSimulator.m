function [rx] = hdCameraSimulator(rXI,S,P)
% hdCameraSimulator : Simulates feature location measurements from the
%                     quad's high-definition camera. 
%
%
% INPUTS
%
% rXI -------- 3x1 location of a feature point expressed in I in meters.
%
% S ---------- Structure with the following elements:
%
%        statek = State of the quad at tk, expressed as a structure with the
%                 following elements:
%                   
%                  rI = 3x1 position of CM in the I frame, in meters
% 
%                 RBI = 3x3 direction cosine matrix indicating the
%                       attitude of B frame wrt I frame
%
% P ---------- Structure with the following elements:
%
%  sensorParams = Structure containing all relevant parameters for the
%                 quad's sensors, as defined in sensorParamsScript.m 
%
% OUTPUTS
%
% rx --------- 2x1 measured position of the feature point projection on the
%              camera's image plane, in pixels.  If the feature point is not
%              visible to the camera (the ray from the feature to the camera
%              center never intersects the image plane, or the feature is
%              behind the camera), then rx is an empty matrix.
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
  ip.addRequired('rXI',@(x)issize(x,3,1));
  ip.addParameter('rI',[],@(x)issize(x,3,1))
  ip.addParameter('RBI',[],@(x)issize(x,3,3))
  ip.addParameter('sensorParams',[],@(x)isstruct(x));
  ip.parse(rXI,S.statek,P);
end

%% Student code

% ????  Insert your code here 


%rXI is given
RCI = P.sensorParams.RCB * S.statek.RBI;
RBI = S.statek.RBI;
% May not need to translate into Camera frame?
% rXC = P.sensorParams.RCB * (S.RBI * (rXI - S.rI) -
% P.sensorParams.rocB);
rXI_h = [rXI' 1]';
t = S.statek.rI + RBI'*P.sensorParams.rocB;
tC = RCI * t; % Main.pdf Page 56

% Calculate x_h

% [RCI, -tC]
% P_ext = [P.sensorParams.K [0;0;0;]] * [RCI -tC; 0, 0, 0, 1];
% x_h = P_ext * rXI_h;
x_h = P.sensorParams.K * [RCI, -tC] * rXI_h;

if x_h(3) <= 0 % Visibility Check
    rx = [];
    return;
end

x = [x_h(1)/x_h(3), x_h(2)/x_h(3)]';


w_c = mvnrnd(zeros(2,1), P.sensorParams.Rc)';



%output
rx = (1 / P.sensorParams.pixelSize) * (x);

if abs(rx(1)) > P.sensorParams.imagePlaneSize(1)
    rx = [];
    return;
end
if(abs(rx(2)) > P.sensorParams.imagePlaneSize(2))
    rx = [];
    return;
end

rx = rx + w_c;


end 