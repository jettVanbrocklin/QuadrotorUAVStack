% Top-level script for calling simulateQuadrotorDynamics
clear all; clc;
global INPUT_PARSING;
INPUT_PARSING = false;
% Total simulation time, in seconds
Tsim = 10;
% Update interval, in seconds.  This value should be small relative to the
% shortest time constant of your system.
delt = 0.005;
% Time vector, in seconds 
N = floor(Tsim/delt);
R.tVec = [0:N-1]'*delt;

% ----CIRCLE----%
omega = 2*(pi)/Tsim;
radius = 2;
height = 1;

x = radius*cos(omega*R.tVec);
y = radius*sin(omega*R.tVec);
z = height*ones(size(R.tVec));


R.rIstar = [x y z];

x_hat = x / norm(x);
y_hat = y / norm(y);

R.xIstar = [-x_hat -y_hat zeros(size(x))];


vx = -radius*omega*sin(omega*R.tVec);
vy =  radius*omega*cos(omega*R.tVec);
vz = zeros(size(R.tVec));

R.vIstar = [vx vy vz];

ax = -radius*omega^(2)*cos(omega*R.tVec);
ay =  -radius*omega^(2)*sin(omega*R.tVec);
az = zeros(size(R.tVec));

R.aIstar = [ax ay az];
% -------------------%

% x = 0.5 * ones(N,1);
% y = 0.5 * ones(N,1);
% z = 0.5 * ones(N,1);
% R.rIstar = [x y z];
% 
% R.vIstar = zeros(N, 3);
% R.aIstar = zeros(N,3);
% 
% R.xIstar =repmat([1 0 0], N, 1);


% Matrix of disturbance forces acting on the body, in Newtons, expressed in I
S.distMat = zeros(N-1,3);

%-HF DOES NOT TAKE IN OMEGAMAT-%
% ------------%
% Rotor speeds at each time, in rad/s
%S.omegaMat = 585*ones(N-1,4);
% ------------

S.eaMat = 2.9283 * ones(N-1,4);

% Initial position in m
S.state0.r = [2 0 1]';
% Initial attitude expressed as Euler angles, in radians
S.state0.e = [0 0 pi]';
% Initial velocity of body with respect to I, expressed in I, in m/s
S.state0.v = [0 0 0]';
% Initial angular rate of body with respect to I, expressed in B, in rad/s
S.state0.omegaB = [0 0 0]';
% Oversampling factor
S.oversampFact = 10;
% Quadrotor parameters and constants
quadParamsScript;
constantsScript;
params.quadParams = quadParams;
params.constants = constants;

%load Stest;
P = simulateQuadrotorControl(R,S,params);

S2.tVec = P.tVec;
S2.rMat = P.state.rMat;
S2.eMat = P.state.eMat;
S2.plotFrequency = 20;
S2.makeGifFlag = false;
S2.gifFileName = 'testGif.gif';
S2.bounds=3*[-1 1 -1 1 -1 1];
visualizeQuad(S2);

figure(1);clf;
plot(P.tVec,P.state.rMat(:,3)); grid on;
xlabel('Time (sec)');
ylabel('Vertical (m)');
title('Vertical position of CM'); 

figure(2);clf;
plot(P.state.rMat(:,1), P.state.rMat(:,2)); 
axis equal; grid on;
xlabel('X (m)');
ylabel('Y (m)');
title('Horizontal position of CM');