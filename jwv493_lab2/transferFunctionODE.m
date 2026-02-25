function [omegaDot] = transferFunctionODE(t, omega, voltage, cm, tm)
%TF Summary of this function goes here
%   Detailed explanation goes here
    

    % use this to call it [tVeck,XMatk] = ode45(@(t,X)quadOdeFunction(t,X,omegaVec,distVec,parameters), tspan, Xbig);

    omegaDot = (-omega/tm) + (cm/tm)*voltage;

end