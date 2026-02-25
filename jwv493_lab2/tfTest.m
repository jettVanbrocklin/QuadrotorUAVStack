
% Script used to test "transferFunctionODE.m"


% Define the time span and initial conditions
tspan = [0 0.3]; % Adjust as necessary
voltage = 1; % Commanded Velocity
omega0 = 0; % Set initial value for omega

[t, omegaOut] = ode45(@(t,omega)transferFunctionODE(t,omega, voltage, quadParams.cm(1), quadParams.taum(1)), tspan, omega0);

plot(t, omegaOut)
xlabel('Time (s)')
ylabel('\omega (rad/s)')
title('Motor Step Response')
grid on