% Author: Jiyoung Hwang (hjy8918@yonsei.ac.kr)

% doubleIntegeral2D_main.m
close all;
clear all;
clc;



% System Parameter Limits
params.posMin = -1;     % minimum position
params.posMax = 1;      % maximum position
params.velMin = -1;     % minimum velocity
params.velMax = 1;      % maximum velocity
params.accMin = -5;     % minimum acceleration
params.accMax = 5;      % maximum acceleration

params.gamma = 1;       % CBF extended class K function gamma



% Define the initial conditions
x0 = [0.5; 0.5];        % [position; velocity]
u_des = -1;             % Desired input constant acceleration)    



% Simulation Parameters
dt = 0.001;
tEnd = 10;
runTime = [0 tEnd];



% Instantiate the DoubleIntegrator system and CBFController
controller = CBFdoubleIntegral2D(params, u_des);



% Define the ODE function for integration
odeFunc = @(t, x) controller.dynamics(x, t);



% Run the simulation using ode45
[t, x] = ode45(odeFunc, runTime, x0);



% Plot the results
figure;
subplot(2, 1, 1);
plot(t, x(:, 1));
grid on
xlabel('Time (s)');
ylabel('Position (m)');
title('Position vs Time');

subplot(2, 1, 2);
plot(t, x(:, 2));
grid on
xlabel('Time (s)'); 
ylabel('Velocity (m/s)');
title('Velocity vs Time');

pic = gcf;
exportgraphics(pic,'time_PosVel_doubleIntegral2D.jpg','Resolution',600);

figure;
hold on
% Indicate the safety threshold boundary (dotted line)
pD = params.posMax+params.velMax^2/(2*params.accMin);
pd = params.posMin+params.velMin^2/(2*params.accMax);
xBoundary = [params.posMin params.posMin pD];
yBoundary = [0 params.velMax params.velMax];
plot(xBoundary, yBoundary,'--k');
xBoundary = [pd params.posMax params.posMax];
yBoundary = [params.velMin params.velMin 0];
plot(xBoundary, yBoundary,'--k');
tBoundary = linspace(0, -params.velMax/params.accMin,100);
xBoundary = pD + params.velMax*tBoundary + 1/2*params.accMin*tBoundary.^2;
yBoundary = params.velMax + params.accMin*tBoundary;
plot(xBoundary, yBoundary,'--k');
tBoundary = linspace(0, -params.velMin/params.accMax,100);
xBoundary = pd + params.velMin*tBoundary + 1/2*params.accMax*tBoundary.^2;
yBoundary = params.velMin + params.accMax*tBoundary;
plot(xBoundary, yBoundary,'--k');

plot(x(:, 1), x(:, 2));
text(x(1,1),x(1,2),{'\leftarrow starting','     point'})

grid on
xlim(1.2*[params.posMin params.posMax]);
ylim(1.2*[params.velMin params.velMax]);

xlabel('Position (m)');
ylabel('Velocity (m/s)');
title('Phase Plot');

pic = gcf;
exportgraphics(pic,'pos_vel_doubleIntegral2D.jpg','Resolution',600);
