%% Constants

v = 2; % m/s
Km = 0.0028; % Nm/A
N = 15.3;
bm = 0.00001; % Nms
r = 0.04; % m
R = 0.64; % ohms
rho = 1.204;
A = 0.01;
Cd = 0.5;
m = 5;
Jm = 0.00005;
L = 0.3;

% Error cost factors
yWeight = 1;
yMax = 1;
thetaWeight = 1;
thetaMax = 1;
speedWeight = 100;
speedMax = 1;

% Energy cost factors
deltaWeight = 10;
deltaMax = 1;
voltWeight = 1;
voltMax = 1;

% References auto-pilot
speedReference = 2;
yReference = 0;

% References parking
xReference = 2;
yReference = 2;

% Initial conditions for states (position y, angle theta, speed v)
x0 = [-1;
      -pi/4;
      0]; 