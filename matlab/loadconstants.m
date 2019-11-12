%% Constants

v = 2; % m/s
Km = 0.0028; % Nm/A
Nsim = 15.3;
bm = 0.00001; % Nms
rsim = 0.04; % m
Rm = 0.64; % ohms
rho = 1.204;
Asim = 0.01;
Cd = 0.5;
m = 5;
Jm = 0.00005;
L = 0.3;
Ts = 0.01;
Cr = 0.01;
Sigma = 0.5;
g = 9.81;
Lm = 0.0001;


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
yReferenceAuto = 0;

% References parking
xReference = 2;
yReference = 0;

% Initial conditions for states (position y, angle theta, speed v)
x0 = [0;
      0;
      0]; 