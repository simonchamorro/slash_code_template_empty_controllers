
%% 1.4.1 - Phase space 
    clear all, clc, close all    
    v=1;
    l=0.4;
    k1 = -0.10;
    k2 = -0.0;
    k3 = -0.10;

    [theta,y] = meshgrid(linspace(-pi,pi,10),linspace(-3,3,10));
    y_prime   = sin(theta)*v;

    delta     = k1*y + k2*y_prime + k3*theta;
    theta_prime = tan(delta)*v/l;

    figure
    quiver(y,theta,y_prime,theta_prime)
    title('Espace de phase')
    xlabel('y (m)')
    ylabel('theta (rad)')
    
%% 1.4.2 - LQR Conception (Auto-Pilot)
    clear all, clc, close all    

% 1) Load constants & matrices (A, B, C, D, Q, N)
    loadconstants
    loadAutoPilotLinearMatrices 
    loadAutoPilotCostMatrices
    
% 2) Compute K matrix and N matrix with LQR technique
    [KautoPilot,~,~] = lqr(A, B, Q, R, N);
    N =  -inv(C*inv(A-(B*KautoPilot))*B);  
    
% 3) Build closed-loop system with K controller and N vector
    autopilotClosedLoopSystem = ss(A-B*KautoPilot, B*N, C, D, 'StateName',{'Position (y)' 'Angle (theta)' 'Speed'}, 'InputName',{'Input 1' 'Input2'} , 'OutputName',{'Velocity' 'Position'}); 

% 4) Analyze LQR closed-loop system
    t = 0:0.01:10;
    r = [yReference*ones(length(t), 1), speedReference*ones(length(t), 1)];
    [y, t, x] = lsim(autopilotClosedLoopSystem, r, t, x0);
    figure()
    subplot(3, 1, 1)
    plot(x(:,1))
    title('Position (y)')
    subplot(3, 1, 2)
    plot(x(:,2))
    title('Theta')
    subplot(3, 1, 3)
    plot(x(:,3))
    title('Speed')

% Closed-Loop System Pole and Zeros
    figure()
    pzmap(CLsys)
    
%% 1.4.3 - Parking 
    clear, clc, close all    

% 1) Load constants & matrices (A, B, C, D, Q, N)
    loadconstants
    loadParkingLinearMatrices 
    
% 2) Compute K matrix and N matrix with poles placement technique 
    p = [-1+0.5i, -1-0.5i, -1];
    Kparking = place(A, B, p);
    Nparking =  -inv(C*inv(A-(B*Kparking))*B);  
    
% 3) Build closed-loop system with K controller and N vector
    parkingClosedLoopSystem = ss(A-B*Kparking, B*Nparking, C, D, 'StateName',{'Position (y)' 'Angle (theta)' 'Speed'}, 'InputName',{'Input 1' 'Input2'} , 'OutputName',{'Velocity' 'Position'}); 

% 4) Analyze LQR closed-loop system
    t = 0:0.01:10;
    r = [xReference*ones(length(t), 1), yReference*ones(length(t), 1)];
    [y, t, x] = lsim(parkingClosedLoopSystem, r, t, x0);
    figure()
    subplot(3, 1, 1)
    plot(x(:,1))
    title('Position (x)')
    subplot(3, 1, 2)
    plot(x(:,2))
    title('Position (y)')
    subplot(3, 1, 3)
    plot(x(:,3))
    title('Theta')

% Closed-Loop System Pole and Zeros
    figure()
    pzmap(parkingClosedLoopSystem)
    
 