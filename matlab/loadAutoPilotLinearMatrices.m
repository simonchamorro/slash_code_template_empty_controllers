A = [0,         v,      0;
     0,         0,      0;
     0,         0       (Km^2*Nsim^2 - Nsim^2*bm*rsim*Rm + rsim^2*Rm*rho*Asim*Cd*v)/(Rm*rsim^2*m + Nsim^2*Jm)];
B = [0,         0;
     0,         v/L;
     Nsim*Km*rsim/(Rm*rsim^2*m + Nsim^2*Jm),         0];
C = [0, 0, 1;
     1, 0, 0];
D = [0, 0;
    0, 0];