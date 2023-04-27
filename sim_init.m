% car sim setup

% steering model
ws = 10;
zetas = 0.7;

% throttle model

wt = 1;
zetat = 0.7;

%sim setup
Iw = 3;
Ic = 30;
m = 80;
r = 3;
R = 15;
wheel_damp = 1;
turn_damp = 1;

wdot0 = 2; %have to make a constant to get reference model
psi0 = 2;  %have to make a constant to get reference model

%base model

Ap = [0 1 0; 0 -wheel_damp 0; 0 0 -turn_damp];
Bp = [0 0; r*Iw/(m*r) 0; Iw*R*psi0/Ic Iw*R*wdot0/Ic];
Cp =  [0 1 0;0 0 1]; 
Dp = [1 0; 0 1];
lambda = [1 0; 0 1];

%extended model

A = [zeros(2,2) Cp;...
     zeros(3,2) Ap];
B = [Dp;...
     Bp];
C = [zeros(2,2) Cp];
D=Dp;
Q = diag([100 10 1 1 10]);
R = diag([1 1]);
Kxt = lqr(A,B,Q,R);
Aref = A-B*Kxt;
Cref = C-D*Kxt;
Bref = [-eye(2);zeros(3,2)];
Qref = diag([10000 1 0 1000 100]);
Pref = lyap(Aref',Qref);
run('carSim')
close all;


