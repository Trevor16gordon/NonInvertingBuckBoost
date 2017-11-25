%% Define the system

% Define Constants
Vin = 5; 
L = 220e-6;
C = 220e-6;
R = 510;


% Define Large Signal - Steady State
U = 0.5;
X1 = Vin*U/((U-1)*(U-1)*R);
X2 = Vin*U/(1-U);

A11 = 0;
A12 = -(1-U)/L;
A21 = (1-U)/C;
A22 = -1/(R*C);

A = [A11 A12; A21 A22];
B = [(Vin+X2)/L; -X1/C];
C = [0 1];
D = [0];

% Find system response
OSys = ss(A, B, C, D);
OPoles = pole(OSys);