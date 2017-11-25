% Trevor Gordon
% Observor and Servomechanism
clear;
clc;

% Define the system
buck_boost_sys_define;

% Closed Loop Tracking System
desP_track  = getPoles(.01, 10);
Kt = place(A,B,desP_track);
Gc = 1/(C*(B*Kt-A)^-1*B);
trackSys = ss(A - B*Kt,B*Gc,C,D);

% Stabilize Servo to desP ITAE
desP_servo = getITAEPoles(.01, 10);
% Servomechanism Matrices
Asrv = [A zeros(2,1);-C 0];
Bsrv = [B; 0];
Csrv = [C  0];
det([A B ; -C 0])
% Place at desP
Ksrv = place(Asrv,Bsrv,desP_servo);
K = Ksrv(1:2);
ki = - Ksrv(3); 

% Closed loop Servo System
Ac = [A-B*K B*ki;-C 0];
Bc = [0 ; 0; 1];
Cc = Csrv;
servoSys = ss(Ac,Bc,Cc,D);

% Observer System
desObsPoles = getPoles(0.1, 10);
L = place(A',C',desObsPoles);  %NOTE: use the "transpose" of A and C 
L=L';
AObs =  A-L*C;
BObs = [B , L];
CObs = eye(2); % number of states
DObs = zeros(2,2); % number of states,number of inputs
ObsSys = ss(AObs,BObs,CObs,DObs);
x0_hat  = [0, 0]';

% CLose loop With Observor and Servo Compensator
% States = [xdot;zetadot; xhatdot]
Aobc = [A, B*ki, -B*K; -C, 0, 0, 0;L*C, B*ki, A-B*K-L*C];
Bobc = [0; 0; 1; 0; 0]+[0; 0; 1; 0; 0];
Cobc = [0, 1, 0, 0, 0];
Dobc = [0];
obcSys = ss(Aobc,Bobc,Cobc,Dobc);

% Finding response
x0 = [0 0]';
dt = 0.0001;
t = 0:dt:0.5;
r = 0.1*ones(1,length(t));
yo = lsim(OSys,r,t,x0);
yc_track = lsim(trackSys, r, t, x0);
yc_servo = lsim(servoSys,r,t,[x0;r(1)-x0(2)]);
yc_servo_observ = lsim(obcSys, r, t, [x0;r(1)-x0(2);0;0]);

%Open Loop Steady state has small input multiplied by gain
olssv = Vin*(U+r(1))/(1-(U+r(1))); %open_loop_steady_state_value 

% plot
figure
subplot(2, 1, 1);
hold on
plot(t,yo+X2,'r','linewidth',2)
plot(t,yc_servo+X2,'b','linewidth',2)
plot(t,yc_servo_observ+X2,'g--','linewidth',2)
hold off
grid on
xlabel('time (sec)')
legend('Output Voltage Open Loop', 'Output Voltage with Disturbance Rejection', 'Output Voltage with Disturbance Rejection and OBbservor')
title({'Control System for a Buckboost Convertor';'Comparison of Open Loop With Servo Compensator and Observer based Servo Compensator'})
subplot(2,1, 2)
plot(t,yc_servo+X2,'b','linewidth',2)
hold on
plot([t(1), t(end)], [X2+r(1), X2+r(1)], 'o--', 'linewidth', 4)
legend('CLosed Loop Output', 'Linearized Voltage + r')
grid on


function desP = getITAEPoles(ts, OS)

zeta = -log(OS/100)/sqrt(pi^2+log(OS/100));
wn = 4/ts/zeta;
% Calculate desired poles
desP  = roots([1 1.75*wn 2.15*wn^2 wn^3]);

end

function desP = getPoles(ts, OS)

zeta = -log(OS/100)/sqrt(pi^2+log(OS/100));
wn = 4/ts/zeta;
sigma   = -wn*zeta;
wd      =  wn*sqrt(1-zeta^2);
% Calculate desired poles
desP  = [sigma-wd ; sigma+wd];

end

