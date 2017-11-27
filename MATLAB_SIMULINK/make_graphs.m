% Trevor Gordon
% Observor and Servomechanism
% clear;
% clc;

% Define the system
buck_boost_sys_define;



% Stabilize Servo to desP ITAE
desP_servo = getITAEPoles(.01, 10);
% Servomechanism Matrices
Asrv = [A zeros(2,1);-C 0];
Bsrv = [B; 0];
Csrv = [C  0];
det([A B ; -C 0])

%Sweep of poles and responses
[servosys_mult, servosys_multi_K, desP_servo_multi] = getManySys(1, 5, 6, A, B, C, D);
% [servosys_mult, servosys_multi_K] = getManySys(0.01, 0.1, 6, A, B, C, D);

% Closed Loop Tracking System
% desP_track  = getPoles(.01, 10);
desP_track = desP_servo_multi(1,2:3);
Kt = place(A,B,desP_track);
Gc = 1/(C*(B*Kt-A)^-1*B);
trackSys = ss(A - B*Kt,B*Gc,C,D);

% Place at desP
% Ksrv = place(Asrv,Bsrv,desP_servo);
Ksrv = servosys_multi_K(1,:);
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
Bobc = [0; 0; 1; 0; 0];
Cobc = [0, 1, 0, 0, 0];
Dobc = [0];
obcSys = ss(Aobc,Bobc,Cobc,Dobc);

% Finding response
x0 = [0 0]';
dt = 0.0001;
t = 0:dt:0.25;
r = 1*ones(1,length(t));
yo = lsim(OSys,r,t,x0);
yc_track = lsim(trackSys, r, t, x0);
yc_servo = lsim(servosys_mult(:,:,1,1),r,t,[x0;0]);
yc_servo_observ = lsim(obcSys, r, t, [x0;0;0;0]);

for i=1:1:5
   
   yc_servo_mult(:,i) = lsim(servosys_mult(:,:,i,1),r,t,[x0;0]);
end

%Open Loop Steady state has small input multiplied by gain
olssv = Vin*(U+r(1))/(1-(U+r(1))); %open_loop_steady_state_value 



% subplot(2,1, 2)
% plot(t,yc_servo+X2,'b','linewidth',2)
% hold on
% plot([t(1), t(end)], [X2+r(1), X2+r(1)], 'o--', 'linewidth', 4)
% legend('CLosed Loop Output', 'Linearized Voltage + r')
% grid on

figure
subplot (4,1,1)
hold on
plot(t,yo+X2, 'Color', [.9 .9 .9], 'linewidth',2)
title('Open Loop Response')
xlabel('Time(s)');
ylabel('Voltage(V)')
grid on
hold off

subplot(4,1,2)
grid on
% linespec = {'b.', 'r-', 'g--o'}; % define your ten linespecs in a cell array
hold on
for i=1:1:5
   plot(t,yc_servo_mult(:,i)+X2, 'linewidth', 3)
   legendInfo{i} = ['Ki = ' num2str(servosys_multi_K(i,3))];
end
legend(legendInfo);
title({'Comparison of Pole Placement','of a BuckBoost Control System and','Its Response to a Desired Step Change'})
xlabel('Time(s)');
ylabel('Voltage(V)')
hold off

subplot(4,1,3)
grid on
% linespec = {'b.', 'r-', 'g--o'}; % define your ten linespecs in a cell array
hold on
for i=1:1:5
   plot(desP_servo_multi(i,:), 'O','linewidth', 3)
end
title('Poles')
xlabel('Real');
ylabel('Imaginary')
hold off

subplot (4,1,4)
hold on
plot(t,yc_track+X2, 'Color', [0.3010,0.7450,0.9330], 'linewidth',2)
plot(t,yc_servo+X2,'Color', [ 0,0.4470, 0.7410],'linewidth',2)
title('Comparison of a Tracking and a ServoMechanism Control System')
legend('Track', 'Servo')
xlabel('Time(s)');
ylabel('Voltage(V)')
grid on
hold off

% subplot (4,1,4)
% hold on
% plot(t,yc_servo+X2,'Color', [ 0,0.4470, 0.7410],'linewidth',2)
% plot(t,yc_servo_observ+X2,'b--','linewidth',2)
% hold off
% grid on
% xlabel('time (sec)')
% legend('Output Voltage Open Loop', 'Output Voltage with Disturbance Rejection', 'Output Voltage with Disturbance Rejection and OBbservor')
% title({'Control System for a Buckboost Convertor';'Comparison of Open Loop With Servo Compensator and Observer based Servo Compensator'})

% To be run after the simulink saves ScopeData
figure
ax1 = subplot(4,1,[1,2,3]);
hold on
simulink_length = length(ScopeData.signals(1).values(:,2));
dt2 = .001;
time_array = 0:dt2:simulink_length*dt2-dt2;
plot(time_array,ScopeData.signals(1).values(:,2), 'Color', [0.3010,0.7450,0.9330],'linewidth', 3)
plot(time_array, ScopeData.signals(1).values(:,3), 'Color', [0,0.4470, 0.7410], 'linewidth', 3)
plot(time_array, ScopeData.signals(1).values(:,1), '--','linewidth', 2)
legend('Tracking', 'Servo', 'Desired Output')
title('Comparison of a Tracking and Servo Control System and their ability to reject disturbances')


ylabel('Voltage (V)')
grid on
hold off
ax2 = subplot(4,1,4);
plot(time_array, ScopeData.signals(2).values(:,1), '--', 'Color', [0.6350,0.0780,0.1840],'linewidth', 2)
legend('0.1% Disturbance in u')
xlabel('Time (s)')
ylabel('Disturbance in U')
linkaxes([ax1, ax2], 'x');

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

function [servo_sys, Ksrv, desP_servo] = getManySys(start, step, num, A, B, C, D)
% AZ = [0,0,0;0,0,0;0,0,0];
% BZ = [0;0;1];
% CZ = [0,0,1];
% DZ = [0];
% servo_sys = repelem(ss(AZ,BZ,CZ,DZ), num);
j = 1;

% Servomechanism Matrices
Asrv = [A zeros(2,1);-C 0];
Bsrv = [B; 0];
Csrv = [C  0];
det([A B ; -C 0])

for i=start:step:start+step*num

% Stabilize Servo to desP ITAE
% desP_servo = getITAEPoles(.01, i)

% desP_servo(j,:) = [-2-20*i, -5*i + 20i*i, -5*i - 20i*i]
desP_servo(j,:) = [-250, -30 + (30i+10i*i), -30 - (30i+10i*i)]


% Place at desP
Ksrv(j,:) = place(Asrv,Bsrv,desP_servo(j,:));
K = Ksrv(j,1:2);
ki = - Ksrv(j,3); 

% Closed loop Servo System
Ac = [A-B*K B*ki;-C 0];
Bc = [0 ; 0; 1];
Cc = Csrv;
servo_sys(:,:,j) = ss(Ac,Bc,Cc,D);
j = j+1;
end
end

