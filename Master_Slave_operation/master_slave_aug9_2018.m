%Master-Slave operation 
%Aug 09, 2018

clear all;
close all;
clc;

%Given parameters:

%1.1 Displacement as observed on the x-axis(zeta*wn)
d = 0.3;

%2.1 Damping ratio of the master robot - zeta
%(Zeta will be varied to observe the response of the system.)
zeta = 0.9;


%3.1 Natural frequency of the master robot(rad/sec)
wn = d/zeta;


%System parameters

%1.1 Environmental stiffness (N/m)
%(KE by calculation is 100N/m - does not include back emf and no weight of the omni arm)
KE = 100.0;

%2.1 Mass calculation(kilograms)
M = KE/(wn * wn)

%3.1 Damping for the robot(Ns/m)
B = 2 * zeta * wn * M

%4.1 Tau - time constant
tau = M/B

%5.1 Constant 'C'
C = 1/B

%Pade approximation for time delay

%1.1 Order of Approximation
N = 1;

%2.1 Time delay(sec)
T = 0.0001;   

%Discretization

%1.1 Sampling time(sec)
Ts = 0.2;

s = tf('s');
z = tf('z');

%A. Master system

num1 = [C];
den1 = [tau 1 0];
sys1 = tf(num1,den1)

%B. Slave system

num2 = [wn^2];
den2 = [1 (2*zeta*wn) (wn^2)];
sys2 = tf(num2,den2)
sys2 = sys2 * (1/KE)

%C. Time delay model

[num3,den3] = pade(T,N);
sys3 = tf(num3,den3)

%A2 - page 1
%Develop closed loop

%Proportional Control
Kp = 0.013;

sys20 = series(Kp,sys2);

sys20 = feedback(sys20,1)

figure(1);
rlocus(sys20);
axis equal;
grid on;
title('Proportional control for the slave system');

figure(2);
step(sys20);
grid on;
title('Step response of Proportional control for the slave system');

%A. No delay,No dynamics

sys21 = feedback(sys1,KE)

unit_step = tf([1],[1 0])

sys21_step = series(sys21, unit_step)

[b1,a1] = tfdata(sys21_step,'v')

[r1,p1,k1] = residue(b1,a1)

figure(3);
rlocus(sys21);
axis equal;
grid on;
title('Closed loop system with no delay and no dynamics');

figure(4);
step(sys21);
grid on;
title('Step response of Closed loop system with no delay and no dynamics');

%B. No delay, with dynamics

sys22cl = series(sys1,sys2);

sys22cl = feedback(sys22cl,KE)

sys22_step = series(sys22cl,unit_step)

[b2,a2] = tfdata(sys22_step,'v')

[r2,p2,k2] = residue(b2,a2)

figure(5);
rlocus(sys22cl);
grid on;
axis equal;
title('Closed loop system no delay, with dynamics');

figure(6);
step(sys22cl);
grid on;
title('Closed loop system no delay, with dynamics');

%C.With delay and dynamics

sys23cl = series(sys1,sys2);

sys23cl = series(sys23cl,sys3);

sys24 = series(sys3,KE);

sys24cl = feedback(sys23cl,sys24)

sys24_step = series(sys24cl,unit_step)

[b3,a3] = tfdata(sys24_step,'v')

[r3,p3,k3] = residue(b3,a3)

figure(7);
rlocus(sys24cl);
axis equal;
grid on;
title('Closed loop system with delay, with dynamics');

figure(8);
step(sys24cl);
grid on;
title('Closed loop system with delay, with dynamics');

%D. Add a zero - designing a PD compensator
%Ziegler Nichols Method

Kpc = 95;

Kdc = 410;

numc = [Kdc Kpc];

denc = [1];

sysc = tf(numc,denc)

sys25 = series(sys1,sys2);

sys25 = series(sys25,sys3);

sys26 = series(sys3,KE);

sys26 = series(sys26,sysc);

sys_filter = feedback(sys25,sys26)

sysfilter_step = series(sys_filter,unit_step)

[b4,a4] = tfdata(sysfilter_step,'v')

[r4,p4,k4] = residue(b4,a4)

figure(9);
rlocus(sys_filter);
axis equal;
grid on;
title('Root locus of the closed loop system(with delay and dynamics) with PD controller : Kp = 95 and Kd = 410');

figure(10);
step(sys_filter);
grid on;
title('Step response closed loop system(with delay and dynamics) of the system with PD controller : Kp = 95 and Kd = 410');

[C_pid,info] = pidtune(sys24cl,'PIDF')

sys251 = series(sys1,sys2);

sys251 = series(sys251,sys3);

sys261 = series(sys3,KE);

sys261 = series(sys261,C_pid);

sys_filter1 = feedback(sys251,sys261)

% figure(11);
% rlocus(sys_filter1);
% axis equal;
% grid on;
% title('Root locus of the system with PID tune controller');
% 
% figure(12);
% step(sys_filter1);
% grid on;
% title('Step response of the system with PID tune controller');

%Lead compensator design

dp0 = -60;
dp1 = 60 * j;
dp = [(dp0 + dp1);(dp0 - dp1)];

ang1 = evalfr(sys24cl,dp(1));
ang2 = angle(ang1);
ang2 = rad2deg(abs(ang2));

phi = 180 - ang2;

%Placing the zero of the compensator exactly below the desired poles

ac1 = abs(real(dp(1)));

x = abs(ac1/cosd(phi));

bc1 = x * sind(phi) + ac1; 

%Gain of the compensator

Kd = bc1/ac1

%Lead filter system 

sysc2 = Kd * (tf([1 ac1],[1 bc1]))

sys27 = series(sys1,sys2);

sys27 = series(sys27,sys3);

sys28 = series(sys3,KE);

sys28 = series(sys28,sysc2);

sys_filter2 = feedback(sys27,sys28)

figure(13);
rlocus(sys_filter2);
axis equal;
grid on;
title('Root locus of the closed loop system(with delay and dynamics) with Lead controller');

figure(14);
step(sys_filter2);
grid on;
title('Step response closed loop system(with delay and dynamics) of the system with Lead controller');