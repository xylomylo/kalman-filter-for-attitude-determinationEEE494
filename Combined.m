clear all
addpath Matlab/
Nsamples = 41500; % Number of data points
EulerSaved = zeros(Nsamples, 3);
dt = 0.01; % seconds of time-step
psi = 0 ; % initial yaw guess
theta=0 ; %pitch
phi = 0 ; %roll
for k=1:Nsamples
  [w1 w2 w3] = GetGyro();  % load gyro data w1, w2, w3
  A = eye(4) + dt*1/2*[ 0   -w1  -w2  -w3 ;
                        w1   0    w3  -w2 ;
                        w2  -w3   0    w1 ;
                        w3   w2  -w1   0  ];
  [f1 f2] = GetAccel(); % load accelerometer data                 
  [phi theta] = EulerAccel(f1, f2); % radians
  z = Euler3212EP([ psi theta phi ]'); %z is the euler params
  [psi theta phi] = EulerKalman(A, z); % radians 
  EulerSaved(k, :) = [ psi theta phi ];
end 
PsiSaved   = EulerSaved(:, 1) * 180/pi;
ThetaSaved = EulerSaved(:, 2) * 180/pi;
PhiSaved   = EulerSaved(:, 3) * 180/pi;
t = 0:dt:Nsamples*dt-dt;
function [psi theta phi] = EulerKalman(A, z)
persistent H Q R
persistent x P
persistent firstRun
if isempty(firstRun)
  H = eye(4);
  Q = 0.0001*eye(4); 
  R = 10*eye(4);
  x = [1 0 0 0]';  
  P = 1*eye(4);
  firstRun = 1;  
end
xp = A*x;
Pp = A*P*A' + Q;
K = Pp*H'*inv(H*Pp*H' + R);
x = xp + K*(z - H*xp);   
P = Pp - K*H*Pp;
e=EP2Euler321(x); % Euler param. to yaw-pitch-roll
psi  =e(1); % yaw [rad]
theta=e(2); % pitch [rad]
phi  =e(3); % roll [rad]
end
function [phi theta] = EulerAccel(ax, ay)
g = 9.8;
theta = asin(ax / g );
phi   = asin(-ay / (g*cos(theta)));
end
function [ax ay az] = GetAccel()
persistent fx fy fz
persistent k firstRun
if isempty(firstRun)
  load ArsAccel
  k = 1;
  firstRun = 1;
end
ax = fx(k);
ay = fy(k);
az = fz(k);
k = k + 1;
end
function [p q r] = GetGyro()
persistent wx wy wz
persistent k firstRun
if isempty(firstRun) 
  load ArsGyro
  k = 1;
  firstRun = 1;
end
p = wx(k);
q = wy(k);
r = wz(k);
k = k + 1;
end