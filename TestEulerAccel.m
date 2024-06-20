clear all
Nsamples = 41500;
EulerSaved = zeros(Nsamples, 2);
for k = 1:Nsamples
  [ax ay] = GetAccel();   
  [phi theta] = EulerAccel(ax, ay); 
  
  EulerSaved(k, :) = [phi theta];
end
PhiSaved   = EulerSaved(:, 1) * 180/pi;
ThetaSaved = EulerSaved(:, 2) * 180/pi;
dt = 0.01;
t  = 0:dt:Nsamples*dt-dt;
figure
plot(t, PhiSaved)
figure
plot(t, ThetaSaved)

figure
subplot(1,3,1)
plot(t, PhiSaved)
title('Roll')
xlabel('time (s)'); ylabel('Roll, \phi (degrees)')
grid on
ylim(40*[-1 1]); xlim([0 420]);
set(gca,'fontsize',18)
subplot(1,3,2)
plot(t, ThetaSaved)
title('Pitch')
xlabel('time (s)'); ylabel('Pitch, \theta (degrees)')
grid on
ylim(40*[-1 1]); xlim([0 420]);
set(gca,'fontsize',18)
subplot(1,3,3)
