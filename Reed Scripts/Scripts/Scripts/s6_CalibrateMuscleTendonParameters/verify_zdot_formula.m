%% verify zdot = vm * sec(pen)

clear
close all
clc

% params
l0 = 1.5;
pen0 = 20*pi/180;
h0 = l0 * sin(pen0);
v0 = 2;

% simulate fiber shortening from 1.8 to 0.4 normalized length in T seconds
% at a constant rate
ln = 1.8:-0.01:0.4;
lm = l0 * ln;
n = length(ln);
T = 1.5;
t = linspace(0,1.5,n);
dt = t(2)-t(1);

% simulate
pen = zeros(1,n);
z = zeros(1,n);
for k = 1:n
    z(k) = sqrt(lm(k)^2 - h0^2);
    pen(k) = atan(h0/z(k));
end

% get true muscle velocity
vm = fdiff5(lm,t);

% verify zdot = vm * sec(pen)
zdot = fdiff5(z,t);
zdot2 = vm .* sec(pen);

% another way
dln_dt = fdiff5(ln,t);
vn = dln_dt / v0;
zdot3 = v0 * l0 * vn .* sec(pen);

% plot
fig = figure;
fig.Position = [872 231 714 670];
subplot(3,1,1)
plot(t,ln)
xlabel('Time (s)')
ylabel('Norm. Fib. Len.')
subplot(3,1,2)
plot(t,pen*180/pi)
xlabel('Time (s)')
ylabel('Pennation (deg)')
subplot(3,1,3)
plot(t,zdot,'k','LineWidth',2)
hold on
plot(t,zdot2,'r--','LineWidth',1.5)
plot(t,zdot3,'g:','LineWidth',1)
leg = legend('zdot = dz/dt','zdot = vm * sec(pen)','zdot = v0 * l0 * vn * sec(pen)');
leg.Box = 'off';
leg.Location = 'southwest';
xlabel('Time (s)')
ylabel('Fib. Vel. Along Tendon (m/s)')


