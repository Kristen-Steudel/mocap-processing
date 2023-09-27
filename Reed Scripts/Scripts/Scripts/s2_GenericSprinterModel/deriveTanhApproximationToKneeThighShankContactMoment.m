%% sprinter passive torque due to calf-thigh contact

% finds a sigmoid (tanh) approximation to Zelle et al. 2009 data for use in
% opensim as an ExpressionBasedCoordinateForce. See zelle et al. 2007 
% thigh-calf contact force measurements in deep flexion, figure 4

clear
close all
clc

format long

% zelle et al. 2009 used their 07 study to model torques due to calf-thigh
% contact, their approach:
Ftc = @(a)4.7133 * 1e-2 * a.^2 - 12.256 * a + 798.68; % eq. 1
Ltc = @(a)6.9596 * 1e-6 * a.^3 - 2.8729 * 1e-3 * a.^2 + 0.39584 * a - 18.088; % eq. 2
Tzelle = @(a)Ftc(a).*Ltc(a);

% fit to zelle data for flexion angles between 140 and 165, set to zero for
% angles between 5 and 120
a_zelle = 140:0.5:165;
a_zero = 5:90;
a = [a_zero, a_zelle];

% get torques corresponding to those angles for fitting sigmoid too
t_zelle = Tzelle(a_zelle);
t_zero = zeros(1,length(a_zero));
t = [t_zero,t_zelle];

% scatter
scatter(a,-t)

% fit tanh params
weights = [1000*ones(1,length(t_zero)),1*ones(1,length(t_zelle))];
fun = @(p)(p(1) * (tanh(p(2) * a - p(3)) + p(4)) - t) .* weights;
options = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt','Display','off','OptimalityTolerance',1e-9);
p = lsqnonlin(fun,[25,0.07,11.935,1],[],[],options);

% adjust p(2) to accept angles in radians
p(2) = p(2) * 180 / pi;

% add to plot
hold on
rom = 0:160;
romrad = rom*pi/180;
plot(rom,-p(1) * (tanh(p(2) * romrad - p(3)) + p(4)))

% report param values
fprintf('Expression: M = a * ( tanh( b * angle - c ) + d )\n')
fprintf('     a = %f\n',-p(1))
fprintf('     b = %f\n',p(2))
fprintf('     c = %f\n',p(3))
fprintf('     d = %f\n',p(4))
fprintf('M(a = 0) = %f\n',-p(1) * (tanh(p(2) * 0 - p(3)) + p(4)))
fprintf('M(a = 90) = %f\n',-p(1) * (tanh(p(2) * pi/2 - p(3)) + p(4)))
fprintf('OpenSim Expression: "%f*(tanh(%f*q-%f)+%f)"',-p(1),p(2),p(3),p(4))

%% test model

import org.opensim.modeling.*
m = Model('sprinter_v3.osim');

% remove old model (uses CoordinateLimitForce)
m.get_ForceSet.remove(m.get_ForceSet.getIndex('right_thigh_shank_contact_torque'));

% construct and set
f = ExpressionBasedCoordinateForce;
f.set_appliesForce(true);
f.set_coordinate('knee_angle_r');
f.set_expression(sprintf("%f*(tanh(%f*q-%f)+%f)",-p(1),p(2),p(3),p(4)));
f.setName('right_thigh_shank_contact_knee_moment');

% add to model
m.addForce(f);

s = m.initSystem;
coord.knee_angle_r = (0:160)*pi/180;
Tosim = zeros(1,161);
for k = 1:161
    [m,s] = osimSetConfiguration(m,s,coord,k);
    Tosim(k) = f.calcExpressionForce(s);
end
plot(coord.knee_angle_r*180/pi,Tosim,'g')
xlim([130 160])
