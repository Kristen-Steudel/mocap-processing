function s = approxScalePassiveForceByAdjustingStrainAtOneNormForce(activeFiberForceScaleFactor,passiveFiberForceScaleFactor,originalPassiveFiberStrainAtOneNormForce)

% let's say we wish to scale F0 differently for active and passive force
% multipliers. For example, we'd like to do f = F0 * (ca * fa + cp * fp).
% OpenSim allows us only to scale F0. One approach is to do:
%
%                   fm = ca * F0 * (fa + cp/ca * fp)
%
% where ca * F0 is done using ModOpScalingMaxIsometricForce(ca). We can't
% actually scale fp in this way as we can only adjust the
% PassiveFiberStrainAtOneNormForce parameter. So this function provides the
% value at which to set this parameter s.t. calcPassiveForceMultiplier 
% returns cp/ca when the original model would return 1 (i.e., when the 
% muscle strain is equal to the original PassiveFiberStrainAtOneNormForce, 
% which the default is 0.6).

% INPUTS
% activeFiberForceScaleFactor - scale factor for active force multiplier, 
%       should be used in:
%       ModelProcessor.append(ModOpScaleMaxIsometricForce(activeFiberForceScaleFactor))
% passiveFiberForceScaleFactor - scale factor for passive force multiplier,
%       see description above 
% originalPassiveFiberStrainAtOneNormForce - original value of the 
%       parameter PassiveFiberStrainAtOneNormForce

% OUTPUTS
% s - adjusted value of the parameter PassiveFiberStrainAtOneNormForce
%       should be set on muscle using
%       muscle.set_passive_fiber_straint_one_norm_force(s)

% note that this is not exactly equal to scaling fp by cp. Derivatives will
% not match generally. The larger the difference between ca and cp, the
% larger the resulting fp multiplier will be from actually scaling via cp *
% fp. However, for reasonable scale factors (ca, cp), the approximation is
% sufficient.

% osim parameters
m_minNormFiberLength = 0.2;
smin = m_minNormFiberLength - 1;
kPE = 4.0;

% solve
options = optimoptions('fsolve','CheckGradients',false,'SpecifyObjectiveGradient',true,'FunctionTolerance',eps,'Display','off');
s = fsolve(@(x)fun(x,smin,kPE,originalPassiveFiberStrainAtOneNormForce,activeFiberForceScaleFactor,passiveFiberForceScaleFactor),originalPassiveFiberStrainAtOneNormForce,options); % trust region method

end

function [z,dz_dx] = fun(x,smin,kPE,s0,ca,cp)

offset = exp( kPE * smin / x );
doffset_dx = -offset * kPE * smin / x / x;

exp1 = exp( kPE * s0 / x);
dexp1_dx = -exp1 * kPE * s0 / x / x;

N = exp1 - offset;
dN_dx = dexp1_dx - doffset_dx;

D = exp( kPE ) - offset;
dD_dx = -doffset_dx;

fp = N / D;
dfp_dx = dN_dx / D - N * dD_dx / D / D;

z = cp/ca - fp;
dz_dx = -dfp_dx;

end