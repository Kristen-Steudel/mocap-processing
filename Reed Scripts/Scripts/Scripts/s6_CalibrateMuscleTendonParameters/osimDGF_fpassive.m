%% normalized passive muscle force function
function [fp,dfp_dln,dfp_ds0] = osimDGF_fpassive(ln,s0)

% derivatives have been verified numerically

% params for passiveFiberForceMultiplier
% see line 902 and 930 here: https://github.com/opensim-org/opensim-core/blob/1fb9ae0caeb8350264b5751f10639c71d516618f/OpenSim/Actuators/DeGrooteFregly2016Muscle.h
m_minNormFiberLength = 0.2;
smin = m_minNormFiberLength - 1.0;
kPE = 4.0; 

offset = exp( kPE * smin / s0 );
doffset_ds0 = -kPE * smin * offset / s0 / s0;
doffset_dln = 0;

exp1 = exp( kPE * (ln - 1.0) / s0);
dexp1_ds0 = -kPE * (ln-1.0) * exp1 / s0 / s0;
dexp1_dln = exp1 * kPE / s0;

N = exp1 - offset;
dN_ds0 = dexp1_ds0 - doffset_ds0;
dN_dln = dexp1_dln - doffset_dln;

D = exp( kPE ) - offset;
dD_dln = -doffset_dln;
dD_ds0 = -doffset_ds0;

fp = N / D; % N * D^-1 => dfp_dx = dNdx / D - N / D / D * dD_dx
dfp_dln = dN_dln / D - fp / D * dD_dln;
dfp_ds0 = dN_ds0 / D - fp / D * dD_ds0;

end