%% normalized active fiber force-length function
function [fa,dfa_dln] = osimDGF_flactive(ln)

% see https://github.com/opensim-org/opensim-core/blob/1fb9ae0caeb8350264b5751f10639c71d516618f/OpenSim/Actuators/DeGrooteFregly2016Muscle.h#L881
b(1,1) = 0.8150671134243542;
b(2,1) = 1.055033428970575;
b(3,1) = 0.162384573599574;
b(4,1) = 0.063303448465465;
b(1,2) = 0.433004984392647;
b(2,2) = 0.716775413397760;
b(3,2) = -0.029947116970696;
b(4,2) = 0.200356847296188;
b(1,3) = 0.1;
b(2,3) = 1.0;
b(3,3) = 0.353553390593274; % 0.5 * sqrt(0.5)
b(4,3) = 0.0;

% see https://github.com/opensim-org/opensim-core/blob/1fb9ae0caeb8350264b5751f10639c71d516618f/OpenSim/Actuators/DeGrooteFregly2016Muscle.h#L376
% ASSUMES DGF scale PROPERTY = 1.0
fa = 0;
dfa_dln = 0;
for i = 1:3
    
    % see https://github.com/opensim-org/opensim-core/blob/1fb9ae0caeb8350264b5751f10639c71d516618f/OpenSim/Actuators/DeGrooteFregly2016Muscle.h#L832
    num = ln - b(2,i);
    den = b(3,i) + b(4,i) * ln;
    x = num ./ den;
    bell = b(1,i) * exp(-x * x / 2);
    fa = fa + bell;
    
    dx_dln = (b(3,i) + b(4,i) * b(2,i)) / den / den;
    dbell_dln = -bell * x * dx_dln;
    dfa_dln = dfa_dln + dbell_dln;
    
end

end