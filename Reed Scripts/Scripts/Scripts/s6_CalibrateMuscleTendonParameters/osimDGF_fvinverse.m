%% inverse of normalized force velocity function
function vn = osimDGF_fvinverse(fn)

% see https://github.com/opensim-org/opensim-core/blob/1fb9ae0caeb8350264b5751f10639c71d516618f/OpenSim/Actuators/DeGrooteFregly2016Muscle.h#L917
d1 = -0.3211346127989808;
d2 = -8.149;
d3 = -0.374;
d4 = 0.8825327733249912;

% see https://github.com/opensim-org/opensim-core/blob/1fb9ae0caeb8350264b5751f10639c71d516618f/OpenSim/Actuators/DeGrooteFregly2016Muscle.h#L430
vn = (sinh(1.0 / d1 * (fn - d4)) - d3) / d2;

end