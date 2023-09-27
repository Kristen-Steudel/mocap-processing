%% normalized tendon force-length function
function [ft,dft_dltn,dft_de0] = osimDGF_ftendon(ltn,e0)

% e0 - tendon strain at F0

% k = tendonForceLengthParameter, m_kT in osim code base
% DeGroote used k = 35;
% osim adjusts k s.t. normalized ft = 1 when tendon strain = e0
% relationship: k = log(6) / e0
% for e0 = 0.049 (default) => k = 36.5665;
% for e0 = 0.1 (plantarflexors) => k = 17.9176

% see here: https://github.com/opensim-org/opensim-core/blob/1fb9ae0caeb8350264b5751f10639c71d516618f/OpenSim/Actuators/DeGrooteFregly2016Muscle.cpp#L161
k = log(6) / e0;
dk_de0 = -k / e0;

% see line 906-915 here: https://github.com/opensim-org/opensim-core/blob/1fb9ae0caeb8350264b5751f10639c71d516618f/OpenSim/Actuators/DeGrooteFregly2016Muscle.h
c1 = 0.2;
c2 = 1.0;
c3 = 0.2;

ft = c1 * exp(k * (ltn - c2)) - c3;
dft_dltn = k * c1 * exp(k * (ltn - c2));
dft_dk = (ltn - c2) * c1 * exp(k * (ltn - c2));
dft_de0 = dft_dk * dk_de0;

end