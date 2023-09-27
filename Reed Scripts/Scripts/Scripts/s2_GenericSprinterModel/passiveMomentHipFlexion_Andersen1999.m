function passiveMomentHipFlexion_Andersen1999(model)

% adds a right/left ExpressionBasedCoordinateForce for the hip flexion
% coordinate modeling the torque due to ligaments. Model is taken from
% table G.2 and eq. 4.4 (pg 74) from Frank Andersen thesis (1999). Note
% that this model predicts 5 Nm flexion  moment at -26.6 degrees extension
% which resembles a conservative model approximating van Arkel's data
% (2015, JOB, 48:3803-3809): they found 5 Nm flexion moment occured at
% -12+/-7 degrees and -26.6 is ~2 S.D. below the mean (hence
% 'conservative')

import org.opensim.modeling.*

%% RIGHT HIP

% construct and set
right_hip_flexion_moment = ExpressionBasedCoordinateForce;
right_hip_flexion_moment.set_appliesForce(true);
right_hip_flexion_moment.set_coordinate('hip_flexion_r');
right_hip_flexion_moment.set_expression("-2.44*exp(5.05*(q-1.81))+1.51*exp(-21.88*(q+0.47))");
right_hip_flexion_moment.setName('rightPassiveMomentHipFlexion_Andersen1999');

% add to model
model.addForce(right_hip_flexion_moment);

%% LEFT HIP

% construct and set
left_hip_flexion_moment = ExpressionBasedCoordinateForce;
left_hip_flexion_moment.set_appliesForce(true);
left_hip_flexion_moment.set_coordinate('hip_flexion_l');
left_hip_flexion_moment.set_expression("-2.44*exp(5.05*(q-1.81))+1.51*exp(-21.88*(q+0.47))");
left_hip_flexion_moment.setName('leftPassiveMomentHipFlexion_Andersen1999');

% add to model
model.addForce(left_hip_flexion_moment);

model.initSystem;

end