function passiveMomentKneeFlexion_Andersen1999(model)

% adds a right/left ExpressionBasedCoordinateForce for the knee flexion
% coordinate modeling the torque due to ligaments. Model is taken from
% table G.2 and eq. 4.4 (pg 74) from Frank Andersen thesis (1999).

import org.opensim.modeling.*

%% RIGHT KNEE

% construct and set
right_knee_flexion_moment = ExpressionBasedCoordinateForce;
right_knee_flexion_moment.set_appliesForce(true);
right_knee_flexion_moment.set_coordinate('knee_angle_r');
right_knee_flexion_moment.set_expression("6.09*exp(-33.94*(q+0.13))-11.03*exp(-11.33*(2.40-q))");
right_knee_flexion_moment.setName('rightPassiveMomentKneeFlexion_Andersen1999');

% add to model
model.addForce(right_knee_flexion_moment);

%% LEFT KNEE

% construct and set
left_knee_flexion_moment = ExpressionBasedCoordinateForce;
left_knee_flexion_moment.set_appliesForce(true);
left_knee_flexion_moment.set_coordinate('knee_angle_l');
left_knee_flexion_moment.set_expression("6.09*exp(-33.94*(q+0.13))-11.03*exp(-11.33*(2.40-q))");
left_knee_flexion_moment.setName('leftPassiveMomentKneeFlexion_Andersen1999');

% add to model
model.addForce(left_knee_flexion_moment);

model.initSystem;

end