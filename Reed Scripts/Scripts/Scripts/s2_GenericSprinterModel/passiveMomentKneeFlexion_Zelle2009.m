function passiveMomentKneeFlexion_Zelle2009(model)

% adds a right/left ExpressionBasedCoordinateForce for the knee flexion
% angle coordinate modeling the thigh-shank contact force. See
% deriveTanhApproximationToKneeThighShankContactMoment for explanation 
% of expression, parameters, etc.

import org.opensim.modeling.*

%% RIGHT KNEE

% construct and set
right_knee_moment = ExpressionBasedCoordinateForce;
right_knee_moment.set_appliesForce(true);
right_knee_moment.set_coordinate('knee_angle_r');
right_knee_moment.set_expression("-25.385677*(tanh(4.066036*q-12.097768)+0.999998)");
right_knee_moment.setName('rightPassiveMomentKneeFlexion_Zelle2009');

% add to model
model.addForce(right_knee_moment);

%% LEFT KNEE

% construct and set
left_knee_moment = ExpressionBasedCoordinateForce;
left_knee_moment.set_appliesForce(true);
left_knee_moment.set_coordinate('knee_angle_l');
left_knee_moment.set_expression("-25.385677*(tanh(4.066036*q-12.097768)+0.999998)");
left_knee_moment.setName('leftPassiveMomentKneeFlexion_Zelle2009');

% add to model
model.addForce(left_knee_moment);

model.initSystem;

end