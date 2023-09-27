function passiveMomentHipAdduction_Andersen1999(model)

% adds a right/left ExpressionBasedCoordinateForce for the hip adduction
% coordinate modeling the torque due to ligaments. Model is taken from
% table G.2 and eq. 4.4 (pg 74) from Frank Andersen thesis (1999). Note
% that this model predicts 5 Nm adduction  moment at ~ -48.2 deg abduction
% which is a bit more than the angle at which van Arkel observed 5 Nm (~30)
% (2015, JOB, 48:3803-3809). Thus Andersen's model presents a very
% conservative approximation to empirical data. Even more so on the
% adduction side: Andersen's model predicts ~0.5 Nm abd moment at 40 deg
% adduction whereas van Arkel reports 5 Nm at 40 deg adduction (for the hip
% flexion to 30 deg). Even still consider that Tim Dorn's 9 m/s sprinting
% data has hip configuration between -20 and 10 deg adduction; even from
% van Arkel's data, moments are low here

import org.opensim.modeling.*

%% RIGHT HIP

% construct and set
right_hip_adduction_moment = ExpressionBasedCoordinateForce;
right_hip_adduction_moment.set_appliesForce(true);
right_hip_adduction_moment.set_coordinate('hip_adduction_r');
right_hip_adduction_moment.set_expression("-0.03*exp(14.94*(q-0.5))+0.03*exp(-14.94*(q+0.5))");
right_hip_adduction_moment.setName('rightPassiveMomentHipAdduction_Andersen1999');

% add to model
model.addForce(right_hip_adduction_moment);

%% LEFT HIP

% construct and set
left_hip_adduction_moment = ExpressionBasedCoordinateForce;
left_hip_adduction_moment.set_appliesForce(true);
left_hip_adduction_moment.set_coordinate('hip_adduction_l');
left_hip_adduction_moment.set_expression("-0.03*exp(14.94*(q-0.5))+0.03*exp(-14.94*(q+0.5))");
left_hip_adduction_moment.setName('leftPassiveMomentHipAdduction_Andersen1999');

% add to model
model.addForce(left_hip_adduction_moment);

model.initSystem;

end