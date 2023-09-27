function passiveMomentHipRotation_vanArkel2015(model)

% adds a right/left ExpressionBasedCoordinateForce for the hip rotation
% coordinate modeling the torque due to ligaments. A 6 parameter 
% bi-exponential model, p1*exp(p2*(q-p3))+p4*exp(p5*(q-p6)), was fit to
% 6 datapoints from Arkel et al. 2015 (JOB, 48:3803-9), Fig.3,
% configuration: hip flexion = 30, hip adduction = 0 (this configuration
% was the closest reported in van Arkel's study to the mean hip flex/add
% angles during 9 m/s sprinting trial from Tim Dorn's data). The six
% datapoints were -5 Nm at 35 deg rot and 5 Nm at -35 deg rot (dashed line
% in figure), 0 Nm at 2.5, 0, and 2.5 deg rot (dotted line +/- 2.5 deg),
% and s.t. the slope = -0.03 Nm/deg at 25 and -25 deg rot (solid black line
% in figure). Note also that for this configuration the data predicts a
% torque-angle curve which is nearly symmetric about 0 deg rotation which
% is consistent with the rotation ROM in the LaiArnold model [-40, 40] deg

% see deriveHipLigamentRotationMoment_vanArkel2015 in nms-dyn

import org.opensim.modeling.*

%% RIGHT HIP

% construct and set
right_hip_rotation_moment = ExpressionBasedCoordinateForce;
right_hip_rotation_moment.set_appliesForce(true);
right_hip_rotation_moment.set_coordinate('hip_rotation_r');
right_hip_rotation_moment.set_expression("-1.799123*exp(24.428054*(q-0.569022))+1.777588*exp(-24.428057*(q+0.568529))");
right_hip_rotation_moment.setName('rightPassiveMomentHipRotation_vanArkel2015');

% add to model
model.addForce(right_hip_rotation_moment);

%% LEFT HIP

% construct and set
left_hip_rotation_moment = ExpressionBasedCoordinateForce;
left_hip_rotation_moment.set_appliesForce(true);
left_hip_rotation_moment.set_coordinate('hip_rotation_l');
left_hip_rotation_moment.set_expression("-1.799123*exp(24.428054*(q-0.569022))+1.777588*exp(-24.428057*(q+0.568529))");
left_hip_rotation_moment.setName('leftPassiveMomentHipRotation_vanArkel2015');

% add to model
model.addForce(left_hip_rotation_moment);

model.initSystem;

end