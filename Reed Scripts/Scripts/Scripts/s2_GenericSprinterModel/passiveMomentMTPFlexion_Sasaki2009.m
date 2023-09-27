function passiveMomentMTPFlexion_Sasaki2009(model)

% adds a right/left SpringGeneralizedForce to model the passive moment 
% about the mtp joint due to non-muscle tissue. Modeled after Sasaki et al.
% (2009): see properties listed after eq (1)

import org.opensim.modeling.*

%% RIGHT MTP SPRING

% build spring
right_mtp_spring = SpringGeneralizedForce('mtp_angle_r');
right_mtp_spring.setName('rightPassiveMomentMTPFlexion_Sasaki2009');
right_mtp_spring.set_coordinate('mtp_angle_r');
right_mtp_spring.setRestLength(0.0);
right_mtp_spring.setStiffness(25.0);
right_mtp_spring.setViscosity(0.03);

% add to model
model.addForce(right_mtp_spring);

%% LEFT MTP SPRING

% build spring
left_mtp_spring = SpringGeneralizedForce('mtp_angle_l');
left_mtp_spring.setName('leftPassiveMomentMTPFlexion_Sasaki2009');
left_mtp_spring.setRestLength(0.0);
left_mtp_spring.setStiffness(25.0);
left_mtp_spring.setViscosity(0.03);

% add to model
model.addForce(left_mtp_spring);

%% FINALIZE

model.finalizeConnections;
model.initSystem;

end