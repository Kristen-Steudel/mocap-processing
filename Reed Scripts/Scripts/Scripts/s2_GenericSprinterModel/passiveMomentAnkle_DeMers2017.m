function passiveMomentAnkle_DeMers2017(model)

% adds a right/left expression based bushing force to model the passive
% moments in ankle due to non-muscle tissue. Model from DeMers et al. 2017

% note that the expressions for Mx and My for the left ankle are different
% here than the ones used by DeMers et al to ensure the moment-angle
% relationship for the left ankle are the same as the right but reflected
% about the y-axis

% note concerning a point of confusion: these expressions express the
% negative of the force applied about the axis of interest, i.e., notice
% that the force computed is negated in the method where the expression is
% evaluated:
% https://github.com/opensim-org/opensim-core/blob/a01cf545485e8bc1f6e50b29ddeab229677de2c5/OpenSim/Simulation/Model/ExpressionBasedBushingForce.cpp#L301)

import org.opensim.modeling.*

model.finalizeConnections;

%% RIGHT ANKLE BUSHING

% construct
shank_r_frame = model.getBodySet.get('tibia_r');
shank_r_ankle_offset = model.getJointSet.get('ankle_r').get_frames(0).get_translation; % use location of ankle joint center: is the talus origin
calcn_r_frame = model.getBodySet.get('calcn_r');
calcn_r_subtalar_offset = model.getJointSet.get('subtalar_r').get_frames(0).get_translation; % subtalar JC is at calcn origin, so offset we want (talus origin relative to calcn) use opposite of this
calcn_r_ankle_offset = Vec3;
for k = 0:2; calcn_r_ankle_offset.set(k,-calcn_r_subtalar_offset.get(k)); end
right_ankle_bushing = ExpressionBasedBushingForce('rightPassiveMomentAnkle_DeMers2017',...
                                shank_r_frame,...
                                shank_r_ankle_offset,...
                                Vec3(0,0,0),...
                                calcn_r_frame,...
                                calcn_r_ankle_offset,...
                                Vec3(0,0,0));

% properties                            
right_ankle_bushing.set_appliesForce(true);
right_ankle_bushing.set_Fx_expression("0.0");
right_ankle_bushing.set_Fy_expression("0.0");
right_ankle_bushing.set_Fz_expression("0.0");
right_ankle_bushing.set_Mx_expression("9.52095038*theta_x-29.57608094*theta_x^2+169.01588105*theta_x^3");
right_ankle_bushing.set_My_expression("3.61796125*theta_y-10.1612135*theta_y^2+64.5279144*theta_y^3");
right_ankle_bushing.set_Mz_expression("5.98187224*theta_z+15.74951827*theta_z^2+33.6780792*theta_z^3");
right_ankle_bushing.set_translational_damping(Vec3(0,0,0));
right_ankle_bushing.set_rotational_damping(Vec3(0,0,0));
right_ankle_bushing.set_force_visual_scale(1.0);
right_ankle_bushing.set_moment_visual_scale(1.0);

% add to model
model.addForce(right_ankle_bushing);

%% LEFT ANKLE BUSHING

% construct
shank_l_frame = model.getBodySet.get('tibia_l');
shank_l_ankle_offset = model.getJointSet.get('ankle_l').get_frames(0).get_translation; % use location of ankle joint center: is the talus origin
calcn_l_frame = model.getBodySet.get('calcn_l');
calcn_l_subtalar_offset = model.getJointSet.get('subtalar_l').get_frames(0).get_translation; % subtalar JC is at calcn origin, so offset we want (talus origin relative to calcn) use opposite of this
calcn_l_ankle_offset = Vec3;
for k = 0:2; calcn_l_ankle_offset.set(k,-calcn_l_subtalar_offset.get(k)); end
left_ankle_bushing = ExpressionBasedBushingForce('leftPassiveMomentAnkle_DeMers2017',...
                                shank_l_frame,...
                                shank_l_ankle_offset,...
                                Vec3(0,0,0),...
                                calcn_l_frame,...
                                calcn_l_ankle_offset,...
                                Vec3(0,0,0));

% properties                            
left_ankle_bushing.set_appliesForce(true);
left_ankle_bushing.set_Fx_expression("0.0");
left_ankle_bushing.set_Fy_expression("0.0");
left_ankle_bushing.set_Fz_expression("0.0");
left_ankle_bushing.set_Mx_expression("-9.52095038*theta_x-29.57608094*theta_x^2-169.01588105*theta_x^3");
left_ankle_bushing.set_My_expression("-3.61796125*theta_y-10.1612135*theta_y^2-64.5279144*theta_y^3");
left_ankle_bushing.set_Mz_expression("5.98187224*theta_z+15.74951827*theta_z^2+33.6780792*theta_z^3");
left_ankle_bushing.set_translational_damping(Vec3(0,0,0));
left_ankle_bushing.set_rotational_damping(Vec3(0,0,0));
left_ankle_bushing.set_force_visual_scale(1.0);
left_ankle_bushing.set_moment_visual_scale(1.0);

% add to model
model.addForce(left_ankle_bushing);

%% FINALIZE

model.finalizeConnections;
model.initSystem;

end