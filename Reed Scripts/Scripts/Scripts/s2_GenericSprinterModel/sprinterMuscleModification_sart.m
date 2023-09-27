function sprinterMuscleModification_sart(model,notes)

import org.opensim.modeling.*

% without modification, this muscle extended the hip when in full extension

state = model.initSystem;

%% FIRST MODIFICATION

% adjust vp 3 s.t. sart remains a knee flexor at full knee flexion

% set knee to max ROM
model.getCoordinateSet.get('knee_angle_r').setValue(state,model.getCoordinateSet.get('knee_angle_r').getRangeMax);

% get 3rd vp location in femur at max knee flexion
rmuscle = model.getForceSet.get('sart_r');
rpath = GeometryPath.safeDownCast(PathActuator.safeDownCast(rmuscle).getGeometryPath);
rpointset = PathPointSet.safeDownCast(rpath.getPathPointSet);
rp2 = PathPoint.safeDownCast(rpointset.get(1));
p2_loc0 = rp2.get_location;
p2_loc0 = [p2_loc0.get(0); p2_loc0.get(1); p2_loc0.get(2)];
f2 = Frame.safeDownCast(rp2.getBody);
rp3 = PathPoint.safeDownCast(rpointset.get(2));
f3 = Frame.safeDownCast(rp3.getBody);
p3_loc = f3.findStationLocationInAnotherFrame(state,rp3.get_location,f2);

% set 2nd vp to this value, this way the sart does not extend the knee in
% deep flexion
rp2.set_location(p3_loc);
lp2new = rp2.get_location;
lp2new = [lp2new.get(0); lp2new.get(1); lp2new.get(2)];
fprintf('-sart VP 2 set equal to VP 3 location in femur at deep knee flexion, moved %4.3f cm from original location\n',vecnorm(lp2new-p2_loc0));
if nargin > 1
    fprintf(notes,'-sart VP 2 set equal to VP 3 location in femur at deep knee flexion, moved %4.3f cm from original location\n',vecnorm(lp2new-p2_loc0)*100);
end

% update left side, same as right but negate z coordinate
lmuscle = model.getForceSet.get('sart_l');
lpath = GeometryPath.safeDownCast(PathActuator.safeDownCast(lmuscle).getGeometryPath);
lpointset = PathPointSet.safeDownCast(lpath.getPathPointSet);
lp2 = PathPoint.safeDownCast(lpointset.get(1));
lp2.set_location(Vec3(p3_loc.get(0),p3_loc.get(1),-p3_loc.get(2)));

% finish
model.finalizeConnections;
model.initSystem;
clearvars -except model notes

%% SECOND MODIFICATION

% clone first vp and insert as a new vp at osim path point set index 1
% then incrementally move it down and forward (in pelvis) until hip flexion
% moment arm is positive at full ext and full flexion

% clone vp 1 and insert as new vp 2
rmuscle = model.getForceSet.get('sart_r');
rpath = GeometryPath.safeDownCast(PathActuator.safeDownCast(rmuscle).getGeometryPath);
rpointset = PathPointSet.safeDownCast(rpath.getPathPointSet);
rp1 = PathPoint.safeDownCast(rpointset.get(0));
rp2 = rp1.clone;
rpointset.insert(1,rp2);
rp2.markAdopted;
rmuscle.finalizeConnections(model);

% update vp names
for k = 6:-1:2
    rpointset.get(k-1).setName(['sart_r-P' num2str(k)]);
end

% now do for left side
lmuscle = model.getForceSet.get('sart_l');
lpath = GeometryPath.safeDownCast(PathActuator.safeDownCast(lmuscle).getGeometryPath);
lpointset = PathPointSet.safeDownCast(lpath.getPathPointSet);
lp1 = PathPoint.safeDownCast(lpointset.get(0));
lp2 = lp1.clone;
lpointset.insert(1,lp2);
lp2.markAdopted;
lmuscle.finalizeConnections(model);

% update vp names
for k = 6:-1:2
    lpointset.get(k-1).setName(['sart_l-P' num2str(k)]);
end

% finalize
model.finalizeConnections;
model.initSystem;
clearvars -except model notes

% get new ref to new vp 2
state = model.initSystem;
rmuscle = model.getForceSet.get('sart_r');
rpath = GeometryPath.safeDownCast(PathActuator.safeDownCast(rmuscle).getGeometryPath);
rpointset = PathPointSet.safeDownCast(rpath.getPathPointSet);
rp2 = PathPoint.safeDownCast(rpointset.get(1));

% move down x and forward x/2 at x = 5 mm increments until sart is a hip
% flexor at both max extension and max flexion
increment = 0.005;
total_increment = 0;
hip = model.getCoordinateSet.get('hip_flexion_r');
hip.setValue(state,hip.getRangeMin);
ma_min = rpath.computeMomentArm(state,hip);
hip.setValue(state,hip.getRangeMax);
ma_max = rpath.computeMomentArm(state,hip);
while ma_min < 0 || ma_max < 0
    total_increment = total_increment + increment;
    loc = rp2.get_location;
    loc.set(0,loc.get(0)+increment/2);
    loc.set(1,loc.get(1)-increment);
    rp2.set_location(loc);
    state = model.initSystem;
    hip = model.getCoordinateSet.get('hip_flexion_r');
    hip.setValue(state,hip.getRangeMin);
    ma_min = rpath.computeMomentArm(state,hip);
    hip.setValue(state,hip.getRangeMax);
    ma_max = rpath.computeMomentArm(state,hip);
end

fprintf('-created new sart vp initially at same location as P1 except moved down %4.2f cm and forward %4.2f cm s.t. sart flexed hip at max hip ext and max hip flex\n',total_increment,total_increment/2);
if nargin > 1
    fprintf(notes,'-created new sart vp initially at same location as P1 except moved down %4.2f cm and forward %4.2f cm s.t. sart flexed hip at max hip ext and max hip flex\n',total_increment,total_increment/2);
end

% now left muscle
lmuscle = model.getForceSet.get('sart_l');
lpath = GeometryPath.safeDownCast(PathActuator.safeDownCast(lmuscle).getGeometryPath);
lpointset = PathPointSet.safeDownCast(lpath.getPathPointSet);
lp2 = PathPoint.safeDownCast(lpointset.get(1));
lp2.set_location(Vec3(loc.get(0),loc.get(1),-loc.get(2)));

% finish
model.finalizeConnections;
model.initSystem;

end