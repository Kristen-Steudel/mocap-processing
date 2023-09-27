function sprinterMuscleModification_bflh(model,notes)

% move P2 and P3 forward (+x) at 5 mm increments until the moment arm at
% max knee flexion is still a flexion moment arm

import org.opensim.modeling.*

state = model.initSystem;

%% RIGHT BFLH

% get knee flexion and set to max
knee = model.getCoordinateSet.get('knee_angle_r');
knee.setValue(state,knee.getRangeMax);

% get bflh
bflh = model.getMuscles.get('bflh_r');

% get geometry path and origin/insertion
path = GeometryPath.safeDownCast(bflh.getGeometryPath);
pps = PathPointSet.safeDownCast(path.getPathPointSet);
p2 = PathPoint.safeDownCast(pps.get(1));
p3 = PathPoint.safeDownCast(pps.get(2));

% move p2 and p3 forward until moment arm is flexion (>0)
increment = 0.005;
total_increment = 0;
momentArm = path.computeMomentArm(state,knee);
while momentArm < 0
    
    p2.get_location.set(0,p2.get_location.get(0) + increment);
    p3.get_location.set(0,p3.get_location.get(0) + increment);
    total_increment = total_increment + increment;
    
    state = model.initSystem;
    knee.setValue(state,knee.getRangeMax);
    momentArm = path.computeMomentArm(state,knee);
    
end
fprintf('-via points 2 and 3 of bflh moved forward (+x) %4.2f cm s.t. muscle remains a flexor at max knee flexion (moment arm = %4.2f cm)\n',total_increment*100,momentArm*100);
if nargin > 1
    fprintf(notes,'-via points 2 and 3 of bflh moved forward (+x) %4.2f cm s.t. muscle remains a flexor at max knee flexion (moment arm = %4.2f cm)\n',total_increment*100,momentArm*100);
end
% finish
model.finalizeConnections;
model.initSystem;

%% LEFT BFLH

% get bflh
bflh = model.getMuscles.get('bflh_l');

% get geometry path and origin/insertion
path = GeometryPath.safeDownCast(bflh.getGeometryPath);
pps = PathPointSet.safeDownCast(path.getPathPointSet);
p2 = PathPoint.safeDownCast(pps.get(1));
p3 = PathPoint.safeDownCast(pps.get(2));

% move p2 and p3 forward as for the right bflh
p2.get_location.set(0,p2.get_location.get(0) + total_increment);
p3.get_location.set(0,p3.get_location.get(0) + total_increment);

% get knee flexion and set to max
state = model.initSystem;
knee = model.getCoordinateSet.get('knee_angle_l');
knee.setValue(state,knee.getRangeMax);
if path.computeMomentArm(state,knee) < 0; error('same adjustments that succesfully kept bflh_r a knee flexor for full knee ROM were not successful for bflh_l'); end

% finish
model.finalizeConnections;
model.initSystem;

end