function sprinterMuscleModification_semiten(model,notes)

% move P2 and P3 forward (+x) at 5 mm increments until the moment arm at
% max knee flexion is still a flexion moment arm

import org.opensim.modeling.*

state = model.initSystem;

%% RIGHT SEMITEN

% get knee flexion and set to max
knee = model.getCoordinateSet.get('knee_angle_r');
knee.setValue(state,knee.getRangeMax);

% get semiten
semiten = model.getMuscles.get('semiten_r');

% get geometry path and origin/insertion
path = GeometryPath.safeDownCast(semiten.getGeometryPath);
pps = PathPointSet.safeDownCast(path.getPathPointSet);
p2 = PathPoint.safeDownCast(pps.get(1));
p3 = PathPoint.safeDownCast(pps.get(2));

% get direction p2 to p3 and distance
r2 = [p2.get_location.get(0); p2.get_location.get(1); p2.get_location.get(2)];
r3 = [p3.get_location.get(0); p3.get_location.get(1); p3.get_location.get(2)];
u = r3 - r2;
dist23 = sqrt(u'*u);
u = u / dist23;

% move p2 toward p3 along u at 10% of d increments until semiten flexes
% knee at full ROM
increment = 0.1;
total_increment = 0;
momentArm = path.computeMomentArm(state,knee);
while momentArm < 0
    
    r2new = r2 + total_increment * dist23 * u;
    p2.set_location(Vec3(r2new(1),r2new(2),r2new(3)));
    total_increment = total_increment + increment;
    
    state = model.initSystem;
    knee.setValue(state,knee.getRangeMax);
    momentArm = path.computeMomentArm(state,knee);
    
end
fprintf('-via point 2 of semiten moved toward via point 3 %4.2f %% of the original distance between the two\n',total_increment*100);
if nargin > 1
    fprintf(notes,'-via point 2 of semiten moved toward via point 3 %4.2f %% of the original distance between the two\n',total_increment*100);
end

% finish
model.finalizeConnections;
model.initSystem;

%% LEFT SEMITEN

% keep new p2 from right side to update left side, negate z coordinate
newLeftP2 = Vec3(r2new(1),r2new(2),-r2new(3));

% get semiten
semiten = model.getMuscles.get('semiten_l');

% get geometry path and origin/insertion
path = GeometryPath.safeDownCast(semiten.getGeometryPath);
pps = PathPointSet.safeDownCast(path.getPathPointSet);
p2 = PathPoint.safeDownCast(pps.get(1));

% set p2 to new location
p2.set_location(newLeftP2);

% verify moment arm signed correctly
state = model.initSystem;
knee = model.getCoordinateSet.get('knee_angle_l');
knee.setValue(state,knee.getRangeMax);
if path.computeMomentArm(state,knee) < 0; error('same adjustments that ensured right semiten flexed knee for full ROM did not do so for left semiten'); end

% finish
model.finalizeConnections;
model.initSystem;

end