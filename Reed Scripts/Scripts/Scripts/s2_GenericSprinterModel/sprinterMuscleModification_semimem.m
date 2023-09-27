function sprinterMuscleModification_semimem(model,notes)

% rotate second via point (only has 2) along cylinder surface until moment
% arm about knee acts to flex the knee at max knee flexion

import org.opensim.modeling.*

model.initSystem;

%% RIGHT SEMIMEM

% get muscle
semimem = model.getMuscles.get('semimem_r');

% get second path point
path = GeometryPath.safeDownCast(semimem.getGeometryPath);
pps = PathPointSet.safeDownCast(path.getPathPointSet);
p2 = PathPoint.safeDownCast(pps.get(1));
r2Vec3 = p2.get_location;
r2 = [r2Vec3.get(0); r2Vec3.get(1); r2Vec3.get(2)];
f2 = Frame.safeDownCast((p2.getBody));

% get wrap cylinder
wrap = path.getWrapSet.get(0); % this assumes there is only one wrap object
cyl = WrapCylinder.safeDownCast(model.getBodySet.get(f2.getName).get_WrapObjectSet.get(wrap.get_wrap_object)); % this assumes cyl body is f2, will throw error if not
cylpos = [cyl.get_translation.get(0); cyl.get_translation.get(1); cyl.get_translation.get(2)];
cylrad = cyl.get_radius;
R.xyz = [cyl.get_xyz_body_rotation.get(0); cyl.get_xyz_body_rotation.get(1); cyl.get_xyz_body_rotation.get(2)];

% get point (A) on cylinder axis that is a minimum distance from p2
v = r2 - cylpos;
w = rot(R,[0;0;1],'inverse'); % cylinder axis represented in f2
x = v' * w; % location of point A relative to cylpos along w
A = cylpos + x * w;

% get vector that points from A to r2
y0 = r2 - A;

% if the distance of r2 from the cylinder axis is less than the cylinde
% radius plus a 5 mm cushion, then move it away from the cylinder until it
% is
ymag = sqrt(y0'*y0);
yhat = y0 / ymag;
if ymag < cylrad + 0.005
    r2 = A + (cylrad + 0.005) * yhat;
    y0 = r2 - A;
    fprintf('-semimem insertion was less than 5 mm away from %s surface, moved radially away from surface to exactly satisfy this condition\n',cyl.getName);
    if nargin > 1
        fprintf(notes,'-semimem insertion was less than 5 mm away from %s surface, moved radially away from surface to exactly satisfy this condition\n',cyl.getName);
    end
end

% now rotate r2 about cylinder axis until moment arm about knee acts to
% flex the knee
knee = model.getCoordinateSet.get('knee_angle_r');
state = model.initSystem;
knee.setValue(state,knee.getRangeMax);
momentArm = path.computeMomentArm(state,knee);
rotationIncrement = pi/180;
rotationAngle = 0;
wskew = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];
while momentArm < 0
    rotationAngle = rotationAngle + rotationIncrement;
    dcm = expm(rotationAngle * wskew);
    y = dcm * y0;
    r2 = A + y;
    p2.set_location(Vec3(r2(1),r2(2),r2(3)));
    state = model.initSystem;
    knee.setValue(state,knee.getRangeMax);
    momentArm = path.computeMomentArm(state,knee);
end
fprintf('-semimem insertion needed to be rotated %4.2f deg about %s to ensure semimem remained a knee flexor for the full knee ROM\n',rotationAngle*180/pi,cyl.getName)
if nargin > 1
    fprintf(notes,'-semimem insertion needed to be rotated %4.2f deg about %s to ensure semimem remained a knee flexor for the full knee ROM\n',rotationAngle*180/pi,cyl.getName);
end

model.finalizeConnections;
model.initSystem;

%% LEFT SEMIMEM

% get muscle
semimem = model.getMuscles.get('semimem_l');

% negate z component of r2 for left side
r2_left_Vec3 = Vec3(r2(1),r2(2),-r2(3));

% get insertion point
path = GeometryPath.safeDownCast(semimem.getGeometryPath);
pps = PathPointSet.safeDownCast(path.getPathPointSet);
p2 = PathPoint.safeDownCast(pps.get(1));

% update location
p2.set_location(r2_left_Vec3);

% verify moment arm still > 0 in max knee flexion
state = model.initSystem;
knee = model.getCoordinateSet.get('knee_angle_l');
knee.setValue(state,knee.getRangeMax);
momentArm = path.computeMomentArm(state,knee);
if momentArm < 0; error('-adjustments for right semimem did not successfully ensure left semimem remains a knee flexor for full ROM'); end

model.finalizeConnections;
model.initSystem;

end