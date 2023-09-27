function sprinterMuscleModification_psoas(model,notes)

import org.opensim.modeling.*

% problematic configuration (SU): -29 flex, -32 add, -36 rot
% problematic configuration: -40 flex, -50 add, 23 rot

% simple fix used here: increase radius by 5 mm, set cylinder length to 11
% cm adjust P2 such that it is moved only radially away from the cylinder 
% and s.t. its distance to the cylinder axis is equal to the cylinder 
% radius + 1 mm (a 1 mm cushion for numerical reasons), move P3 and P4
% forward 5 mm and down 5 mm

fprintf('-psoas cylinder radius increased 5 mm, via point 2 moved radially away from cylinder s.t. distance to cylinder axis = radius + 1 mm, VP 3 and 4 moved forward and down 5 mm\n')
if nargin > 1
    fprintf(notes,'-psoas cylinder radius increased 5 mm, via point 2 moved radially away from cylinder s.t. distance to cylinder axis = radius + 1 mm, VP 3 and 4 moved forward and down 5 mm\n');
end
model.initSystem;

% get via points 2, 3, and 4
rmuscle = model.getForceSet.get('psoas_r');
rpath = GeometryPath.safeDownCast(PathActuator.safeDownCast(rmuscle).getGeometryPath);
rpointset = PathPointSet.safeDownCast(rpath.getPathPointSet);
rp2 = PathPoint.safeDownCast(rpointset.get(1));
P2 = [rp2.get_location.get(0); rp2.get_location.get(1); rp2.get_location.get(2)];
rp3 = PathPoint.safeDownCast(rpointset.get(2));
P3 = [rp3.get_location.get(0); rp3.get_location.get(1); rp3.get_location.get(2)];
rp4 = PathPoint.safeDownCast(rpointset.get(3));
P4 = [rp4.get_location.get(0); rp4.get_location.get(1); rp4.get_location.get(2)];

% get wrapping cylinder
pelvis = model.getBodySet.get('pelvis');
rcylinder = WrapCylinder.safeDownCast(pelvis.get_WrapObjectSet.get('PS_at_brim_r'));
C = [rcylinder.get_translation.get(0); rcylinder.get_translation.get(1); rcylinder.get_translation.get(2)];
R.xyz = [rcylinder.get_xyz_body_rotation.get(0); rcylinder.get_xyz_body_rotation.get(1); rcylinder.get_xyz_body_rotation.get(2)];

% increase radius by 5 mm, found by adding 2.5 mm incrementally until femoral head covered by cylinder
rcylinder.set_radius(rcylinder.get_radius + 0.005); 

% set cylinder length to 11 cm (to ensure P3 remains within the cylinder
% end faces at max extension and max abduction
rcylinder.set_length(0.11);

% get point on cylinder axis s.t. line segment formed by that point and P2
% is normal to the cylinder axis. This pointed denoted A below.
z = rot(R,[0;0;1],'inverse'); % cylinder axis represented in pelvis frame
A = C + z * dot(P2 - C,z);

% adjust original P2 s.t. it is still on the line defined by A and P2 but
% with the distance from the axis is equal to the cylinder radius. This
% would put P2 exactly on the cylinder surface. Instead increase slightly
% for potential floating point error (e.g., increase by 1 mm)
v = normalize(P2 - A,1,'norm'); % this vector should be orthogonal to u
P2new = A + v * (rcylinder.get_radius + 1e-3); % puts P2 radius + 1 mm away (radially) from cylinder
rp2.set_location(Vec3(P2new(1),P2new(2),P2new(3)));

% move P3 and P4: x forward 5 mm, y down 5 mm, z unchanged
rp3.set_location(Vec3(P3(1) + 5e-3, P3(2) - 5e-3, P3(3)));
rp4.set_location(Vec3(P4(1) + 5e-3, P4(2) - 5e-3, P4(3)));

model.finalizeConnections;
model.initSystem;

% get left psoas and geometry
lmuscle = model.getForceSet.get('psoas_l');
lpath = GeometryPath.safeDownCast(PathActuator.safeDownCast(lmuscle).getGeometryPath);
lpointset = PathPointSet.safeDownCast(lpath.getPathPointSet);
lp2 = PathPoint.safeDownCast(lpointset.get(1));
lp3 = PathPoint.safeDownCast(lpointset.get(2));
lp4 = PathPoint.safeDownCast(lpointset.get(3));
lcylinder = WrapCylinder.safeDownCast(pelvis.get_WrapObjectSet.get('PS_at_brim_l'));

% adjust assuming symmetricity
lp2.set_location(Vec3(rp2.get_location.get(0),rp2.get_location.get(1),-rp2.get_location.get(2)));
lp3.set_location(Vec3(rp3.get_location.get(0),rp3.get_location.get(1),-rp3.get_location.get(2)));
lp4.set_location(Vec3(rp4.get_location.get(0),rp4.get_location.get(1),-rp4.get_location.get(2)));
lcylinder.set_radius(rcylinder.get_radius);
lcylinder.set_length(rcylinder.get_length);

model.finalizeConnections;
model.initSystem;

end