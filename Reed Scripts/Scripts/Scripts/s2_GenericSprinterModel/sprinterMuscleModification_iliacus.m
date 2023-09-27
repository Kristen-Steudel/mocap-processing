function sprinterMuscleModification_iliacus(model,notes)

import org.opensim.modeling.*

% problematic configuration (SU): -29 flex, -32 add, -36 rot
% problematic configuration: -40 flex, -50 add, 23 rot

% simple fix used here: adjust P2 such that it is moved only radially away
% from the cylinder and s.t. its distance to the cylinder axis is equal to
% the cylinder radius + 1 mm (a 1 mm cushion for numerical reasons)

fprintf('-iliacus via point 2 moved radially away from cylinder s.t. distance to cylinder axis = cylinder radius + 1 mm\n')
if nargin > 1
    fprintf(notes,'-iliacus via point 2 moved radially away from cylinder s.t. distance to cylinder axis = cylinder radius + 1 mm\n');
end

model.initSystem;

% get via points 2, 3, and 4
rmuscle = model.getForceSet.get('iliacus_r');
rpath = GeometryPath.safeDownCast(PathActuator.safeDownCast(rmuscle).getGeometryPath);
rpointset = PathPointSet.safeDownCast(rpath.getPathPointSet);
rp2 = PathPoint.safeDownCast(rpointset.get(1));
P2 = [rp2.get_location.get(0); rp2.get_location.get(1); rp2.get_location.get(2)];

% get wrapping cylinder
pelvis = model.getBodySet.get('pelvis');
rcylinder = WrapCylinder.safeDownCast(pelvis.get_WrapObjectSet.get('IL_at_brim_r'));
C = [rcylinder.get_translation.get(0); rcylinder.get_translation.get(1); rcylinder.get_translation.get(2)];
R.xyz = [rcylinder.get_xyz_body_rotation.get(0); rcylinder.get_xyz_body_rotation.get(1); rcylinder.get_xyz_body_rotation.get(2)];

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

model.finalizeConnections;
model.initSystem;

% get left iliacus and geometry
lmuscle = model.getForceSet.get('iliacus_l');
lpath = GeometryPath.safeDownCast(PathActuator.safeDownCast(lmuscle).getGeometryPath);
lpointset = PathPointSet.safeDownCast(lpath.getPathPointSet);
lp2 = PathPoint.safeDownCast(lpointset.get(1));

% adjust assuming symmetricity
lp2.set_location(Vec3(rp2.get_location.get(0),rp2.get_location.get(1),-rp2.get_location.get(2)));

model.finalizeConnections;
model.initSystem;

end