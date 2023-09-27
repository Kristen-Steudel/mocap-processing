function sprinterMuscleModification_glmax1(model,notes)

import org.opensim.modeling.*

% problematic configuration (SU): 90 flex, -26 add, 40 rot
% problematic configuration: >95 flex, -20 to 5 add, 40 rot

% simple fix used here: reduce radius by 1 cm, reduce second rotation
% (about y-axis) by 33% (from 0.45 rad to 0.3 rad, ~26 deg to ~17 deg),
% translate along cylinder axis by 2.5 cm and back 5 mm along the pelvis x axis

% this works for both configurations above, but note that for full flexion
% and full abduction (with arbitrary rotation), the mtu slightly penetrates
% trochanter. However, cylinder not penetrated (for this, nor any other
% found config), this config rarely would be observed in sprinting, and if
% moment arms ok then that's all that matters ultimately

fprintf('-glmax1 cylinder radius reduced by 1 cm\n')
fprintf('-glmax1 cylinder rotation about y axis set to 17 deg (from ~26 deg, ~30%% reduction)\n')
fprintf('-glmax1 cylinder translated along cyl axis by 2.5 cm and back 5 mm along pelvis x\n')
if nargin > 1
    fprintf(notes,'-glmax1 cylinder radius reduced by 1 cm\n');
    fprintf(notes,'-glmax1 cylinder rotation about y axis set to 17 deg (from ~26 deg, ~30%% reduction)\n');
    fprintf(notes,'-glmax1 cylinder translated along cyl axis by 2.5 cm and back 5 mm along pelvis x\n');
end
model.initSystem;

% get wrapping cylinder
pelvis = model.getBodySet.get('pelvis');
rcylinder = WrapCylinder.safeDownCast(pelvis.get_WrapObjectSet.get('Gmax1_at_pelvis_r'));
C = [rcylinder.get_translation.get(0); rcylinder.get_translation.get(1); rcylinder.get_translation.get(2)];
R.xyz = [rcylinder.get_xyz_body_rotation.get(0); rcylinder.get_xyz_body_rotation.get(1); rcylinder.get_xyz_body_rotation.get(2)];

% reduce cylinder radius by 1 cm
rcylinder.set_radius(rcylinder.get_radius - 0.01);

% set second rotation angle (about y-axis) to 0.3 rad
R.xyz(2) = 0.3;
rcylinder.set_xyz_body_rotation(Vec3(R.xyz(1),R.xyz(2),R.xyz(3)));

% translate along cylinder axis by 2.5 cm and back 5 mm along the pelvis x axis
px = [1;0;0];
cz = rot(R,[0;0;1],'inverse');
Cnew = C - px * 5e-3 + cz * 2.5e-2;
rcylinder.set_translation(Vec3(Cnew(1),Cnew(2),Cnew(3)));

model.finalizeConnections;
model.initSystem;

% left side assuming symmetricity
lcylinder = WrapCylinder.safeDownCast(pelvis.get_WrapObjectSet.get('Gmax1_at_pelvis_l'));
lcylinder.set_radius(rcylinder.get_radius);
lcylinder.set_xyz_body_rotation(Vec3(-R.xyz(1),-R.xyz(2),R.xyz(3)));
lcylinder.set_translation(Vec3(Cnew(1),Cnew(2),-Cnew(3)));

model.finalizeConnections;
model.initSystem;

end