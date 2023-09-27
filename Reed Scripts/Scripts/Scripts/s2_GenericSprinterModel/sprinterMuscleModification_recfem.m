function sprinterMuscleModification_recfem(model,notes)

% give RF its own wrapping cylinder which is exact same as VL's except only
% +x quadrant is active and is rotated about z axis -30 degrees

fprintf('-recfem given own wrapping cylinder, exact same as KnExt_at_fem except only +x quadrant active and rotated -30 deg about z axis\n')
if nargin > 1
    fprintf(notes,'-recfem given own wrapping cylinder, exact same as KnExt_at_fem except only +x quadrant active and rotated -30 deg about z axis\n');
end

import org.opensim.modeling.*

model.initSystem;

%% RIGHT RF

% get RF
rf = model.getMuscles.get('recfem_r');

% get geometry path
path = GeometryPath.safeDownCast(rf.getGeometryPath);

% get wrapping cylinder
rfwrapset = path.getWrapSet.get(0); % this assumes there is only one wrap object
femwrapset = model.getBodySet.get('femur_r').get_WrapObjectSet;
cyl0 = WrapCylinder.safeDownCast(femwrapset.get(rfwrapset.get_wrap_object));

% clone the original cylinder
cylnew = cyl0.clone;

% rename
cylnew.setName('KnExtRF_at_fem_r');

% active quadrant +x
cylnew.set_quadrant('+x');

% rotation about z axis = -30 deg
cylnew.get_xyz_body_rotation.set(2,-30*pi/180);

% insert into femur wrap set
femwrapset.insert(femwrapset.getIndex(rfwrapset.get_wrap_object)+1,cylnew);

% update wrap object for RF geometry path
rfwrapset.setWrapObject(cylnew);

% finish
cylnew.markAdopted;
model.finalizeConnections;
model.initSystem;

clearvars -except model notes

%% LEFT RF

% get RF
rf = model.getMuscles.get('recfem_l');

% get geometry path
path = GeometryPath.safeDownCast(rf.getGeometryPath);

% get wrapping cylinder
rfwrapset = path.getWrapSet.get(0); % this assumes there is only one wrap object
femwrapset = model.getBodySet.get('femur_l').get_WrapObjectSet;
cyl0 = WrapCylinder.safeDownCast(femwrapset.get(rfwrapset.get_wrap_object));

% clone the original cylinder
cylnew = cyl0.clone;

% rename
cylnew.setName('KnExtRF_at_fem_l');

% active quadrant +x
cylnew.set_quadrant('+x');

% rotation about z axis = -30 deg
cylnew.get_xyz_body_rotation.set(2,-30*pi/180);

% insert into femur wrap set
femwrapset.insert(femwrapset.getIndex(rfwrapset.get_wrap_object)+1,cylnew);

% update wrap object for RF geometry path
rfwrapset.setWrapObject(cylnew);

% finish
cylnew.markAdopted;
model.finalizeConnections;
model.initSystem;

end