function sprinterMuscleModification_glmax3(model, notes)

import org.opensim.modeling.*

% without modification, this muscle flexed the hip when in full extension

fprintf('-glmax3 via point 2 moved back 1.5 cm\n')
if nargin > 1
    fprintf(notes,'-glmax3 via point 2 moved back 1.5 cm\n');
end

model.initSystem;

% move right via point 2 back 1.5 cm
rmuscle = model.getForceSet.get('glmax3_r');
rpath = GeometryPath.safeDownCast(PathActuator.safeDownCast(rmuscle).getGeometryPath);
rpointset = PathPointSet.safeDownCast(rpath.getPathPointSet);
rp2 = PathPoint.safeDownCast(rpointset.get(1));
rp2.set_location(Vec3(rp2.get_location.get(0)-0.015,rp2.get_location.get(1),rp2.get_location.get(2)));

model.finalizeConnections;
model.initSystem;

% move left via point 2 back 1.5 cm
lmuscle = model.getForceSet.get('glmax3_l');
lpath = GeometryPath.safeDownCast(PathActuator.safeDownCast(lmuscle).getGeometryPath);
lpointset = PathPointSet.safeDownCast(lpath.getPathPointSet);
lp2 = PathPoint.safeDownCast(lpointset.get(1));
lp2.set_location(Vec3(lp2.get_location.get(0)-0.015,lp2.get_location.get(1),lp2.get_location.get(2)));

model.finalizeConnections;
model.initSystem;

end