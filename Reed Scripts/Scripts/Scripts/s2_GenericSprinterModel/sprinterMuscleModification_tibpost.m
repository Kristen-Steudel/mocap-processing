function sprinterMuscleModification_tibpost(model,notes)

import org.opensim.modeling.*

% without modification, this muscle dorsiflexed the ankle for all ankle
% angles

fprintf('-tibpost via points 2 and 3 moved back 2 cm, via point 3 moved down 2 cm\n');
if nargin > 1
    fprintf(notes,'-tibpost via points 2 and 3 moved back 2 cm, via point 3 moved down 2 cm\n');
end

model.initSystem;

% move right vp 2 and 3 back 2 cm, and vp 3 down 2 cm
rmuscle = model.getForceSet.get('tibpost_r');
rpath = GeometryPath.safeDownCast(PathActuator.safeDownCast(rmuscle).getGeometryPath);
rpointset = PathPointSet.safeDownCast(rpath.getPathPointSet);
rp2 = PathPoint.safeDownCast(rpointset.get(1));
rp2.set_location(Vec3(rp2.get_location.get(0)-0.02,rp2.get_location.get(1),rp2.get_location.get(2)));
rp3 = PathPoint.safeDownCast(rpointset.get(2));
rp3.set_location(Vec3(rp3.get_location.get(0)-0.02,rp3.get_location.get(1)-0.02,rp3.get_location.get(2)));

% move left vp 2 and 3 back 2 cm, and vp 3 down 2 cm
lmuscle = model.getForceSet.get('tibpost_l');
lpath = GeometryPath.safeDownCast(PathActuator.safeDownCast(lmuscle).getGeometryPath);
lpointset = PathPointSet.safeDownCast(lpath.getPathPointSet);
lp2 = PathPoint.safeDownCast(lpointset.get(1));
lp2.set_location(Vec3(lp2.get_location.get(0)-0.02,lp2.get_location.get(1),lp2.get_location.get(2)));
lp3 = PathPoint.safeDownCast(lpointset.get(2));
lp3.set_location(Vec3(lp3.get_location.get(0)-0.02,lp3.get_location.get(1)-0.02,lp3.get_location.get(2)));

model.finalizeConnections;
model.initSystem;

end