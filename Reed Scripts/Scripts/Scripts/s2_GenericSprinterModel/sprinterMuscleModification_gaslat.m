function sprinterMuscleModification_gaslat(model,notes)

% move GasLat_at_shank inferiorly until no longer penetrated by mtu at max
% knee flexion

import org.opensim.modeling.*

state = model.initSystem;

%% RIGHT GASLAT

% get knee flexion and set to max
knee = model.getCoordinateSet.get('knee_angle_r');
knee.setValue(state,knee.getRangeMax);

% get gaslat
gaslat = model.getMuscles.get('gaslat_r');

% get geometry path and origin/insertion
path = GeometryPath.safeDownCast(gaslat.getGeometryPath);
pps = PathPointSet.safeDownCast(path.getPathPointSet);
p1 = PathPoint.safeDownCast(pps.get(0));
f1 = Frame.safeDownCast(p1.getBody);
p2 = PathPoint.safeDownCast(pps.get(1));
f2 = Frame.safeDownCast(p2.getBody);

% get GasLat_at_shank_r
for k = 0:path.getWrapSet.getSize-1
    if strcmp(path.getWrapSet.get(k).get_wrap_object,'GasLat_at_shank_r')
        cyl = path.getWrapSet.get(k).getWrapObject;
        break;
    end
    if k == path.getWrapSet.getSize-1; error('could not find GasLat_at_shank_r'); end
end

% the mtu length should be longer than the point to point distance
% move cylinder down at 5 mm increments until satisfied
decrement = 0.005;
lenpath = path.getLength(state);
lenp2p = distp2p(p1,f1,p2,f2,state);
total_decrement = 0;
while abs(lenpath - lenp2p) < eps
    
    cyl.get_translation.set(1,cyl.get_translation.get(1) - decrement);
    total_decrement = total_decrement + decrement;
    
    state = model.initSystem;
    knee.setValue(state,knee.getRangeMax);
    lenpath = path.getLength(state);
    lenp2p = distp2p(p1,f1,p2,f2,state);
    
end
fprintf('-cylinder GasLat_at_shank_r moved inferiorly %4.2f cm\n',total_decrement*100);
if nargin > 1
    fprintf(notes,'-cylinder GasLat_at_shank_r moved inferiorly %4.2f cm\n',total_decrement*100);
end

% finish
model.finalizeConnections;
model.initSystem;

%% LEFT GASLAT

% get gaslat
gaslat = model.getMuscles.get('gaslat_l');

% get geometry path and origin/insertion
path = GeometryPath.safeDownCast(gaslat.getGeometryPath);
pps = PathPointSet.safeDownCast(path.getPathPointSet);
p1 = PathPoint.safeDownCast(pps.get(0));
f1 = Frame.safeDownCast(p1.getBody);
p2 = PathPoint.safeDownCast(pps.get(1));
f2 = Frame.safeDownCast(p2.getBody);

% get GasLat_at_shank_l
for k = 0:path.getWrapSet.getSize-1
    if strcmp(path.getWrapSet.get(k).get_wrap_object,'GasLat_at_shank_l')
        cyl = path.getWrapSet.get(k).getWrapObject;
        break;
    end
    if k == path.getWrapSet.getSize-1; error('could not find GasLat_at_shank_l'); end
end

% move inferiorly by total decrement determined above
cyl.get_translation.set(1,cyl.get_translation.get(1) - total_decrement);

% get knee flexion and set to max
state = model.initSystem;
knee = model.getCoordinateSet.get('knee_angle_l');
knee.setValue(state,knee.getRangeMax);

% verify cylinder not penetrated
lenpath = path.getLength(state);
lenp2p = distp2p(p1,f1,p2,f2,state);
if abs(lenpath - lenp2p) < eps
    error('cylinder penetrated for left gaslat using cylinder modifications that successfully disallowed penetration for right gaslat');
end

% finish
model.finalizeConnections;
model.initSystem;

end

function len = distp2p(p1,f1,p2,f2,state)

r1 = p1.get_location;
r2 = f2.findStationLocationInAnotherFrame(state,p2.get_location,f1);
len = sqrt((r1.get(0) - r2.get(0))^2 + (r1.get(1) - r2.get(1))^2 + (r1.get(2) - r2.get(2))^2 );

end