function sprinterMuscleModification_vaslat(model,notes)

import org.opensim.modeling.*

state = model.initSystem;

%% RIGHT VL

% get muscle
vl = model.getMuscles.get('vaslat_r');

% get path points and their frames
path = GeometryPath.safeDownCast(vl.getGeometryPath);
pps = PathPointSet.safeDownCast(path.getPathPointSet);
p = cell(1,pps.getSize);
f = cell(1,pps.getSize);
for k = 0:pps.getSize-1
    p{k+1} = PathPoint.safeDownCast(pps.get(k));
    f{k+1} = Frame.safeDownCast(p{k+1}.getBody);
end

% get relevant coordinate
coord = model.getCoordinateSet.get('knee_angle_r');

% find config where path just touches cyl (search in 1 deg increments)
inc = pi/180;
q = 0;
coord.setValue(state,q);
lentrue = path.getLength(state);
[lenconn,segcurr] = connectpoints(p,f,state);
while abs(lentrue - lenconn) < eps
    segprev = segcurr;
    q = q + inc;
    coord.setValue(state,q);
    lentrue = path.getLength(state);
    [lenconn,segcurr] = connectpoints(p,f,state);
end

% get the points defining the segment which touched the cylinder
[~,segtouch] = max(abs(segcurr - segprev));
p1 = p{segtouch};
f1 = f{segtouch};
p2 = p{segtouch+1};
f2 = f{segtouch+1};
qstart = q - inc;

% create conditional path point active at the config determined above just
% prior to the one for which the MTU touches the cylinder located on the
% line between p1 and p2 at a distance away from the point on that line
% that is closest to the cylinder axis equal to the radius of the cylinder
% and in the direction of p1

% get point locations in f1 at qstart
coord.setValue(state,qstart);
r1Vec3 = p1.get_location;
r2Vec3 = f2.findStationLocationInAnotherFrame(state,p2.get_location,f1);
r1 = [r1Vec3.get(0); r1Vec3.get(1); r1Vec3.get(2)];
r2 = [r2Vec3.get(0); r2Vec3.get(1); r2Vec3.get(2)];

% get spatial properties of relevant wrap cylinder
wrap = path.getWrapSet.get(0); % this assumes there is only one wrap object
cyl = WrapCylinder.safeDownCast(model.getBodySet.get(f1.getName).get_WrapObjectSet.get(wrap.get_wrap_object)); % this assumes cyl body is f1, will throw error if not
c = [cyl.get_translation.get(0); cyl.get_translation.get(1); cyl.get_translation.get(2)];
rad = cyl.get_radius;
R.xyz = [cyl.get_xyz_body_rotation.get(0); cyl.get_xyz_body_rotation.get(1); cyl.get_xyz_body_rotation.get(2)];

% get point (A) on cylinder axis that is a minimum distance from the segment
u = r1 - r2;
u = u / sqrt(u' * u); % unit vector, points from p1 to p2
U = u * u';
B = eye(3) - U;
w = rot(R,[0;0;1],'inverse'); % cylinder axis represented in f1
h = r2 - c; % vector that points from cylinder position to p2
x = h' * B * w / (w' * B * w); % location of point A relative to c along w
A = c + x * w;

% project A onto segment
v = A - r2;
z = r2 + U * v; % point on line p2 to p1 that is closest to cylinder axis

% final point is located a distance = radius away from z in direction +u
rnew = z + rad * u;

% create new conditional path point
pnew = ConditionalPathPoint;
pnew.setName(['vaslat_r-P' num2str(segtouch) 'a'])
pnew.setRangeMin(qstart)
pnew.setRangeMax(pi);
pnew.setCoordinate(coord);
pnew.setParentFrame(PhysicalFrame.safeDownCast(f1));
pnew.getSocket('coordinate').setConnecteePath(coord.getAbsolutePathString);
pnew.getSocket('parent_frame').setConnecteePath(f1.getAbsolutePathString);
pnew.setLocation(Vec3(rnew(1),rnew(2),rnew(3)));

% insert into path point set and finalize
pps.insert(segtouch,pnew);
model.finalizeConnections;
model.initSystem;

fprintf('-vaslat modification:\n')
fprintf('     -touches cylinder at %s = %f deg\n', char(coord.getName), (qstart+inc)*180/pi)
fprintf('     -new conditional path point:\n')
fprintf('           -location in %s: %f, %f, %f\n', char(f1.getName), rnew)
fprintf('           -range (deg): %f, %f\n',qstart*180/pi,180)

if nargin > 1
    fprintf(notes,'-vaslat modification:\n');
    fprintf(notes,'     -touches cylinder at %s = %f deg\n', char(coord.getName), (qstart+inc)*180/pi);
    fprintf(notes,'     -new conditional path point:\n');
    fprintf(notes,'           -location in %s: %f, %f, %f\n', char(f1.getName), rnew);
    fprintf(notes,'           -range (deg): %f, %f\n',qstart*180/pi,180);
end

%% LEFT VL

% get muscle
vl = model.getMuscles.get('vaslat_l');

% get path points and their frames
path = GeometryPath.safeDownCast(vl.getGeometryPath);
pps = PathPointSet.safeDownCast(path.getPathPointSet);

% get relevant coordinate
coord = model.getCoordinateSet.get('knee_angle_l');

% get frame on left side
frname = char(f1.getName);
flname = [frname(1:end-2) '_l'];
fl = PhysicalFrame.safeDownCast(model.getBodySet.get(flname));

% create new conditional path point
pnew = ConditionalPathPoint;
pnew.setName(['vaslat_l-P' num2str(segtouch) 'a'])
pnew.setRangeMin(qstart)
pnew.setRangeMax(pi);
pnew.setCoordinate(coord);
pnew.setParentFrame(PhysicalFrame.safeDownCast(fl));
pnew.getSocket('coordinate').setConnecteePath(coord.getAbsolutePathString);
pnew.getSocket('parent_frame').setConnecteePath(f1.getAbsolutePathString);
pnew.setLocation(Vec3(rnew(1),rnew(2),-rnew(3)));

% insert into path point set and finalize
pps.insert(segtouch,pnew);
model.finalizeConnections;
model.initSystem;

end

function [len,seg] = connectpoints(p,f,state)
len = 0;
a = p{1}.get_location;
seg = zeros(1,length(p)-1);
for k = 2:length(p)
    b = f{k}.findStationLocationInAnotherFrame(state,p{k}.get_location,f{1});
    seg(k-1) = sqrt((a.get(0) - b.get(0))^2 + (a.get(1) - b.get(1))^2 + (a.get(2) - b.get(2))^2 );
    len = len + seg(k-1);
    a = b;
end
end