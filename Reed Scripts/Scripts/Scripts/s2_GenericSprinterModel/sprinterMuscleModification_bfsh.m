function sprinterMuscleModification_bfsh(model,notes)

% empty wrapset for bfsh, this wrapping surface pushes against the bfsh
% posteriorly in deep flexion, had minimal effect on momen arm without it
% and caused the bfsh to extend the knee in deep flexion

fprintf('-bfsh cylinder removed\n');
if nargin > 1
    fprintf(notes,'-bfsh cylinder removed\n');
end

import org.opensim.modeling.*

state = model.initSystem;

model.getMuscles.get('bfsh_r').getGeometryPath.deletePathWrap(state,0);
model.getMuscles.get('bfsh_l').getGeometryPath.deletePathWrap(state,0);

model.initSystem;

end