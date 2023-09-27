function muscles = osimCalculateMTUGeometry(model,coordinates,muscles)

% Reed Gurchiek, 2021
% calculate mtu length and moment arm

% INPUT
% model - OpenSim Model object
% coordinates - struct, fields are coordinate names and are 1xn arrays of coordinate values
% muscles - struct, fields are structs and fieldnames are muscle names,
% these structs have one field when input and contain a cell array called
% coordinatesActuated which lists the coordinate names it actuates

% OUTPUT
% muscles - same muscles input struct but now with an mtuLength field with
% values corresponding to the mtu length at the coordinate values given in
% the coordinates input struct and with a momentArm field which contains
% fieldnames for each coordinate named in that muscles coordinatesActuated
% cell array

% EXAMPLE
% INPUT
%   coordinates.knee_angle_r = [0 pi/2]
%              .ankle_angle_r = [0 pi/4]
%   muscles.soleus_r.coordinatesActuated = {'ankle_angle_r'}, could do knee_angle_r here but we know this is zero
%          .vaslat_r.coordinatesActuated = {'knee_angle_r'}
%          .gastroc_r.coordinatesActuated = {'knee_angle_r','ankle_angle_r'}
% OUTPU
%   muscles.soleus_r.coordinatesActuated = {'ankle_angle_r'}
%                   .mtuLength = [l1 l2]
%                   .momentArm.ankle_angle_r ] [r1 r2]
%          .vaslat_r.coordinatesActuated = {'knee_angle_r'}
%                   .mtuLength = [l1 l2]
%                   .momentArm.knee_angle_r = [r1 r2]
%          .gastroc_r.coordinatesActuated = {'knee_angle_r','ankle_angle_r'}
%                    .mtuLength = [l1 l2]
%                    .momentArm.knee_angle_r = [r1 r2]
%                              .ankle_angle_r = [r1 r2]


import org.opensim.modeling.*

if ischar(model); model = Model(model); end

% initialize output fields
coordinate_names = fieldnames(coordinates);
muscle_names = fieldnames(muscles);
n = length(coordinates.(coordinate_names{1}));
for k = 1:length(muscle_names)
    muscles.(muscle_names{k}).mtuLength = zeros(1,n);
    for j = 1:length(muscles.(muscle_names{k}).coordinatesActuated)
        muscles.(muscle_names{k}).momentArm.(muscles.(muscle_names{k}).coordinatesActuated{j}) = zeros(1,n);
    end
end

% get reference to state, muscle geometry paths, and coordinates (those
% with values given and all those actuated by the specified muscles)
state = model.initSystem;
for k = 1:length(coordinate_names)
    osimCoordinates.(coordinate_names{k}) = model.getCoordinateSet.get(coordinate_names{k});
end
for k = 1:length(muscle_names)
    temp = model.getForceSet.get(muscle_names{k}); % temporary reference to muscle
    osimGeometryPaths.(muscle_names{k}) = eval(temp.getConcreteClassName.toCharArray').safeDownCast(temp);
    for j = 1:length(muscles.(muscle_names{k}).coordinatesActuated)
        if ~isfield(osimCoordinates,muscles.(muscle_names{k}).coordinatesActuated{j})
            osimCoordinates.(muscles.(muscle_names{k}).coordinatesActuated{j}) = model.getCoordinateSet.get(muscles.(muscle_names{k}).coordinatesActuated{j});
        end
    end
end


% for each configuration
for k = 1:n
    
    currstr = sprintf('-calculating mtu geometry for configuration %d/%d',k,n);
    fprintf(currstr);
    
    % set configuration
    for j = 1:length(coordinate_names)
        osimCoordinates.(coordinate_names{j}).setValue(state,coordinates.(coordinate_names{j})(k))
    end
    
    % for each muscle
    for j = 1:length(muscle_names)
        
        % get length
        muscles.(muscle_names{j}).mtuLength(k) = osimGeometryPaths.(muscle_names{j}).getLength(state);
        
        % for each coordinate actuated
        for i = 1:length(muscles.(muscle_names{j}).coordinatesActuated)
            
            % get moment arm
            muscles.(muscle_names{j}).momentArm.(muscles.(muscle_names{j}).coordinatesActuated{i})(k) = osimGeometryPaths.(muscle_names{j}).computeMomentArm(state,osimCoordinates.(muscles.(muscle_names{j}).coordinatesActuated{i}));
            
        end
        
    end
    
    fprintf(repmat('\b',[1 length(currstr)]))

end

end
