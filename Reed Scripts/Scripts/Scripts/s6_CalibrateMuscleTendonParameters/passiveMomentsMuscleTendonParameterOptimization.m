function [model,notes] = passiveMomentsMuscleTendonParameterOptimization(model,configuration,parameterBounds,specialParameterBounds,stateBounds,weights,rightLigamentMoment,rightKneeThighShankContactMoment,minLengthMTU,maxLengthMTU,maxShorteningVelocityMTU,muscleGroups,notes,resultsDir)

% Riener and Edrich, section 4: best fit for ankle, largest error for knee
% moment at 0 deg knee flexion + 45 hip flexion, largest deviation at hip
% joint observed when knee flexed to 90 deg, model error increases at
% extremes of joint motion

% use of min/max mtu lengths:
% we will impose constraints ensuring that (1) muscle-tendon equilibrium is
% feasible (within user set bounds for allowable norm fiber lengths) at
% maxLengthMTU under exactly zero activation (worst-case for violating the
% long fiber bound) and (2) ensuring that muscle-teond equilibrium is
% feasible at minLengthMTU under maximum activation (worst-case for
% violating the short fiber bound)

% objective/constraint gradients checked against central-difference based
% numerical approximations with 1e-3 step size and 1e-3 tolerance (in
% MATLAB's validateFirstDerivatives)

% v3 includes tendon strain at one norm force as a tunable parameter

import org.opensim.modeling.*

%% INITIALIZE MODEL

% get reference to necessary model coordinates
hip = model.getCoordinateSet.get('hip_flexion_r');
knee = model.getCoordinateSet.get('knee_angle_r');
ankle = model.getCoordinateSet.get('ankle_angle_r');

% set to default pose
state = model.initSystem;
hip.setValue(state,0.0);
knee.setValue(state,0.0);
ankle.setValue(state,0.0);

% turn off all muscles
for k = 0:model.getMuscles.getSize-1
    model.getMuscles.get(k).setActivation(state,0.0);
end

%% CONFIGURATIONS

% unpack configurations to optimize over
hip_flexion_r = configuration.hip_flexion_r;
knee_angle_r = configuration.knee_angle_r;
ankle_angle_r = configuration.ankle_angle_r;

% get num configurations
num_hip_angles = length(hip_flexion_r);
num_knee_angles = length(knee_angle_r);
num_ankle_angles = length(ankle_angle_r);
if num_hip_angles ~= num_knee_angles || num_hip_angles ~= num_ankle_angles
    error('Must have same number of coordinate values for hip, knee, and ankle angles')
end
num_configurations = num_hip_angles;
fprintf('-num configurations considered for matching passive moments: %d\n',num_configurations);
fprintf(notes,'-num configurations considered for matching passive moments: %d\n',num_configurations);

%% ORIGINAL MUSCLE-TENDON PARAMETERS

% allocation
num_muscles = model.getMuscles.getSize;
muscle_names = cell(num_muscles,1);

% original muscle parameters
originalParams.tendon_slack_length = zeros(num_muscles,1); % tendon slack length
originalParams.optimal_fiber_length = zeros(num_muscles,1); % optimal fiber length
originalParams.passive_fiber_strain_at_one_norm_force = zeros(num_muscles,1); % fiber strain at one norm force
originalParams.pennation_angle_at_optimal = zeros(num_muscles,1); % pennation at optimal fiber length
originalParams.max_isometric_force = zeros(num_muscles,1); % max isometric force
originalParams.max_contraction_velocity = zeros(num_muscles,1); % max contraction velocity
originalParams.tendon_strain_at_one_norm_force = zeros(num_muscles,1);
originalParams.tendonForceLengthParameter = zeros(num_muscles,1);

% for each muscle in the model
i = 1;
for k = 1:model.getMuscles.getSize
    
    % get muscle reference
    thisMuscle = DeGrooteFregly2016Muscle.safeDownCast(model.getMuscles.get(k-1));
    
    % consider if on right side
    if endsWith(thisMuscle.getName,'_r')
        
        % store properties
        muscle_names{i} = char(thisMuscle.getName);
        originalParams.tendon_slack_length(i) = thisMuscle.get_tendon_slack_length;
        originalParams.optimal_fiber_length(i) = thisMuscle.get_optimal_fiber_length;
        originalParams.passive_fiber_strain_at_one_norm_force(i) = thisMuscle.get_passive_fiber_strain_at_one_norm_force;
        originalParams.pennation_angle_at_optimal(i) = thisMuscle.get_pennation_angle_at_optimal;
        originalParams.max_isometric_force(i) = thisMuscle.get_max_isometric_force;
        originalParams.max_contraction_velocity(i) = thisMuscle.get_max_contraction_velocity;
        originalParams.tendon_strain_at_one_norm_force(i) = thisMuscle.get_tendon_strain_at_one_norm_force;
        originalParams.tendonForceLengthParameter(i) = log(6) / originalParams.tendon_strain_at_one_norm_force(i);
        
        % increment counter
        i = i+1;
        
    % otherwise delete instance
    else
        muscle_names(i) = [];
        originalParams.tendon_slack_length(i) = [];
        originalParams.optimal_fiber_length(i) = [];
        originalParams.passive_fiber_strain_at_one_norm_force(i) = [];
        originalParams.pennation_angle_at_optimal(i) = [];
        originalParams.max_isometric_force(i) = [];
        originalParams.max_contraction_velocity(i) = [];
        originalParams.tendon_strain_at_one_norm_force(i) = [];
        originalParams.tendonForceLengthParameter(i) = [];
        num_muscles = num_muscles - 1;
    end
end

%% UNPACK MIN/MAX MTU LENGTH RANGE

% these used as a constraint: we require muscle-tendon equilibrium at the
% longest mtu length with no activation and at the shortest mtu length with
% full activation without violating the normalized fiber length bounds

% init
min_mtu_length = zeros(num_muscles,1);
max_mtu_length = zeros(num_muscles,1);

% if none given (min/max LengthMTU = [] or [] for any particular muscle)
% then the min_mtu_length and max_mtu_length are both set to the mtu length
% in the default pose
if isempty(minLengthMTU)
    for k = 1:num_muscles
        minLengthMTU.(muscle_names{k}) = [];
    end
end
if isempty(maxLengthMTU)
    for k = 1:num_muscles
        maxLengthMTU.(muscle_names{k}) = [];
    end
end
for k = 1:num_muscles
    
    % set min/max mtu length
    min_mtu_length(k) = minLengthMTU.(muscle_names{k});
    max_mtu_length(k) = maxLengthMTU.(muscle_names{k});
    
    % if empty, set to default
    if isempty(min_mtu_length(k))
        thisMuscle = DeGrooteFregly2016Muscle.safeDownCast(model.getMuscles.get(muscle_names{k}));
        min_mtu_length(k) = thisMuscle.getLength(state);
    end
    if isempty(max_mtu_length(k))
        thisMuscle = DeGrooteFregly2016Muscle.safeDownCast(model.getMuscles.get(muscle_names{k}));
        max_mtu_length(k) = thisMuscle.getLength(state);
    end
    
end

%% UNPACK MAX MTU SHORTENING VELOCITY

% if none given, set all to 0
max_mtu_shortening_velocity = zeros(num_muscles,1);
if ~isempty(maxShorteningVelocityMTU)
    for k = 1:num_muscles
        if ~isempty(maxShorteningVelocityMTU.(muscle_names{k}))
            max_mtu_shortening_velocity(k) = maxShorteningVelocityMTU.(muscle_names{k});
        end
        if max_mtu_shortening_velocity(k) > 0; error('maxShorteningVelocityMTU must be < 0'); end

    end
end

%% MAKE SURE SPECIAL BOUNDS ARE RIGHT MUSCLE ONLY

if ~isempty(specialParameterBounds)
    
    % get muscles with special bounds
    specialBoundMuscles = fieldnames(specialParameterBounds);
    
    % change to right if specified for left unless bound already exists for
    % right
    k = 1;
    while k <= length(specialBoundMuscles)
        if endsWith(specialBoundMuscles{k},'_l')
            if any(strcmp(specialBoundMuscles,[specialBoundMuscles{k}(1:end-2) '_r']))
                
                % report
                fprintf('-special bound provided for %s and %s. Only right muscles are optimized (symmetry assumed). Using the bounds specified for the right muscle\n',specialBoundMuscles{k},[specialBoundMuscles{k}(1:end-2) '_r'])
                fprintf(notes,'-special bound provided for %s and %s. Only right muscles are optimized (symmetry assumed). Using the bounds specified for the right muscle\n',specialBoundMuscles{k},[specialBoundMuscles{k}(1:end-2) '_r']);
                
                % remove _l bound from struct and name from cell array
                specialParameterBounds = rmfield(specialParameterBounds,specialBoundMuscles{k});
                specialBoundMuscles(k) = [];
                
            else
                
                % report
                fprintf('-special bound provided for %s. Only right muscles are optimized (symmetry assumed). Applying bound to right-side muscle\n',specialBoundMuscles{k})
                fprintf(notes,'-special bound provided for %s. Only right muscles are optimized (symmetry assumed). Applying bound to right-side muscle\n',specialBoundMuscles{k});
                
                % change name in cell array: _l to _r
                specialBoundMuscles{k} = [specialBoundMuscles{k}(1:end-2) '_r'];
                
                % duplicate _l entry as _r in struct
                specialBoundsMuscles.(specialBoundMuscles{k}) = specialBoundsMuscles.([specialBoundMuscles{k}(1:end-1) '_l']);
                
                % remove _l entry
                specialBoundsMuscles = rmfield(specialBoundsMuscles,[specialBoundMuscles{k}(1:end-1) '_l']);
                k = k+1;
                
            end
        else
            k=k+1;
        end
    end
end

%% IMPLEMENT VALUE SPECIAL BOUNDS AS PERCENT CHANGE SPECIAL BOUNDS

if ~isempty(specialParameterBounds)
    
    % get muscles with special bounds
    specialBoundMuscles = fieldnames(specialParameterBounds);
    
    % for each specially bounded muscle
    for k = 1:length(specialBoundMuscles)
        
        % for each bound
        thisMuscleName = specialBoundMuscles{k};
        thisMuscleIndex = strcmp(muscle_names,thisMuscleName);
        thisMuscleBounds = specialParameterBounds.(thisMuscleName);
        thisMuscleBoundNames = fieldnames(thisMuscleBounds);
        for j = 1:length(thisMuscleBoundNames)
                
            % get lower/upper bounds
            thisBoundName = thisMuscleBoundNames{j};
            thisLB = thisMuscleBounds.(thisBoundName)(1);
            thisUB = thisMuscleBounds.(thisBoundName)(2);
            
            % if value bound
            if startsWith(thisMuscleBoundNames{j},'value_')

                % get property
                thisPropertyName = replace(thisBoundName,'value_','');

                % get original parameter value
                thisOriginalParam = originalParams.(thisPropertyName)(thisMuscleIndex);
                
                % get lower/upper percent changes
                thisLB_pc = (thisLB - thisOriginalParam) / thisOriginalParam;
                thisUB_pc = (thisUB - thisOriginalParam) / thisOriginalParam;
                
                % set in special bounds struct and remove value_ bound
                specialParameterBounds.(thisMuscleName).(['percent_change_' thisPropertyName]) = [thisLB_pc, thisUB_pc];
                specialParameterBounds.(thisMuscleName) = rmfield(specialParameterBounds.(thisMuscleName),['value_' thisPropertyName]);
                fprintf('-%s special bound on %s was value based [%f, %f], now implemented as percent change bound [%f, %f]\n',...
                    thisMuscleName, thisPropertyName, thisLB, thisUB, thisLB_pc, thisUB_pc);
                fprintf(notes,'-%s special bound on %s was value based [%f, %f], now implemented as percent change bound [%f, %f]\n',...
                    thisMuscleName, thisPropertyName, thisLB, thisUB, thisLB_pc, thisUB_pc);
               
            % if percent change
            else
                if ~startsWith(thisMuscleBoundNames{j},'percent_change_'); error('all bounds must be either percent_change_ or value_'); end

                % get property
                thisPropertyName = replace(thisBoundName,'percent_change_','');

                % get original parameter value
                thisOriginalParam = originalParams.(thisPropertyName)(thisMuscleIndex);
                
                % report
                fprintf('-%s special bound on %s: [%f, %f] percent change (as a value bound: [%f, %f]\n',...
                    thisMuscleName,thisPropertyName,thisLB,thisUB,(1+thisLB)*thisOriginalParam,(1+thisUB)*thisOriginalParam);
                fprintf(notes,'-%s special bound on %s: [%f, %f] percent change (as a value bound: [%f, %f]\n',...
                    thisMuscleName,thisPropertyName,thisLB,thisUB,(1+thisLB)*thisOriginalParam,(1+thisUB)*thisOriginalParam);
                
            end
        end
    end
end

%% IMPLEMENT VALUE BOUNDS AS PERCENT CHANGE SPECIAL BOUNDS

% for each bound
boundNames = fieldnames(parameterBounds);
for k = 1:length(boundNames)
        
    % get lower and upper bound
    thisBoundName = boundNames{k};
    thisLB = parameterBounds.(thisBoundName)(1);
    thisUB = parameterBounds.(thisBoundName)(2);
    
    % if value bound
    if startsWith(thisBoundName,'value_')
        if thisLB > thisUB; error('lower bounds must be <= upper bounds'); end

        % get muscle property
        thisPropertyName = replace(thisBoundName,'value_','');
        fprintf('-generic bound placed on %s [%f, %f] will now be implemented as a percent change special bound\n',thisPropertyName,thisLB,thisUB);
        fprintf(notes,'-generic bound placed on %s [%f, %f] will now be implemented as a percent change special bound\n',thisPropertyName,thisLB,thisUB);

        % for each muscle
        for j = 1:num_muscles

            % if special bound already given for this muscle and property,
            % then ignore the general bound, continue loop
            if ~isempty(specialParameterBounds)
                if isfield(specialParameterBounds,muscle_names{j})
                    if isfield(specialParameterBounds.(muscle_names{j}),['percent_change_' thisPropertyName])
                        fprintf('     -bound %s on %s NOT IMPLEMENTED since a special bound already exists for this muscle and property\n',thisPropertyName,muscle_names{j});
                        fprintf(notes,'     -bound %s on %s NOT IMPLEMENTED since a special bound already exists for this muscle and property\n',thisPropertyName,muscle_names{j});
                        continue;
                    end
                end
            end

            % get original parameter value
            thisOriginalParam = originalParams.(thisPropertyName)(j);

            % get lower/upper percent changes
            thisLB_pc = (thisLB - thisOriginalParam) / thisOriginalParam;
            thisUB_pc = (thisUB - thisOriginalParam) / thisOriginalParam;

            % set in special bounds struct
            specialParameterBounds.(muscle_names{j}).(['percent_change_' thisPropertyName]) = [thisLB_pc, thisUB_pc];
            fprintf('     -%s: implementation as a percent change special bound: [%f, %f]\n',muscle_names{j},thisLB_pc,thisUB_pc);
            fprintf(notes,'     -%s: implementation as a percent change special bound: [%f, %f]\n',muscle_names{j},thisLB_pc,thisUB_pc);

        end

        % remove this value bound from bound struct
        parameterBounds = rmfield(parameterBounds,thisBoundName);

        % replace with percent_change_ bound set to [nan,nan]
        % will be replaced with special bounds later
        parameterBounds.(['percent_change_' thisPropertyName]) = [nan,nan];

    % if percent change bound
    else
        if ~startsWith(thisBoundName,'percent_change_'); error('all bounds must be either percent_change_ or value_'); end

        % get muscle property
        thisPropertyName = replace(thisBoundName,'percent_change_','');

        % report
        fprintf('-generic bound on %s: [%f, %f] percent change\n',thisPropertyName,thisLB,thisUB);
        fprintf(notes,'-generic bound on %s: [%f, %f] percent change\n',thisPropertyName,thisLB,thisUB);

    end
    
end

%% GET PASSIVE MOMENTS DUE TO NON MUSCLE TISSUE FROM MODEL

% ankle moment assumed to be a bushing force (as in DeMers 2017)
% rest are expression based coordinate forces

% hip ligament moment
model_ligament_moment.hip = false;
if ~isempty(rightLigamentMoment.hip)
    hip_ligament_moment = model.getForceSet.get(rightLigamentMoment.hip);
    hip_ligament_moment = eval(char(hip_ligament_moment.getConcreteClassName)).safeDownCast(hip_ligament_moment);
    model_ligament_moment.hip = true;
end

% knee ligament moment
model_ligament_moment.knee = false;
if ~isempty(rightLigamentMoment.knee)
    knee_ligament_moment = model.getForceSet.get(rightLigamentMoment.knee);
    knee_ligament_moment = eval(char(knee_ligament_moment.getConcreteClassName)).safeDownCast(knee_ligament_moment);
    model_ligament_moment.knee = true;
end

% ankle ligament moment
model_ligament_moment.ankle = false;
if ~isempty(rightLigamentMoment.ankle)
    ankle_ligament_moment = model.getForceSet.get(rightLigamentMoment.ankle);
    ankle_ligament_moment = eval(char(ankle_ligament_moment.getConcreteClassName)).safeDownCast(ankle_ligament_moment);
    model_ligament_moment.ankle = true;
end

% knee contact moment
model_contact_moment.knee = false;
if ~isempty(rightKneeThighShankContactMoment)
    knee_thighShankContact_moment = model.getForceSet.get(rightKneeThighShankContactMoment);
    knee_thighShankContact_moment = eval(char(knee_thighShankContact_moment.getConcreteClassName)).safeDownCast(knee_thighShankContact_moment);
    model_contact_moment.knee = true;
end

%% GET CONSTANT MTU LENGTH, MOMENT ARM, AND PASSIVE MOMENTS IN EACH CONFIGURATION TO PASS TO OPTIMIZER

% allocation
mtu_length = zeros(num_muscles,num_configurations); % mtu length in each config
moment_arm = zeros(num_muscles,num_configurations,3); % moment arm in each config
passive_moment.net_experimental = zeros(3,num_configurations); % net passive moment in each config: row1 = hip, row2 = knee, row3 = ankle
passive_moment.ligament = zeros(3,num_configurations); % moment due to ligaments
passive_moment.contact = zeros(3,num_configurations); % moment due to segment contact
original_norm_fiber_length = zeros(num_muscles,num_configurations); % fiber lengths from initial params: fsolve(ft - fm)

% osim validation for initial params
original_norm_fiber_length_osim = zeros(num_muscles,num_configurations); % equilibrating in each config
original_norm_tendon_force_osim = zeros(num_muscles,num_configurations);
passive_moment.original_muscle_estimate_osim = zeros(3,num_configurations);

% for each configuration
wait_bar = waitbar(0,sprintf('Configuration 0 of %d',num_configurations));
for c = 1:num_configurations
            
    waitbar(c/num_configurations,wait_bar,sprintf('Configurations %d of %d',c,num_configurations))

    % set model configuration
    hip.setValue(state,hip_flexion_r(c));
    knee.setValue(state,knee_angle_r(c));
    ankle.setValue(state,ankle_angle_r(c));

    % equilibrate muscles
    model.equilibrateMuscles(state);

    % get net passive moment
    passive_moment.net_experimental(1,c) = passiveHipMomentRienerEdrich1999(hip_flexion_r(c),knee_angle_r(c));
    passive_moment.net_experimental(2,c) = passiveKneeMomentRienerEdrich1999(hip_flexion_r(c),knee_angle_r(c),ankle_angle_r(c));
    passive_moment.net_experimental(3,c) = passiveAnkleMomentRienerEdrich1999(knee_angle_r(c),ankle_angle_r(c));

    % get moment due to ligaments
    if model_ligament_moment.hip;   passive_moment.ligament(1,c) = hip_ligament_moment.calcExpressionForce(state); end
    if model_ligament_moment.knee;  passive_moment.ligament(2,c) = knee_ligament_moment.calcExpressionForce(state); end
    if model_ligament_moment.ankle; passive_moment.ligament(3,c) = ankle_ligament_moment.calcStiffnessForce(state).get(2); end

    % get moment due to segment contact
    if model_contact_moment.knee;   passive_moment.contact(2,c) = knee_thighShankContact_moment.calcExpressionForce(state); end

    % for each muscle
    for m = 1:num_muscles

        % compute moment arm and mtu length
        thisMuscle = DeGrooteFregly2016Muscle.safeDownCast(model.getMuscles.get(muscle_names{m}));
        mtu_length(m,c) = thisMuscle.getLength(state);
        moment_arm(m,c,1) = thisMuscle.computeMomentArm(state,hip);
        moment_arm(m,c,2) = thisMuscle.computeMomentArm(state,knee);
        moment_arm(m,c,3) = thisMuscle.computeMomentArm(state,ankle);
        original_norm_fiber_length_osim(m,c) = thisMuscle.getNormalizedFiberLength(state);
        original_norm_fiber_length(m,c) = solveMuscleTendonEquilibrium(original_norm_fiber_length_osim(m,c),mtu_length(m,c),originalParams,m,'off');

        % osim estimate of passive moment due to muscles from original muscle model
        original_norm_tendon_force_osim(m,c) = thisMuscle.getTendonForce(state) / originalParams.max_isometric_force(m);
        for h = 1:3 % for each coordinate: hip, knee, ankle
            passive_moment.original_muscle_estimate_osim(h,c) = passive_moment.original_muscle_estimate_osim(h,c) + moment_arm(m,c,h) * original_norm_tendon_force_osim(m,c) * originalParams.max_isometric_force(m);
        end

    end
            
end
close(wait_bar)

% get the moment that must be contributed by the muscles, this is value the
% optimizer will try to match
passive_moment.muscle = passive_moment.net_experimental - passive_moment.ligament - passive_moment.contact;

%% OPTIMIZE

% decision variables
xlmn = original_norm_fiber_length(:); % normalized fiber length
xlmn_max_mtu = ones(num_muscles,1); % normalized fiber length at max mtu length
xlmn_min_mtu = ones(num_muscles,1); % normalized fiber length at min mtu length
xlts = ones(num_muscles,1); % tendon slack length
xlm0 = ones(num_muscles,1); % optimal fiber length
xfm0 = ones(num_muscles,1); % max isometric force
xe0 = ones(num_muscles,1); % tendon strain at F0
xpen0 = ones(num_muscles,1); % pennation angle at optimal fiber length
xvm0 = ones(num_muscles,1); % max contraction velocity
xs0 = ones(num_muscles,1); % muscle strain at F0

% initial guess: lmn = 1, scale factors = 1 
x0 = [xlmn; xlmn_max_mtu; xlmn_min_mtu; xlts; xlm0; xfm0; xe0; xpen0; xvm0; xs0];

% indices: the first N correspond the normalized fiber length for each
% muscle and each configuration
indices.xlmn = 1 : num_muscles * num_configurations;
indices.xlmn_max_mtu_length = indices.xlmn(end) + 1 : indices.xlmn(end) + num_muscles;
indices.xlmn_min_mtu_length = indices.xlmn_max_mtu_length(end) + 1 : indices.xlmn_max_mtu_length(end) + num_muscles;
indices.xlts = indices.xlmn_min_mtu_length(end) + 1 : indices.xlmn_min_mtu_length(end) + num_muscles;
indices.xlm0 = indices.xlts(end) + 1 : indices.xlts(end) + num_muscles;
indices.xfm0 = indices.xlm0(end) + 1 : indices.xlm0(end) + num_muscles;
indices.xe0 = indices.xfm0(end) + 1 : indices.xfm0(end) + num_muscles;
indices.xpen0 = indices.xe0(end) + 1 : indices.xe0(end) + num_muscles;
indices.xvm0 = indices.xpen0(end) + 1 : indices.xpen0(end) + num_muscles;
indices.xs0 =  indices.xvm0(end) + 1 : length(x0);

% lower bounds
lb = zeros(length(x0),1);
lb(indices.xlmn) = stateBounds.norm_fiber_length(1);
lb(indices.xlmn_max_mtu_length) = stateBounds.norm_fiber_length(1);
lb(indices.xlmn_min_mtu_length) = stateBounds.norm_fiber_length(1);
lb(indices.xlts) = 1 + parameterBounds.percent_change_tendon_slack_length(1);
lb(indices.xlm0) = 1 + parameterBounds.percent_change_optimal_fiber_length(1);
lb(indices.xfm0) = 1 + parameterBounds.percent_change_max_isometric_force(1);
lb(indices.xe0) = 1 + parameterBounds.percent_change_tendon_strain_at_one_norm_force(1);
lb(indices.xpen0) = 1 + parameterBounds.percent_change_pennation_angle_at_optimal(1);
lb(indices.xvm0) = 1 + parameterBounds.percent_change_max_contraction_velocity(1);
lb(indices.xs0) = 1 + parameterBounds.percent_change_passive_fiber_strain_at_one_norm_force(1);

% upper bounds
ub = zeros(length(x0),1);
ub(indices.xlmn) = stateBounds.norm_fiber_length(2);
ub(indices.xlmn_max_mtu_length) = stateBounds.norm_fiber_length(2);
ub(indices.xlmn_min_mtu_length) = stateBounds.norm_fiber_length(2);
ub(indices.xlts) = 1 + parameterBounds.percent_change_tendon_slack_length(2);
ub(indices.xlm0) = 1 + parameterBounds.percent_change_optimal_fiber_length(2);
ub(indices.xfm0) = 1 + parameterBounds.percent_change_max_isometric_force(2);
ub(indices.xe0) = 1 + parameterBounds.percent_change_tendon_strain_at_one_norm_force(2);
ub(indices.xpen0) = 1 + parameterBounds.percent_change_pennation_angle_at_optimal(2);
ub(indices.xvm0) = 1 + parameterBounds.percent_change_max_contraction_velocity(2);
ub(indices.xs0) = 1 + parameterBounds.percent_change_passive_fiber_strain_at_one_norm_force(2);

% special bounds
boundNames = fieldnames(parameterBounds);
if ~isempty(specialParameterBounds)
    
    % get muscles with special bounds
    specialBoundMuscles = fieldnames(specialParameterBounds);
    
    % for each muscle
    for k = 1:length(specialBoundMuscles)
        
        % get index for this muscle
        thisMuscleName = specialBoundMuscles{k};
        thisMuscleIndex = strcmp(muscle_names,thisMuscleName);
        
        % get these bounds
        theseBounds = specialParameterBounds.(thisMuscleName);
        
        % for each bound name
        for j = 1:length(boundNames)
        
            % if available for update
            thisBoundName = boundNames{j};
            if isfield(theseBounds,thisBoundName)
                
                % get bounds
                thisBound = theseBounds.(thisBoundName);
                
                % error if only 1 given
                if length(thisBound) ~= 2; error('special [lower, upper] bounds must both be given. If only one is special, set the other to nan'); end
                
                % scale factor abbreviation
                if contains(thisBoundName,'tendon_slack')
                    scaleFactorAbb = 'xlts';
                elseif contains(thisBoundName,'optimal_fiber')
                    scaleFactorAbb = 'xlm0';
                elseif contains(thisBoundName,'max_isometric')
                    scaleFactorAbb = 'xfm0';
                elseif contains(thisBoundName,'tendon_strain')
                    scaleFactorAbb = 'xe0';
                elseif contains(thisBoundName,'pennation')
                    scaleFactorAbb = 'xpen0';
                elseif contains(thisBoundName,'velocity')
                    scaleFactorAbb = 'xvm0';
                elseif contains(thisBoundName,'fiber_strain')
                    scaleFactorAbb = 'xs0';
                end
                
                % lower bound?
                if ~isnan(thisBound(1))
                    lb(indices.(scaleFactorAbb)(thisMuscleIndex)) = 1 + thisBound(1);
                end
                
                % upper bound?
                if ~isnan(thisBound(2))
                    ub(indices.(scaleFactorAbb)(thisMuscleIndex)) = 1 + thisBound(2);
                end
                
            end
            
        end
        
    end
            
end

% sanity check
if any(isnan(ub)); error('this should not happen'); end
if any(isnan(lb)); error('this should not happen'); end

% muscle group constraints
% muscles in a group share the same scale factors
% muscle groups is a cell array of cell arrays of chars listing the muscles
% in each group
% each group of muscles must obviously be more than one
% the constraints are implemented as muscle1 - muscle2, muscle1 - muscle3,
% ..., muscle1 - muscleN, s.t. the number of constraints added for each
% group is equal to (N - 1) * 7 where 7 is the number of scale factors
% muscle groups are recognized by the optimization as a matrix where each
% row corresponds to a group and each column specifies whether or not that
% muscle is in the group (1) or not (0). Column k corresponds to muscle i
% in muscle_names
if ~isempty(muscleGroups)
    muscleGroupsMatrix = zeros(length(muscleGroups),num_muscles);
    for j = 1:length(muscleGroups)
        for k = 1:num_muscles
            muscleGroupsMatrix(j,k) = any(strcmp(muscleGroups{j},muscle_names{k}));
        end
    end
    
    % check to make sure each muscle in one group at most
    if any(sum(muscleGroupsMatrix) > 1)
        error('at least one muscle is a member of more than one muscleGroup')
    end
else
    muscleGroupsMatrix = [];
end

% max mtu shortening velocity is used as a lower bound on the fiber
% shortening velocity projected onto the mtu:
%           zdot = vm0 * lm0 * vn * sec(pen)
% we need to make sure vm0, lm0, and pen0 are s.t. zdot CAN be less than
% min mtu velocity. To answer CAN, we'd set vn = -1. But this is right on
% the edge of the bound and there the fv multiplier is 0. For some wiggle
% room we'd like for it to be able to produce some force there. So consider
% vn s.t. fv = fl at the min allowable norm fiber length (more restrictive
% than vn = -1). We need also consider some instantaneous pen. Worst case
% scenario is when sec(pen) = 1/cos(pen) is as small as possible, i.e.,
% when cos(pen) is as large as possible, i.e., when pen is as small as
% possible, i.e., when norm fiber length is as long as possible. So set
% sec(pen) s.t. norm fiber length is the max allowed. This way, by
% satisfying this bound, we know the fiber can handle the fastest mtu
% shortening velocities for instances where the tendon is lengthening and
% for the worst case configuration (longest possible fiber length) and s.t.
% the fiber has at least some force producing capacity (as much as the fl
% multiplier at the min allowable fiber length)

% that said this is quite restrictive, so the following sets the percentage
% of the max mtu shortening vel that we wish to require could be satisfied
% by the muscle given the above worst case scenario
min_percentage_mtu_vel_from_fiber = 0.50;
fprintf('-minimum percentage (p) of the max mtu shortening velocity contributed by fiber for inequality constraint: %4.2f%%\n',min_percentage_mtu_vel_from_fiber*100);
fprintf(notes,'-minimum percentage (p) of the max mtu shortening velocity contributed by fiber for inequality constraint: %4.2f%%\n',min_percentage_mtu_vel_from_fiber*100);

% max fiber shortening velocity: vn in above equation
max_normalized_fiber_shortening_velocity = osimDGF_fvinverse(osimDGF_flactive(stateBounds.norm_fiber_length(1)));
fprintf('-fiber shortening velocity considered for max mtu shortening velocity inequality constraint: %f\n',max_normalized_fiber_shortening_velocity);
fprintf(notes,'-fiber shortening velocity considered for max mtu shortening velocity inequality constraint: %f\n',max_normalized_fiber_shortening_velocity);

% now the constraint is:
%    p * min_mtu_vel > v0 * l0 * vn * sec(pen)
%        min_mtu_vel > v0 * l0 * vn/p * sec(pen)
% so adjust max_normalized_fiber_shortening_velocity to incorporate 1/p
max_normalized_fiber_shortening_velocity = max_normalized_fiber_shortening_velocity / min_percentage_mtu_vel_from_fiber;

% now make sure every muscle has room to satisfy the bound
fprintf('-feasibility for satisfying min(mtudot) < zdot = vm0 * lm0 * vn/p * sec(pen)\n');
fprintf('     -following muscles do not satisfy the bound for nominal values (all muscles not listed do):\n');
fprintf(notes,'-feasibility for satisfying min(mtudot) < zdot = vm0 * lm0 * vn/p * sec(pen)\n');
fprintf(notes,'     -following muscles do not satisfy the bound for nominal values (all muscles not listed do):\n');
problematic_muscles = false(1,num_muscles);
for k = 1:num_muscles
    
    % get relevant params
    this_vm0 = originalParams.max_contraction_velocity(k);
    this_lm0 = originalParams.optimal_fiber_length(k);
    this_pen0 = originalParams.pennation_angle_at_optimal(k);
    
    % get scale factor index
    this_xvm0_index = indices.xvm0(k);
    this_xlm0_index = indices.xlm0(k);
    this_xpen0_index = indices.xpen0(k);
    
    % directions that make satisfying easier:
    %   increased vm0
    %   increased lm0
    %   increased pen0
    % so get upper bounds of each scale factor
    this_xvm0_ub = ub(this_xvm0_index);
    this_xlm0_ub = ub(this_xlm0_index);
    this_xpen0_ub = ub(this_xpen0_index);
    
    % get upper bounds of each value
    this_vm0_ub = this_xvm0_ub * this_vm0;
    this_lm0_ub = this_xlm0_ub * this_lm0;
    this_pen0_ub = this_xpen0_ub * this_pen0;
    
    % worst case pen: see above
    worst_case_pen_ub = asin(sin(this_pen0_ub) / stateBounds.norm_fiber_length(2));
    worst_case_pen = asin(sin(this_pen0) / stateBounds.norm_fiber_length(2));
    
    % throw error if doesn't satisfy
    this_best_case_zdot = this_vm0_ub * this_lm0_ub * max_normalized_fiber_shortening_velocity * sec(worst_case_pen_ub);
    if max_mtu_shortening_velocity(k) <= this_best_case_zdot
        problematic_muscles(k) = true;
        fprintf('-%s max mtu shortening velocity = %4.3f m/s, but best possible case given bounds for zdot, cannot satisfy zdot = %4.3f m/s < %3.2f * min(mtudot) = %4.3f m/s\n',...
            muscle_names{k},max_mtu_shortening_velocity(k),...
            this_best_case_zdot * min_percentage_mtu_vel_from_fiber,...
            min_percentage_mtu_vel_from_fiber,...
            min_percentage_mtu_vel_from_fiber * max_mtu_shortening_velocity(k));
        fprintf(notes,'-%s max mtu shortening velocity = %4.3f m/s, but best possible case given bounds for zdot, cannot satisfy zdot = %4.3f m/s < %3.2f * min(mtudot) = %4.3f m/s\n',...
            muscle_names{k},max_mtu_shortening_velocity(k),...
            this_best_case_zdot * min_percentage_mtu_vel_from_fiber,...
            min_percentage_mtu_vel_from_fiber,...
            min_percentage_mtu_vel_from_fiber * max_mtu_shortening_velocity(k));
    else
        
        % satisfy for nominal values?
        % report only if doesnt
        nominal_zdot = this_vm0 * this_lm0 * max_normalized_fiber_shortening_velocity * sec(worst_case_pen);
        if max_mtu_shortening_velocity(k) <= nominal_zdot
            fprintf('           -%s: %3.2f * min(mtudot) = %4.3f m/s, best case zdot = %4.3f m/s, (wiggle room: %3.2f%%) \n',...
                muscle_names{k},min_percentage_mtu_vel_from_fiber,...
                min_percentage_mtu_vel_from_fiber * max_mtu_shortening_velocity(k),...
                min_percentage_mtu_vel_from_fiber * this_best_case_zdot,...
                (1-max_mtu_shortening_velocity(k)/this_best_case_zdot)*100);
            fprintf(notes,'           -%s: %3.2f * min(mtudot) = %4.3f m/s, best case zdot = %4.3f m/s, (wiggle room: %3.2f%%) \n',...
                muscle_names{k},min_percentage_mtu_vel_from_fiber,...
                min_percentage_mtu_vel_from_fiber * max_mtu_shortening_velocity(k),...
                min_percentage_mtu_vel_from_fiber * this_best_case_zdot,...
                (1-max_mtu_shortening_velocity(k)/this_best_case_zdot)*100);
        end
    end
    
end

% error if any problematic muscles
if any(problematic_muscles)
    fprintf('Problematic muscles which have no chance of satisfying min(mtudot) > min(zdot):\n');
    fprintf(notes,'Problematic muscles which have no chance of satisfying min(mtudot) > min(zdot):\n');
    disp(muscle_names(problematic_muscles))
    error('see notes above');
end

% optimize
options = optimoptions('fmincon','Algorithm','interior-point','HessianApproximation','lbfgs','SpecifyConstraintGradient',true,'SpecifyObjectiveGradient',true,'CheckGradients',false,'FiniteDifferenceType','central','FiniteDifferenceStepSize',1e-3,...
                       'StepTolerance',1e-15,'ConstraintTolerance',1e-9,'OptimalityTolerance',1e-9,...
                       'Display','iter','MaxIterations',inf,'MaxFunctionEvaluations',inf);
x = fmincon(@objective,x0,[],[],[],[],lb,ub,@nonlcon,options,...
                        originalParams,...
                        passive_moment.muscle,...
                        moment_arm,...
                        mtu_length,...
                        weights,...
                        min_mtu_length,...
                        max_mtu_length,...
                        stateBounds.norm_fiber_length(2),...
                        max_mtu_shortening_velocity,...
                        indices,...
                        max_normalized_fiber_shortening_velocity,...
                        muscleGroupsMatrix,...
                        stateBounds);

fprintf('\n\n')

%% ADJUST MODEL WITH OPTIMIZED VARIABLES

% get optimized variables
xlmn = x(indices.xlmn);
xlmn_max_mtu = x(indices.xlmn_max_mtu_length);
xlmn_min_mtu = x(indices.xlmn_min_mtu_length);
xlts = x(indices.xlts);
xlm0 = x(indices.xlm0);
xfm0 = x(indices.xfm0);
xe0 = x(indices.xe0);
xs0 = x(indices.xs0);
xvm0 = x(indices.xvm0);
xpen0 = x(indices.xpen0);

% for each muscle
for m = 1:num_muscles
    
    % get adjusted params
    lts = xlts(m) * originalParams.tendon_slack_length(m);
    lm0 = xlm0(m) * originalParams.optimal_fiber_length(m);
    fm0 = xfm0(m) * originalParams.max_isometric_force(m);
    e0 = xe0(m) * originalParams.tendon_strain_at_one_norm_force(m);
    s0 = xs0(m) * originalParams.passive_fiber_strain_at_one_norm_force(m);
    vm0 = xvm0(m) * originalParams.max_contraction_velocity(m);
    pen0 = xpen0(m) * originalParams.pennation_angle_at_optimal(m);
    
    % get right and left muscle names
    rmusc = DeGrooteFregly2016Muscle.safeDownCast(model.getMuscles.get(muscle_names{m}));
    lmusc = DeGrooteFregly2016Muscle.safeDownCast(model.getMuscles.get([muscle_names{m}(1:end-2) '_l']));
    
    % adjust right
    rmusc.set_tendon_slack_length(lts);
    rmusc.set_optimal_fiber_length(lm0);
    rmusc.set_max_isometric_force(fm0);
    rmusc.set_tendon_strain_at_one_norm_force(e0);
    rmusc.set_passive_fiber_strain_at_one_norm_force(s0);
    rmusc.set_max_contraction_velocity(vm0);
    rmusc.set_pennation_angle_at_optimal(pen0);
    
    % adjust left
    lmusc.set_tendon_slack_length(lts);
    lmusc.set_optimal_fiber_length(lm0);
    lmusc.set_max_isometric_force(fm0);
    lmusc.set_tendon_strain_at_one_norm_force(e0);
    lmusc.set_passive_fiber_strain_at_one_norm_force(s0);
    lmusc.set_max_contraction_velocity(vm0);
    lmusc.set_pennation_angle_at_optimal(pen0);
    
end

%% compare

% original model passive muscle moment
[~,~,passive_moment.original_muscle_estimate,original_norm_tendon_force] = objective(x0,...
                                                                                     originalParams,...
                                                                                     passive_moment.muscle,...
                                                                                     moment_arm,...
                                                                                     mtu_length,...
                                                                                     weights,...
                                                                                     min_mtu_length,...
                                                                                     max_mtu_length,...
                                                                                     stateBounds.norm_fiber_length(2),...
                                                                                     max_mtu_shortening_velocity,...
                                                                                     indices,...
                                                                                     max_normalized_fiber_shortening_velocity,...
                                                                                     muscleGroupsMatrix,...
                                                                                     stateBounds);
% optimzed muscle moments
[optimized_total_cost,~,passive_moment.optim_muscle_estimate,optim_norm_tendon_force,F_squared_error,F_lts_dev,F_lm0_dev,F_fm0_dev,F_e0_dev,F_s0_dev,F_vm0_dev,F_pen0_dev] = ...
    objective(x,...
              originalParams,...
              passive_moment.muscle,...
              moment_arm,...
              mtu_length,...
              weights,...
              min_mtu_length,...
              max_mtu_length,...
              stateBounds.norm_fiber_length(2),...
              max_mtu_shortening_velocity,...
              indices,...
              max_normalized_fiber_shortening_velocity,...
              muscleGroupsMatrix,...
              stateBounds);
          
% init system, set activation to 0
state = model.initSystem;
for k = 0:model.getMuscles.getSize-1; model.getMuscles.get(k).setActivation(state,0.0); end

% get fiber lengths in each config (that satisfy M-T force equilibrium)
optim_norm_fiber_length = xlmn;

% get osim estimates post-optimization
passive_moment.optim_muscle_estimate_osim = zeros(3,num_configurations);
optim_norm_fiber_length_osim = zeros(num_muscles,num_configurations);
optim_norm_tendon_force_osim = zeros(num_muscles,num_configurations);

% for each configuration
wait_bar = waitbar(0,sprintf('Configuration 0 of %d',num_configurations));
for c = 1:num_configurations
            
    waitbar(c/num_configurations,wait_bar,sprintf('Configurations %d of %d',c,num_configurations))

    % set/store configuration
    hip.setValue(state,hip_flexion_r(c));
    knee.setValue(state,knee_angle_r(c));
    ankle.setValue(state,ankle_angle_r(c));

    % equilibrate muscles
    model.equilibrateMuscles(state);

    % for each muscle
    for m = 1:num_muscles

        % compute norm fiber length
        thisMuscle = DeGrooteFregly2016Muscle.safeDownCast(model.getMuscles.get(muscle_names{m}));
        optim_norm_fiber_length_osim(m,c) = thisMuscle.getNormalizedFiberLength(state);

        % osim estimate from optimized muscle model
        optim_norm_tendon_force_osim(m,c) = thisMuscle.getTendonForce(state) / thisMuscle.get_max_isometric_force;
        for h = 1:3
            passive_moment.optim_muscle_estimate_osim(h,c) = passive_moment.optim_muscle_estimate_osim(h,c) + moment_arm(m,c,h) * optim_norm_tendon_force_osim(m,c) * thisMuscle.get_max_isometric_force;
        end

    end
            
end
hip.setValue(state,0);
knee.setValue(state,0);
ankle.setValue(state,0);
close(wait_bar)

%% REPORT

% plot norm fiber lengths and min/max mtu length configs
figure
plot(xlmn_min_mtu,'b')
hold on
plot(xlmn_max_mtu,'r')
ylabel('Norm. Fib. Len.')
legend('min MTU','max MTU')

% plot hip
figure
plot(passive_moment.net_experimental(1,:),'k')
hold on
plot(passive_moment.ligament(1,:),'b')
plot(passive_moment.muscle(1,:),'r')
plot(passive_moment.original_muscle_estimate(1,:),'m--')
plot(passive_moment.original_muscle_estimate_osim(1,:),'m:')
plot(passive_moment.optim_muscle_estimate(1,:),'g--')
plot(passive_moment.optim_muscle_estimate_osim(1,:),'g:')
title('Hip')
xlabel('Hip Flexion')
ylabel('Moment (Nm)')
legend('Net','Lig','Muscle','script-pre','osim-pre','script-post','osim-post')

% plot knee
figure
plot(passive_moment.net_experimental(2,:),'k')
hold on
plot(passive_moment.ligament(2,:),'b')
plot(passive_moment.muscle(2,:),'r')
plot(passive_moment.original_muscle_estimate(2,:),'m--')
plot(passive_moment.original_muscle_estimate_osim(2,:),'m:')
plot(passive_moment.optim_muscle_estimate(2,:),'g--')
plot(passive_moment.optim_muscle_estimate_osim(2,:),'g:')
title('Knee')
xlabel('Knee Flexion')
ylabel('Moment (Nm)')
legend('Net','Lig','Muscle','script-pre','osim-pre','script-post','osim-post')

% plot ankle
figure
plot(passive_moment.net_experimental(3,:),'k')
hold on
plot(passive_moment.ligament(3,:),'b')
plot(passive_moment.muscle(3,:),'r')
plot(passive_moment.original_muscle_estimate(3,:),'m--')
plot(passive_moment.original_muscle_estimate_osim(3,:),'m:')
plot(passive_moment.optim_muscle_estimate(3,:),'g--')
plot(passive_moment.optim_muscle_estimate_osim(3,:),'g:')
title('Ankle')
xlabel('Ankle Dorsiflexion')
ylabel('Moment (Nm)')
legend('Net','Lig','Muscle','script-pre','osim-pre','script-post','osim-post')

% compare Lm/Ft with osim
muscle_fig2 = figure;
muscle_fig2.Position = [1005 688 1102 376];
original_osim_fiber_length_error = original_norm_fiber_length(:) - original_norm_fiber_length_osim(:);
original_osim_tendon_force_error = original_norm_tendon_force(:) - original_norm_tendon_force_osim(:);
optim_osim_fiber_length_error = optim_norm_fiber_length(:) - optim_norm_fiber_length_osim(:);
optim_osim_tendon_force_error = optim_norm_tendon_force(:) - optim_norm_tendon_force_osim(:);
subplot(2,1,1)
plot(optim_osim_fiber_length_error,'g')
hold on
plot(original_osim_fiber_length_error,'k')
ylabel('Lm osim error')
title('Script vs OpenSim Fiber-Tendon Equilibrium')
ylim([-0.2 0.2])
subplot(2,1,2)
plot(optim_osim_tendon_force_error,'g')
hold on
plot(original_osim_tendon_force_error,'k')
ylabel('Ft osim error')
ylim([-0.2 0.2])

% RMSE for each
fprintf('\n\n')
fprintf('-Initial error in passive muscle moments: script vs experimental data\n')
fprintf('     -hip:   %f Nm RMSE\n',rms(passive_moment.original_muscle_estimate(1,:) - passive_moment.muscle(1,:)))
fprintf('     -knee:  %f Nm RMSE\n',rms(passive_moment.original_muscle_estimate(2,:) - passive_moment.muscle(2,:)))
fprintf('     -ankle: %f Nm RMSE\n',rms(passive_moment.original_muscle_estimate(3,:) - passive_moment.muscle(3,:)))
fprintf('-Initial error in passive muscle moments: OpenSim vs experimental data\n')
fprintf('     -hip:   %f Nm RMSE\n',rms(passive_moment.original_muscle_estimate_osim(1,:) - passive_moment.muscle(1,:)))
fprintf('     -knee:  %f Nm RMSE\n',rms(passive_moment.original_muscle_estimate_osim(2,:) - passive_moment.muscle(2,:)))
fprintf('     -ankle: %f Nm RMSE\n',rms(passive_moment.original_muscle_estimate_osim(3,:) - passive_moment.muscle(3,:)))
fprintf('-Initial error in passive muscle moment: script vs OpenSim\n')
fprintf('     -hip:   %f Nm RMSE\n',rms(passive_moment.original_muscle_estimate(1,:) - passive_moment.original_muscle_estimate_osim(1,:)))
fprintf('     -knee:  %f Nm RMSE\n',rms(passive_moment.original_muscle_estimate(2,:) - passive_moment.original_muscle_estimate_osim(2,:)))
fprintf('     -ankle: %f Nm RMSE\n',rms(passive_moment.original_muscle_estimate(3,:) - passive_moment.original_muscle_estimate_osim(3,:)))
fprintf('-Initial error in fiber length and tendon force: script vs OpenSim\n')
fprintf('      -Norm. fiber length = %f RMSE (max abs error = %f)\n',rms(original_osim_fiber_length_error),max(abs(original_osim_fiber_length_error)))
fprintf('      -Norm. tendon force = %f RMSE (max abs error = %f)\n',rms(original_osim_tendon_force_error),max(abs(original_osim_tendon_force_error)))
fprintf('\n------------------------------------------------------------------------------------')
fprintf('\n------------------------------------------------------------------------------------\n\n')
fprintf('-OBJECTIVE TERMS:\n')
fprintf('     -total = %f\n',optimized_total_cost)
fprintf('     -squared error = %f\n',F_squared_error)
fprintf('     -tendon slack length deviations = %f\n',F_lts_dev)
fprintf('     -optimal fiber length deviations = %f\n',F_lm0_dev)
fprintf('     -max isometric force deviations = %f\n',F_fm0_dev)
fprintf('     -tendon strain at one norm force deviations = %f\n',F_e0_dev)
fprintf('     -strain at one norm force deviations = %f\n',F_s0_dev)
fprintf('     -max contraction velocity deviations = %f\n',F_vm0_dev)
fprintf('     -pennation angle at optimal deviations = %f\n',F_pen0_dev)
fprintf('-Post optimization error in passive muscle moments: script vs experimental data\n')
fprintf('     -hip:   %f Nm RMSE\n',rms(passive_moment.optim_muscle_estimate(1,:) - passive_moment.muscle(1,:)))
fprintf('     -knee:  %f Nm RMSE\n',rms(passive_moment.optim_muscle_estimate(2,:) - passive_moment.muscle(2,:)))
fprintf('     -ankle: %f Nm RMSE\n',rms(passive_moment.optim_muscle_estimate(3,:) - passive_moment.muscle(3,:)))
fprintf('-Post optimization error in passive muscle moments: OpenSim vs experimental data\n')
fprintf('     -hip:   %f Nm RMSE\n',rms(passive_moment.optim_muscle_estimate_osim(1,:) - passive_moment.muscle(1,:)))
fprintf('     -knee:  %f Nm RMSE\n',rms(passive_moment.optim_muscle_estimate_osim(2,:) - passive_moment.muscle(2,:)))
fprintf('     -ankle: %f Nm RMSE\n',rms(passive_moment.optim_muscle_estimate_osim(3,:) - passive_moment.muscle(3,:)))
fprintf('-Post optimization error in passive muscle moment: script vs OpenSim\n')
fprintf('     -hip:   %f Nm RMSE\n',rms(passive_moment.optim_muscle_estimate(1,:) - passive_moment.optim_muscle_estimate_osim(1,:)))
fprintf('     -knee:  %f Nm RMSE\n',rms(passive_moment.optim_muscle_estimate(2,:) - passive_moment.optim_muscle_estimate_osim(2,:)))
fprintf('     -ankle: %f Nm RMSE\n',rms(passive_moment.optim_muscle_estimate(3,:) - passive_moment.optim_muscle_estimate_osim(3,:)))
fprintf('-Post optimization error in fiber length and tendon force: script vs OpenSim\n')
fprintf('      -Norm. fiber length = %f RMSE (max abs error = %f)\n',rms(optim_osim_fiber_length_error),max(abs(optim_osim_fiber_length_error)))
fprintf('      -Norm. tendon force = %f RMSE (max abs error = %f)\n',rms(optim_osim_tendon_force_error),max(abs(optim_osim_tendon_force_error)))

% greatest changes for each parameter
fprintf('\n------------------------------------------------------------------------------------')
fprintf('\n------------------------------------------------------------------------------------\n\n')
fprintf('-GREATEST PERCENT CHANGES FOR EACH PARAMETER:\n')
[~,imax] = max(abs(xlts-1));
fprintf('     -tendon slack length: %s, percent change = %4.2f%%\n',replace(muscle_names{imax}(1:end-2),'_',' '),(xlts(imax)-1)*100)
[~,imax] = max(abs(xlm0-1));
fprintf('     -optimal fiber length: %s, percent change = %4.2f%%\n',replace(muscle_names{imax}(1:end-2),'_',' '),(xlm0(imax)-1)*100)
[~,imax] = max(abs(xfm0-1));
fprintf('     -max isometric force: %s, percent change = %4.2f%%\n',replace(muscle_names{imax}(1:end-2),'_',' '),(xfm0(imax)-1)*100)
[~,imax] = max(abs(xe0-1));
fprintf('     -tendon strain at one norm force: %s, percent change = %4.2f%%\n',replace(muscle_names{imax}(1:end-2),'_',' '),(xe0(imax)-1)*100)
[~,imax] = max(abs(xs0-1));
fprintf('     -fiber strain at one norm force: %s, percent change = %4.2f%%\n',replace(muscle_names{imax}(1:end-2),'_',' '),(xs0(imax)-1)*100)
[~,imax] = max(abs(xvm0-1));
fprintf('     -max contraction velocity: %s, percent change = %4.2f%%\n',replace(muscle_names{imax}(1:end-2),'_',' '),(xvm0(imax)-1)*100)
[~,imax] = max(abs(xpen0-1));
fprintf('     -pennation angle at optimal: %s, percent change = %4.2f%%\n',replace(muscle_names{imax}(1:end-2),'_',' '),(xpen0(imax)-1)*100)

% print results
fprintf('\n------------------------------------------------------------------------------------');
fprintf('\n------------------------------------------------------------------------------------\n\n');
fprintf('-SCALARS\n');
fprintf('-legend:\n');
fprintf('     -lts: tendon slack length\n');
fprintf('     -lm0: optimal fiber length\n');
fprintf('     -fm0: max isometric force\n');
fprintf('     -e0: tendon strain at one norm force\n');
fprintf('     -s0: passive fiber strain at one norm force\n');
fprintf('     -vm0: max contraction velocity\n');
fprintf('     -pen0: pennation angle at optimal\n');
for k = 1:num_muscles
    fprintf('(%d) %s: lts = %3.2f, lm0 = %3.2f, fm0 = %3.2f, e0 = %3.2f, s0 = %3.2f, vm0 = %3.2f, pen0 = %3.2f\n',...
        k,replace(muscle_names{k}(1:end-2),'_',' '),xlts(k),xlm0(k),xfm0(k),xe0(k),xs0(k),xvm0(k),xpen0(k))
end

%% SAVE IMPORTANT DATA

save(fullfile(resultsDir,['calibration_' datestr(datetime,'yyyymmddTHHMMSS') '.mat']),'passive_moment','x','x0','indices',...
    'original_norm_fiber_length','original_norm_fiber_length_osim','original_norm_tendon_force','original_norm_tendon_force_osim',...
    'optim_norm_fiber_length','optim_norm_fiber_length_osim','optim_norm_tendon_force','optim_norm_tendon_force_osim',...
    'configuration');

%% SAVE TO NOTES

% RMSE for each
fprintf(notes,'\n\n');
fprintf(notes,'-Initial error in passive muscle moments: script vs experimental data\n');
fprintf(notes,'     -hip:   %f Nm RMSE\n',rms(passive_moment.original_muscle_estimate(1,:) - passive_moment.muscle(1,:)));
fprintf(notes,'     -knee:  %f Nm RMSE\n',rms(passive_moment.original_muscle_estimate(2,:) - passive_moment.muscle(2,:)));
fprintf(notes,'     -ankle: %f Nm RMSE\n',rms(passive_moment.original_muscle_estimate(3,:) - passive_moment.muscle(3,:)));
fprintf(notes,'-Initial error in passive muscle moments: OpenSim vs experimental data\n');
fprintf(notes,'     -hip:   %f Nm RMSE\n',rms(passive_moment.original_muscle_estimate_osim(1,:) - passive_moment.muscle(1,:)));
fprintf(notes,'     -knee:  %f Nm RMSE\n',rms(passive_moment.original_muscle_estimate_osim(2,:) - passive_moment.muscle(2,:)));
fprintf(notes,'     -ankle: %f Nm RMSE\n',rms(passive_moment.original_muscle_estimate_osim(3,:) - passive_moment.muscle(3,:)));
fprintf(notes,'-Initial error in passive muscle moment: script vs OpenSim\n');
fprintf(notes,'     -hip:   %f Nm RMSE\n',rms(passive_moment.original_muscle_estimate(1,:) - passive_moment.original_muscle_estimate_osim(1,:)));
fprintf(notes,'     -knee:  %f Nm RMSE\n',rms(passive_moment.original_muscle_estimate(2,:) - passive_moment.original_muscle_estimate_osim(2,:)));
fprintf(notes,'     -ankle: %f Nm RMSE\n',rms(passive_moment.original_muscle_estimate(3,:) - passive_moment.original_muscle_estimate_osim(3,:)));
fprintf(notes,'-Initial error in fiber length and tendon force: script vs OpenSim\n');
fprintf(notes,'      -Norm. fiber length = %f RMSE (max abs error = %f)\n',rms(original_osim_fiber_length_error),max(abs(original_osim_fiber_length_error)));
fprintf(notes,'      -Norm. tendon force = %f RMSE (max abs error = %f)\n',rms(original_osim_tendon_force_error),max(abs(original_osim_tendon_force_error)));
fprintf(notes,'\n------------------------------------------------------------------------------------');
fprintf(notes,'\n------------------------------------------------------------------------------------\n\n');
fprintf(notes,'-OBJECTIVE TERMS:\n');
fprintf(notes,'     -total = %f\n',optimized_total_cost);
fprintf(notes,'     -squared error = %f\n',F_squared_error);
fprintf(notes,'     -tendon slack length deviations = %f\n',F_lts_dev);
fprintf(notes,'     -optimal fiber length deviations = %f\n',F_lm0_dev);
fprintf(notes,'     -max isometric force deviations = %f\n',F_fm0_dev);
fprintf(notes,'     -tendon strain at one norm force deviations = %f\n',F_e0_dev);
fprintf(notes,'     -strain at one norm force deviations = %f\n',F_s0_dev);
fprintf(notes,'     -max contraction velocity deviations = %f\n',F_vm0_dev);
fprintf(notes,'     -pennation angle at optimal deviations = %f\n',F_pen0_dev);
fprintf(notes,'-Post optimization error in passive muscle moments: script vs experimental data\n');
fprintf(notes,'     -hip:   %f Nm RMSE\n',rms(passive_moment.optim_muscle_estimate(1,:) - passive_moment.muscle(1,:)));
fprintf(notes,'     -knee:  %f Nm RMSE\n',rms(passive_moment.optim_muscle_estimate(2,:) - passive_moment.muscle(2,:)));
fprintf(notes,'     -ankle: %f Nm RMSE\n',rms(passive_moment.optim_muscle_estimate(3,:) - passive_moment.muscle(3,:)));
fprintf(notes,'-Post optimization error in passive muscle moments: OpenSim vs experimental data\n');
fprintf(notes,'     -hip:   %f Nm RMSE\n',rms(passive_moment.optim_muscle_estimate_osim(1,:) - passive_moment.muscle(1,:)));
fprintf(notes,'     -knee:  %f Nm RMSE\n',rms(passive_moment.optim_muscle_estimate_osim(2,:) - passive_moment.muscle(2,:)));
fprintf(notes,'     -ankle: %f Nm RMSE\n',rms(passive_moment.optim_muscle_estimate_osim(3,:) - passive_moment.muscle(3,:)));
fprintf(notes,'-Post optimization error in passive muscle moment: script vs OpenSim\n');
fprintf(notes,'     -hip:   %f Nm RMSE\n',rms(passive_moment.optim_muscle_estimate(1,:) - passive_moment.optim_muscle_estimate_osim(1,:)));
fprintf(notes,'     -knee:  %f Nm RMSE\n',rms(passive_moment.optim_muscle_estimate(2,:) - passive_moment.optim_muscle_estimate_osim(2,:)));
fprintf(notes,'     -ankle: %f Nm RMSE\n',rms(passive_moment.optim_muscle_estimate(3,:) - passive_moment.optim_muscle_estimate_osim(3,:)));
fprintf(notes,'-Post optimization error in fiber length and tendon force: script vs OpenSim\n');
fprintf(notes,'      -Norm. fiber length = %f RMSE (max abs error = %f)\n',rms(optim_osim_fiber_length_error),max(abs(optim_osim_fiber_length_error)));
fprintf(notes,'      -Norm. tendon force = %f RMSE (max abs error = %f)\n',rms(optim_osim_tendon_force_error),max(abs(optim_osim_tendon_force_error)));

% greatest changes for each parameter
fprintf(notes,'\n------------------------------------------------------------------------------------');
fprintf(notes,'\n------------------------------------------------------------------------------------\n\n');
fprintf(notes,'-GREATEST PERCENT CHANGES FOR EACH PARAMETER:\n');
[~,imax] = max(abs(xlts-1));
fprintf(notes,'     -tendon slack length: %s, percent change = %4.2f%%\n',replace(muscle_names{imax}(1:end-2),'_',' '),(xlts(imax)-1)*100);
[~,imax] = max(abs(xlm0-1));
fprintf(notes,'     -optimal fiber length: %s, percent change = %4.2f%%\n',replace(muscle_names{imax}(1:end-2),'_',' '),(xlm0(imax)-1)*100);
[~,imax] = max(abs(xfm0-1));
fprintf(notes,'     -max isometric force: %s, percent change = %4.2f%%\n',replace(muscle_names{imax}(1:end-2),'_',' '),(xfm0(imax)-1)*100);
[~,imax] = max(abs(xe0-1));
fprintf(notes,'     -tendon strain at one norm force: %s, percent change = %4.2f%%\n',replace(muscle_names{imax}(1:end-2),'_',' '),(xe0(imax)-1)*100);
[~,imax] = max(abs(xs0-1));
fprintf(notes,'     -fiber strain at one norm force: %s, percent change = %4.2f%%\n',replace(muscle_names{imax}(1:end-2),'_',' '),(xs0(imax)-1)*100);
[~,imax] = max(abs(xvm0-1));
fprintf(notes,'     -max contraction velocity: %s, percent change = %4.2f%%\n',replace(muscle_names{imax}(1:end-2),'_',' '),(xvm0(imax)-1)*100);
[~,imax] = max(abs(xpen0-1));
fprintf(notes,'     -pennation angle at optimal: %s, percent change = %4.2f%%\n',replace(muscle_names{imax}(1:end-2),'_',' '),(xpen0(imax)-1)*100);

% print results
fprintf(notes,'\n------------------------------------------------------------------------------------');
fprintf(notes,'\n------------------------------------------------------------------------------------\n\n');
fprintf(notes,'-SCALARS\n');
fprintf(notes,'-legend:\n');
fprintf(notes,'     -lts: tendon slack length\n');
fprintf(notes,'     -lm0: optimal fiber length\n');
fprintf(notes,'     -fm0: max isometric force\n');
fprintf(notes,'     -e0: tendon strain at one norm force\n');
fprintf(notes,'     -s0: passive fiber strain at one norm force\n');
fprintf(notes,'     -vm0: max contraction velocity\n');
fprintf(notes,'     -pen0: pennation angle at optimal\n');
for k = 1:num_muscles
    fprintf(notes,'(%d) %s: lts = %3.2f, lm0 = %3.2f, fm0 = %3.2f, e0 = %3.2f, s0 = %3.2f, vm0 = %3.2f, pen0 = %3.2f\n',...
        k,replace(muscle_names{k}(1:end-2),'_',' '),xlts(k),xlm0(k),xfm0(k),xe0(k),xs0(k),xvm0(k),xpen0(k));
end

end

%% OBJECTIVE
function [F,dF_dx,est_M_muscle,est_nF_tendon,F_squared_error,F_lts_dev,F_lm0_dev,F_fm0_dev,F_e0_dev,F_s0_dev,F_vm0_dev,F_pen0_dev] = objective(x,originalParams,M_muscle,moment_arm,mtu_length,weights,min_mtu_length,max_mtu_length,max_lmn,max_mtu_shortening_velocity,indices,max_vn,muscleGroupsMatrix,stateBounds)

% range of muscle moments for normalization
range_M_muscle = max(M_muscle,[],2) - min(M_muscle,[],2);
range_M_muscle(abs(range_M_muscle) < 1) = 1;

% init objective/gradient
F  = 0;
dF_dx = zeros(1,length(x));
est_M_muscle = zeros(size(M_muscle));
est_nF_tendon = zeros(size(mtu_length));

% for each configuration
num_configurations = size(mtu_length,2); % num configurations
num_muscles = size(mtu_length,1); % num muscles
num_errors = num_configurations * 3;
for c = 1:num_configurations
    
    % for each muscle
    dM_dxlmn = zeros(3,num_muscles);
    dM_dxfm0 = zeros(3,num_muscles);
    dM_dxpen0 = zeros(3,num_muscles);
    dM_dxs0 = zeros(3,num_muscles);
    for m = 1:num_muscles
        
        % get index of this muscle normalized length for this configuration
        xlmn_index = num_muscles * c - num_muscles + m;
        
        % get index of this muscles F0 scale factor
        xfm0_index = indices.xfm0(m);
        
        % get index of this muscles pen0 scale factor
        xpen0_index = indices.xpen0(m);
        
        % get index of this muscles strain at F0 scale factor
        xs0_index = indices.xs0(m);
        
        % get normalized muscle length
        lmn = x(xlmn_index);
        dlmn_dxlmn = 1;
        
        % get F0
        fm0 = x(xfm0_index) * originalParams.max_isometric_force(m);
        dfm0_dxfm0 = originalParams.max_isometric_force(m);
        
        % get pen0
        pen0 = x(xpen0_index) * originalParams.pennation_angle_at_optimal(m);
        dpen0_dxpen0 = originalParams.pennation_angle_at_optimal(m);
        
        % get muscle strain at F0 from scale factor
        s0 = x(xs0_index) * originalParams.passive_fiber_strain_at_one_norm_force(m);
        ds0_dxs0 = originalParams.passive_fiber_strain_at_one_norm_force(m);
        
        % get passive force
        [fp,dfp_dlmn,dfp_ds0] = osimDGF_fpassive(lmn,s0);
        dfp_dxlmn = dfp_dlmn * dlmn_dxlmn;
        dfp_dxs0 = dfp_ds0 * ds0_dxs0;
        
        % get pennation angle
        asinarg = sin(pen0) / lmn; % sin(p) * lmn^-1
        dasinarg_dxlmn = -asinarg / lmn * dlmn_dxlmn;
        dasinarg_dxpen0 = cos(pen0) / lmn * dpen0_dxpen0;
        pen = asin(asinarg);
        dpen_dxlmn = 1 / sqrt(1-asinarg^2) * dasinarg_dxlmn;
        dpen_dxpen0 = 1 / sqrt(1-asinarg^2) * dasinarg_dxpen0;
        
        % get muscle force
        f = fm0 * fp * cos(pen);
        est_nF_tendon(m,c) = f / fm0;
        df_dxlmn = fm0 * (dfp_dxlmn * cos(pen) - fp * sin(pen) * dpen_dxlmn);
        df_dxfm0 = dfm0_dxfm0 * fp * cos(pen);
        df_dxpen0 = -fm0 * fp * sin(pen) * dpen_dxpen0;
        df_dxs0 = fm0 * dfp_dxs0 * cos(pen);
        
        % for dof j, contribute net moment from this muscle
        for j = 1:3
            est_M_muscle(j,c) = est_M_muscle(j,c) + moment_arm(m,c,j) * f;
            dM_dxlmn(j,m) = moment_arm(m,c,j) * df_dxlmn;
            dM_dxfm0(j,m) = moment_arm(m,c,j) * df_dxfm0;
            dM_dxpen0(j,m) = moment_arm(m,c,j) * df_dxpen0;
            dM_dxs0(j,m) = moment_arm(m,c,j) * df_dxs0;
        end
        
    end
    
    % for each dof
    for j = 1:3
        
        % get error
        err = M_muscle(j,c) - est_M_muscle(j,c);
        
        % normalize by range
        norm_err = err / range_M_muscle(j);
        
        % square
        sq_err = norm_err * norm_err;
        
        % accumulate objective
        F = F + sq_err / 2 / num_errors;
        
        % for each muscle
        for m = 1:num_muscles
            
            % get error derivatives
            derr_dxlmn = -dM_dxlmn(j,m);
            derr_dxfm0 = -dM_dxfm0(j,m);
            derr_dxpen0 = -dM_dxpen0(j,m);
            derr_dxs0 = -dM_dxs0(j,m);
            
            % get normalized error derivatives
            dnerr_dxlmn = derr_dxlmn / range_M_muscle(j);
            dnerr_dxfm0 = derr_dxfm0 / range_M_muscle(j);
            dnerr_dxpen0 = derr_dxpen0 / range_M_muscle(j);
            dnerr_dxs0 = derr_dxs0 / range_M_muscle(j);
            
            % get squared error derivatives
            dsqerr_dxlmn = 2 * norm_err * dnerr_dxlmn;
            dsqerr_dxfm0 = 2 * norm_err * dnerr_dxfm0;
            dsqerr_dxpen0 = 2 * norm_err * dnerr_dxpen0;
            dsqerr_dxs0 = 2 * norm_err * dnerr_dxs0;
            
            % get final objective derivatives
            xlmn_index = num_muscles * c - num_muscles + m;
            xfm0_index = indices.xfm0(m);
            xpen0_index = indices.xpen0(m);
            xs0_index = indices.xs0(m);
            
            dF_dx(xlmn_index) = dF_dx(xlmn_index) + dsqerr_dxlmn / 2 / num_errors;
            dF_dx(xfm0_index) = dF_dx(xfm0_index) + dsqerr_dxfm0 / 2 / num_errors;
            dF_dx(xpen0_index) = dF_dx(xpen0_index) + dsqerr_dxpen0 / 2 / num_errors;
            dF_dx(xs0_index) = dF_dx(xs0_index) + dsqerr_dxs0 / 2 / num_errors;
            
        end
        
    end
    
end

% weights for each term
w_sq_err = weights.squared_error;
w_lts = weights.tendon_slack_length_deviation_from_one;
w_lm0 = weights.optimal_fiber_length_deviation_from_one;
w_fm0 = weights.max_isometric_force_deviation_from_one;
w_e0 = weights.tendon_strain_at_one_norm_force_deviation_from_one;
w_pen0 = weights.pennation_angle_at_optimal_deviation_from_one;
w_vm0 = weights.max_contraction_velocity_deviation_from_one;
w_s0 = weights.passive_fiber_strain_at_one_norm_force_deviation_from_one;

% store squared error objective
F = w_sq_err * F;
dF_dx = w_sq_err * dF_dx;
F_squared_error = F;

% get params
xlts = x(indices.xlts);
xlm0 = x(indices.xlm0);
xfm0 = x(indices.xfm0);
xe0 = x(indices.xe0);
xpen0 = x(indices.xpen0);
xvm0 = x(indices.xvm0);
xs0 = x(indices.xs0);

% penalize deviations from 1
F_lts_dev = w_lts * (xlts - 1)' * (xlts - 1) / 2 / num_muscles;
F_lm0_dev = w_lm0 * (xlm0 - 1)' * (xlm0 - 1) / 2 / num_muscles;
F_fm0_dev = w_fm0 * (xfm0 - 1)' * (xfm0 - 1) / 2 / num_muscles;
F_e0_dev = w_e0 * (xe0 - 1)' * (xe0 - 1) / 2 / num_muscles;
F_pen0_dev = w_pen0 * (xpen0 - 1)' * (xpen0 - 1) / 2 / num_muscles;
F_vm0_dev = w_vm0 * (xvm0 - 1)' * (xvm0 - 1) / 2 / num_muscles;
F_s0_dev = w_s0 * (xs0 - 1)' * (xs0 - 1) / 2 / num_muscles;

% add to total objective
F = F + F_lts_dev + F_lm0_dev + F_fm0_dev + F_e0_dev + F_pen0_dev + F_vm0_dev + F_s0_dev;
dF_dx(indices.xlts) = dF_dx(indices.xlts) + w_lts * (xlts - 1)' / 2 / num_muscles;
dF_dx(indices.xlm0) = dF_dx(indices.xlm0) + w_lm0 * (xlm0 - 1)' / 2 / num_muscles;
dF_dx(indices.xfm0) = dF_dx(indices.xfm0) + w_fm0 * (xfm0 - 1)' / 2 / num_muscles;
dF_dx(indices.xe0) = dF_dx(indices.xe0) + w_e0 * (xe0 - 1)' / 2 / num_muscles;
dF_dx(indices.xpen0) = dF_dx(indices.xpen0) + w_pen0 * (xpen0 - 1)' / 2 / num_muscles;
dF_dx(indices.xvm0) = dF_dx(indices.xvm0) + w_vm0 * (xvm0 - 1)' / 2 / num_muscles;
dF_dx(indices.xs0) = dF_dx(indices.xs0) + w_s0 * (xs0 - 1)' / 2 / num_muscles;

end

%% NONLINEAR CONSTRAINTS
function [cineq,ceq,dcineq_dx,dceq_dx] = nonlcon(x,originalParams,M_muscle,moment_arm,mtu_length,weights,min_mtu_length,max_mtu_length,max_lmn,max_mtu_shortening_velocity,indices,max_vn,muscleGroupsMatrix,stateBounds)

% init inequality constraints
num_configurations = size(mtu_length,2); % num configurations
num_muscles = size(mtu_length,1); % num muscles
num_cineq_from_pennation_upper_bound = (num_configurations + 2) * num_muscles; % must satisfy inst. penn lb for all static config + min/max mtu length config
num_cineq_from_max_mtu_shortening_velocity = num_muscles; % just make sure v0, l0, pen0 s.t. vm * sec(pen) < min mtu vel
num_cineq = num_cineq_from_pennation_upper_bound + num_cineq_from_max_mtu_shortening_velocity;
cineq = zeros(num_cineq,1);
dcineq_dx = zeros(num_cineq,length(x));

% for each configuration that muscle-tendon equilibrium must hold for (all
% joint configurations over which passive moments are being optimized and
% min/max mtu length), make sure inst. pen angle not too large
cineq_index = 1;
for c = 1:num_configurations+2 % the +2 is the two configuration corresponding to max/min mtu length
    
    % for each muscle
    for m = 1:num_muscles
        
        % get index to muscle length for this muscle/configuration
        xlmn_index = num_muscles * c - num_muscles + m;
        
        % sanity check
        if xlmn_index ~= cineq_index; error('this should not happen'); end
        
        % get normalized fiber length
        lmn = x(xlmn_index);
        dlmn_dxlmn = 1;
        
        % get index to pen0 scale factor
        xpen0_index = indices.xpen0(m);
        
        % get pen0
        pen0 = x(xpen0_index) * originalParams.pennation_angle_at_optimal(m);
        dpen0_dxpen0 = originalParams.pennation_angle_at_optimal(m);
        
        % get pennation angle
        asinarg = sin(pen0) / lmn; % sin(p) * lmn^-1
        dasinarg_dxlmn = -asinarg / lmn * dlmn_dxlmn;
        dasinarg_dxpen0 = cos(pen0) / lmn * dpen0_dxpen0;
        pen = asin(asinarg);
        dpen_dxlmn = 1 / sqrt(1-asinarg^2) * dasinarg_dxlmn;
        dpen_dxpen0 = 1 / sqrt(1-asinarg^2) * dasinarg_dxpen0;
        
        % calculate inequality: pen < upper bound => 0 > pen - ub
        cineq(cineq_index) = pen - stateBounds.pennation_angle(2);
        dcineq_dx(cineq_index,xlmn_index) = dpen_dxlmn;
        dcineq_dx(cineq_index,xpen0_index) = dpen_dxpen0;
        cineq_index = cineq_index + 1;
        
    end
    
end

% now make sure v0 * l0 * vn * sec(pen) < min mtu vel
% for each muscle
for m = 1:num_muscles
        
    % get index to params for this muscle
    xlm0_index = indices.xlm0(m);
    xpen0_index = indices.xpen0(m);
    xvm0_index = indices.xvm0(m);
    
    % get Lopt
    lm0 = x(xlm0_index) * originalParams.optimal_fiber_length(m);
    dlm0_dxlm0 = originalParams.optimal_fiber_length(m);

    % get pen0
    pen0 = x(xpen0_index) * originalParams.pennation_angle_at_optimal(m);
    dpen0_dxpen0 = originalParams.pennation_angle_at_optimal(m);
    
    % get vm0
    vm0 = x(xvm0_index) * originalParams.max_contraction_velocity(m);
    dvm0_dxvm0 = originalParams.max_contraction_velocity(m);
    
    % get pennation angle at worst case scenario (max norm fiber length)
    asinarg = sin(pen0) / max_lmn; % sin(p) * lmn^-1
    dasinarg_dxpen0 = cos(pen0) / max_lmn * dpen0_dxpen0;
    pen = asin(asinarg);
    dpen_dxpen0 = 1 / sqrt(1-asinarg^2) * dasinarg_dxpen0;
    
    % get velocity of fiber along tendon
    zdot = vm0 * lm0 * max_vn * sec(pen);
    dzdot_dxvm0 = lm0 * max_vn * sec(pen) * dvm0_dxvm0;
    dzdot_dxlm0 = vm0 * max_vn * sec(pen) * dlm0_dxlm0;
    dzdot_dxpen0 = zdot * tan(pen) * dpen_dxpen0;
    
    % get inequality and derivative
    cineq(cineq_index) = zdot - max_mtu_shortening_velocity(m);
    dcineq_dx(cineq_index,xlm0_index) = dzdot_dxlm0;
    dcineq_dx(cineq_index,xvm0_index) = dzdot_dxvm0;
    dcineq_dx(cineq_index,xpen0_index) = dzdot_dxpen0;
    
    cineq_index = cineq_index + 1;
    
end

% sanity check
if cineq_index-1 ~= length(cineq); error('this should not happen'); end

% num constraints due to muscle groups
num_ceq_muscle_groups = 0;
if ~isempty(muscleGroupsMatrix)
    for k = 1:size(muscleGroupsMatrix,1)
        num_ceq_muscle_groups = num_ceq_muscle_groups + (sum(muscleGroupsMatrix(k,:)) - 1)*7;
    end
end

% num eq constraints due to satisfying equilibrium at each configuration
num_ceq_muscle_tendon_equilibrium = (num_configurations + 2) * num_muscles;

% total num eq constraints
num_ceq = num_ceq_muscle_tendon_equilibrium + num_ceq_muscle_groups;

% init equality constraints
ceq = zeros(num_ceq,1);
dceq_dx = zeros(num_ceq,length(x));

% for each configuration that muscle-tendon equilibrium must hold for (all
% joint configurations over which passive moments are being optimized and
% max mtu length)
ceq_index = 1;
for c = 1:num_configurations+2 % the +2 is the two configuration corresponding to max/min mtu length
    
    % for each muscle
    for m = 1:num_muscles
        
        % get index to muscle length for this muscle/configuration
        xlmn_index = num_muscles * c - num_muscles + m;
        
        % sanity check
        if ceq_index ~= xlmn_index; error('this should not happen'); end
        
        % get index to params for this muscle
        xlts_index = indices.xlts(m);
        xlm0_index = indices.xlm0(m);
        xe0_index = indices.xe0(m);
        xpen0_index = indices.xpen0(m);
        xs0_index = indices.xs0(m);
        
        % get normalized fiber length
        lmn = x(xlmn_index);
        dlmn_dxlmn = 1;
        
        % get params from scale factor and original parameter values
        lts = x(xlts_index) * originalParams.tendon_slack_length(m);
        dlts_dxlts = originalParams.tendon_slack_length(m);
        
        lm0 = x(xlm0_index) * originalParams.optimal_fiber_length(m);
        dlm0_dxlm0 = originalParams.optimal_fiber_length(m);
        
        e0 = x(xe0_index) * originalParams.tendon_strain_at_one_norm_force(m);
        de0_dxe0 = originalParams.tendon_strain_at_one_norm_force(m);
        
        pen0 = x(xpen0_index) * originalParams.pennation_angle_at_optimal(m);
        dpen0_dxpen0 = originalParams.pennation_angle_at_optimal(m);
        
        s0 = x(xs0_index) * originalParams.passive_fiber_strain_at_one_norm_force(m);
        ds0_dxs0 = originalParams.passive_fiber_strain_at_one_norm_force(m);
        
        % get lmtu and active fiber force length multiplier
        if c <= num_configurations % calibration configurations
            lmtu = mtu_length(m,c);
            fa = 0;
            dfa_dxlmn = 0;
        elseif c == num_configurations + 1 % max mtu length
            lmtu = max_mtu_length(m);
            fa = 0;
            dfa_dxlmn = 0;
        else % min mtu length
            lmtu = min_mtu_length(m);
            [fa,dfa_dlmn] = osimDGF_flactive(lmn);
            dfa_dxlmn = dfa_dlmn * dlmn_dxlmn;
        end
        
        % get normalized passive fiber force
        [fp,dfp_dlmn,dfp_ds0] = osimDGF_fpassive(lmn,s0);
        dfp_dxlmn = dfp_dlmn * dlmn_dxlmn;
        dfp_dxs0 = dfp_ds0 * ds0_dxs0;
        
        % get pennation angle
        asinarg = sin(pen0) / lmn; % sin(p) * lmn^-1
        dasinarg_dxlmn = -asinarg / lmn * dlmn_dxlmn;
        dasinarg_dxpen0 = cos(pen0) / lmn * dpen0_dxpen0;
        pen = asin(asinarg);
        dpen_dxlmn = 1 / sqrt(1-asinarg^2) * dasinarg_dxlmn;
        dpen_dxpen0 = 1 / sqrt(1-asinarg^2) * dasinarg_dxpen0;
        
        % project onto mtu line of action
        fm = (fa + fp) * cos(pen);
        dfm_dxlmn = (dfa_dxlmn + dfp_dxlmn) * cos(pen) - (fa + fp) * sin(pen) * dpen_dxlmn;
        dfm_dxlts = 0;
        dfm_dxlm0 = 0;
        dfm_dxe0 = 0;
        dfm_dxpen0 = -(fa + fp) * sin(pen) * dpen_dxpen0;
        dfm_dxs0 = dfp_dxs0 * cos(pen);
        
        % get actual fiber length
        lm = lmn * lm0;
        dlm_dxlmn = lm0 * dlmn_dxlmn;
        dlm_dxlm0 = lmn * dlm0_dxlm0;
        
        % project onto mtu line of action and subtract from lmtu to get lt
        lt = lmtu - lm * cos(pen);
        dlt_dxlmn = -dlm_dxlmn * cos(pen) + lm * sin(pen) * dpen_dxlmn;
        dlt_dxlm0 = -dlm_dxlm0 * cos(pen);
        dlt_dxpen0 = lm * sin(pen) * dpen_dxpen0;
        
        % get normalized tendon length
        ltn = lt / lts; % lt * lts^-1
        dltn_dxlmn = dlt_dxlmn / lts;
        dltn_dxlts = -ltn / lts * dlts_dxlts;
        dltn_dxlm0 = dlt_dxlm0 / lts;
        dltn_dxpen0 = dlt_dxpen0 / lts;
        
        % get normalized tendon force
        [ft, dft_dltn, dft_de0] = osimDGF_ftendon(ltn,e0);
        dft_dxlmn = dft_dltn * dltn_dxlmn;
        dft_dxlts = dft_dltn * dltn_dxlts;
        dft_dxlm0 = dft_dltn * dltn_dxlm0;
        dft_dxe0 = dft_de0 * de0_dxe0;
        dft_dxpen0 = dft_dltn * dltn_dxpen0;
        dft_dxs0 = 0;
        
        % compute constraint violation
        ceq(xlmn_index) = ft - fm;
        dceq_dx(xlmn_index,xlmn_index) = dft_dxlmn - dfm_dxlmn;
        dceq_dx(xlmn_index,xlts_index) = dft_dxlts - dfm_dxlts;
        dceq_dx(xlmn_index,xlm0_index) = dft_dxlm0 - dfm_dxlm0;
        dceq_dx(xlmn_index,xe0_index) = dft_dxe0 - dfm_dxe0;
        dceq_dx(xlmn_index,xpen0_index) = dft_dxpen0 - dfm_dxpen0;
        dceq_dx(xlmn_index,xs0_index) = dft_dxs0 - dfm_dxs0;
        
        ceq_index = ceq_index + 1;
        
    end
    
end

% muscle group constraints
if ceq_index-1 ~= (num_configurations+2)*num_muscles; error('this should not happen'); end % sanity check
if ~isempty(muscleGroupsMatrix)
    
    % for each group
    for k = 1:size(muscleGroupsMatrix,1)
        
        % indices of muscles in group
        theseGroupIndices = find(muscleGroupsMatrix(k,:));
        
        % for each muscle in the group other than the first (each pair of
        % constraints)
        for j = 2:length(theseGroupIndices)
        
            % lts
            xlts_index1 = indices.xlts(theseGroupIndices(1));
            xlts_index2 = indices.xlts(theseGroupIndices(j));
            xlts1 = x(xlts_index1);
            xlts2 = x(xlts_index2);
            ceq(ceq_index) = xlts1 - xlts2;
            dceq_dx(ceq_index,xlts_index1) = 1;
            dceq_dx(ceq_index,xlts_index2) = -1;
            ceq_index = ceq_index + 1;

            % lm0
            xlm0_index1 = indices.xlm0(theseGroupIndices(1));
            xlm0_index2 = indices.xlm0(theseGroupIndices(j));
            xlm01 = x(xlm0_index1);
            xlm02 = x(xlm0_index2);
            ceq(ceq_index) = xlm01 - xlm02;
            dceq_dx(ceq_index,xlm0_index1) = 1;
            dceq_dx(ceq_index,xlm0_index2) = -1;
            ceq_index = ceq_index + 1;

            % fm0
            xfm0_index1 = indices.xfm0(theseGroupIndices(1));
            xfm0_index2 = indices.xfm0(theseGroupIndices(j));
            xfm01 = x(xfm0_index1);
            xfm02 = x(xfm0_index2);
            ceq(ceq_index) = xfm01 - xfm02;
            dceq_dx(ceq_index,xfm0_index1) = 1;
            dceq_dx(ceq_index,xfm0_index2) = -1;
            ceq_index = ceq_index + 1;

            % pen0
            xpen0_index1 = indices.xpen0(theseGroupIndices(1));
            xpen0_index2 = indices.xpen0(theseGroupIndices(j));
            xpen01 = x(xpen0_index1);
            xpen02 = x(xpen0_index2);
            ceq(ceq_index) = xpen01 - xpen02;
            dceq_dx(ceq_index,xpen0_index1) = 1;
            dceq_dx(ceq_index,xpen0_index2) = -1;
            ceq_index = ceq_index + 1;

            % vm0
            xvm0_index1 = indices.xvm0(theseGroupIndices(1));
            xvm0_index2 = indices.xvm0(theseGroupIndices(j));
            xvm01 = x(xvm0_index1);
            xvm02 = x(xvm0_index2);
            ceq(ceq_index) = xvm01 - xvm02;
            dceq_dx(ceq_index,xvm0_index1) = 1;
            dceq_dx(ceq_index,xvm0_index2) = -1;
            ceq_index = ceq_index + 1;

            % e0
            xe0_index1 = indices.xe0(theseGroupIndices(1));
            xe0_index2 = indices.xe0(theseGroupIndices(j));
            xe01 = x(xe0_index1);
            xe02 = x(xe0_index2);
            ceq(ceq_index) = xe01 - xe02;
            dceq_dx(ceq_index,xe0_index1) = 1;
            dceq_dx(ceq_index,xe0_index2) = -1;
            ceq_index = ceq_index + 1;

            % s0
            xs0_index1 = indices.xs0(theseGroupIndices(1));
            xs0_index2 = indices.xs0(theseGroupIndices(j));
            xs01 = x(xs0_index1);
            xs02 = x(xs0_index2);
            ceq(ceq_index) = xs01 - xs02;
            dceq_dx(ceq_index,xs0_index1) = 1;
            dceq_dx(ceq_index,xs0_index2) = -1;
            ceq_index = ceq_index + 1;
            
        end
        
    end
end

% sanity check
if ceq_index-1 ~= length(ceq); error('this should not happen'); end

% transpose for matlab
dceq_dx = dceq_dx';
dcineq_dx = dcineq_dx';

end

%% SOLVE MUSCLE TENDON EQUILIBRIUM FOR NORM FIBER LENGTH
function lmn = solveMuscleTendonEquilibrium(x0,lmtu,params,i,display)

% Jacobian has been verified with CheckGradients
options = optimoptions('fsolve','SpecifyObjectiveGradient',true,'CheckGradients',false,'FiniteDifferenceType','central','Display',display,'OptimalityTolerance',1e-3);
lmn = fsolve(@equilibrateMuscleTendonForces,x0,options,lmtu,params,i);

end
function [eq, deq_dlmn] = equilibrateMuscleTendonForces(lmn,lmtu,params,i)

% get params for muscle i
lts = params.tendon_slack_length(i);
lm0 = params.optimal_fiber_length(i);
s0 = params.passive_fiber_strain_at_one_norm_force(i);
pen0 = params.pennation_angle_at_optimal(i);
e0 = params.tendon_strain_at_one_norm_force(i);

% get normalized passive fiber force
[fp,dfp_dlmn] = osimDGF_fpassive(lmn,s0);

% get pennation angle
asinarg = sin(pen0) / lmn; % sin(p) * lmn^-1
dasinarg_dlmn = -sin(pen0) / lmn / lmn;
pen = asin(asinarg);
dpen_dlmn = 1 / sqrt(1-asinarg^2) * dasinarg_dlmn;

% project onto mtu line of action
fm = fp * cos(pen);
dfm_dlmn = dfp_dlmn * cos(pen) - fp * sin(pen) * dpen_dlmn;

% get actual fiber length
lm = lmn * lm0;
dlm_dlmn = lm0;

% project onto mtu line of action and subtract from lmtu to get lt
lt = lmtu - lm * cos(pen);
dlt_dlmn = -dlm_dlmn * cos(pen) + lm * sin(pen) * dpen_dlmn;

% get tendon length normalized by lts
ltn = lt / lts; % lt * lts^-1
dltn_dlmn = dlt_dlmn / lts;

% get normalized tendon force
[ft, dft_dltn] = osimDGF_ftendon(ltn,e0);
dft_dlmn = dft_dltn * dltn_dlmn;

% compute constraint violation
eq = ft - fm;
deq_dlmn = dft_dlmn - dfm_dlmn;

end