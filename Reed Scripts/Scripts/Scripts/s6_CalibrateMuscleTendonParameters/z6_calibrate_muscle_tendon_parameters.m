%% CALIBRATE MUSCLE TENDON PARAMETERS USING EXPERIMENTAL PASSIVE MOMENTS

close all
clear
clc

import org.opensim.modeling.*

% make results directory
mkdir('Results');

% init notes
notes = fopen(fullfile('Results','notes.txt'),'w');
fprintf(notes,'-date and time: %s\n',datestr(datetime,'yyyymmddTHHMMSS'));

% set calibrated model file name
calibratedModelFile = fullfile('..','..','Models','sprinter_scaled_calibrated.osim');

% model name post calibration
calibratedModelName = 'sprinter_scaled_calibrated_JA1';

% get model
% G:\Shared drives\HPL_Drive\Nord Sprinting Study\Data Processing Scripts\mocap-processing\Reed Scripts
%modelFile = fullfile('..','..','Models','sprinter_scaled.osim');
modelFile = fullfile('G:','Shared drives','HPL_drive','Nord Sprinting Study','Data Processing Scripts','mocap-processing','Reed Scripts','Models','sprinter_scaled.osim');
model = Model(modelFile);
model.initSystem;
fprintf(notes,'-model file: %s\n',modelFile);

%% FORCE NAMES MODELING NON MUSCLE SAGITTAL PLANE COORD PASSIVE MOMENTS

% names of forces corresponding to ligament moments (i.e., all due to non
% muscle tissue, collectively referred to as a ligament moment0

% set any to [] if none included in the model
% e.g., rightLigamentMoment.knee = [];

rightLigamentMoment.hip = 'rightPassiveMomentHipFlexion_Andersen1999';
rightLigamentMoment.knee = [];
rightLigamentMoment.ankle = 'rightPassiveMomentAnkle_DeMers2017';

% name of force corresponding to knee moment due to thigh shank contact
% unique to knee
rightKneeThighShankContactMoment = 'rightPassiveMomentKneeFlexion_Zelle2009';

%% MTU KINEMATICS DURING 9 m/s SPRINT

% set minLengthMTU = []; if none to enforce
% set maxLengthMTU = []; if none to enforce
% set maxShorteningVelocityMTU = []; if none to enforce

% if none to enforce for a specific muscle, then set = [], e.g.,
%            minLengthMTU.soleus_r = [];

% min/max mtu lengths during 9 m/s trial and max mtu shortening velocity
% use 75% of max mtu shortening velocity for bound
load(fullfile('..','..','..','..','..','..','Subject001','IK','rotated_Sprint_0855_ik.mat')); %.mat file
for k = 1:length(mtuKinematics.muscleNames)
    thisLengthMTU = mtuKinematics.muscles.(mtuKinematics.muscleNames{k}).mtuLength;
    thisVelocityMTU = mtuKinematics.muscles.(mtuKinematics.muscleNames{k}).mtuVelocity;
    minLengthMTU.(mtuKinematics.muscleNames{k}) = min(thisLengthMTU);
    maxLengthMTU.(mtuKinematics.muscleNames{k}) = max(thisLengthMTU);
    maxShorteningVelocityMTU.(mtuKinematics.muscleNames{k}) = min(thisVelocityMTU);
end

%% PENALTIES FOR DEVIATION FROM NOMINAL VALUES

% weights for different cost terms
weights.squared_error = 10;
weights.tendon_slack_length_deviation_from_one = 10;
weights.optimal_fiber_length_deviation_from_one = 10;
weights.max_isometric_force_deviation_from_one = 10;
weights.tendon_strain_at_one_norm_force_deviation_from_one = 10;
weights.max_contraction_velocity_deviation_from_one = 10;
weights.pennation_angle_at_optimal_deviation_from_one = 10;
weights.passive_fiber_strain_at_one_norm_force_deviation_from_one = 10;

% report
fprintf('-objective term weights:\n');
fprintf('     -squared error: %f\n',weights.squared_error)
fprintf('     -deviation from nominal: tendon slack length: %f\n',weights.tendon_slack_length_deviation_from_one);
fprintf('     -deviation from nominal: optimal fiber length: %f\n',weights.optimal_fiber_length_deviation_from_one);
fprintf('     -deviation from nominal: max isometric force: %f\n',weights.max_isometric_force_deviation_from_one);
fprintf('     -deviation from nominal: tendon strain at one norm force: %f\n',weights.tendon_strain_at_one_norm_force_deviation_from_one);
fprintf('     -deviation from nominal: max contraction velocity: %f\n',weights.max_contraction_velocity_deviation_from_one);
fprintf('     -deviation from nominal: pennation angle at optimal: %f\n',weights.pennation_angle_at_optimal_deviation_from_one);
fprintf('     -deviation from nominal: passive fiber strain at one norm force: %f\n',weights.passive_fiber_strain_at_one_norm_force_deviation_from_one);

fprintf(notes,'-objective term weights:\n');
fprintf(notes,'     -squared error: %f\n',weights.squared_error);
fprintf(notes,'     -deviation from nominal: tendon slack length: %f\n',weights.tendon_slack_length_deviation_from_one);
fprintf(notes,'     -deviation from nominal: optimal fiber length: %f\n',weights.optimal_fiber_length_deviation_from_one);
fprintf(notes,'     -deviation from nominal: max isometric force: %f\n',weights.max_isometric_force_deviation_from_one);
fprintf(notes,'     -deviation from nominal: tendon strain at one norm force: %f\n',weights.tendon_strain_at_one_norm_force_deviation_from_one);
fprintf(notes,'     -deviation from nominal: max contraction velocity: %f\n',weights.max_contraction_velocity_deviation_from_one);
fprintf(notes,'     -deviation from nominal: pennation angle at optimal: %f\n',weights.pennation_angle_at_optimal_deviation_from_one);
fprintf(notes,'     -deviation from nominal: passive fiber strain at one norm force: %f\n',weights.passive_fiber_strain_at_one_norm_force_deviation_from_one);

%% BOUNDS ON MT PARAMETERS

% bounds on allowable percent changes in params or in value of params
% if values specified then will be converted to percentage inside
% optimization routine
% lit. considerations: Lloyd/Besier 03, van Campen 14, Falisse 16, Carbone 16
% popular model: rajagopal, lai/arnold
% model prev used for sprinting: hamner
% consider F0 differences between Hamner vs Rajagopal
% consider difference in Lm0 and Lts between Hamner vs Rajagopal

% for allowable percent change in lm0 and pen0, use mean cv from ward 2009
% cv for lm0 also used for lts since rajagopal determined lts inadvertently
% from lm0 following muscle equilibration, etc.
ward = table3Ward2009;
wardMuscles = fieldnames(ward);
flwardcv = zeros(1,length(wardMuscles));
pawardcv = zeros(1,length(wardMuscles));
for k = 1:length(wardMuscles)
    flwardcv(k) = ward.(wardMuscles{k}).fiber_length(2) / ward.(wardMuscles{k}).fiber_length(1);
    pawardcv(k) = ward.(wardMuscles{k}).pennation_angle(2) / ward.(wardMuscles{k}).pennation_angle(1);
end
fprintf('-Ward 2009: optimal fiber length median CV: %f (used as percent change bound in optimization for lm0 and lts)\n',median(flwardcv))
fprintf('-Ward 2009: pennation angle median CV: %f (used as percent change bound in optimization for pen0)\n',median(pawardcv))


fprintf(notes,'-Ward 2009: optimal fiber length median CV: %f (used as percent change bound in optimization for lm0 and lts)\n',median(flwardcv));
fprintf(notes,'-Ward 2009: pennation angle median CV: %f (used as percent change bound in optimization for pen0)\n',median(pawardcv));


% default tendon strain at F0 in this model is 0.049, but 0.033 also
% popular. To allow 0.033 from 0.049, set the percent change bound to 0.33
% we allow 33% increases as well b/c some muscles def have more compliance,
% e.g. achilles up to 0.1 (their default value)
% percent change allowed in fiber strain at F0 set to same as tendon strain
% at F0

% max isometric force eventually doubled below, rajagopal initially assumed
% 60 MPa specific tension but note it can be as low as 15 and as high as
% 100, plus it's dependent on lm0 and pen0 and muscle volumes which all
% have their own variation as well. So doubling F0, not so crazy (RE:
% haralabidis justification). A lower bound on percent change of 0.5 makes
% sense (allows returning to default value), but plus 0.5 may be too high
% since already doubled (although, note Tim Dorn tripled...), 0.5 may also
% weaken the muscles too much to allow the model to run 9 m/s. 
% conservative solution: allow +/- 10% changes

parameterBounds.percent_change_tendon_slack_length = [-1 1] * median(flwardcv); 
parameterBounds.percent_change_optimal_fiber_length = [-1 1] * median(flwardcv); 
parameterBounds.percent_change_max_isometric_force = [-0.10 0.10]; 
parameterBounds.percent_change_tendon_strain_at_one_norm_force = [-1 1] * 0.33; 
parameterBounds.value_max_contraction_velocity = [12 16];
parameterBounds.percent_change_pennation_angle_at_optimal = [-1 1] * median(pawardcv);
parameterBounds.percent_change_passive_fiber_strain_at_one_norm_force = [-1 1] * 0.33;

% report
bnds = fieldnames(parameterBounds);
fprintf('-parameter bounds:\n');
fprintf(notes,'-parameter bounds:\n');
for k = 1:7
    bnd = bnds{k};
    if startsWith(bnd,'percent_change_')
        type = 'percent_change';
    elseif startsWith(bnd,'value_')
        type = 'value';
    else
        error('bounds must be percent_change_ or value_');
    end
    if contains(bnd,'tendon_slack')
        prop = 'tendon_slack_length';
    elseif contains(bnd,'optimal_fiber')
        prop = 'optimal_fiber_length';
    elseif contains(bnd,'isometric')
        prop = 'max_isometric_force';
    elseif contains(bnd,'tendon_strain')
        prop = 'tendon_strain_at_one_norm_force';
    elseif contains(bnd,'contraction')
        prop = 'max_contraction_velocity';
    elseif contains(bnd,'pennation')
        prop = 'pennation_angle_at_optimal';
    elseif contains(bnd,'fiber_strain')
        prop = 'passive_fiber_strain_at_one_norm_force';
    else
        error('all 7 params must be specified in parameterBounds\n');
    end
    fprintf('     (%d) %s bound placed on %s: %f, %f\n',k,replace(type,'_',' '),replace(prop,'_',' '),parameterBounds.([type '_' prop])(1),parameterBounds.([type '_' prop])(2));
    fprintf(notes,'     (%d) %s bound placed on %s: %f, %f\n',k,replace(type,'_',' '),replace(prop,'_',' '),parameterBounds.([type '_' prop])(1),parameterBounds.([type '_' prop])(2));
end

%% SPECIAL BOUNDS

% e.g., specialParameterBounds.soleus_r.value_tendon_strain_at_one_norm_force = [0.049, 0.10];
% set specialParameterBounds = []; if no muscle specific parameter bounds

specialParameterBounds.soleus_r.value_tendon_strain_at_one_norm_force = [0.049 0.10];
specialParameterBounds.gasmed_r.value_tendon_strain_at_one_norm_force = [0.049 0.10];
specialParameterBounds.gaslat_r.value_tendon_strain_at_one_norm_force = [0.049 0.10];

% report
if ~isempty(specialParameterBounds)
    fprintf('-muscle specific parameter bounds:\n');
    fprintf(notes,'-muscle specific parameter bounds:\n');
    specialMuscles = fieldnames(specialParameterBounds);
    for k = 1:length(specialMuscles)
        fprintf('     (%d) %s:\n',k,specialMuscles{k});
        fprintf(notes,'     (%d) %s:\n',k,specialMuscles{k});
        bnd = fieldnames(specialParameterBounds.(specialMuscles{k}));
        for j = 1:length(bnd)
            thisbnd = bnd{j};
            if startsWith(thisbnd,'value_')
                thisprop = replace(thisbnd,'value_','');
                fprintf('           -value bound on %s: %f, %f\n',thisprop,specialParameterBounds.(specialMuscles{k}).(thisbnd)(1),specialParameterBounds.(specialMuscles{k}).(thisbnd)(2));
                fprintf(notes,'           -value bound on %s: %f, %f\n',thisprop,specialParameterBounds.(specialMuscles{k}).(thisbnd)(1),specialParameterBounds.(specialMuscles{k}).(thisbnd)(2));
            elseif startsWith(thisbnd,'percent_change_')
                thisprop = replace(thisbnd,'percent_change_','');
                fprintf('           -percent change bound on %s: %f, %f\n',thisprop,specialParameterBounds.(specialMuscles{k}).(thisbnd)(1),specialParameterBounds.(specialMuscles{k}).(thisbnd)(2));
                fprintf(notes,'           -percent change bound on %s: %f, %f\n',thisprop,specialParameterBounds.(specialMuscles{k}).(thisbnd)(1),specialParameterBounds.(specialMuscles{k}).(thisbnd)(2));
            else
                error('bounds must be percent_change_ or value_');
            end
        end
        
    end
else
    fprintf('-no special parameter bounds\n');
end

%% MUSCLE GROUPS

% e.g., a gastroc group and vasti group could be specified as 
% muscleGroups = {{'gasmed_r','gaslat_r',{'vaslat_r,'vasmed_r','vasint_r'}}

% set muscleGroups = []; if no groups

% muscle groups
muscleGroups{1} = {'gasmed_r','gaslat_r'};
muscleGroups{2} = {'vaslat_r','vasmed_r','vasint_r'};
muscleGroups{3} = {'edl_r','ehl_r'};
muscleGroups{4} = {'fdl_r','fhl_r'};
muscleGroups{5} = {'perbrev_r','perlong_r'};
muscleGroups{6} = {'addbrev_r','addlong_r','addmagDist_r','addmagIsch_r','addmagMid_r','addmagProx_r'};
muscleGroups{7} = {'iliacus_r','psoas_r'};

% report
if ~isempty(muscleGroups)
    fprintf('-muscle groups (n=%d):\n',length(muscleGroups));
    fprintf(notes,'-muscle groups (n=%d):\n',length(muscleGroups));
    for k = 1:length(muscleGroups)
        fprintf('     (%d) %s',k,muscleGroups{k}{1});
        fprintf(notes,'     (%d) %s',k,muscleGroups{k}{1});
        for j = 2:length(muscleGroups{k})
            fprintf(', %s',muscleGroups{k}{j});
            fprintf(notes,', %s',muscleGroups{k}{j});
        end
        fprintf('\n');
        fprintf(notes,'\n');
    end
else
    fprintf('-no muscle groups\n');
    fprintf(notes,'-no muscle groups\n');
end

%% ACTIVE AND PASSIVE FORCE SCALING

% double muscle forces (ca = 2). Note this also doubles the passive fiber
% force at the original passive_fiber_strain_at_one_norm_force (thus cp =
% 2) effectively modeling that the joints are less flexibile, instead
% here we adjust this parameter s.t. the passive fiber force is the same as
% it was in the unscaled case (i.e., prior to doubling max isometric force)
% when fiber strain = the original passive_fiber_strain_... parameter
activeFiberForceScaleFactor = 2.0;
passiveFiberForceScaleFactor = 1.0;
muscles = model.getMuscles;
for k = 0:muscles.getSize-1
    
    % get reference to muscle
    thisMuscle = DeGrooteFregly2016Muscle.safeDownCast(muscles.get(k));
    
    % double force
    thisMuscle.set_max_isometric_force(activeFiberForceScaleFactor * thisMuscle.get_max_isometric_force);
    
    % adjust fiber strain @ F0
    original_strain_at_F0 = thisMuscle.get_passive_fiber_strain_at_one_norm_force;
    new_strain_at_F0 = approxScalePassiveForceByAdjustingStrainAtOneNormForce(activeFiberForceScaleFactor,passiveFiberForceScaleFactor,original_strain_at_F0);
    thisMuscle.set_passive_fiber_strain_at_one_norm_force(new_strain_at_F0);
    
end
fprintf('-max isometric force for all muscles scaled by %4.2f\n',activeFiberForceScaleFactor);
fprintf('-original passive fiber strain at one norm force scaled s.t. passive fiber force is approximately scaled by %4.2f (given the scaling of F0)\n',passiveFiberForceScaleFactor);

fprintf(notes,'-max isometric force for all muscles scaled by %4.2f\n',activeFiberForceScaleFactor);
fprintf(notes,'-original passive fiber strain at one norm force scaled s.t. passive fiber force is approximately scaled by %4.2f (given the scaling of F0)\n',passiveFiberForceScaleFactor);
%% STATE BOUNDS: PENNATION ANGLE + NORM FIBER  LENGTH

% these bound the instantaneous fiber length and pennation angle
% (configuration dependent)

% Millard et al. 2013: upper bound for inst. pennation angle is 84.26 deg
upper_bound_pennation_angle = 84.26 * pi/180;
fprintf('-max allowable instantaneous pennation angle: %4.2f\n',upper_bound_pennation_angle*180/pi);
fprintf(notes,'-max allowable instantaneous pennation angle: %4.2f\n',upper_bound_pennation_angle*180/pi);
lower_bound_pennation_angle = 0; % obviously, but included for consistent specification of bounds [lower, upper]

% get largest pennation angle at optimal across all muscles
max_pennation_angle_at_optimal = 0;
max_pennation_angle_at_optimal_muscle = '';
for k = 0:muscles.getSize-1
    
    % get reference to muscle
    thisMuscle = DeGrooteFregly2016Muscle.safeDownCast(muscles.get(k));
    
    % update max pennation if this muscle's is larger than current max
    thisMusclePen0 = thisMuscle.get_pennation_angle_at_optimal;
    if thisMusclePen0 > max_pennation_angle_at_optimal
        max_pennation_angle_at_optimal = thisMusclePen0;
        max_pennation_angle_at_optimal_muscle = char(thisMuscle.getName);
    end

end
fprintf('-largest pennation angle at optimal fiber length: %4.2f deg, %s\n',max_pennation_angle_at_optimal*180/pi,max_pennation_angle_at_optimal_muscle);
fprintf(notes,'-largest pennation angle at optimal fiber length: %4.2f deg, %s\n',max_pennation_angle_at_optimal*180/pi,max_pennation_angle_at_optimal_muscle);

% use this to set the lower bound on norm fiber length
% is inadvertently the most restrictive lower bound placed norm fib length
% from Millard 2013 min pennation
lower_bound_norm_fiber_length = sin(max_pennation_angle_at_optimal)/sin(upper_bound_pennation_angle);
fprintf('-lower bound on normalized fiber length: %f\n',lower_bound_norm_fiber_length);
fprintf(notes,'-lower bound on normalized fiber length: %f\n',lower_bound_norm_fiber_length);

% set the upper bound on norm fiber length such that the muscle has the
% same active force capacity based on normalized fiber length as at the
% lower bound
flactive_at_lower_bound_norm_fiber_length = osimDGF_flactive(lower_bound_norm_fiber_length);
fprintf('-active force length multiplier at normalized fiber length lower bound: %f\n',flactive_at_lower_bound_norm_fiber_length);
fprintf(notes,'-active force length multiplier at normalized fiber length lower bound: %f\n',flactive_at_lower_bound_norm_fiber_length);

upper_bound_norm_fiber_length = fsolve(@(x)osimDGF_flactive(x)-osimDGF_flactive(lower_bound_norm_fiber_length),1.8,...
    optimoptions('fsolve','Display','off'));
fprintf('-upper bound on normalized fiber length: %f\n',upper_bound_norm_fiber_length);
fprintf(notes,'-upper bound on normalized fiber length: %f\n',upper_bound_norm_fiber_length);

% set on state bounds struct
stateBounds.norm_fiber_length = [lower_bound_norm_fiber_length, upper_bound_norm_fiber_length];
stateBounds.pennation_angle = [lower_bound_pennation_angle, upper_bound_pennation_angle];

%% CONFIGURATION BASED PRE-OPTIMIZATION MODIFICATIONS TO LTS AND LOPT

% some tendon slack lengths may need an initial adjustment to even allow 
% muscle-tendon equilibrium at some configurations used for calibration. 
% For example, take the shortest MTU length for each muscle during the 
% sprint. If this length is less than the tendon slack length, then it is 
% nearly impossible to equilibrateMuscles. 

% In reality, the tendon slack length need be even less
% than the min MTU length since there must be room for the fiber. So, as an
% initial (configuration-based) adjustment, adjust Lts if the fiber length
% is less than a user-set minimum at the min MTU length. To
% ensure, adjustments to Lts do not result in overly large fiber lengths at
% large MTU lengths, we also check the fiber length (assuming a rigid
% tendon) at the max observed MTU length during sprinting. If it exceeds a
% user-set threshold, then the Lts and Lm0 are adjusted simultaneously
% (using methods in my dissertation). If not exceeded, then Lts only
% adjusted. 

% a more restrictive bound is set here since in reality tendon is not rigid
% when equilibrating muscles. Using a 10% adjustment to min/max allowable
% fiber lengths determined above

[model,notes] = configurationBasedLtsLoptAdjustment(model,1.1*stateBounds.norm_fiber_length(1),0.9*stateBounds.norm_fiber_length(2),minLengthMTU,maxLengthMTU,notes);

%% CONFIGURATIONS

% riener edrich sweeps that include most data points:
% ankle moment: ankle = -50 to 15, knee = 0 and 60
% knee moment: knee = 30 to 140, ankle = 0, hip = 0, 45, 90, and 120
% hip moment: hip = -30 to 100, ankle = 0, knee = 0, 45, 90

% configurations to optimize over
sweep.hip_flexion_r.hip_flexion_r = (-20:10:80)*pi/180; % sweep through these hip angles
sweep.hip_flexion_r.knee_angle_r = [30 60 90]*pi/180; % sweep hip at these knee and ankle angles
sweep.hip_flexion_r.ankle_angle_r = 0; % sweep hip at these ankle angles

sweep.knee_angle_r.knee_angle_r = (30:10:140)*pi/180;
sweep.knee_angle_r.hip_flexion_r = [0 40 80]*pi/180;
sweep.knee_angle_r.ankle_angle_r = 0;

sweep.ankle_angle_r.ankle_angle_r = (-30:5:20)*pi/180;
sweep.ankle_angle_r.knee_angle_r = [10 35 60]*pi/180;
sweep.ankle_angle_r.hip_flexion_r = 0;

% concatenate all configurations
configurations.hip_flexion_r = [];
configurations.knee_angle_r = [];
configurations.ankle_angle_r = [];
coordNames = fieldnames(sweep);
for c1 = 1:3
    
    % get index of other coordinates
    if c1 == 1; c2 = 2; c3 = 3; end
    if c1 == 2; c2 = 1; c3 = 3; end
    if c1 == 3; c2 = 1; c3 = 2; end
    
    % get sweep for primary coordinate (c1)
    sweep1 = sweep.(coordNames{c1}).(coordNames{c1});
    
    % get constant values of other coordinates at which sweep1 will occur
    value2 = sweep.(coordNames{c1}).(coordNames{c2});
    value3 = sweep.(coordNames{c1}).(coordNames{c3});
    
    n1 = length(sweep1);
    n2 = length(value2);
    n3 = length(value3);
    
    % for each combination of constant values of other coordinates
    for j = 1:n2
        for i = 1:n3
            
            % create array with equivalent length as sweep1 but that has
            % constant values for other coordinates
            sweep2 = value2(j) * ones(1,n1);
            sweep3 = value3(i) * ones(1,n1);
            
            % concatenate
            configurations.(coordNames{c1}) = horzcat(configurations.(coordNames{c1}),sweep1);
            configurations.(coordNames{c2}) = horzcat(configurations.(coordNames{c2}),sweep2);
            configurations.(coordNames{c3}) = horzcat(configurations.(coordNames{c3}),sweep3);
            
        end
    end
end

% for testing only
% configurations.hip_flexion_r = 60 * pi/180;
% configurations.knee_angle_r = 30 * pi/180;
% configurations.ankle_angle_r = 20 * pi/180;

%% OPTIMIZE

% optimize
[model,notes] = passiveMomentsMuscleTendonParameterOptimization(model,...
                                                                configurations,...
                                                                parameterBounds,...
                                                                specialParameterBounds,...
                                                                stateBounds,...
                                                                weights,...
                                                                rightLigamentMoment,...
                                                                rightKneeThighShankContactMoment,...
                                                                minLengthMTU,...
                                                                maxLengthMTU,...
                                                                maxShorteningVelocityMTU,...
                                                                muscleGroups,...
                                                                notes,...
                                                                'Results');

fclose(notes);

%% SAVE

model.setName(calibratedModelName);
model.print(calibratedModelFile);
