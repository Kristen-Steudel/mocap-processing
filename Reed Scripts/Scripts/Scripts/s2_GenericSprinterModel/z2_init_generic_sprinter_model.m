%% initiate sprinter model before scaling with addbiomechanics

% uses LaiArnold2017 model as a base

% uses custom coordinate ranges (detailed below) which were determined by
% using the lesser/greater limit for the lower/upper bound between the dorn
% and laiArnold models except for hip adduction (uses dorn lower bound of
% -30 deg), knee (uses laiArnold upper bound of 0 whereas dorn allowed for
% 10 deg hyperextension), elbow (same for knee)

% radioulnar joint is welded with palms facing medially
% wrist joint welded in neutral position
% mtp joint unlocked

% convert muscles to DGF, set vmax to 12 Lopt/s, damping = 0.01, muscle
% strain at F0 = 0.6, tendon strain at F0 = 0.049 for all muscles except
% for right/left soleus, med/lat gastroc = 0.1, tendon compliance dynamics
% mode = implicit, activation time constant = 10 ms, deactivation time
% constant = 40 ms, ignore activation dynamics = false

% uses marker set from dorn's JA1_SCALED.osim

% glute med, glute min, and tfl path points adjusted as in Uhlrich et al.
% 2022. Lopt/Lts scaled according to Lmtu change

% lumbar coordinate actuator names changed to match coordinate name

% knee_angle coordinate used for patellar kinematics (knee_angle_r_beta)
% name is changed to satsify convention for bilateral coordinate names
% ending with _r or _l (knee_angle_beta_r). Also updated in constraint set
% and patellofemoral SpatialTransform

% pelvis mesh file set to r/l_pelvis_raja.vtp

% mtu geometry adjusted for muscles specified below in 
% sprinterMuscleGeometriesModified, see individual scripts for details
% about modifications

% passive moments added for joints/coordiantes specified below in 
% passiveMomentsAdded

% generic sprinter model does not contain contact spheres nor any reserves

clear
close all
clc

import org.opensim.modeling.*

% path to lai arnold model
laiarnoldModelFilepath = '';

% notes to record updates
notes = fopen('notes.txt','w');

% model name
modelName = 'generic_sprinter';

% joints to weld
weldTheseJoints = {'radioulnar_r','radioulnar_l','radius_hand_r','radius_hand_l'};

% coordinate actuators to remove
removeTheseCoordinateActuators = {'pro_sup_r','pro_sup_l','wrist_flex_r','wrist_flex_l','wrist_dev_r','wrist_dev_l'};

% muscles for which path points set using Uhlrich et al. 2022 modifications
uhlrichModifiedMusclePathPoints = {'glmed1_r','glmed2_r','glmed3_r','glmin1_r','glmin2_r','glmin3_r','tfl_r','glmed1_l','glmed2_l','glmed3_l','glmin1_l','glmin2_l','glmin3_l','tfl_l'};

% generic muscle parameters
params.appliesForce = true;
params.min_control = 0.01;
params.max_control = 1.0;
params.ignore_activation_dynamics = false;
params.activation_time_constant = 0.01;
params.deactivation_time_constant = 0.04;
params.max_contraction_velocity = 12.0;
params.ignore_passive_fiber_force = false;
params.passive_fiber_strain_at_one_norm_force = 0.6;
params.ignore_tendon_compliance = false;
params.tendon_compliance_dynamics_mode = 'implicit';
params.fiber_damping = 0.01;

% coordinate ranges
coordinateRanges = {'pelvis_tilt',-1.570796,1.570796;...
                    'pelvis_list',-1.570796,1.570796;...
                    'pelvis_rotation',-3.140000,3.140000;...
                    'pelvis_tx',-20.000000,20.000000;...
                    'pelvis_ty',-2.000000,4.000000;...
                    'pelvis_tz',-5.000000,5.000000;...
                    'hip_flexion_r',-0.698130,2.094395;...
                    'hip_adduction_r',-0.523599,0.523599;...
                    'hip_rotation_r',-0.698132,0.698132;...
                    'knee_angle_r',0.000000,2.792500;...
                    'knee_angle_r_beta',0.000000,2.792500;...
                    'ankle_angle_r',-1.047200,0.872665;...
                    'subtalar_angle_r',-0.872660,0.872660;...
                    'mtp_angle_r',-1.047200,1.047200;...
                    'hip_flexion_l',-0.698130,2.094395;...
                    'hip_adduction_l',-0.523599,0.523599;...
                    'hip_rotation_l',-0.698132,0.698132;...
                    'knee_angle_l',0.000000,2.792500;...
                    'knee_angle_l_beta',0.000000,2.792500;...
                    'ankle_angle_l',-1.047200,0.872665;...
                    'subtalar_angle_l',-0.872660,0.872660;...
                    'mtp_angle_l',-1.047200,1.047200;...
                    'lumbar_extension',-1.570796,1.570796;...
                    'lumbar_bending',-1.570796,1.570796;...
                    'lumbar_rotation',-1.570796,1.570796;...
                    'arm_flex_r',-1.570796,1.570796;...
                    'arm_add_r',-2.094395,1.570796;...
                    'arm_rot_r',-1.570796,1.570796;...
                    'elbow_flex_r',0.000000,2.618000;...
                    'arm_flex_l',-1.570796,1.570796;...
                    'arm_add_l',-2.094395,1.570796;...
                    'arm_rot_l',-1.570796,1.570796;...
                    'elbow_flex_l',0.000000,2.618000};

% change these coordinate names
changeTheseCoordinateNames = {'knee_angle_r_beta','knee_angle_beta_r';...
                              'knee_angle_l_beta','knee_angle_beta_l'};
                          
% change these force names
changeTheseForceNames = {'lumbar_ext','lumbar_extension';...
                         'lumbar_rot','lumbar_rotation';...
                         'lumbar_bend','lumbar_bending';...
                         'vaslat140_r','vaslat_r';...
                         'vaslat140_l','vaslat_l';...
                         'gaslat140_r','gaslat_r';...
                         'gaslat140_l','gaslat_l'
                         'bflh140_r','bflh_r';...
                         'bflh140_l','bflh_l';...
                         'bfsh140_r','bfsh_r';...
                         'bfsh140_l','bfsh_l'};
                     
% execute sprinterMuscleModification_(muscle)() function on these muscles
% had previously modified glmax3 and tibpost also, however, this was to
% compensate for incorrectly signed moment arms at only the most extreme
% ranges of motion and that are not obtained in sprinting
sprinterMuscleGeometriesModified = {'recfem','psoas','iliacus','glmax1','sart','vasmed','vaslat','vasint','bflh','bfsh','gasmed','gaslat','semiten','semimem'};

% add passive moments for coords/joints as specified in this array
passiveMomentsAdded = {'MTPFlexion_Sasaki2009','KneeFlexion_Zelle2009','HipRotation_vanArkel2015','HipFlexion_Andersen1999','HipAdduction_Andersen1999','Ankle_DeMers2017'};

%% INIT SPRINTER MODEL

% initiate sprinter model with lai arnold 2017 model
model = Model(laiarnoldModelFilepath);
model.setName(modelName);
fprintf(notes,'-date and time: %s\n',datestr(datetime,'yyyymmddTHHMMSS'));
fprintf(notes,'-base model: LaiArnold2017_refined_arms.osim\n');
fprintf(notes,'-no contact spheres added\n');
fprintf(notes,'-muscle tendon parameters not yet calibrated to match passive moments; should happen after scaling\n');

% remove requested coordinate acutators
if ~isempty(removeTheseCoordinateActuators)
    for k = 1:length(removeTheseCoordinateActuators)
        removeSuccessful = model.getForceSet.remove(model.getForceSet.getIndex(removeTheseCoordinateActuators{k}));
        if ~removeSuccessful; error('Unsuccessful removal of %s',removeTheseCoordinateActuators{k}); end
        fprintf('-removed actuator: %s\n',removeTheseCoordinateActuators{k})
        fprintf(notes,'-removed actuator: %s\n',removeTheseCoordinateActuators{k});
    end
end
model.initSystem;

% weld requested joints
modelProcessor = ModelProcessor(model);
if ~isempty(weldTheseJoints)
    weldedJoints = StdVectorString;
    for k = 1:length(weldTheseJoints)
        weldedJoints.add(weldTheseJoints{k}); 
        fprintf('-welded joint: %s\n',weldTheseJoints{k})
        fprintf(notes,'-welded joint: %s\n',weldTheseJoints{k});
    end
    modelProcessor.append(ModOpReplaceJointsWithWelds(weldedJoints));
end
model.initSystem;

% convert muscles to DGF
fprintf('-muscles converted to DGF\n')
fprintf(notes,'-muscles converted to DGF\n');
modelProcessor.append(ModOpReplaceMusclesWithDeGrooteFregly2016);

% process model
model = modelProcessor.process;
model.initSystem;

% unlock any locked coordinates, should only be mtp
for k = 0:model.getCoordinateSet.getSize-1
    if model.getCoordinateSet.get(k).get_locked
        fprintf('-%s coordinate is now unlocked\n',char(model.getCoordinateSet.get(k).getName))
        fprintf(notes,'-%s coordinate is now unlocked\n',char(model.getCoordinateSet.get(k).getName));
        model.getCoordinateSet.get(k).set_locked(false);
    end
end
model.initSystem;

% update welded radioulnar s.t. palm faces medially
PhysicalOffsetFrame.safeDownCast(model.getJointSet.get('radioulnar_r').getChildFrame).set_orientation(Vec3(-pi/2,0,0));
PhysicalOffsetFrame.safeDownCast(model.getJointSet.get('radioulnar_l').getChildFrame).set_orientation(Vec3(pi/2,0,0));
fprintf('-welded radioulnar joint radius physical offset frame adjusted s.t. palms face medially\n');
fprintf(notes,'-welded radioulnar joint radius physical offset frame adjusted s.t. palms face medially\n');
model.initSystem;

% set coordinate ranges
for k = 1:size(coordinateRanges,1)
    thisCoord = model.getCoordinateSet.get(coordinateRanges{k,1});
    thisCoord.setRangeMin(coordinateRanges{k,2});
    thisCoord.setRangeMax(coordinateRanges{k,3});
    fprintf('-set coordinate range: %s: [%4.3f, %4.3f]\n',coordinateRanges{k,1},coordinateRanges{k,2},coordinateRanges{k,3})
    fprintf(notes,'-set coordinate range: %s: [%4.3f, %4.3f]\n',coordinateRanges{k,1},coordinateRanges{k,2},coordinateRanges{k,3});
end
model.initSystem;

% set generic muscle parameters
muscles = model.getMuscles;
paramNames = fieldnames(params);
fprintf('-setting default muscle parameters:\n')
fprintf(notes,'-default muscle parameters:\n');
for j = 1:length(paramNames)
    if isa(params.(paramNames{j}),'char')
        fprintf('     -%s: %s\n',paramNames{j},params.(paramNames{j}))
        fprintf(notes,'     -%s: %s\n',paramNames{j},params.(paramNames{j}));
    elseif isa(params.(paramNames{j}),'logical')
        fprintf('     -%s: %d\n',paramNames{j},params.(paramNames{j}))
        fprintf(notes,'     -%s: %d\n',paramNames{j},params.(paramNames{j}));
    else
        fprintf('     -%s: %f\n',paramNames{j},params.(paramNames{j}))
        fprintf(notes,'     -%s: %f\n',paramNames{j},params.(paramNames{j}));
    end      
end
for k = 0:muscles.getSize-1
    thisMuscle = DeGrooteFregly2016Muscle.safeDownCast(muscles.get(k));
    for j = 1:length(paramNames)
        thisMuscle.(['set_' paramNames{j}])(params.(paramNames{j})); 
    end
    
    % set tendon strain at F0 to 0.1 if soleus or med/lat gastroc
    if contains(thisMuscle.getName.toCharArray',{'soleus','gaslat','gasmed'})
        thisMuscle.set_tendon_strain_at_one_norm_force(0.1);
    end
    
end
fprintf(notes,'-tendon strain at one nor force for soleus, gaslat, gasmed: 0.1\n');
model.initSystem;

% adjust muscle path points using Uhlrich et al. 2022 modifications
uhlrich = Model(fullfile('..','..','Models','RajagopalModified_generic.osim'));
uhlrich.initSystem;
for k = 1:length(uhlrichModifiedMusclePathPoints)
    
    fprintf('-adjusting path points as in Uhlrich et al. 2022: %s\n',uhlrichModifiedMusclePathPoints{k})
    fprintf(notes,'-adjusting path points as in Uhlrich et al. 2022: %s\n',uhlrichModifiedMusclePathPoints{k});
    
    % get muscle
    thisMuscle = model.getMuscles.get(uhlrichModifiedMusclePathPoints{k});
    newMuscle = uhlrich.getMuscles.get(uhlrichModifiedMusclePathPoints{k});
    
    % get muscle path
    thisPath = GeometryPath.safeDownCast(thisMuscle.getGeometryPath);
    newPath = GeometryPath.safeDownCast(newMuscle.getGeometryPath);
    
    % get Lmtu pre adjustment
    preLmtu = thisPath.getLength(model.initSystem);
    
    % path point sets
    thisPathPointSet = PathPointSet.safeDownCast(thisPath.getPathPointSet);
    newPathPointSet = PathPointSet.safeDownCast(newPath.getPathPointSet);
    
    % set newpath onto thispath
    for j = 0:thisPathPointSet.getSize-1
        thisPoint = PathPoint.safeDownCast(thisPathPointSet.get(j));
        newPoint = PathPoint.safeDownCast(newPathPointSet.get(j));
        if ~strcmp(thisPoint.getBodyName.toCharArray',newPoint.getBodyName.toCharArray'); error('New PathPoint is located in a different body frame than This PathPoint'); end
        thisPoint.set_location(newPoint.get_location);
    end
    
    % get Lmtu post adjustment
    postLmtu = thisPath.getLength(model.initSystem);
    
    % adjust Lopt/Lts
    thisScaleFactor = postLmtu / preLmtu;
    thisMuscle.set_optimal_fiber_length(thisScaleFactor * thisMuscle.get_optimal_fiber_length);
    thisMuscle.set_tendon_slack_length(thisScaleFactor * thisMuscle.get_tendon_slack_length);
    
end

% change specified coordinate names
constraintset = model.getConstraintSet;
jointset = model.getJointSet;
for k = 1:size(changeTheseCoordinateNames,1)
    fprintf('-changed coordinate name: %s to %s\n',changeTheseCoordinateNames{k,1},changeTheseCoordinateNames{k,2})
    fprintf(notes,'-changed coordinate name: %s to %s\n',changeTheseCoordinateNames{k,1},changeTheseCoordinateNames{k,2});
    model.getCoordinateSet.get(changeTheseCoordinateNames{k,1}).setName(changeTheseCoordinateNames{k,2});
    
    % for each constraint
    for j = 0:constraintset.getSize-1
        thisConstraint = constraintset.get(j);
        thisConstraintType = thisConstraint.getConcreteClassName.toCharArray';
        
        % if coordinate coupler
        if strcmp(thisConstraintType,'CoordinateCouplerConstraint')
            thisConstraint = CoordinateCouplerConstraint.safeDownCast(thisConstraint);
            
            % change dependent coordinate name if match one modified
            if strcmp(thisConstraint.getDependentCoordinateName.toCharArray',changeTheseCoordinateNames{k,1})
                thisConstraint.setDependentCoordinateName(changeTheseCoordinateNames{k,2})
            end
            
            % change independent coordinate name if match one modified
            thisInd = thisConstraint.getIndependentCoordinateNames;
            for i = 0:thisInd.getSize-1
                if strcmp(thisInd.get(i).toCharArray',changeTheseCoordinateNames{k,1})
                    thisConstraint.setIndependentCoordinateName(changeTheseCoordinateNames{k,2})
                end
            end
        end
    end
    
    % for each joint
    for j = 0:jointset.getSize-1
        thisJoint = jointset.get(j);
        thisJointType = thisJoint.getConcreteClassName.toCharArray';
       
        % if custom
        if strcmp(thisJointType,'CustomJoint')
            
            % get transform and parametrizing coords
            thisJoint = CustomJoint.safeDownCast(thisJoint);
            thisTransform = SpatialTransform.safeDownCast(thisJoint.getSpatialTransform);
            
            % for each rotation
            for i = 1:3
                thisRotation = thisTransform.(['get_rotation' num2str(i)]);
                
                % for each coordinate
                for c = 0:thisRotation.getCoordinateNamesInArray.getSize-1
                    
                    % modify if necessary
                    if strcmp(thisRotation.get_coordinates(c).toCharArray',changeTheseCoordinateNames{k,1})
                        thisRotation.set_coordinates(c,changeTheseCoordinateNames{k,2});
                    end
                    
                end
                
            end
            
            % for each translation
            for i = 1:3
                thisTranslation = thisTransform.(['get_translation' num2str(i)]);
                
                % for each coordinate
                for c = 0:thisTranslation.getCoordinateNamesInArray.getSize-1
                    
                    % modify if necessary
                    if strcmp(thisTranslation.get_coordinates(c).toCharArray',changeTheseCoordinateNames{k,1})
                        thisTranslation.set_coordinates(c,changeTheseCoordinateNames{k,2});
                    end
                    
                end
                
            end
            
        end
        
    end
            
end
model.initSystem;

% change specified force names
forceset = model.getForceSet;
for k = 1:size(changeTheseForceNames,1)
    fprintf('-changed force name: %s to %s\n',changeTheseForceNames{k,1},changeTheseForceNames{k,2})
    fprintf(notes,'-changed force name: %s to %s\n',changeTheseForceNames{k,1},changeTheseForceNames{k,2});
    forceset.get(changeTheseForceNames{k,1}).setName(changeTheseForceNames{k,2});
end
model.initSystem;

% pelvis mesh files from rajagopal
Mesh.safeDownCast(model.getBodySet.get('pelvis').get_attached_geometry(0)).set_mesh_file('r_pelvis_raja.vtp');
Mesh.safeDownCast(model.getBodySet.get('pelvis').get_attached_geometry(1)).set_mesh_file('l_pelvis_raja.vtp');
Mesh.safeDownCast(model.getBodySet.get('pelvis').get_attached_geometry(2)).set_mesh_file('sacrum.vtp');
model.initSystem;

% add models of passive moments due to non muscle tissue
for k = 1:length(passiveMomentsAdded)
    fprintf('-added passive moment: %s\n',passiveMomentsAdded{k})
    fprintf(notes,'-added passive moment: %s\n',passiveMomentsAdded{k});
    feval(['passiveMoment' passiveMomentsAdded{k}],model);
end
model.initSystem;

% save model without following muscle modifications modifications
model.print('generic_sprinter_UNMODIFIED_MUSCLE_GEOMETRY.osim');

% NOTE: I could not modify the muscle geometry programmatically without
% printing a temporary model, clearing the current model/state instance,
% and then reloading the model. The matlab crash interface may appear
% during the script, but just let it continue to run, it will finish and
% print the model, then click Don't Send after which matlab will crash

% muscle geometry modifications
temp = ['temp' datestr(datetime,'yyyymmddTHHMMSS') '.osim'];
model.print(temp);
clear model
for k = 1:length(sprinterMuscleGeometriesModified)
    
    model = Model(temp);
    
    % pre Lmtu
    state = model.initSystem;
    preLmtuR = model.getMuscles.get([sprinterMuscleGeometriesModified{k} '_r']).getLength(state);
    preLmtuL = model.getMuscles.get([sprinterMuscleGeometriesModified{k} '_l']).getLength(state);
    
    % modify geometry
    feval(str2func(['sprinterMuscleModification_' sprinterMuscleGeometriesModified{k}]),model,notes);
    
    model.print(temp);
    clear model
    clear state
    model = Model(temp);
    
    % muscle ref
    rightMuscle = model.getMuscles.get([sprinterMuscleGeometriesModified{k} '_r']);
    leftMuscle = model.getMuscles.get([sprinterMuscleGeometriesModified{k} '_l']);
    
    % post Lmtu
    state = model.initSystem;
    postLmtuR = rightMuscle.getLength(state);
    postLmtuL = leftMuscle.getLength(state);
    
    fprintf('-modifying muscle geometry: %s, scaling Lopt/Lts based on Lmtu pre- and post-modification: Lmtu (pre) = %4.2f cm, Lmtu (post) = %4.2f\n',sprinterMuscleGeometriesModified{k},preLmtuR*100,postLmtuR*100);
    fprintf(notes,'-modifying muscle geometry: %s, scaling Lopt/Lts based on Lmtu pre- and post-modification: Lmtu (pre) = %4.2f cm, Lmtu (post) = %4.2f\n',sprinterMuscleGeometriesModified{k},preLmtuR*100,postLmtuR*100);
    
    % adjust Lopt/Lts
    rightScaleFactor = postLmtuR / preLmtuR;
    leftScaleFactor = postLmtuL / preLmtuL;
    rightMuscle.set_optimal_fiber_length(rightScaleFactor * rightMuscle.get_optimal_fiber_length);
    rightMuscle.set_tendon_slack_length(rightScaleFactor * rightMuscle.get_tendon_slack_length);
    leftMuscle.set_optimal_fiber_length(leftScaleFactor * leftMuscle.get_optimal_fiber_length);
    leftMuscle.set_tendon_slack_length(leftScaleFactor * leftMuscle.get_tendon_slack_length);
    
    model.print(temp);
    clear model
    clear state
    
end
model = Model(temp);
delete(temp);

% save model without markers
model.initSystem;
model.print('generic_sprinter.osim');
