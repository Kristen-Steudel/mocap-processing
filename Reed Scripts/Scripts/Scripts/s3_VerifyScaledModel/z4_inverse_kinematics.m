%% INVERSE KINEMATICS

% this script just moves the addbiomechanics IK results to this folder,
% replace .mot extension with .sto and computes MTU geometry for 9 m/s
% trial

clear 
close all
clc

import org.opensim.modeling.*


% get experimental data, contains information about foot on and off in the
% strides, this processes multiple running strides.

load(fullfile('data.mat'));

% model
model = Model(fullfile('..','..','..','Models','sprinter_scaled.osim'));

% for each run
run_names = fieldnames(data);
nRuns = length(run_names);
mkdir('Results');
for k = 1:nRuns
    
    % copy/paste, change extension
    addbiomech = fullfile('..','..','..','..','..','..','Subject001','IK',[run_names{k} '_ik.mot']);
    ikfile = fullfile('..','..','..','..','..','..','Subject001','IK',[run_names{k} '_coordinates.sto']);
    copyfile(addbiomech,ikfile)
    
end

%% Create checks for the data structure
% Check if data is a structure
if ~isstruct(data)
    error('The variable "data" is not a structure.');
end

% % Check if the expected fields and subfields exist
% if ~isfield(data, run_names{k}) || ~isfield(data.(run_names{k}), 'step') || ~isfield(data.(run_names{k}).step(j), 'startTime')
%     error('The structure "data" does not have the expected fields.');
% end


%% STEP/STRIDE KINEMATICS + REPORT/DOCUMENT COORDINATE RANGES

% for each run
for k = 1:nRuns
    
    % load coordinates
    ikfile = fullfile('..','..','..','..','..','..','Subject001','IK',[run_names{k} '_coordinates.sto']);
    tst = TimeSeriesTable(ikfile);
    
    % for each step: clone, trim, print
    for j = 1:2
        step = tst.clone;
        step.trimFrom(data.(run_names{k}).(['step' num2str(j)]).startTime);
        step.trimTo(data.(run_names{k}).(['step' num2str(j)]).endTime);
        STOFileAdapter.write(step,fullfile('Results',[run_names{k} '_step' num2str(j) '_coordinates.sto']));
        clear step;
    end
    
    % now for stride
    tst.trimFrom(data.(run_names{k}).step1.startTime);
    tst.trimTo(data.(run_names{k}).step2.endTime);
    STOFileAdapter.write(tst,fullfile('Results',[run_names{k} '_stride_coordinates.sto']));
    
    % coordinate ranges
    fprintf('-coordinate ranges:\n')
    ranges = fopen(fullfile('Results',[run_names{k} '_coordinate_ranges.txt']),'w');
    coord = importCoordinatesSTO(ikfile);
    for j = 1:coord.nCoordinates
        thisCoordName = coord.coordinateNames{j};
        thisCoord = coord.coordinate.(thisCoordName);
        cmin = min(thisCoord);
        cmax = max(thisCoord);
        if endsWith(thisCoordName,{'_tx','_ty','_tz'})
            fprintf('%s = %f, %f m\n',thisCoordName,cmin,cmax)
            fprintf(ranges,'%s = %f, %f m\n',thisCoordName,cmin,cmax);
        else
            if strcmp(coord.rotationalUnits,'degrees')
                cmin = cmin * pi/180;
                cmax = cmax * pi/180;
            end
            fprintf('%s = %f, %f rad (%f, %f deg)\n',thisCoordName,cmin,cmax,cmin*180/pi,cmax*180/pi)
            fprintf(ranges,'%s = %f, %f rad (%f, %f deg)\n',thisCoordName,cmin,cmax,cmin*180/pi,cmax*180/pi);
        end
        
    end
    fclose(ranges);
    
end   

%% PLOT COORDINATES FOR 9 m/s TRIAL

coordinates = importCoordinatesSTO(fullfile('Results','run1_stride_coordinates.sto'));
plotcoord = {'hip_flexion_r','hip_rotation_r','hip_adduction_r','knee_angle_r','ankle_angle_r','subtalar_angle_r','mtp_angle_r'};
for j = 1:length(plotcoord)
    fig = figure;
    fig.Position = [1200 500 300 150];
    plot(coordinates.time,coordinates.coordinate.(plotcoord{j})*180/pi,'k','LineWidth',1.5)
    xlabel('Time (s)')
    ylabel('Angle (deg)')
    title([run_names{k} ': ' replace(plotcoord{j},'_',' ')])
end

%% MTU KINEMATICS FOR 9 m/s TRIAL

% muscles and coordinates to compute moment arms for (right leg)
muscles.piri_r.coordinatesActuated = {'hip_flexion_r','hip_adduction_r','hip_rotation_r'};
muscles.iliacus_r.coordinatesActuated = {'hip_flexion_r','hip_adduction_r','hip_rotation_r'};
muscles.psoas_r.coordinatesActuated = {'hip_flexion_r','hip_adduction_r','hip_rotation_r'};
muscles.addbrev_r.coordinatesActuated = {'hip_flexion_r','hip_adduction_r','hip_rotation_r'};
muscles.addlong_r.coordinatesActuated = {'hip_flexion_r','hip_adduction_r','hip_rotation_r'};
muscles.addmagDist_r.coordinatesActuated = {'hip_flexion_r','hip_adduction_r','hip_rotation_r'};
muscles.addmagIsch_r.coordinatesActuated = {'hip_flexion_r','hip_adduction_r','hip_rotation_r'};
muscles.addmagMid_r.coordinatesActuated = {'hip_flexion_r','hip_adduction_r','hip_rotation_r'};
muscles.addmagProx_r.coordinatesActuated = {'hip_flexion_r','hip_adduction_r','hip_rotation_r'};
muscles.glmax1_r.coordinatesActuated = {'hip_flexion_r','hip_adduction_r','hip_rotation_r'};
muscles.glmax2_r.coordinatesActuated = {'hip_flexion_r','hip_adduction_r','hip_rotation_r'};
muscles.glmax3_r.coordinatesActuated = {'hip_flexion_r','hip_adduction_r','hip_rotation_r'};
muscles.glmed1_r.coordinatesActuated = {'hip_flexion_r','hip_adduction_r','hip_rotation_r'};
muscles.glmed2_r.coordinatesActuated = {'hip_flexion_r','hip_adduction_r','hip_rotation_r'};
muscles.glmed3_r.coordinatesActuated = {'hip_flexion_r','hip_adduction_r','hip_rotation_r'};
muscles.glmin1_r.coordinatesActuated = {'hip_flexion_r','hip_adduction_r','hip_rotation_r'};
muscles.glmin2_r.coordinatesActuated = {'hip_flexion_r','hip_adduction_r','hip_rotation_r'};
muscles.glmin3_r.coordinatesActuated = {'hip_flexion_r','hip_adduction_r','hip_rotation_r'};
muscles.recfem_r.coordinatesActuated = {'hip_flexion_r','hip_adduction_r','hip_rotation_r','knee_angle_r'};
muscles.bflh_r.coordinatesActuated = {'hip_flexion_r','hip_adduction_r','hip_rotation_r','knee_angle_r'};
muscles.grac_r.coordinatesActuated = {'hip_flexion_r','hip_adduction_r','hip_rotation_r','knee_angle_r'};
muscles.sart_r.coordinatesActuated = {'hip_flexion_r','hip_adduction_r','hip_rotation_r','knee_angle_r'};
muscles.semimem_r.coordinatesActuated = {'hip_flexion_r','hip_adduction_r','hip_rotation_r','knee_angle_r'};
muscles.semiten_r.coordinatesActuated = {'hip_flexion_r','hip_adduction_r','hip_rotation_r','knee_angle_r'};
muscles.tfl_r.coordinatesActuated = {'hip_flexion_r','hip_adduction_r','hip_rotation_r','knee_angle_r'};
muscles.bfsh_r.coordinatesActuated = {'knee_angle_r'};
muscles.vasmed_r.coordinatesActuated = {'knee_angle_r'};
muscles.vasint_r.coordinatesActuated = {'knee_angle_r'};
muscles.vaslat_r.coordinatesActuated = {'knee_angle_r'};
muscles.gasmed_r.coordinatesActuated = {'knee_angle_r','ankle_angle_r','subtalar_angle_r'};
muscles.gaslat_r.coordinatesActuated = {'knee_angle_r','ankle_angle_r','subtalar_angle_r'};
muscles.soleus_r.coordinatesActuated = {'ankle_angle_r','subtalar_angle_r'};
muscles.tibant_r.coordinatesActuated = {'ankle_angle_r','subtalar_angle_r'};
muscles.perbrev_r.coordinatesActuated = {'ankle_angle_r','subtalar_angle_r'};
muscles.perlong_r.coordinatesActuated = {'ankle_angle_r','subtalar_angle_r'};
muscles.tibpost_r.coordinatesActuated = {'ankle_angle_r','subtalar_angle_r'};
muscles.edl_r.coordinatesActuated = {'ankle_angle_r','subtalar_angle_r','mtp_angle_r'};
muscles.ehl_r.coordinatesActuated = {'ankle_angle_r','subtalar_angle_r','mtp_angle_r'};
muscles.fdl_r.coordinatesActuated = {'ankle_angle_r','subtalar_angle_r','mtp_angle_r'};
muscles.fhl_r.coordinatesActuated = {'ankle_angle_r','subtalar_angle_r','mtp_angle_r'};

% muscles and coordinates to compute moment arms for (left leg)
muscles.piri_l.coordinatesActuated = {'hip_flexion_l','hip_adduction_l','hip_rotation_l'};
muscles.iliacus_l.coordinatesActuated = {'hip_flexion_l','hip_adduction_l','hip_rotation_l'};
muscles.psoas_l.coordinatesActuated = {'hip_flexion_l','hip_adduction_l','hip_rotation_l'};
muscles.addbrev_l.coordinatesActuated = {'hip_flexion_l','hip_adduction_l','hip_rotation_l'};
muscles.addlong_l.coordinatesActuated = {'hip_flexion_l','hip_adduction_l','hip_rotation_l'};
muscles.addmagDist_l.coordinatesActuated = {'hip_flexion_l','hip_adduction_l','hip_rotation_l'};
muscles.addmagIsch_l.coordinatesActuated = {'hip_flexion_l','hip_adduction_l','hip_rotation_l'};
muscles.addmagMid_l.coordinatesActuated = {'hip_flexion_l','hip_adduction_l','hip_rotation_l'};
muscles.addmagProx_l.coordinatesActuated = {'hip_flexion_l','hip_adduction_l','hip_rotation_l'};
muscles.glmax1_l.coordinatesActuated = {'hip_flexion_l','hip_adduction_l','hip_rotation_l'};
muscles.glmax2_l.coordinatesActuated = {'hip_flexion_l','hip_adduction_l','hip_rotation_l'};
muscles.glmax3_l.coordinatesActuated = {'hip_flexion_l','hip_adduction_l','hip_rotation_l'};
muscles.glmed1_l.coordinatesActuated = {'hip_flexion_l','hip_adduction_l','hip_rotation_l'};
muscles.glmed2_l.coordinatesActuated = {'hip_flexion_l','hip_adduction_l','hip_rotation_l'};
muscles.glmed3_l.coordinatesActuated = {'hip_flexion_l','hip_adduction_l','hip_rotation_l'};
muscles.glmin1_l.coordinatesActuated = {'hip_flexion_l','hip_adduction_l','hip_rotation_l'};
muscles.glmin2_l.coordinatesActuated = {'hip_flexion_l','hip_adduction_l','hip_rotation_l'};
muscles.glmin3_l.coordinatesActuated = {'hip_flexion_l','hip_adduction_l','hip_rotation_l'};
muscles.recfem_l.coordinatesActuated = {'hip_flexion_l','hip_adduction_l','hip_rotation_l','knee_angle_l'};
muscles.bflh_l.coordinatesActuated = {'hip_flexion_l','hip_adduction_l','hip_rotation_l','knee_angle_l'};
muscles.grac_l.coordinatesActuated = {'hip_flexion_l','hip_adduction_l','hip_rotation_l','knee_angle_l'};
muscles.sart_l.coordinatesActuated = {'hip_flexion_l','hip_adduction_l','hip_rotation_l','knee_angle_l'};
muscles.semimem_l.coordinatesActuated = {'hip_flexion_l','hip_adduction_l','hip_rotation_l','knee_angle_l'};
muscles.semiten_l.coordinatesActuated = {'hip_flexion_l','hip_adduction_l','hip_rotation_l','knee_angle_l'};
muscles.tfl_l.coordinatesActuated = {'hip_flexion_l','hip_adduction_l','hip_rotation_l','knee_angle_l'};
muscles.bfsh_l.coordinatesActuated = {'knee_angle_l'};
muscles.vasmed_l.coordinatesActuated = {'knee_angle_l'};
muscles.vasint_l.coordinatesActuated = {'knee_angle_l'};
muscles.vaslat_l.coordinatesActuated = {'knee_angle_l'};
muscles.gasmed_l.coordinatesActuated = {'knee_angle_l','ankle_angle_l','subtalar_angle_l'};
muscles.gaslat_l.coordinatesActuated = {'knee_angle_l','ankle_angle_l','subtalar_angle_l'};
muscles.soleus_l.coordinatesActuated = {'ankle_angle_l','subtalar_angle_l'};
muscles.tibant_l.coordinatesActuated = {'ankle_angle_l','subtalar_angle_l'};
muscles.perbrev_l.coordinatesActuated = {'ankle_angle_l','subtalar_angle_l'};
muscles.perlong_l.coordinatesActuated = {'ankle_angle_l','subtalar_angle_l'};
muscles.tibpost_l.coordinatesActuated = {'ankle_angle_l','subtalar_angle_l'};
muscles.edl_l.coordinatesActuated = {'ankle_angle_l','subtalar_angle_l','mtp_angle_l'};
muscles.ehl_l.coordinatesActuated = {'ankle_angle_l','subtalar_angle_l','mtp_angle_l'};
muscles.fdl_l.coordinatesActuated = {'ankle_angle_l','subtalar_angle_l','mtp_angle_l'};
muscles.fhl_l.coordinatesActuated = {'ankle_angle_l','subtalar_angle_l','mtp_angle_l'};
    
% load coordinates
coordinates = importCoordinatesSTO(fullfile('Results','run1_stride_coordinates.sto'));

% get mtu geometry
mtuKinematics.muscles = osimCalculateMTUGeometry(model,coordinates.coordinate,muscles);
mtuKinematics.time = coordinates.time;
mtuKinematics.muscleNames = fieldnames(mtuKinematics.muscles);

% get mtu velocity
for j = 1:length(mtuKinematics.muscleNames)
    %mtuKinematics.muscles.(mtuKinematics.muscleNames{j}).mtuVelocity = fdiff5(mtuKinematics.muscles.(mtuKinematics.muscleNames{j}).mtuLength,mtuKinematics.time);
    
    % Compute the difference between consecutive muscle lengths
    deltaLength = diff(mtuKinematics.muscles.(mtuKinematics.muscleNames{j}).mtuLength);
    
    % Compute the time differences
    deltaTime = diff(mtuKinematics.time);
    
    % Compute muscle velocities by dividing deltaLength by deltaTime
    mtuKinematics.muscles.(mtuKinematics.muscleNames{j}).mtuVelocity = deltaLength ./ deltaTime;

end

% save
save(fullfile('Results','run1_mtuKinematics.mat'),'mtuKinematics')

% step mtu kinematics
mtukin = mtuKinematics;
for i = 1:2
    mtuKinematics = mtukin;
    t1 = data.run0900cms.step(i).startTime;
    t2 = data.run0900cms.step(i).endTime;
    ind = mtuKinematics.time >= t1 & mtuKinematics.time <= t2;
    mtuKinematics.time = mtuKinematics.time(ind);
    for k = 1:length(mtuKinematics.muscleNames)
        msc = mtuKinematics.muscleNames{k};
        mtuKinematics.muscles.(msc).mtuLength = mtuKinematics.muscles.(msc).mtuLength(ind);
        mtuKinematics.muscles.(msc).mtuVelocity = mtuKinematics.muscles.(msc).mtuVelocity(ind);
        for j = 1:length(mtuKinematics.muscles.(msc).coordinatesActuated)
            coord = mtuKinematics.muscles.(msc).coordinatesActuated{j};
            mtuKinematics.muscles.(msc).momentArm.(coord) = mtuKinematics.muscles.(msc).momentArm.(coord)(ind);
        end
    end
    save(fullfile('Results',['run855_step' num2str(i) '_mtuKinematics.mat']),'mtuKinematics');
    clear mtuKinematics
end

% stride mtu kinematics
for i = 1:2
    mtuKinematics = mtukin;
    t1 = data.run0900cms.step(i).startTime;
    t2 = data.run0900cms.step(i).endTime;
    ind = mtuKinematics.time >= t1 & mtuKinematics.time <= t2;
    mtuKinematics.time = mtuKinematics.time(ind);
    for k = 1:length(mtuKinematics.muscleNames)
        msc = mtuKinematics.muscleNames{k};
        mtuKinematics.muscles.(msc).mtuLength = mtuKinematics.muscles.(msc).mtuLength(ind);
        mtuKinematics.muscles.(msc).mtuVelocity = mtuKinematics.muscles.(msc).mtuVelocity(ind);
        for j = 1:length(mtuKinematics.muscles.(msc).coordinatesActuated)
            coord = mtuKinematics.muscles.(msc).coordinatesActuated{j};
            mtuKinematics.muscles.(msc).momentArm.(coord) = mtuKinematics.muscles.(msc).momentArm.(coord)(ind);
        end
    end
    save(fullfile('Results','run1_stride_mtuKinematics.mat'),'mtuKinematics');
    clear mtuKinematics
end

%% PLOT 9 m/s MTU KINEMATICS

% load mtuKinematics for 9 m/s
load(fullfile('Results','run1_mtuKinematics.mat'));

% for each muscle
mnames = mtuKinematics.muscleNames;
time = mtuKinematics.time;
for k = 1:length(mnames)
    fig = figure;
    msc = mtuKinematics.muscles.(mnames{k});
    coord = msc.coordinatesActuated;
    nsp = length(coord) + 1;
    sp = subplot(1,nsp,1);
    plot(time,msc.mtuLength*100,'Color',softblue,'LineWidth',2)
    xlabel('Time (s)')
    ylabel('MTU Len. (cm)')
    title(replace(mnames{k},'_',' '))
    sp.Box = 'off';
    sp.YAxis.LineWidth = 1.5;
    sp.XAxis.LineWidth = 1.5;
    for j = 2:nsp
        sp = subplot(1,nsp,j);
        plot(time,msc.momentArm.(coord{j-1})*100,'Color',softblue,'LineWidth',2)
        ylabel('Mom. Arm (cm)')
        title(replace(coord{j-1},'_',' '))
        sp.Box = 'off';
        sp.YAxis.LineWidth = 1.5;
        sp.XAxis.LineWidth = 1.5;
    end
    fig.Position = [700 600 230*nsp 150];
end