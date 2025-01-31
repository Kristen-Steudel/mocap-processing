% Kristen Steudel August 1, 2023
% Combine the force data from the ankle straps and force plates into
% one force file

close all; clear all; clc;

import org.opensim.modeling.*

% Load in .mot force file
plate_forces_filename = 'COP_Left_Board_FP2000002_forces_filt12Hz.mot';
grf_data = importdata(plate_forces_filename);
grf_time = grf_data.data(8:end, 1);
% 1 is left and 2 is right
grf_1x = grf_data.data(8:end, 2);
grf_1y = grf_data.data(8:end, 3);
grf_1z = grf_data.data(8:end, 4);
grf_p1x = grf_data.data(8:end, 5);
grf_p1y = grf_data.data(8:end, 6);
grf_p1z = grf_data.data(8:end, 7);
grf_t1x = grf_data.data(8:end, 8);
grf_t1y = grf_data.data(8:end, 9);
grf_t1z = grf_data.data(8:end, 10);
grf_2x = grf_data.data(8:end, 11);
grf_2y = grf_data.data(8:end, 12);
grf_2z = grf_data.data(8:end, 13);
grf_p2x = grf_data.data(8:end, 14);
grf_p2y = grf_data.data(8:end, 15);
grf_p2z = grf_data.data(8:end, 16);
grf_t2x = grf_data.data(8:end, 17);
grf_t2y = grf_data.data(8:end, 18);
grf_t2z = grf_data.data(8:end, 19);


% Load in .dat force transducer file, column 1 is time, 2 left, 3 right, 4
% total, 5 trigger
ankle_forces_filename = 'NordBoard_COP_Left_01.dat';
ankle_data = readtable(ankle_forces_filename);
ankle_time = ankle_data.Time;
Trigger_ankle = ankle_data.Trigger;

%Find index of when the nordic forces began and sync that time with Cortex
trigger = find(Trigger_ankle(:) == 0);
time_trigger = ankle_time(trigger(1,1));
ankle_time = ankle_time(trigger,:);

left_ankle = ankle_data.Left(trigger,:);
right_ankle = ankle_data.Right(trigger,:);
total_ankle = ankle_data.Total(trigger,:);
Trigger_ankle = ankle_data.Trigger(trigger,:);

% interpolate data from .dat force file to each time point in the .mot
% force file
left_ankle_interp = interp1(ankle_time, left_ankle, grf_time);
right_ankle_interp = interp1(ankle_time, right_ankle, grf_time);
total_ankle_interp = interp1(ankle_time, total_ankle, grf_time);
Trigger_ankle_interp = interp1(ankle_time, Trigger_ankle, grf_time);

% Load in trc file, these markers are just 5 markers from the COP tester
trc_filename = 'COP_Left_Board_FP2000002.trc';
trc_data_all = TRCload(trc_filename);
%trc_time = 
trc_data = load_trc(trc_filename);
M1X = trc_data(5:end,1);
M1Y = trc_data(5:end,2);
M1Z = trc_data(5:end,3);
M2X = trc_data(5:end,4);
M2Y = trc_data(5:end,5);
M2Z = trc_data(5:end,6);
M3X = trc_data(5:end,7);
M3Y = trc_data(5:end,8);
M3Z = trc_data(5:end,9);
M4X = trc_data(5:end,10);
M4Y = trc_data(5:end,11);
M4Z = trc_data(5:end,12);
M5X = trc_data(5:end,13);
M5Y = trc_data(5:end,14);
M5Z = trc_data(5:end,15);

% add in three columns using data from the trc file to include the
% direction that the force is acting on the tibia


% add two columns of force data for the left and right tibia forces


% Subtract the mass of the nordboard from the Z force in the left and right
% patella forces
grf_1z = grf_1z - 456;
grf_2z = grf_2z - 466;


% Change the px, py, pz values in the patella forces to the knee marker
% location in gathered from the .trc file.
% M2 is the point of force action in this case for COP testing
% use M5 as the point of action for the right patella force since that is
% not being acted on in this case
%input_file = strrep(trc_filename, '.trc', '_ankle_knee_forces.mot');
input_file = 'ankle_knee_forces.mot';
inpath = cd;
fid = fopen([tempdir,input_file],'w');

nforces = 4;
forceName = {'left_patella', 'left_tibia', 'right_patella', 'right_tibia'};
colNames = {'time'};
dTypes = {'force_v','force_p','torque_'};
dims = {'x','y','z'};
for iforce = 1:nforces
    for j = 1:length(dTypes)
        for k = 1:length(dims)
            colNames{end+1} = [forceName{iforce} '_' dTypes{j} dims{k}];
        end
    end
end


% Write the header
fprintf(fid,'%s\n',input_file);
fprintf(fid,'%s\n','version=1');
fprintf(fid,'%s\n',['nRows=' num2str(length(grf_time))]);
fprintf(fid,'%s\n',['nColumns=',num2str(9*nforces+1)]);
fprintf(fid,'%s\n','inDegrees=yes');
fprintf(fid,'%s\n','endheader');
fprintf(fid,repmat('%s\t',1,9*nforces+1),colNames{:});
fprintf(fid,'\n') ;
% 
% % Write the data
% for j=1:npts
%     % Data order is 1Fxyz,1COPxyz,1Mxyz,2Fxyz...
%     fprintf(fid,'%f',time_forces(j));
%     fprintf(fid,'\t%10.6f',GRF_write(j,:));
%     fprintf(fid,'\n');
% end
fprintf(fid, '%f', grf_time);

disp(['Wrote ',num2str(length(grf_time)),' frames of force data to ',input_file]);
fclose(fid);
copyfile([tempdir,input_file],[inpath,input_file])
%delete([tempdir,input_file])



