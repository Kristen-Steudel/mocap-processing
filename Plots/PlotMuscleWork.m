% Plotting the total work done by each muscle after getting the moco
% results
close all; clear all; clc;

import org.opensim.modeling.*;

%Add the paths where the moco results are stored
addpath('C:\Users\15086\Documents\GitHub\mocap-processing\NordTesting\KristenNordTestZeroed\K_Nord_Zeroed\Moco')

modelFile = 'final.osim';
model = Model(modelFile);

%Import the data for the trial we are looking at and create a moco report
fileContent = TimeSeriesTable('rotated_Sprint_08100001_moco.sto');

trajReport = osimMocoTrajectoryReport(model, ...
             'rotated_Sprint_08100001_moco.sto','outputFilepath', 'S001_0810_report.pdf', ...
             'refFiles', {'Trimmed_Sprint_0810_EMG.sto'});
reportFilepath = trajReport.generate();
open(reportFilepath);

% https://www.ps2pdf.com/ Use this site to convert ps report to pdf form


%% Calculating Work and Power 
% Separate header from the data
numHeaderLines = 17;
% lines = splitlines(fileContent);
data = fileContent(numHeaderLines + 1:end,:);

% Identify the columns that contain muscle force information
time = data(:,1); %Assuming time is in the first column
forces = data(:,2:end); %Assuming forces start from the second column

% Calculate Muscle Work, integrate the area under the force vs. time curve
% for each hamstring muscle

work = trapz(time, forces);
total_work_per_muscle = sum(work); % Total work done by each muscle

% Calculate Muscle Power
musclePower = musclesForces .* muscleVelocities; %element-wise multiplication


