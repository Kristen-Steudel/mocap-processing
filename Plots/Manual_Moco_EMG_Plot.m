% Kristen Steudel, September 12, 2023
% Plot the moco results and emg results together

close all; clear all; clc;
addpath('C:\Users\15086\Documents\GitHub\mocap-processing\common')
import opensim.*

%% Load in the moco results
% Define the input file path and name
inpath = 'C:\Users\15086\Documents\Github\mocap-processing\Plots';  %Specify the file path you have the moco results in
infile = 'rotated_Sprint_04500001_moco.sto';                        %Specify the file name of the moco results you want to look at

% Call the load_sto function
[data, headers] = load_sto(inpath, infile);

% Now you have your data and headers
% You can work with 'data' and 'headers' in your script

% Identify the columns that contain muscle force information
%time = data(:,1); %Assuming time is in the first column
%forces = data(:,2:end); %Assuming forces start from the second column

%% Load in the EMG signals that have been normalized and converted from bits
% to volts
emg_infile = 'emg_data4500001.sto';                                     %Specify the file name of the emg data you want to look at

% Call the load_sto function
[emg_data, emg_headers] = load_sto(inpath, emg_infile);


%% Plot the semitendinosus activation (ST)

%% Plot the biceps femoris activation (BF)



