%% VERIFY SCALED MODEL

% generic model is scaled using addbiomechanics.org from which the scaled
% model along with other optimization info (IK, ID, etc.) are downloaded
% and moved to the addbiomechanics folder (see README).

% this script moves the scaled model to the Models folder and renames it
% it also places the model in certain configurations and sweeps certain
% coordinate ranges to verify moment arms are correctly signed

clear
close all
clc

import org.opensim.modeling.*
addpath('G:\Shared drives\HPL_Drive\Nord Sprinting Study\Data Processing Scripts\mocap-processing\Reed Scripts\Scripts.zip\Scripts\s3_VerifyScaledModel\')
addpath('C:\Users\15086\AppData\Local\Temp\Temp232749e3-44c4-42b7-b085-1b823d887c06_Scripts.zip\Scripts\s3_VerifyScaledModel\')
addpath('G:\Shared drives\HPL_Drive\Nord Sprinting Study\Data Processing Scripts\mocap-processing\Reed Scripts')

% path to scaled model
%scaledModelFilepath
scaledModelFilepath = 'C:\Users\15086\Documents\GitHub\mocap-processing\S001\Models\sprinter_scaled.osim';


% copy scaled model from addbiomechanics and rename
model = Model(scaledModelFilepath);
model.setName('sprinter_scaled_JA1');
model.print('sprinter_scaled.osim');

%% verify scaled MTU geometry

v3_verify_scaled_MTU_geometry

