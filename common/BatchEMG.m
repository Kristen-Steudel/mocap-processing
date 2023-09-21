% September 16, 2023, Kristen Steudel
% Learn to use structures

close all; clear all; clc;

[file_list, inpath] = uigetfile('.sto', 'Grab the .sto EMG files you want to process', 'MultiSelect','on');

data_struct = [];

for i = 1:length(file_list)
    infile = char(cell2mat(file_list(i)));
    file = fullfile(inpath, infile);
    data_in = load_sto(inpath, infile);
    % Get subject ID
    sub_id = inpath(end-15:end-11);

    % Create a structure for each subject if it doesn't exist
    if ~isfield(data_struct, sub_id)
        data_struct.(sub_id) = struct();
    end

    %parse data
    data_struct.(sub_id).ADD = data_in(:, 14);

    %Remove NANs
    data_struct.(sub_id).ADD(isnan(data_struct.(sub_id).ADD)) = [];

end
%%
FILT = [];
FILT.EMGfiltFreq_LP = 10;
FILT.EMGfiltFreq_BP = [10,20];
% pathNames are stored in file_list, FILT is a structure containing the
% proper filtering frequencies, 14 muscleNames starting with ADD
muscleNames = 
% processMaxEMGTrials(FILT,muscleNames,pathNames)
