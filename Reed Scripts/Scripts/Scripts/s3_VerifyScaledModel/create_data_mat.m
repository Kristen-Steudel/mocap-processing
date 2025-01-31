%% Create data.mat for z4 script
clear; 
close all;
clc;

% data = struct();
% data.run1 = 'run1.mot';
% data.run2 = 'run2.mot';
% data.run3 = 'run3.mot';
% 
% data.run1.step.startTime = 0.320;
% % data.run1.endTime = 0.865;
% % data.run2.step.startTime = 0.865;
% % data.run2.step.endTime = 1.375;
% % data.run3.step.startTime = 1.375;
% % data.run3.step.endTime = 1.875;
startTimes = {0.32,0.865, 1.375};
endTimes = {0.865,1.375,1.875};
% 
% save('data.mat', 'data');
% Create a sample data structure with fields and subfields
data = struct();

% Define the number of runs
numRuns = 3;

% Loop to create data for each run
for runIdx = 1:numRuns
    % Create a substructure for each run
    runName = sprintf('run%d', runIdx);
    data.(runName) = struct();
    
    % Define step information for each run
    numSteps = 2;
    for stepIdx = 1:numSteps
        stepName = sprintf('step%d', stepIdx);
        data.(runName).(stepName) = struct();
        
        % Add start and end times for each step (example values)
        data.(runName).(stepName).startTime = startTimes{runIdx};
        data.(runName).(stepName).endTime = endTimes{runIdx};
    end
end

% Save the data structure to a .mat file
save('data.mat', 'data');

% Display the generated data structure
disp('Generated data structure:');
disp(data);