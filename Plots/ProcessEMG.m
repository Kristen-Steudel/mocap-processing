% Read in and plot EMG Files
close all; clear all; clc;

%% Import the OpenSim libraries.
import org.opensim.modeling.*;
load('EMG_maxes.mat');

trackLegend = [];
for trial = 5
    AD = [];
    RF = [];
    VL = [];
    VM = [];
    TA = [];
    BF = [];
    ST = [];
    LG = [];
    MG = [];
    SOL = [];
    t = [];
    %% Set filenames
    % Import Trial Info
    TrialInfo = readtable('S001EMGDataInfo.xlsx');
    tStart = TrialInfo.StartTime(trial);%define start time
    tEnd = TrialInfo.EndTime(trial);    %define end time
    sIndex = TrialInfo.StartIndex(trial);
    eIndex = TrialInfo.EndIndex(trial);
    cutoff = TrialInfo.Cutoff(trial);
    trialID = TrialInfo.TrialID(trial);
    strID = num2str(trialID);

    EMGFile = strcat("Trimmed_Sprint_0",strID,"_Analog.sto");

    f = TimeSeriesTable(EMGFile);

    t = f.getIndependentColumn();
    EMG_names = f.getColumnLabels();


    %for i = 0:t.size()-1
    for i = 0:cutoff
        % if EMGFile == 'Trimmed_Sprint_08550001_Analog.sto'
        %     if i > 3000
        %         break
        %     end
        % end
        %t(i+1) = f.getIndependentColumn().get(i);
        AD(i+1,1) = f.getDependentColumn('ADD').get(i);
        RF(i+1,1) = f.getDependentColumn('RF').get(i);
        VL(i+1,1) = f.getDependentColumn('VL').get(i);
        VM(i+1,1) = f.getDependentColumn('VM').get(i);
        TA(i+1,1) = f.getDependentColumn('TA').get(i);
        BF(i+1,1) = f.getDependentColumn('BF').get(i);
        ST(i+1,1) = f.getDependentColumn('ST').get(i);
        LG(i+1,1) = f.getDependentColumn('LG').get(i);
        MG(i+1,1) = f.getDependentColumn('MG').get(i);
        SOL(i+1,1) = f.getDependentColumn('SOL').get(i);
        VL_cont(i+1,1) = f.getDependentColumn('VL_cont').get(i);
        BF_cont(i+1,1) = f.getDependentColumn('BF_cont').get(i);
        MG_cont(i+1,1) = f.getDependentColumn('MG_cont').get(i);
        SOL_cont(i+1,1) = f.getDependentColumn('SOL_cont').get(i);
    end
    % names = [AD, RF, VL, VM, TA, BF, ST, LG, MG, SOL];
    % for i = 1:length(names)
    %     strcat(i,'_rec') = abs(i);
    % end

    % Set the filter specifications, bandpass filter
    fs = 2000;               % Sampling frequency (Hz)
    fpass = [40, 500];       % Passband frequencies (Hz)
    wpass = fpass / (fs/2);  % Passband frequencies (normalized)
    order = 4;               % Filter order bandpass quads order
    [b,a] = butter(order/4, wpass, 'bandpass');

    AD_b = filtfilt(b,a, AD);
    RF_b = filtfilt(b,a, RF);
    VL_b = filtfilt(b,a, VL);
    VM_b = filtfilt(b,a, VM);
    TA_b = filtfilt(b,a, TA);
    BF_b = filtfilt(b,a,BF);
    ST_b = filtfilt(b,a,ST);
    LG_b = filtfilt(b,a, LG);
    MG_b = filtfilt(b,a,MG);
    SOL_b = filtfilt(b,a,SOL);
    VL_cont_b = filtfilt(b,a,VL_cont);
    BF_cont_b = filtfilt(b,a,BF_cont);
    MG_cont_b = filtfilt(b,a,MG_cont);
    SOL_cont_b = filtfilt(b,a,SOL_cont);

    % Rectify
    AD_rec = abs(AD_b);
    RF_rec = abs(RF_b);
    VL_rec = abs(VL_b);
    VM_rec = abs(VM_b);
    TA_rec = abs(TA_b);
    BF_rec = abs(BF_b);
    ST_rec = abs(ST_b);
    LG_rec = abs(LG_b);
    MG_rec = abs(MG_b);
    SOL_rec = abs(SOL_b);
    VL_cont_rec = abs(VL_cont_b);
    BF_cont_rec = abs(BF_cont_b);
    MG_cont_rec = abs(MG_cont_b);
    SOL_cont_rec = abs(SOL_cont_b);

    %Lowpass filter
    [b,a] = butter(2,20/(2000/2),'low'); %cutoff/(sampling/2)
    % make this a variable

    AD_filt = filtfilt(b,a, AD_rec);
    RF_filt = filtfilt(b,a, RF_rec);
    VL_filt = filtfilt(b,a, VL_rec);
    VM_filt = filtfilt(b,a, VM_rec);
    TA_filt = filtfilt(b,a, TA_rec);
    BF_filt = filtfilt(b,a,BF_rec);
    ST_filt = filtfilt(b,a,ST_rec);
    LG_filt = filtfilt(b,a, LG_rec);
    MG_filt = filtfilt(b,a,MG_rec);
    SOL_filt = filtfilt(b,a,SOL_rec);
    VL_cont_filt = filtfilt(b,a,VL_cont_rec);
    BF_cont_filt = filtfilt(b,a, BF_cont_rec);
    MG_cont_filt = filtfilt(b,a,MG_cont_rec);
    SOL_cont_filt = filtfilt(b,a,SOL_cont_rec);
   

    AD_norm = AD_filt/AD_max;
    RF_norm = RF_filt/RF_max;
    VL_norm = VL_filt/VL_max;
    VM_norm = VM_filt/VM_max;
    TA_norm = TA_filt/TA_max;
    BF_norm = BF_filt/BF_max;
    ST_norm = ST_filt/ST_max;
    LG_norm = LG_filt/LG_max;
    MG_norm = MG_filt/MG_max;
    SOL_norm = SOL_filt/SOL_max;
    VL_cont_norm = VL_cont_filt/VL_cont_max;
    BF_cont_norm = BF_cont_filt/BF_cont_max;
    MG_cont_norm = MG_cont_filt/MG_cont_max;
    SOL_cont_norm = SOL_cont_filt/SOL_cont_max;
        

    EMG_norm = [AD_norm, RF_norm, VL_norm, VM_norm, TA_norm, BF_norm, ST_norm, LG_norm, MG_norm, SOL_norm, VL_cont_norm, BF_cont_norm, MG_cont_norm, SOL_cont_norm];
    EMG_name = ['AD', 'RF', 'VL', 'VM', 'TA', 'BF', 'ST', 'LG', 'MG', 'SOL', 'VL_cont', 'BF_cont', 'MG_cont', 'SOL_cont'];
    figure(1);
    hold on
    set(gcf,'Position',[0 0 1000 800])

    percentFlight = linspace(0,1000,eIndex-sIndex); %10 strides
    
    for i = 1:14
        subplot(3,5,i)
        hold on %Doesn't overwrite what's in subplot
        plot(percentFlight, EMG_norm((sIndex:(eIndex-1)),i), 'lineWidth', 2) % to Normalize plots to bodyweight divide by 95
        title(EMG_name(2*i-1:2*i))
        xlabel('time (s)')
        %xlim([sIndex, eIndex]);
        %xlim([t.getIndex(tStart, 100), t.getIndex(tEnd, 0)])
        ylim([0,1])
        xlim([0,1000])
    end
    EMG_norm = EMG_norm((sIndex:(eIndex-1)),:)
    trackLegend = [trackLegend, string(TrialInfo.PercentTopSpeed(trial)*100)+" %"];
    xlim([0,1000]);
    save(strcat("EMGPlot_",num2str(trial),".mat"), "percentFlight", "EMG_norm","EMG_name");


end

subplot(3,5,1)
legend(trackLegend)

%% Create a new file containing the filtered EMG data to plot with Moco Trajectory Results
% Number of data points
numDataPoints = cutoff;

% Number of EMG channels (columns)
numChannels = 10; %Currently only have dominant leg in here

% Create matrix of EMG data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Combine time and EMG data
%dataMatrix = [t, AD_norm, RF_norm, VL_norm, VM_norm, TA_norm, BF_norm, ST_norm, LG_norm, MG_norm, SOL_norm];
dataMatrix = [AD_norm, RF_norm, VL_norm, VM_norm, TA_norm, BF_norm, ST_norm, LG_norm, MG_norm, SOL_norm,VL_cont_norm, BF_cont_norm, MG_cont_norm, SOL_cont_norm];


% Define header for the columns
%header = 'Time\t/forceset/addlong_r/activation\t/forceset/recfem_r/activation\t/forceset/vaslat_r/activation\t/forceset/vasmed_r/activation\t/forceset/tibant_r/activation\t/forceset/bflh_r/activation\t/forceset/semiten_r/activation\t/forceset/gaslat_r/activation\t/forceset/gasmed_r/activation\t/forceset/soleus_r/activation';
header = '/forceset/addlong_r/activation\t/forceset/recfem_r/activation\t/forceset/vaslat_r/activation\t/forceset/vasmed_r/activation\t/forceset/tibant_r/activation\t/forceset/bflh_r/activation\t/forceset/semiten_r/activation\t/forceset/gaslat_r/activation\t/forceset/gasmed_r/activation\t/forceset/soleus_r/activation\t/forceset/vaslat_l/activation\t/forceset/bflh_l/activation\t/forceset/vasmed_l/activation\t/forceset/soleus_l/activation';

header = [header, '\n'];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Save data to a file
filename = strcat('emg_data', strID ,'.sto');
dlmwrite(filename, dataMatrix, 'delimiter', '\t', 'precision', '%.6f');

% Write header to the beginning of the file
fid = fopen(filename, 'r+');
fileContent = fread(fid, '*char');
frewind(fid);
fprintf(fid, header);
fwrite(fid, fileContent);
fclose(fid);

disp(['EMG data saved to ', filename]);
