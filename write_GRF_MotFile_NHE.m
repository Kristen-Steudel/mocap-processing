clear ; close all ; format compact ; clc
repoDir = [pwd,'\'] ;
addpath([repoDir, 'common']);

% User inputs if you'd like
dataDir = [repoDir 'NordTesting\'] ;
cd(dataDir)

freq_filtering = 12 ; % lpCutoffFreq for generic force data
freq_filtering_walk = 12; %lpCutoffFreq for walking and treadmill force data
freq_filtering_run = 20; % lpCutoffFreq for force and marker data
zero_threshold = 20 ; % forces below this go to 0

% thresholds for treadmill running
thresh_high_TMrun = 300 ; % N Everything below this goes to 0. ~300 needed for sprinting
thresh_low_TMrun = 10 ; % N Everything below this goes to 0 of the filtered version
thresh_COP_TMrun = 200 ; % N When Fz is below this, COP tracks heel and toe markers. ~200 is good for sprointing

plateNamesOG = {'R','L','3'} ; % Reset this to 1,2,3 for generality if desired
plateNamesWalking = {'R','L'} ; % Reset this to '','1_' for old style

rotateOG_xForward = false ; 

manuallySelectTrials = true;
walking= 1 ; % this does not sum the treadmill forces if true

% if not manual, change these
isGait = false ;
% gaitPrefixes = {'walking','running'}
% nonGaitPrefixes = {'squat','STS','DJ','static'}
gaitPrefixes = {'walking'} ;
nonGaitPrefixes = {'DJ','squat','STS', 'static'} ;

% % % % End user inputs

if manuallySelectTrials
% Load file(s) to be converted.
    display('Select *.anc files to convert into motion files.');
    [files,inpath]=uigetfile([dataDir '*.anc'],'Select analog files with forces','multiselect','on');
    files=cellstr(files);
    cd(inpath)
else
    if isGait
        filePrefixes = gaitPrefixes ;
        walking = true ; 
    else
        filePrefixes = nonGaitPrefixes ;
        walking = false ;
    end
    
    files = {} ;
    for i=1:length(filePrefixes)
        temp = dir([dataDir '/' filePrefixes{i} '*.anc']) ;
        files= [files, {temp(:).name}] ;
        inpath = dataDir ;
    end
end

[a b] = size(files);
for i=1:b
    clear FPData
    clear rightmoments
    clear leftmoments
    infile=char(files(:,i));
    [samp_rate, channel_names, range, time, data, inpath, fileroot]=open_anc(inpath,infile);
%     time_forces=[time(1:size(time,1)-1)];
    time_forces = time ;
    samprate_a = samp_rate(strmatch('F1X',channel_names));

        % 16-bit system 2^16 = 65536
    % range is given in milivolts, so multiply by 0.001 to make volts
    data=(ones(size(data,1),1)*range*2).*(data/65536)*0.001; % Convert all data into volts
    
            forcenames = ['F1X';'F1Y';'F1Z';'M1X';'M1Y';'M1Z';'F2X';'F2Y';'F2Z';'M2X';'M2Y';'M2Z';'F3X';'F3Y';'F3Z';'M3X';'M3Y';'M3Z'] ;
        
        % Create raw matrix of forces
        forceraw = zeros(size(data,1),length(forcenames)) ;
        for p = 1:size(forcenames,1) ;
            forceindicies(p) = find(strcmp(channel_names,forcenames(p,:))) ;
            forceraw(:,p) = data(:,forceindicies(p)) ;
        end
        
        filt_freq = freq_filtering ; % lpCutoffFreq for force and marker data
        threshold = zero_threshold ; % Fz threshold in Newtons - this is how we define steps
        
        % Turning Forces from V to Newtons, Filtering, and Zeroing during swing
        % This structure includes forces and COP for FP1 (right foot) and FP2 (left foot) in the
        % following order: Fx Fy Fz COPx COPy COPz Tz
        [forces_proc_meters] = Analog2Force_OG(forceraw,threshold,filt_freq,samp_rate(1)) ;

            % for general OG trials - output by forceplate number
            % Transform forces and COP into x-forward, y-up, and z-right for
            % OpenSim
            R=[1  0  0;
                0  0 -1;
                0  1  0];

            R = R* [0 0 -1;
                    0 1 0;
                    1 0 0] ;


            R9 = zeros(9,9) ; R9(1:3,1:3) = R ; R9(4:6,4:6) = R ; R9(7:9,7:9) = R ;
            R27 = blkdiag(R9,R9,R9) ;
            
            % Get into F,COP,M order (add zeros for the Mx and My entries)
            zeroCols = zeros(size(forces_proc_meters,1),2) ;
            forces_proc_meters = horzcat(forces_proc_meters(:,1:6),zeroCols,...
                                         forces_proc_meters(:,7:13),zeroCols,...
                                         forces_proc_meters(:,14:20),zeroCols,...
                                         forces_proc_meters(:,21)) ;
            
            % Rotate into Opensim frame
            GRF_write = forces_proc_meters * R27 ;
 
        
    end % overground
    
    
%% Write forces files

    % write out a per-plate Grf file
        % Write Forces File
        npts = size(GRF_write,1);

        input_file = strrep(infile, '.anc', ['_forces_filt' num2str(freq_filtering) 'Hz.mot']);

        fid = fopen([tempdir,input_file],'w');
        
        colNames = {'time'} ;
        nPlates = 3 ;
        dTypes = {'ground_force_v','ground_force_p','ground_torque_'} ;
        dims = {'x','y','z'} ;
        for iPlate = 1:nPlates
            for j = 1:length(dTypes)
                for k = 1:length(dims) 
                    colNames{end+1} = [plateNamesOG{iPlate} '_' dTypes{j} dims{k}] ;
                end
            end
        end
           
        % Write the header
        fprintf(fid,'%s\n',input_file);
        fprintf(fid,'%s\n','version=1');
        fprintf(fid,'%s\n',['nRows=' num2str(length(GRF_write))]);
        fprintf(fid,'%s\n',['nColumns=',num2str(9*nPlates+1)]);
        fprintf(fid,'%s\n','inDegrees=yes');
        fprintf(fid,'%s\n','endheader');
        fprintf(fid,repmat('%s\t',1,9*nPlates+1),colNames{:});
        fprintf(fid,'\n') ;
        
        % Write the data
        for j=1:npts
            % Data order is 1Fxyz,1COPxyz,1Mxyz,2Fxyz...
            fprintf(fid,'%f',time_forces(j));
            fprintf(fid,'\t%10.6f',GRF_write(j,:));
            fprintf(fid,'\n');
        end

        disp(['Wrote ',num2str(npts),' frames of force data to ',input_file]);
        fclose(fid);
        copyfile([tempdir,input_file],[inpath,input_file])
        delete([tempdir,input_file])
