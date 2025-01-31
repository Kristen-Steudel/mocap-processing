%% This script Rotates TRC files to Opensim land in batch
clear all;
close all;
clc;

rotate_xForward = 0 % for overground data facing the screen

% basedir = 'C:\Users\15086\Documents\NMBL_Research\NordBoard_Files\K_Nord_Zeroed';
basedir = 'C:\Users\15086\Documents\Github\mocap-processing\BertecTesting2023-09-14';
[files, inpath]=uigetfile('*.trc','Select input file','multiselect','on');
files=cellstr(files);
[a b] = size(files);

%should be 46 markers 
%pos should be (#x138) not 168 or 156
for index=1:b;
    infile=char(files(:,index));
    [pos,time,f,n,nmrk,mrk_names,file,inpath]=load_trc(infile,inpath);

% rotate the marker data into an OpenSim model coordinate system

% %For Nordics
% R = [0 0 1; 
%     1 0 0; 
%     0 1 0];
%For Sprinting and Nordics

R = [1 0 0; 
    0 0 -1; 
    0 1 0];

% % TEST
% R = [1 0 0;
%      0 1 0;
%      0 0 1];

%rotate x forward
if rotate_xForward
    R = R* [0 0 -1;
            0 1 0;
            1 0 0] ;
end

mrkdata = pos;
for i=1:3:size(mrkdata,2)-2;
    mrkdata(:,i:i+2)=mrkdata(:,i:i+2)*R;
end
S= strcat('rotated_',infile);
S = S(1:end-4);

done = writeTRCFile(time,mrkdata,mrk_names,inpath,S);
if done==1
    display(['File ' infile ' written with OpenSimRotation']);
end
end