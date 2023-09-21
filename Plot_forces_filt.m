% Plot ground reaction forces from grftomot.m
% Kristen Steudel July 26, 2023

close all; clear all; clc;
repoDir = [pwd,''];
addpath([repoDir, '']);

% User inputs if you'd like
%dataDir = [repoDir '/S001/S001_anc_files'];%
dataDir = [repoDir '/BertecTesting2023-09-14/'];%
cd(dataDir)
filter = '20';
speed = '600';
%grf_data = importdata(strcat('rotated_Sprint_0',speed,'0001_forces_filt',filter,'Hz.mot'));  
grf_data = importdata(strcat('Trimmed_running_0',speed,'000001_forces_filt',filter,'Hz.mot'));  

time = grf_data.data(8:end, 1);
grf_rx = grf_data.data(8:end, 2);
grf_ry = grf_data.data(8:end, 3);
grf_rz = grf_data.data(8:end, 4);
grf_r = grf_rx + grf_ry + grf_rz;
plot(time, grf_r)

grf_lx = grf_data.data(8:end, 11);
grf_ly = grf_data.data(8:end, 12);
grf_lz = grf_data.data(8:end, 13);
grf_l = grf_lx + grf_ly + grf_lz;
ylabel('Newtons')
xlabel('time (seconds)')
hold on
plot(time, grf_l)
title(strcat(filter,' Hz ', speed, ' m/s'))

exportgraphics(gcf, strcat(filter, '_Hz_','GRFs_', speed, '.png'), 'Resolution', 300);