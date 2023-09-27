%% VERIFY SCALED MTU GEOMETRY
% see z3_verify_scaled_model

function [] = v3_verify_scaled_MTU_geometry()

clear
close all
clc

import org.opensim.modeling.*

addpath('G:\Shared drives\HPL_Drive\Nord Sprinting Study\Data Processing Scripts\mocap-processing\Reed Scripts\Models\')

% set cmodel = [] to skip comparing, otherwise set it to the model to
% compare to
% cmodel = Model(fullfile('..','..','Models','generic_sprinter_UNMODIFIED_MUSCLE_GEOMETRY.osim'));
cmodel = Model(fullfile('generic_sprinter_UNMODIFIED_MUSCLE_GEOMETRY.osim'));
scaledModelFilepath = 'C:\Users\15086\Documents\GitHub\mocap-processing\S001\Models\sprinter_scaled.osim';



% load model and get coordinates used
%model = Model(fullfile('..','..','Models','sprinter_scaled.osim'));
%model = Model(fullfile('sprinter_scaled.osim'));
model = Model(scaledModelFilepath);
knee = model.getCoordinateSet.get('knee_angle_r');
ankle = model.getCoordinateSet.get('ankle_angle_r');
subtalar = model.getCoordinateSet.get('subtalar_angle_r');
hipf = model.getCoordinateSet.get('hip_flexion_r');
hipa = model.getCoordinateSet.get('hip_adduction_r');
hipr = model.getCoordinateSet.get('hip_rotation_r');

%% UNIARTICULAR KNEE MUSCLES

mnames = {'vaslat_r','vasmed_r','vasint_r','bfsh_r'};
coordinates.knee_angle_r = knee.getRangeMin:5*pi/180:knee.getRangeMax;
for k = 1:length(mnames); msc.(mnames{k}).coordinatesActuated = {'knee_angle_r'}; end

% plot
plotfun(model,cmodel,msc,coordinates,'knee_angle_r',[])


clear mnames msc coordinates

%% BIARTICULAR KNEE/HIP MUSCLES

mnames = {'bflh_r','semimem_r','semiten_r','recfem_r','sart_r'};
coordinates.knee_angle_r = knee.getRangeMin:5*pi/180:knee.getRangeMax;
coordinates.hip_flexion_r = [0.0, hipf.getRangeMin, hipf.getRangeMax];
for k = 1:length(mnames); msc.(mnames{k}).coordinatesActuated = {'knee_angle_r','hip_flexion_r'}; end

% plot
plotfun(model,cmodel,msc,coordinates,'knee_angle_r','hip_flexion_r')

clear mnames msc coordinates

%% BIARTICULAR KNEE/ANKLE MUSCLES

mnames = {'gaslat_r','gasmed_r'};
coordinates.knee_angle_r = knee.getRangeMin:5*pi/180:knee.getRangeMax;
coordinates.ankle_angle_r = [0.0, ankle.getRangeMin, ankle.getRangeMax];
for k = 1:length(mnames); msc.(mnames{k}).coordinatesActuated = {'knee_angle_r','ankle_angle_r'}; end

% plot
plotfun(model,cmodel,msc,coordinates,'knee_angle_r','ankle_angle_r')

clear mnames msc coordinates

%% UNIARTICULAR ANKLE MUSCLES

mnames = {'soleus_r','tibant_r','tibpost_r'};
coordinates.ankle_angle_r = ankle.getRangeMin:5*pi/180:ankle.getRangeMax;
coordinates.subtalar_angle_r = [0.0, subtalar.getRangeMin, subtalar.getRangeMax];
for k = 1:length(mnames); msc.(mnames{k}).coordinatesActuated = {'ankle_angle_r'}; end

% plot
plotfun(model,cmodel,msc,coordinates,'ankle_angle_r','subtalar_angle_r')

clear mnames msc coordinates

%% UNIARTICULAR HIP MUSCLES (1)

mnames = {'glmax1_r','glmax3_r','psoas_r','iliacus_r'};
coordinates.hip_flexion_r = hipf.getRangeMin:5*pi/180:hipf.getRangeMax;
coordinates.hip_rotation_r = [0.0, hipr.getRangeMin, hipr.getRangeMax];
coordinates.hip_adduction_r = 0.0;
for k = 1:length(mnames); msc.(mnames{k}).coordinatesActuated = {'hip_flexion_r','hip_adduction_r','hip_rotation_r'}; end

% plot
plotfun(model,cmodel,msc,coordinates,'hip_flexion_r','hip_rotation_r')

clear mnames msc coordinates

%% UNIARTICULAR HIP MUSCLES (2)

mnames = {'glmax1_r','glmax3_r','psoas_r','iliacus_r'};
coordinates.hip_flexion_r = hipf.getRangeMin:5*pi/180:hipf.getRangeMax;
coordinates.hip_rotation_r = [0.0, hipr.getRangeMin, hipr.getRangeMax];
coordinates.hip_adduction_r = hipa.getRangeMin;
for k = 1:length(mnames); msc.(mnames{k}).coordinatesActuated = {'hip_flexion_r','hip_adduction_r','hip_rotation_r'}; end

% plot
plotfun(model,cmodel,msc,coordinates,'hip_flexion_r','hip_rotation_r')

clear mnames msc coordinates

%% UNIARTICULAR HIP MUSCLES (3)

mnames = {'glmax1_r','glmax3_r','psoas_r','iliacus_r'};
coordinates.hip_flexion_r = hipf.getRangeMin:5*pi/180:hipf.getRangeMax;
coordinates.hip_rotation_r = [0.0, hipr.getRangeMin, hipr.getRangeMax];
coordinates.hip_adduction_r = hipa.getRangeMax;
for k = 1:length(mnames); msc.(mnames{k}).coordinatesActuated = {'hip_flexion_r','hip_adduction_r','hip_rotation_r'}; end

% plot
plotfun(model,cmodel,msc,coordinates,'hip_flexion_r','hip_rotation_r')

clear mnames msc coordinates

end

%% PLOT MOMENT ARMS
function [] = plotfun(model,cmodel,msc,coord,sweep1,sweep2)
    
% get muscle names
mnames = fieldnames(msc);

% if no sweep2 specified, create a dummy one
if isempty(sweep2)
    sweep2coord = 0;
    sweep2 = 'pelvis_tx';
    coord.(sweep2) = sweep2coord;
    sweep2dummy = true;
else
    
    % otherwise get angles to sweep over sweep2 coordinate
    sweep2coord = coord.(sweep2);
    sweep2dummy = false;
    
end

% remove sweep 2 from all coordinates set, will be added from sweep2coord
coord = rmfield(coord,sweep2);
n = length(coord.(sweep1));
coordnames = fieldnames(coord);
for k = 1:length(coordnames)
    if length(coord.(coordnames{k})) ~= n
        coord.(coordnames{k}) = coord.(coordnames{k})(1) * ones(1,n);
    end
end
    
% colors for different configs, black always the default one
%colors = [softblack; softblue; softred; softlightblue]; 
colors = ['k';'b';'r';'g'];     %My matlab doesn't know what softblack is

% for each sweep2 coord to sweep over
for j = 1:length(sweep2coord)
        
    % get a temp ref to set of coordinates
    thisCoord = coord;
    
    % set sweep2 coord, print current config (other than sweep1)
    thisCoord.(sweep2) = sweep2coord(j) * ones(1,n);

    % get mtu geometry
    m = osimCalculateMTUGeometry(model,thisCoord,msc);
    if ~isempty(cmodel)
        cm = osimCalculateMTUGeometry(cmodel,thisCoord,msc);
    end

    % for each muscle
    for k = 1:length(mnames)
        
        % get coordinates actuated, this determines num subplots
        thisCoordsActuated = m.(mnames{k}).coordinatesActuated;
        numSubPlots = length(thisCoordsActuated);

        % muscle specific figure
        if j == 1
            fig(k).fig = figure;
            fig(k).fig.Position = [650 650 300*numSubPlots 180];
        else
            figure(fig(k).fig);
        end
        
        % for each moment arm to plot
        for i = 1:numSubPlots
            
            % plot
            sp = subplot(1,numSubPlots,i);
            thisMomentArm = m.(mnames{k}).momentArm.(thisCoordsActuated{i})*100;
            plot(thisCoord.(sweep1)*180/pi,thisMomentArm,'LineStyle','-','LineWidth',1.5,'Color',colors(j,:))
            hold on
            if ~isempty(cmodel)
                compThisMomentArm = cm.(mnames{k}).momentArm.(thisCoordsActuated{i})*100;
                plot(thisCoord.(sweep1)*180/pi,compThisMomentArm,'LineStyle','--','LineWidth',1.5,'Color',colors(j,:))
            end
            sp.Box = 'off';
            title(mnames{k}(1:end-2))
            xlabcoord = sweep1(1:end-2);
            xlabcoord = replace(xlabcoord,'_',' ');
            xlabel([xlabcoord ' (\circ)'])
            ylabel([replace(thisCoordsActuated{i}(1:end-2),'_',' ') ' MA (cm)'])
        end

    end
  
end

% set legend if more than one sweep 2
if length(sweep2coord) > 1
    
    % get legend name for each solid line
    thisLegend = cell(1,length(sweep2coord));
    for j = 1:length(sweep2coord)
        thisLegend{j} = [replace(sweep2(1:end-2),'_',' '), ' = ' num2str(round(sweep2coord(j)*180/pi)) '\circ'];
    end
    if isempty(cmodel)
        lineIndices = (length(sweep2coord):-1:1);
    else
        lineIndices = (length(sweep2coord):-1:1) * 2;
    end
    
    % for each muscle
    for k = 1:length(mnames)
        
        % for each subplot
        for i = 1:length(fig(k).fig.Children)
            
            % reset lims
            ylimmin = inf;
            ylimmax = -inf;
            
            % for each plot
            for j = 1:length(fig(k).fig.Children(i).Children)
                if max(fig(k).fig.Children(i).Children(j).YData) > ylimmax
                    ylimmax = max(fig(k).fig.Children(i).Children(j).YData);
                end
                if min(fig(k).fig.Children(i).Children(j).YData) < ylimmin
                    ylimmin = min(fig(k).fig.Children(i).Children(j).YData);
                end
            end
            
            % set lims
            ylimmin = floor(ylimmin) - 2;
            ylimmax = ceil(ylimmax) + 2;
            fig(k).fig.Children(i).YLim = [ylimmin ylimmax];
        end
        
        % get solid line plot objects and legendize
        subset = fig(k).fig.Children(end).Children(lineIndices);
        figure(fig(k).fig);
        leg = legend(subset,thisLegend);
        leg.Box = 'off';
    end
end

clear fig
    
end