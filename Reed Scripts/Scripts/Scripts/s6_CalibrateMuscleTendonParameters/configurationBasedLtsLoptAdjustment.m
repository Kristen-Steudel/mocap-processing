function [model,notes] = configurationBasedLtsLoptAdjustment(model,min_allowable_fiber_length,max_allowable_fiber_length,min_mtu_length,max_mtu_length,notes)

% this adjusts lts and lm0 if fiber length violates bounds for max/min mtu
% lengths assuming a rigid tendon. 

% First, it attempts to adjust by setting Lts s.t. it exactly satisfies the
% min allowable fiber length in the min mtu length configuration. If the 
% max allowable fiber length is not violated in the max mtu length 
% configuration following that adjustment, then only the Lts adjustment is
% made. If it is violated, then Lts and Lopt are adjustment simulataneously
% s.t. they exactly satisfy the bounds at the min/max mtu length
% configurations

% note that because of (1) the rigid tendon assumption and (2) the min/max
% mtu lengths may not be the exact min/max that could be observed, then the
% min/max allowable fiber lengths should be MORE RESTRICTIVE than what one
% actually wish to allow. So, for example, if you really wish to limit
% fiber length on the lower end by 0.4, then it may be best to use a bound
% like 0.45 or 0.5 when making the adjustments here.

% right/left symmetry assumed

import org.opensim.modeling.*

% for each muscle
muscles = model.getMuscles;
for k = 1:muscles.getSize
    
    % get this muscle
    thisMuscle = muscles.get(k-1);
    
    % do only right muscles, continue to next muscle if left side
    muscleName = char(thisMuscle.getName);
    if muscleName(end) == 'l'; continue; end
    
    % get current params
    lts = thisMuscle.get_tendon_slack_length;
    lm0 = thisMuscle.get_optimal_fiber_length;
    pen0 = thisMuscle.get_pennation_angle_at_optimal;
    lmtu_max = max_mtu_length.(muscleName);
    lmtu_min = min_mtu_length.(muscleName);
    h = lm0*sin(pen0);
    
    % let z be fiber length along tendon
    % find z correspoding to min/max fiber length setting
    pen_min = asin(sin(pen0) / min_allowable_fiber_length);
    pen_max = asin(sin(pen0) / max_allowable_fiber_length);
    min_allowable_z = cos(pen_min) * min_allowable_fiber_length * lm0;
    max_allowable_z = cos(pen_max) * max_allowable_fiber_length * lm0;
    
    % find z corresponding to min/max mtu length (assuming rigid tendon)
    min_observed_z = lmtu_min - lts;
    max_observed_z = lmtu_max - lts;
    
    % any problematic?
    if min_observed_z < min_allowable_z || max_observed_z >= max_allowable_z
        
        % report
        fprintf('-%s: original optimal fiber length = %4.2f cm, original tendon slack length = %4.2f cm\n',replace(muscleName(1:end-2),'_',' '),lm0*100,lts*100);
        fprintf(notes,'-%s: original optimal fiber length = %4.2f cm, original tendon slack length = %4.2f cm\n',replace(muscleName(1:end-2),'_',' '),lm0*100,lts*100);
            
        % adjust lts based on lmtu_min and lm_min
        lts_new = lmtu_min - min_allowable_fiber_length * cos(pen_min) * lm0;

        % get fiber length at lmtu_max given new lts and see if exceeds
        % threshold
        new_lm_max =  sqrt(h^2 + (lmtu_max - lts_new)^2) / lm0;

        % if exceeds, then adjust lts/lm0 simultaneously
        if new_lm_max > max_allowable_fiber_length
            A = [1, max_allowable_fiber_length * cos(pen_max);...
                 1, min_allowable_fiber_length * cos(pen_min)];
            p = A \ [lmtu_max; lmtu_min];
            lts_new = p(1);
            lm0_new = p(2);
            fprintf('     -for normalized fiber range (rigid tendon): %2.1f - %2.1f\n',min_allowable_fiber_length,max_allowable_fiber_length)
            fprintf('           -if adjust lts only, keep l0 fixed, max norm fiber length assuming rigid tendon (%3.2f) EXCEEDS user set upper bound (%3.2f)\n',new_lm_max,max_allowable_fiber_length)
            fprintf('           -so adjusting both lts and l0 simultaneously\n')
            fprintf('                -new optimal fiber length: %4.2f cm (percent change: %4.2f%%)\n',lm0_new * 100,(lm0_new-lm0)/lm0*100)
            fprintf('                -new tendon slack length:  %4.2f cm (percent change: %4.2f%%)\n',lts_new * 100,(lts_new-lts)/lts*100)
            
            fprintf(notes,'     -for normalized fiber range (rigid tendon): %2.1f - %2.1f\n',min_allowable_fiber_length,max_allowable_fiber_length);
            fprintf(notes,'           -if adjust lts only, keep l0 fixed, max norm fiber length assuming rigid tendon (%3.2f) EXCEEDS user set upper bound (%3.2f)\n',new_lm_max,max_allowable_fiber_length);
            fprintf(notes,'           -so adjusting both lts and l0 simultaneously\n');
            fprintf(notes,'                -new optimal fiber length: %4.2f cm (percent change: %4.2f%%)\n',lm0_new * 100,(lm0_new-lm0)/lm0*100);
            fprintf(notes,'                -new tendon slack length:  %4.2f cm (percent change: %4.2f%%)\n',lts_new * 100,(lts_new-lts)/lts*100);

        % if doesn't,then use lts-only adjustment
        else
            lm0_new = lm0;
            fprintf('     -for normalized fiber range (rigid tendon): %2.1f - %2.1f\n',min_allowable_fiber_length,max_allowable_fiber_length)
            fprintf('           -if adjust lts only, keep l0 fixed, max norm fiber length (%3.2f) is below user set upper bound (%3.2f)\n',new_lm_max,max_allowable_fiber_length)
            fprintf('           -so adjusting only lts only\n')
            fprintf('                -new tendon slack length:  %4.2f cm (percent change: %4.2f%%)\n',lts_new * 100,(lts_new-lts)/lts*100)
            
            fprintf(notes,'     -for normalized fiber range (rigid tendon): %2.1f - %2.1f\n',min_allowable_fiber_length,max_allowable_fiber_length);
            fprintf(notes,'           -if adjust lts only, keep l0 fixed, max norm fiber length (%3.2f) is below user set upper bound (%3.2f)\n',new_lm_max,max_allowable_fiber_length);
            fprintf(notes,'           -so adjusting only lts only\n');
            fprintf(notes,'                -new tendon slack length:  %4.2f cm (percent change: %4.2f%%)\n',lts_new * 100,(lts_new-lts)/lts*100);
        end
    
        % adjust in model
        thisMuscle.set_tendon_slack_length(lts_new);
        thisMuscle.set_optimal_fiber_length(lm0_new);

        % get left side of this muscle
        leftMuscle = muscles.get([muscleName(1:end-2) '_l']);
        leftMuscle.set_tendon_slack_length(lts_new);
        leftMuscle.set_optimal_fiber_length(lm0_new);
    
        % clear new l0/lts
        clear lts_new lm0_new
            
    end
    
end

