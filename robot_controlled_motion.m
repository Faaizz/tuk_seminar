% THIS SCRIPT PLOTS THE RESULT OF CONTROLLING A ROBOT IN THE CONFIGURATION
% SPACE

%% CONFIGURATION SPACE
% X-AXIS: 0...50
% Y_AXIS: 0...50


%% PLOT OBSTACLES

figure

viscircles(obstacles', (obs_radius+zeros(1, size(obstacles, 2))));
   
hold on

%% PLOT REFERENCE
reference_y= trajectory(3,:)';
reference_x= trajectory(2,:)';

plot(reference_x, reference_y);


%% PLOT OUTPUT
assummed_output_y= trajectory(3,:)';
output_x= sim_out.output{1}.Values.Data;

plot(output_x, assummed_output_y);