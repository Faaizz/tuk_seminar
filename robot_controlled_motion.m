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
reference_lon= trajectory(3,:)';
reference_lat= trajectory(2,:)';

plot(reference_lat, reference_lon);


%% PLOT OUTPUT
output_lon= sim_out.output_lon{1}.Values.Data;
output_lon= max(output_lon)-output_lon;
output_lat= sim_out.output{1}.Values.Data;

plot(output_lat, output_lon);