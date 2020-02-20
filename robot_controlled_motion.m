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
reference_lon= trajectory(2,:)';
reference_lat= trajectory(3,:)';

plot(reference_lon, reference_lat);


%% PLOT OUTPUT
output_lon= sim_out.output_lon{1}.Values.Data;
output_lat= sim_out.output{1}.Values.Data;

plot(output_lon, output_lat);
xlabel('Longitudinal Position');
ylabel('Lateral Position');

