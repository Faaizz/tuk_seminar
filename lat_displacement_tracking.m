% THIS SCRIPT PLOTS THE RESULT OF CONTROLLING A ROBOT IN THE CONFIGURATION
% SPACE

%% CONFIGURATION SPACE
% X-AXIS: 0...50
% Y_AXIS: 0...50


%% PLOT OBSTACLES

figure

subplot(2,1,1)

hold on

%% PLOT REFERENCE
reference_time= trajectory(1,:)';
reference_lat= trajectory(3,:)';

plot(reference_time, reference_lat, 'LineStyle', '--', 'LineWidth', 1.5);

%% PLOT OUTPUT
output_lat= sim_out.output{1}.Values.Data;

plot(reference_time, output_lat, 'LineStyle', ':', 'Color', 'r', 'LineWidth', 1.5);

legend('reference', 'output');
xlabel('Time');
ylabel('Lateral Position');



%% PLOT CONTROL INPUT (WHEEL ANGLE)

subplot(2,1,2)

control_input= sim_out.control_signal{1}.Values.Data;

stairs(reference_time, control_input, 'Color', 'g', 'LineWidth', 1.5);
legend('control signal');


