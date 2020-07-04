% THIS SCRIPT PLOTS THE RESULT OF CONTROLLING A ROBOT IN THE CONFIGURATION
% SPACE

%% CONFIGURATION SPACE
% X-AXIS: 0...50
% Y_AXIS: 0...50


%% ROBOT MOTION PLOT

% PLOT OBSTACLES

figure

viscircles(obstacles', (obs_radius+zeros(1, size(obstacles, 2))), 'Color', 'k');
   
hold on

% PLOT REFERENCE
reference_lon= trajectory(2,:)';
reference_lat= trajectory(3,:)';

plot(reference_lon, reference_lat, 'Color', 'b', 'LineStyle', '--', 'LineWidth', 1.5);


% PLOT OUTPUT
output_lon= sim_out.output_lon{1}.Values.Data;
output_lat= sim_out.output{1}.Values.Data;

%plot(output_lon, output_lat);
plot(reference_lon, output_lat, 'Color', 'r', 'LineStyle', ':', 'LineWidth', 1.5);
% plot(output_lon, output_lat, 'Color', 'r', 'LineStyle', ':', 'LineWidth', 1.5);

legend('reference', 'output');
xlabel('Longitudinal Position(m)');
ylabel('Lateral Position(m)');
title('Robot Motion Plot');



%% TIME PLOT OF LATERAL TRACKING

% ERROR NORM
scaled_error_norm= sqrt((sum((reference_lat - output_lat).^2, 1))./size(trajectory, 2));

figure

subplot(2,1,1)

hold on

% PLOT REFERENCE
reference_time= trajectory(1,:)';
reference_lat= trajectory(3,:)';

plot(reference_time, reference_lat, 'LineStyle', '--', 'LineWidth', 1.5);

% PLOT OUTPUT
output_lat= sim_out.output{1}.Values.Data;

plot(reference_time, output_lat, 'LineStyle', ':', 'Color', 'r', 'LineWidth', 1.5);

legend('reference', 'output');
xlabel('Time(s)');
ylabel('Lateral Position(m)');
title(['Time Plot of Lateral Displacement Tracking', ' ', '(Error Norm: ', num2str(scaled_error_norm),')']);


% PLOT CONTROL INPUT (WHEEL ANGLE)
subplot(2,1,2)

control_input= sim_out.control_signal{1}.Values.Data;

stairs(reference_time, control_input, 'Color', 'g', 'LineWidth', 1.5);
xlabel('Time(s)');
ylabel('Front Wheel Angle(rad)');
legend('control signal');
title('Time Plot of Control Signal');


%% TIME PLOT OF YAW ANGLE AND YAW RATE

figure

% Yaw Angle
subplot(2, 1, 1);

% Yaw Angle Reference
yaw_ang_ref= yaw_ang;

% Plot Reference
plot(reference_time, yaw_ang_ref, 'LineStyle', '--', 'LineWidth', 1.5);
hold on
% Plot Output
yaw_angle= sim_out.states{3}.Values.Data;
plot(reference_time, yaw_angle, 'Color', 'r', 'LineWidth', 1.5);
xlabel('Time(s)');
ylabel('Yaw Angle(rad)');
legend('reference', 'output');
title('Time Plot of Yaw Angle');

% Yaw Rate
subplot(2, 1, 2);

yaw_rate= sim_out.states{6}.Values.Data;

plot(reference_time, yaw_rate, 'Color', 'b', 'LineWidth', 1.5);
xlabel('Time(s)');
ylabel('Yaw Velocity(rad/sec)');
legend('yaw velocity');
title('Time Plot of Yaw Velocity');




