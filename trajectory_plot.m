%% TIME PLOT OF TRAJECTORY


figure

% PLOT LATERAL
reference_time= trajectory(1,:)';
reference_lat= trajectory(3,:)';

plot(reference_time, reference_lat, '.', 'LineWidth', .01);

xlabel('Time(s)');
ylabel('Lateral Position(m)');
title(['Time Plot of Lateral Displacement']);



figure

% PLOT LONGITUDINAL
reference_time= trajectory(1,:)';
reference_lon= trajectory(2,:)';

plot(reference_time, reference_lon,  '.', 'LineWidth', .05);

xlabel('Time(s)');
ylabel('Longitudinal Position(m)');
title(['Time Plot of Lateral Displacement']);
