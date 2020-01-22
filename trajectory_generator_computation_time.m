% THIS SCRIPT COMPUTES THE TIME TAKEN TO GENERATE TRAJECTORIES FOR RANDOM
% STARTING POINTS

%% CONSTANTS

% Robot velocity (m/s)
rob_vel= 2;

% Configuration space
min_x= 0;
max_x= 50;
min_y= 0;
max_y= 50;

%% SAMPLE TIME= 0.05

fprintf('\n\n\nSAMPLING TIME= 0.05\n\n');

% Sampling time
h= 0.05;

% First

% Random starting point
rob_pos= [(min_x + rand()*(max_x-min_x)) (min_y + rand()*(max_y-min_y))]';

handle_point_one= @() trajectory_generator_apf(rob_vel, rob_pos, h);

% Time execution
fprintf('First\n');
fprintf('Robot initial position: [%.4f %.4f]\n', rob_pos(1), rob_pos(2));
fprintf('Computation time: %.4f seconds\n\n', timeit(handle_point_one));

% Second

% Random starting point
rob_pos= [(min_x + rand()*(max_x-min_x)) (min_y + rand()*(max_y-min_y))]';

handle_point_one= @() trajectory_generator_apf(rob_vel, rob_pos, h);

% Time execution
fprintf('Second\n');
fprintf('Robot initial position: [%.4f %.4f]\n', rob_pos(1), rob_pos(2));
fprintf('Computation time: %.4f seconds\n\n', timeit(handle_point_one));

% Third

% Random starting point
rob_pos= [(min_x + rand()*(max_x-min_x)) (min_y + rand()*(max_y-min_y))]';

handle_point_one= @() trajectory_generator_apf(rob_vel, rob_pos, h);

% Time execution
fprintf('Third\n');
fprintf('Robot initial position: [%.4f %.4f]\n', rob_pos(1), rob_pos(2));
fprintf('Computation time: %.4f seconds\n\n', timeit(handle_point_one));


%% SAMPLE TIME= 0.1

fprintf('\n\n\nSAMPLING TIME= 0.1\n\n');

% Sampling time
h= 0.1;

% First

% Random starting point
rob_pos= [(min_x + rand()*(max_x-min_x)) (min_y + rand()*(max_y-min_y))]';

handle_point_one= @() trajectory_generator_apf(rob_vel, rob_pos, h);

% Time execution
fprintf('First\n');
fprintf('Robot initial position: [%.4f %.4f]\n', rob_pos(1), rob_pos(2));
fprintf('Computation time: %.4f seconds\n\n', timeit(handle_point_one));

% Second

% Random starting point
rob_pos= [(min_x + rand()*(max_x-min_x)) (min_y + rand()*(max_y-min_y))]';

handle_point_one= @() trajectory_generator_apf(rob_vel, rob_pos, h);

% Time execution
fprintf('Second\n');
fprintf('Robot initial position: [%.4f %.4f]\n', rob_pos(1), rob_pos(2));
fprintf('Computation time: %.4f seconds\n\n', timeit(handle_point_one));

% Third

% Random starting point
rob_pos= [(min_x + rand()*(max_x-min_x)) (min_y + rand()*(max_y-min_y))]';

handle_point_one= @() trajectory_generator_apf(rob_vel, rob_pos, h);

% Time execution
fprintf('Third\n');
fprintf('Robot initial position: [%.4f %.4f]\n', rob_pos(1), rob_pos(2));
fprintf('Computation time: %.4f seconds\n\n', timeit(handle_point_one));



%% SAMPLE TIME= 0.2

fprintf('\n\n\nSAMPLING TIME= 0.2\n\n');

% Sampling time
h= 0.2;

% First

% Random starting point
rob_pos= [(min_x + rand()*(max_x-min_x)) (min_y + rand()*(max_y-min_y))]';

handle_point_one= @() trajectory_generator_apf(rob_vel, rob_pos, h);

% Time execution
fprintf('First\n');
fprintf('Robot initial position: [%.4f %.4f]\n', rob_pos(1), rob_pos(2));
fprintf('Computation time: %.4f seconds\n\n', timeit(handle_point_one));

% Second

% Random starting point
rob_pos= [(min_x + rand()*(max_x-min_x)) (min_y + rand()*(max_y-min_y))]';

handle_point_one= @() trajectory_generator_apf(rob_vel, rob_pos, h);

% Time execution
fprintf('Second\n');
fprintf('Robot initial position: [%.4f %.4f]\n', rob_pos(1), rob_pos(2));
fprintf('Computation time: %.4f seconds\n\n', timeit(handle_point_one));

% Third

% Random starting point
rob_pos= [(min_x + rand()*(max_x-min_x)) (min_y + rand()*(max_y-min_y))]';

handle_point_one= @() trajectory_generator_apf(rob_vel, rob_pos, h);

% Time execution
fprintf('Third\n');
fprintf('Robot initial position: [%.4f %.4f]\n', rob_pos(1), rob_pos(2));
fprintf('Computation time: %.4f seconds\n\n', timeit(handle_point_one));










