function trajectory= robot_motion(robot_vel, robot_pos, h)
% Generate a Universal Potential Field which directs a robot to its
% destination via an Attractive Potential Field (with its minimum at goal 
% point) and keeps it away from obstacles via a Repulsive Potential Field 
% (with its peaks at obstacle locations).

% Configuration Space:  x-axis- 0...50
%                       y-axis- 0...50

% INPUTS
% robot_vel (1x1)- magnitude of longitudinal robot velocity
% robot_pos (2x1)- robot position
% h- Sampling period


%% Constant Definition

% Attractive Potential Constant
k_att= 1;

% Repulsive Potential Constant
k_rep= 500;

% Obstacle Radius (Assuming circular obstacle of constant radius)
obs_radius= 2;

% Maximum Longitudinal Velocity
V_MAX= robot_vel;  % m/sec

% Maximum Longitudinal Deceleration
A_MAX= 0.4;   % m/sec^2

% Obstacle Influence Range
rol_not= V_MAX/(2*A_MAX);

% Stopping Criteria
stopping_criteria= 0.2;   % Stop when Attractive Potential is at/lower than this value

% Robot Size Alllowance
robot_size_allowance= 1.50; % 150 cm

%% Define Goal
X_goal= [40 5]';


%% Plot Attractive Potential
x_plot_sub= 0:0.2:50;
y_plot_sub= 0:0.2:50;

[x_plot, y_plot]= meshgrid(x_plot_sub, y_plot_sub);

z_plot_att= 0.5 .* k_att .* ( (x_plot-X_goal(1)).^2 + (y_plot-X_goal(2)).^2 );

% mesh(x_plot, y_plot, z_plot_att);

% Hold Plot
% hold on


%% Define Obstacle Positions
obstacles= [ [14.87 33.28]' [32 30]' [19 19]' [34.49 17.25]' ];

%% Plot Universal Potential

% Maximum Repulsive Potential
max_rep_pot= max(max(z_plot_att));

% Initialize to Attractive Potential
z_plot_uni= z_plot_att;

% Initialize Obstacle Counter
obs_counter= 1;

% Get number of obstacles (number of columns of *obstacles*)
size_of_obstacles= size(obstacles);
no_of_obstacles= size_of_obstacles(2);

% Loop through obstacles
while (obs_counter <= no_of_obstacles)
   
     % Current Obstacle
     current_obs= obstacles(:, obs_counter);
     
    % Loop through y_plot_sub
    for y_index= 1:length(y_plot_sub)

        % Loop through x_plot_sub
        for x_index= 1:length(x_plot_sub)

            % Calculate Minimum Distance to obstacle
            min_dis_plot= sqrt( (x_plot_sub(x_index)-current_obs(1))^2 + ... 
                            (y_plot_sub(y_index)-current_obs(2))^2 )- ...
                            (obs_radius + robot_size_allowance);
            
           % If point is within obstacle range
           if(min_dis_plot <= rol_not)
               
                % If minimum obstacle distance is zero or negative (i.e. robot is at obstacle)
                if(min_dis_plot <= 0)
                   %Set maximum repulsive potential
                   z_plot_temp= max_rep_pot;
                   % Add to Universal Potential
                   z_plot_uni(y_index, x_index)= z_plot_uni(y_index, x_index) + z_plot_temp;

                % Otherwise
                else
                   % Compute Repulsive Potential
                   z_plot_temp= 0.5 * k_rep * ( (1/min_dis_plot) -...
                                            (1/rol_not) )^2;  

                   % Clip Repulsive Potential to max of attractive potential
                   if(z_plot_temp > max_rep_pot)
                       z_plot_temp= max_rep_pot;
                   end                                                                            
                   % Add to Universal Potential
                   z_plot_uni(y_index, x_index)= z_plot_uni(y_index, x_index) + z_plot_temp;
                end
               
           end

        end

    end
    
    obs_counter= obs_counter + 1;
    
    
end

% Plot Universal Potential for obstacles
subplot(2, 2, 1);
mesh(x_plot, y_plot, z_plot_uni);
title('Universal Potential Field');
xlabel("x-axis (fixed earth)");
ylabel("y-axis (fixed earth)");
hold on

%% Robot Position
X_robot= robot_pos;

% Plot Robot initial position
pos_plot= plot3(X_robot(1), X_robot(2), max(max(z_plot_att)), '*m', 'MarkerSize', 13 );
hold on

% Plot marker at goal point
goal_plot= plot(X_goal(1), X_goal(2), '*r', 'MarkerSize', 13 );
legend([pos_plot goal_plot], "Start Position", "Goal Position");
legend("Location", "northwest");

% Initialize trajectory. Preallocate zeros to optimize run speed
trajectory= zeros(4, 1000);

% Initailize timestep
trajectory_index= 1;    
timestep= trajectory_index-1;

% Plot Universal Potential upon which planned path will be plotted
subplot(2, 2, 2);
mesh(x_plot, y_plot, z_plot_uni);
title('Path Planning (Gradient Descent)');
xlabel("x-axis (fixed earth)");
ylabel("y-axis (fixed earth)");
%colorbar
hold on

%% Forever Loop
while 1
   
    % Calculate Attractive Potential and Force
    Ua= 0.5 .* k_att .* (X_robot - X_goal)' * (X_robot - X_goal);
    %fprintf('Ua: %.2f\n', Ua);
    Fa= -k_att .* (X_robot - X_goal);
    %fprintf('Fa: %.2f\n', Fa);
    
    % Initialize Repulsive Potential and Force to zero
    Ur= 0;
    Fr= 0;
    
    % Initialize Obstacle Counter
    obs_counter= 1;
    
    % Loop through obstacles
    while (obs_counter <= no_of_obstacles)
        
        % Current Obstacle
        current_obs= obstacles(:, obs_counter);
        
        % Calculate minimum distance to obstacle center
        obs_min_dis= sqrt( (X_robot - current_obs)' *  ...
                       (X_robot - current_obs)     ...
                      );
                    
        % Factor in obstacle radius
        obs_min_dis= obs_min_dis - (obs_radius + robot_size_allowance);
                    
        % Check if Robot is within obstacle influence range
        if(obs_min_dis <= rol_not)
            
            % If yes, calculate Repulsive Potential for obstacle
            Ur_obs= 0.5 * k_rep * ( (1/obs_min_dis) - (1/rol_not) )^2;
            
            % Calculate Repulsive FOrce for obstacle
            Fr_obs= k_rep .* ( (1/obs_min_dis) - (1/rol_not) ) .* ...
                    (1/(obs_min_dis^3)) .* (X_robot - current_obs);
            
            % Update Repulsive Potential and Force
            Ur= Ur + Ur_obs;
            Fr= Fr + Fr_obs;
            %fprintf('Fr: %.2f\n', Fr);
            
        end
        
        % Increment obstacle counter
        obs_counter= obs_counter + 1;
        
    end
    
    % Evaluate Universal Potential
    Uuni= Ua + Ur;
    %fprintf('Uuni: %.2f\n', Uuni);
    
    % Check if stopping criteria is satisfied
    if(Ua <= stopping_criteria)
        % break the forever loop
        break;
    end
    
    % If stopping criteria is not satisfied, Evaluate Universal Force
    Funi= Fa + Fr;
    
    % Log Current Robot Position into trajectory
    trajectory(1:3,trajectory_index)= [(timestep*h) X_robot(1) X_robot(2)]';
    %fprintf( 'Robot position: [%.2f,%.2f]\n\n', X_robot(1), X_robot(2) );

    
    % Scale Movement
    movement= scale_movement(Funi, V_MAX, h);
    
    % Update Robot Position
    X_robot= X_robot + movement;
    
    % Update timestep
    trajectory_index= trajectory_index + 1;
    timestep= timestep + 1;
    
    % Log yaw into trajectory
    trajectory= add_yaw_to_trajectory(trajectory, trajectory_index, X_robot);
    
    % Plot Robot Position and corresponding Universal Potential
    plot3(X_robot(1), X_robot(2), Uuni, '*k', 'MarkerSize', 10);
    
    % Plot Robot Position and corresponding Attractive Potential
    % plot3(X_robot(1), X_robot(2), Ua, '*k', 'MarkerSize', 10);
    
    % Repeat Loop
    
end

last_pos= plot3(X_robot(1), X_robot(2), Uuni, '*k', 'MarkerSize', 10);
% Legend Robot Position
legend(last_pos, "Robot Position");
legend("Location", "northwest");

% Release Plot
hold off


% Set all zero columns to empty vectors, effectively removing them from the
% array
trajectory( :, all(~trajectory,1) )= [];

end









