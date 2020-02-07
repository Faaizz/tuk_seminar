function trajectory= trajectory_generator_apf(robot_vel, robot_pos, h)
% Generate trajectory which directs a robot to its
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
stopping_criteria= 0.005;   % Stop when Attractive Potential is at/lower than this value

% Robot Size Alllowance
robot_size_allowance= 1.50; % 150 cm

%% Define Goal
X_goal= [40 5]';


%% Define Obstacle Positions
obstacles= [ [14.87 33.28]' [32 30]' [19 19]' [34.49 17.25]' ];

% Get number of obstacles (number of columns of *obstacles*)
size_of_obstacles= size(obstacles);
no_of_obstacles= size_of_obstacles(2);

%% Robot Position
X_robot= robot_pos;

% Initialize trajectory. Preallocate zeros to optimize run speed
trajectory= zeros(3, 1000);

% Initailize timestep
trajectory_index= 1;    
timestep= trajectory_index-1;


%% Forever Loop

% Initialize loop_counter
loop_counter= 0;

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
     
 
    % If 2000 iterations have been performed without getting to the goal 
    % point, set trajectory to zero and exit the loop
    if(loop_counter >= 2000)
        trajectory= zeros(3, 1000);
        break;
    end
    
    % Repeat Loop
    loop_counter= loop_counter + 1;
    
end


% Set all zero columns to empty vectors, effectively removing them from the
% array
trajectory( :, all(~trajectory,1) )= [];

end









