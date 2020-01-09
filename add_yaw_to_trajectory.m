function trajectory= add_yaw_to_trajectory(trajectory, traj_index, new_pos)

% This function computes the yaw and adds it to trajectory
% INPUTS
% - trajectory (4xn) matrix
% - traj_index: updated trajectory index
% - new_pos: new robot coorodinate

if(traj_index >= 3)
   
    % Get lower bound
    lower= trajectory(1:2, traj_index-2);
    % Get upper bound
    upper= new_pos;
    
    % Compute yaw angle
    yaw_angle= ( upper(2) - lower(2) )/ ( upper(1) - lower(1) );
    
    % Update trajectory to include yaw angle
    trajectory(4, (traj_index-1))= yaw_angle;
    
end

end