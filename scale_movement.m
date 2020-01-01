function movement = scale_movement(Funi, long_vel, h)

%SCALE_MOVEMENT Scales the movement of Robot based on its set lonigtudinal
%velocity which in this case corresponds to the resultant of the components
%of vector Funi

% Timesteps per second
% k_per_sec= 10;

% Velocity per time step
k_vel= long_vel*h;

% y coordinate
y_mov= Funi(2);
% x coordinate
x_mov= Funi(1);

% Calculate angle between resultant vector and x coordinate
theta= atan2(y_mov,x_mov);

% Scale y coordinate based on longitudinal velocity of robot per timestep
movement(2)= k_vel*sin(theta);

% Scale x coordinate based on longitudinal velocity of robot per timestep
movement(1)= k_vel*cos(theta);

movement= movement';

end

