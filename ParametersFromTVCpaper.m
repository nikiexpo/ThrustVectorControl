g = 9.81;

% Rocket Parameters from the paper
total_mass = 82.9;
dry_mass = 40;
rocket_length = 3.57;
max_diameter = 24*10^-2;

% Trajectory Parameters
apogee = 4945;
max_velocity = 82;
max_acceleration = 1.7;
time_to_apogee = 100;
velocity_at_burnout = 27;

% Assumptions (not from paper)
center_of_mass = rocket_length / 2;
torque_arm = center_of_mass; %l
Thrust = 50; %assuming constant now, check if this value is sensible