g = 9.81;

% Rocket Parameters from the paper
total_mass = 82.9;
dry_mass = 40;
rocket_length = 3.57;
max_diameter = 24*10^-2;
cross_sectional_area = pi*max_diameter^2 /4;

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

ROOTCHORD_FIN = 85*10^-3;
TIPCHORD_FIN = 50*10^-3;
%AVERAGECHORD_FIN = (ROOTCHORD_FIN + TIPCHORD_FIN)/2;
%THICKNESS_FIN = 0.238*10^-3;
HEIGHT_FIN = 53*10^-3;
MIDCHORD_FIN = 53*10^-3;

CN_NOSE = 2;
CN_BODY = 0;
CN_FINS = 16*(HEIGHT_FIN/max_diameter)^2 / (1 + sqrt(1 + (2*MIDCHORD_FIN / (ROOTCHORD_FIN + TIPCHORD_FIN))^2));
%correction 
K = 1  +TIPCHORD_FIN/(HEIGHT_FIN + TIPCHORD_FIN);
CN_INTERFERENCE = K*CN_FINS;
CN = CN_INTERFERENCE + CN_BODY + CN_NOSE + CN_FINS;

kinematic_viscosity = 1.495*10^-5;
J_vec = [0.034, 6.3, 6.3];